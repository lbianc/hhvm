/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | Copyright (c) 2010-2013 Facebook, Inc. (http://www.facebook.com)     |
   +----------------------------------------------------------------------+
   | This source file is subject to version 3.01 of the PHP license,      |
   | that is bundled with this package in the file LICENSE, and is        |
   | available through the world-wide-web at the following url:           |
   | http://www.php.net/license/3_01.txt                                  |
   | If you did not receive a copy of the PHP license and are unable to   |
   | obtain it through the world-wide-web, please send a note to          |
   | license@php.net so we can mail you a copy immediately.               |
   +----------------------------------------------------------------------+
*/

#include "hphp/runtime/vm/jit/vasm-emit.h"

#include "hphp/runtime/base/arch.h"
#include "hphp/runtime/vm/jit/abi-ppc64.h"
#include "hphp/runtime/vm/jit/block.h"
#include "hphp/runtime/vm/jit/code-gen-helpers.h"
#include "hphp/runtime/vm/jit/func-guard-ppc64.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/print.h"
#include "hphp/runtime/vm/jit/prof-data.h"
#include "hphp/runtime/vm/jit/service-requests.h"
#include "hphp/runtime/vm/jit/smashable-instr-ppc64.h"
#include "hphp/runtime/vm/jit/target-cache.h"
#include "hphp/runtime/vm/jit/timer.h"
#include "hphp/runtime/vm/jit/vasm.h"
#include "hphp/runtime/vm/jit/vasm-instr.h"
#include "hphp/runtime/vm/jit/vasm-internal.h"
#include "hphp/runtime/vm/jit/vasm-lower.h"
#include "hphp/runtime/vm/jit/vasm-print.h"
#include "hphp/runtime/vm/jit/vasm-unit.h"
#include "hphp/runtime/vm/jit/vasm-util.h"
#include "hphp/runtime/vm/jit/vasm-visit.h"

#include <algorithm>
#include <tuple>

TRACE_SET_MOD(vasm);

namespace HPHP { namespace jit {

///////////////////////////////////////////////////////////////////////////////

using namespace ppc64;
using namespace ppc64_asm;

namespace {
///////////////////////////////////////////////////////////////////////////////

struct Vgen {
  explicit Vgen(Venv& env)
    : text(env.text)
    , assem(*env.cb)
    , a(&assem)
    , current(env.current)
    , next(env.next)
    , jmps(env.jmps)
    , jccs(env.jccs)
    , catches(env.catches)
  {}

  static void patch(Venv& env);
  static void pad(CodeBlock& cb);

  /////////////////////////////////////////////////////////////////////////////

  template<class Inst> void emit(const Inst& i) {
    always_assert_flog(false, "unimplemented instruction: {} in B{}\n",
                       vinst_names[Vinstr(i).op], size_t(current));
  }

  // intrinsics
  void emit(const copy& i) {
    if (i.s == i.d) return;
    if (i.s.isGP()) {
      if (i.d.isGP()) {                 // GP => GP
        a->mr(i.d, i.s);
      } else {                             // GP => XMM
        assertx(i.d.isSIMD());
        not_implemented();
      }
    } else {
      if (i.d.isGP()) {                 // XMM => GP
        not_implemented();
      } else {                             // XMM => XMM
        assertx(i.d.isSIMD());
        not_implemented();
      }
    }
  }
  void emit(const copy2& i) {
    assertx(i.s0.isValid() && i.s1.isValid() &&
            i.d0.isValid() && i.d1.isValid());
    auto s0 = i.s0, s1 = i.s1, d0 = i.d0, d1 = i.d1;
    assertx(d0 != d1);
    if (d0 == s1) {
      if (d1 == s0) {
        a->mr(rAsm, s1);
        a->mr(d0, s0);
        a->mr(d1, rAsm);
      } else {
        // could do this in a simplify pass
        if (s1 != d1) a->mr(d1, s1); // save s1 first; d1 != s0
        if (s0 != d0) a->mr(d0, s0);
      }
    } else {
      // could do this in a simplify pass
      if (s0 != d0) a->mr(d0, s0);
      if (s1 != d1) a->mr(d1, s1);
    }
  }
  void emit(const fallthru& i) {}
  void emit(const ldimmb& i) {
    if(i.d.isGP()) {
      a->li(i.d, i.s); // should be only 8bits available
    } else {
      // TODO(rcardoso): SIMD instruction
      not_implemented();
    }
  }
  void emit(const ldimmw& i) {
    if(i.d.isGP()) {
      a->li(Reg64(i.d), i.s); // should be only 16bits available
    } else {
      // TODO(rcardoso): SIMD instruction
      not_implemented();
    }
  }
  void emit(const ldimml& i) {
    // ldimml is for Vconst::Long, which is treated as unsigned uint32_t
    auto val = i.s.l();
    if (i.d.isGP()) {
      Vreg64 d = i.d;
      a->li32un(d,val);
    } else {
      // TODO(igornunes): SIMD instruction
      not_implemented();
    }
  }
  void emit(const ldimmq& i) {
    auto val = i.s.q();
    if (i.d.isGP()) {
      if (val == 0) {
        a->xor_(i.d, i.d, i.d);
        // emit nops to fill a standard li64 instruction block
        // this will be useful on patching and smashable operations
        a->emitNop(ppc64_asm::Assembler::kLi64InstrLen -
            1 * ppc64_asm::Assembler::kBytesPerInstr);
      } else {
        a->li64(i.d, val);
      }
    } else {
      not_implemented();
    }
  }
  void emit(const ldimmqs& i) { emitSmashableMovq(a->code(), i.s.q(), i.d); }
  void emit(const nothrow& i) {
    mcg->registerCatchBlock(a->frontier(), nullptr);
  }
  void emit(const landingpad& i) {}

  // instructions
  void emit(addl i) { a->add(Reg64(i.d), Reg64(i.s1), Reg64(i.s0)); }
  void emit(addli i) { a->addi(Reg64(i.d), Reg64(i.s1), i.s0); }
  void emit(addq i) { a->add(i.d, i.s0, i.s1, false); }
  void emit(addqi i) { a->addi(i.d, i.s1, i.s0); }
  void emit(andli i) { a->andi(Reg64(i.d), Reg64(i.s1), i.s0); }
  void emit(andq i) { a->and_(i.d, i.s0, i.s1, false); }
  void emit(andqi i) { a->andi(i.d, i.s1, i.s0); }
  void emit(const cmpl& i) { a->cmpw(Reg64(i.s1), Reg64(i.s0)); }
  void emit(const cmpli& i) { a->cmpwi(Reg64(i.s1), i.s0); }
  void emit(const cmpq& i) { a->cmpd(i.s1, i.s0); }
  void emit(const cmpqi& i) { a->cmpdi(i.s1, i.s0); }
  void emit(const xscvdpsxds& i) { a->xscvdpsxds(i.d, i.s); }
  void emit(const xscvsxddp& i) { a->xscvsxddp(i.d, i.s); }
  void emit(const xxlxor& i) { a->xxlxor(i.d, i.s1, i.s0); }
  void emit(const xxpermdi& i) { a->xxpermdi(i.d, i.s1, i.s0); }
  void emit(const mfvsrd& i) { a->mfvsrd(i.d, i.s); }
  void emit(const mtvsrd& i) { a->mtvsrd(i.d, i.s); }
  void emit(decl i) { a->addi(Reg64(i.d), Reg64(i.s), -1); }
  void emit(decq i) { a->addi(i.d, i.s, -1); }
  void emit(imul i) { a->mullw(i.d, i.s1, i.s0, false); }
  void emit(const srem& i) { a->divd(i.d,  i.s0, i.s1, false); }
  void emit(incw i) { a->addi(Reg64(i.d), Reg64(i.s), 1); }
  void emit(incl i) { a->addi(Reg64(i.d), Reg64(i.s), 1); }
  void emit(incq i) { a->addi(i.d, i.s, 1); }
  void emit(const jmpi& i) {
    a->branchAuto(i.target, BranchConditions::Always, LinkReg::DoNotTouch);
  }
  void emit(const jmpr& i) {
    a->mtctr(i.target);
    a->bctr();
  }
  void emit(const leap& i) { a->li64(i.d, i.s.r.disp); }
  void emit(const loadups& i) { a->lxvw4x(i.d,i.s); }
  void emit(const loadtqb& i) { a->lbz(Reg64(i.d),i.s); }
  void emit(const mflr& i) { a->mflr(i.d); }
  void emit(const mtlr& i) { a->mtlr(i.s); }
  void emit(const movb& i) { a->ori(Reg64(i.d), Reg64(i.s), 0); }
  void emit(const movl& i) { a->ori(Reg64(i.d), Reg64(i.s), 0); }
  void emit(const movzbl& i) { a->ori(Reg64(i.d), Reg64(i.s), 0); }
  void emit(const movzbq& i) { a->ori(i.d, Reg64(i.s), 0); }
  void emit(neg i) { a->neg(i.d, i.s, false); }
  void emit(const nop& i) { a->ori(Reg64(0), Reg64(0), 0); } // no-op form
  void emit(not i) { a->nor(i.d, i.s, i.s, false); }
  void emit(orq i) { a->or_(i.d, i.s0, i.s1, false); }
  void emit(orqi i) { a->ori(i.d, i.s1, i.s0); }
  void emit(const roundsd& i) { a->xsrdpi(i.d, i.s); }
  void emit(const ret& i) {
    a->blr();
  }
  /*Immediate-form logical (unsigned) shift operations are
    obtained by specifying appropriate masks and shift values for
    certain Rotate instructions.
  */
  void emit(sar i) { a->srad(i.d, i.s1, i.s0); }
  void emit(sarqi i) { a->srawi(i.d, i.s1, Reg64(i.s0.w()), false); }
  void emit(const setcc& i) {
    ppc64_asm::Label l_true, l_end;
    Reg64 d(i.d);

    a->bc(l_true, i.cc);
    a->xor_(d, d, d);   /* set output to 0 */
    a->b(l_end);

    l_true.asm_label(*a);
    a->li(d, 1);        /* set output to 1 */

    l_end.asm_label(*a);
  }
  void emit(shlli i) { a->slwi(Reg64(i.d), Reg64(i.s1), i.s0.b()); }
  /*TODO Rc=1*/
  void emit(shl i) { a->sld(i.d, i.s1, i.s0); }
  void emit(shlqi i) { a->sldi(i.d, i.s1, i.s0.b()); }
  void emit(shrli i) { a->srwi(Reg64(i.d), Reg64(i.s1), i.s0.b()); }
  void emit(shrqi i) { a->srdi(i.d, i.s1, i.s0.b()); }
  void emit(const sqrtsd& i) { a->xssqrtdp(i.d,i.s); }
  void emit(const storeups& i) { a->stxvw4x(i.s,i.m); }
  void emit(const loadb& i) { a->lbz(Reg64(i.d), i.s); }

  // macro for commonlizing X-/D-form of load/store instructions
#define X(instr, dst, ptr)                                \
  do {                                                    \
    if (ptr.index.isValid()) a->instr##x(dst, ptr);       \
    else                     a->instr   (dst, ptr);       \
  } while(0)

  void emit(const loadw& i)   { X(lhz,  Reg64(i.d), i.s); }
  void emit(const loadl& i)   { X(lwz,  Reg64(i.d), i.s); }
  void emit(const loadzbl& i) { X(lbz,  Reg64(i.d), i.s); }
  void emit(const loadzbq& i) { X(lbz,  i.d,        i.s); }
  void emit(const loadzlq& i) { X(lwz,  i.d,        i.s); }
  void emit(const storeb& i)  { X(stb,  Reg64(i.s), i.m); }
  void emit(const storel& i)  { X(stw,  Reg64(i.s), i.m); }
  void emit(const storew& i)  { X(sth,  Reg64(i.s), i.m); }
  void emit(const storesd& i) { X(stfd, i.s,        i.m); }

#undef X

  void emit(subq i) { a->subf(i.d, i.s1, i.s0, false); }
  void emit(subqi i) { a->addi(i.s1, i.d, i.s0); /*addi with negative value*/ }
  void emit(subsd i) { a->fsub(i.d, i.s0, i.s1); /* d = s1 - s0 */ }
  void emit(const testq& i) {
    // More information on:
    // https://www.freelists.org/post/hhvm-ppc/Review-on-testb-vasm-change-aka-how-to-translate-x64s-test-operator-to-ppc64
    if (i.s0 != i.s1)
      a->and_(rAsm, i.s0, i.s1, true); // result is not used, only flags
    else
      a->cmpdi(i.s0, Immed(0));
  }
  void emit(const testqi& i) { a->andi(rAsm, i.s1, i.s0); }
  void emit(const ucomisd& i) { a->dcmpu(i.s0,i.s1); }
  void emit(const ud2& i) { a->trap(); }
  void emit(xorb i) { a->xor_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
  void emit(xorbi i) { a->xori(Reg64(i.d), Reg64(i.s1), i.s0); }
  void emit(xorl i) { a->xor_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
  void emit(xorq i) { a->xor_(i.d, i.s0, i.s1, false); }
  void emit(xorqi i) { a->xori(i.d, i.s1, i.s0); }

  // The following vasms reemit other vasms. They are implemented afterwards in
  // order to guarantee that the desired vasm is already defined or else it'll
  // fallback to the templated emit function.
  void emit(const callfaststub& i);
  void emit(const callstub& i);
  void emit(const jcc& i);
  void emit(const jcci& i);
  void emit(const jmp& i);
  void emit(const load& i);
  void emit(const pop& i);
  void emit(const push& i);
  void emit(const store& i);
  void emit(const mcprep&);
  void emit(const syncpoint& i);
  void emit(const unwind& i);
  void emit(const callphp&);
  void emit(const cmovq&);
  void emit(const leavetc&);

  // auxiliary for emit(call) and emit(callr)
  template<class Func>
  void callExtern(Func func);
  void emit(const call& i);
  void emit(const callr& i);
  void emit(const stublogue& i);
  void emit(const stubret& i);
  void emit(const tailcallstub& i);

private:
  CodeBlock& frozen() { return text.frozen().code; }

  Vtext& text;
  ppc64_asm::Assembler assem;
  ppc64_asm::Assembler* a;

  const Vlabel current;
  const Vlabel next;
  jit::vector<Venv::LabelPatch>& jmps;
  jit::vector<Venv::LabelPatch>& jccs;
  jit::vector<Venv::LabelPatch>& catches;
};

void Vgen::emit(const syncpoint& i) {
  FTRACE(5, "IR recordSyncPoint: {} {} {}\n", a->frontier(),
         i.fix.pcOffset, i.fix.spOffset);
  mcg->recordSyncPoint(a->frontier(), i.fix);
}

/*
 * Push/pop mechanism is as simple as X64: it stores 8 bytes below the SP.
 */
void Vgen::emit(const pop& i) {
  a->ld(i.d, rsp()[0]);                         // popped element
  a->addi(rsp(), rsp(), push_pop_position);     // recover stack
}
void Vgen::emit(const push& i) {
  a->stdu(i.s, rsp()[-push_pop_position]);      // pushed element
}

void Vgen::emit(const load& i) {
  if (i.d.isGP()) {
    if (i.s.index.isValid()){
      a->ldx(i.d, i.s);
    } else {
      a->ld(i.d, i.s);
    }
  } else {
    assertx(i.d.isSIMD());
    // TODO(rcardoso): Needs to check if needs to change to vec instruction
    a->lfs(i.d, i.s);
  }
}

void Vgen::emit(const callstub& i) {
  emit(call{ i.target, i.args });
}

void Vgen::emit(const callfaststub& i) {
  emit(call{i.target, i.args});
  emit(syncpoint{i.fix});
}
void Vgen::emit(const unwind& i) {
  catches.push_back({a->frontier(), i.targets[1]});
  emit(jmp{i.targets[0]});
}
void Vgen::emit(const jmp& i) {
  if (next == i.target) return;
  jmps.push_back({a->frontier(), i.target});

  // offset to be determined by a->patchBctr
  a->branchAuto(a->frontier());
}
void Vgen::emit(const jcc& i) {
  if (i.targets[1] != i.targets[0]) {
    if (next == i.targets[1]) {
      return emit(jcc{ccNegate(i.cc), i.sf, {i.targets[1], i.targets[0]}});
    }
    auto taken = i.targets[1];
    jccs.push_back({a->frontier(), taken});

    // offset to be determined by a->patchBctr
    a->branchAuto(a->frontier(), i.cc);
  }
  emit(jmp{i.targets[0]});
}
void Vgen::emit(const jcci& i) {
  a->branchAuto(i.taken, i.cc);
  emit(jmp{i.target});
}

void Vgen::emit(const cmovq& i) {
  BranchParams bp (i.cc);
  a->isel(i.d, i.t, i.f, bp.bi());
}

void Vgen::patch(Venv& env) {
  for (auto& p : env.jmps) {
    assertx(env.addrs[p.target]);
    ppc64_asm::Assembler::patchBctr(p.instr, env.addrs[p.target]);
  }
  for (auto& p : env.jccs) {
    assertx(env.addrs[p.target]);
    ppc64_asm::Assembler::patchBctr(p.instr, env.addrs[p.target]);
  }
  assertx(env.bccs.empty());
}

void Vgen::pad(CodeBlock& cb) {
  ppc64_asm::Assembler a {cb};
  while (a.available() >= 4) a.trap();
  assertx(a.available() == 0);
}

void Vgen::emit(const store& i) {
  if (i.s.isGP()) {
    if (i.d.index.isValid()){
      a->stdx(i.s, i.d);
    } else {
      a->std(i.s, i.d);
    }
  } else {
    assertx(i.s.isSIMD());
    // TODO(rcardoso): Needs to check if needs to change to vec instruction
    a->stfs(i.s, i.d);
  }
}

void Vgen::emit(const mcprep& i) {
  /*
   * Initially, we set the cache to hold (addr << 1) | 1 (where `addr' is the
   * address of the movq) so that we can find the movq from the handler.
   *
   * We set the low bit for two reasons: the Class* will never be a valid
   * Class*, so we'll always miss the inline check before it's smashed, and
   * handlePrimeCacheInit can tell it's not been smashed yet
   */
  auto const mov_addr = emitSmashableMovq(a->code(), 0, r64(i.d));
  auto const imm = reinterpret_cast<uint64_t>(mov_addr);
  smashMovq(mov_addr, (imm << 1) | 1);

  mcg->cgFixups().m_addressImmediates.insert(reinterpret_cast<TCA>(~imm));
}

void Vgen::emit(const callphp& i) {
  emitSmashableCall(a->code(), i.stub);
  emit(unwind{{i.targets[0], i.targets[1]}});
}

void Vgen::emit(const leavetc&) {
  emit(ret{});
}

template<class Func>
void Vgen::callExtern(Func func) {
  // save caller's return address to a new callstack as pushing would destroy
  // the ppc64's ABI call stack format
  a->stdu(rsp(), rsp()[-min_callstack_size]);

  a->mflr(rfuncln());
  a->std(rfuncln(), rsp()[lr_position_on_callstack]); // ABI expected position
  a->std(rfuncln(), rsp()[8]);         // ActRec->m_savedRip expected position
  a->stdu(rsp(), rsp()[-min_callstack_size]);

  // branch
  func();

  // pop caller's return address
  a->addi(rsp(), rsp(), min_callstack_size);
  a->ld(rfuncln(), rsp()[lr_position_on_callstack]);
  a->mtlr(rfuncln());

  // and restore the additional stack
  a->addi(rsp(), rsp(), min_callstack_size);
}

void Vgen::emit(const call& i) {
  callExtern([&]() {
      a->branchAuto(i.target, BranchConditions::Always, LinkReg::Save);
  });
}

void Vgen::emit(const callr& i) {
  callExtern([&]() {
      a->mtctr(i.target);
      a->bctrl();
  });
}

/////////////////////////////////////////////////////////////////////////////
/*
 * Stub function ABI
 */
/*
 * Unlike X64, the return address is not stored in the stack but on LR. Perform
 * the prologue simply by saving the rvmfp on current stack
 */
void Vgen::emit(const stublogue& i) {
  if (i.saveframe) {
    // will not be lowered, but this Vptr doesn't need to be patched, so it's ok
    emit(store{rvmfp(), rsp()[rvmfp_position_on_callstack]});
  }
}

void Vgen::emit(const stubret& i) {
  if (i.saveframe) {
    // will not be lowered, but this Vptr doesn't need to be patched, so it's ok
    emit(load{rsp()[rvmfp_position_on_callstack], rvmfp()});
  }
  emit(ret{});
}

void Vgen::emit(const tailcallstub& i) {
  emit(jmpi{i.target, i.args});
}

///////////////////////////////////////////////////////////////////////////////

/*
 * Native ppc64 instructions can't handle an immediate bigger than 16 bits and
 * need to be loaded into a register in order to be used.
 */
template <typename typeImm>
bool patchImm(typeImm imm, Vout& v, Vreg& tmpRegister) {
  uint64_t imm64 = static_cast<uint64_t>(imm);
  if (!(imm64 >> 16)) {
    return false;
  } else {
    tmpRegister  = v.makeReg();
    v << ldimmq{imm64, tmpRegister};
    return true;
  }
}

/*
 * Vptr struct supports fancy x64 addressing modes.
 * So we need to patch it to avoid ppc64el unsuported address modes.
 *
 * After patching, the Vptr @p will only have either base and index or base and
 * displacement.
 */
void patchVptr(Vptr& p, Vout& v) {
  // Map all address modes that Vptr can be so it can be handled.
  enum class AddressModes {
    kInvalid         = 0,
    kDisp            = 1, //            Displacement
    kBase            = 2, //       Base
    kBase_Disp       = 3, //       Base+Displacement
    kIndex           = 4, // Index
    kIndex_Disp      = 5, // Index     +Dispacement
    kIndex_Base      = 6, // Index+Base
    kIndex_Base_Disp = 7  // Index+Base+Displacement
  };

  AddressModes mode = static_cast<AddressModes>(
                    (((p.disp != 0)     & 0x1) << 0) |
                    ((p.base.isValid()  & 0x1) << 1) |
                    ((p.index.isValid() & 0x1) << 2));

  // Index can never be used directly if shifting is necessary. Handling it here
  uint8_t shift = p.scale == 2 ? 1 :
                  p.scale == 4 ? 2 :
                  p.scale == 8 ? 3 : 0;

  if (p.index.isValid() && shift) {
    Vreg shifted_index_reg = v.makeReg();
    v << shlqi{shift, p.index, shifted_index_reg, VregSF(RegSF{0})};
    p.index = shifted_index_reg;
    p.scale = 1;  // scale is now normalized.
  }

  // taking care of the displacement, in case it is > 16bits
  Vreg disp_reg;
  bool patched_disp = patchImm(p.disp, v, disp_reg);
  switch (mode) {
    case AddressModes::kBase:
    case AddressModes::kIndex_Base:
      // ppc64 can handle these address modes. Nothing to do here.
      break;

    case AddressModes::kBase_Disp:
      // ppc64 can handle this address mode if displacement < 16bits
      if (patched_disp) {
        // disp is loaded on a register. Change address mode to kBase_Index
        p.index = disp_reg;
        p.disp = 0;
      }
      break;

    case AddressModes::kIndex:
        // treat it as kBase to avoid a kIndex_Disp asm handling.
        std::swap(p.base, p.index);
      break;

    case AddressModes::kDisp:
    case AddressModes::kIndex_Disp:
      if (patched_disp) {
        // disp is loaded on a register. Change address mode to kBase_Index
        p.base = disp_reg;
        p.disp = 0;
      }
      else {
        // treat it as kBase_Disp to avoid a kIndex_Disp asm handling.
        p.base = p.index;
      }
      break;

    case AddressModes::kIndex_Base_Disp: {
      // This mode is not supported: Displacement will be embedded on Index
      Vreg index_disp_reg = v.makeReg();
      if (patched_disp) {
        v << addq{disp_reg, p.index, index_disp_reg, VregSF(RegSF{0})};
      } else {
        v << addqi{p.disp, p.index, index_disp_reg, VregSF(RegSF{0})};
      }
      p.index = index_disp_reg;
      p.disp = 0;
      break;
    }

    case AddressModes::kInvalid:
    default:
      assert(false && "Invalid address mode");
      break;
  }
}


/*
 * Rules for the lowering of these vasms:
 * 1) All vasms emitted in lowering are already adjusted/patched.
 *   In other words, it will not be lowered afterwards.
 * 2) If a vasm has a Vptr that can be removed by emitting load/store, do it!
 *
 * Parameter description for every lowering:
 * Vout& v : the Vout instance so vasms can be emitted
 * <Type> inst : the current vasm to be lowered
 */

/* fallback, when a vasm is not lowered */
template <typename Inst>
void lowerForPPC64(Vout& v, Inst& inst) {}

/*
 * Using macro to commonlize vasms lowering
 */

// Patches the Vptr, retrieve the immediate and emmit a related direct vasm
#define X(vasm_src, attr_data, vasm_dst, attr_addr, vasm_imm)           \
void lowerForPPC64(Vout& v, vasm_src& inst) {                           \
  Vreg tmp = v.makeReg();                                               \
  Vptr p = inst.attr_addr;                                              \
  patchVptr(p, v);                                                      \
  v << vasm_imm{inst.attr_data, tmp} << vasm_dst{tmp, p};               \
}

X(storebi, s, storeb, m, ldimmb)
X(storewi, s, storew, m, ldimmw)
X(storeli, s, storel, m, ldimml)
// X(storeqi, s, store,  m, ldimmq)  // not possible due to Immed64

#undef X

void lowerForPPC64(Vout& v, storeqi& inst) {
  auto ir = v.makeReg();
  v << ldimmq {Immed64(inst.s.q()), ir};

  Vptr p = inst.m;
  patchVptr(p, v);
  v << store {ir, p};
}

// Simply take care of the vasm's Vptr, reemmiting it if patch occured
#define X(vasm, attr_addr, attr_1, attr_2)                              \
void lowerForPPC64(Vout& v, vasm& inst) {                               \
  patchVptr(inst.attr_addr, v);                                         \
  if (!v.empty()) v << vasm{inst.attr_1, inst.attr_2};                  \
}

X(storeb,   m, s, m);
X(storew,   m, s, m);
X(storel,   m, s, m);
X(store,    d, s, d);
X(storeups, m, s, m);
X(storesd,  m, s, m);
X(load,     s, s, d);
X(loadl,    s, s, d);
X(loadzbl,  s, s, d);
X(loadzbq,  s, s, d);
X(loadzlq,  s, s, d);
X(loadups,  s, s, d);

#undef X

// Auxiliary macros to handle vasms with different attributes
#define NONE
#define ONE(attr_1)         inst.attr_1,
#define TWO(attr_1, attr_2) inst.attr_1, inst.attr_2,

// If it patches the Immed, replace the vasm for its non-immediate variant
#define X(vasm_src, vasm_dst, attr_imm, attrs)                          \
void lowerForPPC64(Vout& v, vasm_src& inst) {                           \
  Vreg tmp;                                                             \
  if (patchImm(inst.attr_imm, v, tmp)) v << vasm_dst{tmp, attrs inst.sf}; \
}

X(addli,  addl,  s0.l(), TWO(s1, d))
X(addqi,  addq,  s0.q(), TWO(s1, d))
X(andli,  andl,  s0.l(), TWO(s1, d))
X(andqi,  andq,  s0.q(), TWO(s1, d))
X(testqi, testq, s0.q(), ONE(s1))
X(cmpqi,  cmpq,  s0.q(), ONE(s1))

#undef X

// Simplify MemoryRef vasm types by their direct variant as ppc64 can't
// change data directly in memory. Patches the Vptr, grab and save the data.
#define X(vasm_src, vasm_dst, vasm_load, vasm_store, attr_addr, attrs)  \
void lowerForPPC64(Vout& v, vasm_src& inst) {                           \
  Vreg tmp = v.makeReg(), tmp2 = v.makeReg();                           \
  Vptr p = inst.attr_addr;                                              \
  patchVptr(p, v);                                                      \
  v << vasm_load{p, tmp} << vasm_dst{attrs tmp, tmp2, inst.sf};         \
  v << vasm_store{tmp2, p};                                             \
}

X(incwm, incw, loadw, storew, m, NONE)
X(inclm, incl, loadl, storel, m, NONE)
X(incqm, incq, load,  store,  m, NONE)
X(declm, decl, loadl, storel, m, NONE)
X(decqm, decq, load,  store,  m, NONE)
X(addlm, addl, loadw, storew, m, ONE(s0))

#undef X

#undef NONE
#undef ONE
#undef TWO

// Also deals with MemoryRef vasms like above but these ones have Immed data
// too. Load data and emit a new vasm depending if the Immed fits a direct
// ppc64 instruction.
#define X(vasm_src, vasm_dst_reg, vasm_dst_imm, vasm_load,              \
                  attr_addr, attr_data)                                 \
void lowerForPPC64(Vout& v, vasm_src& inst) {                           \
  Vptr p = inst.attr_addr;                                              \
  patchVptr(p, v);                                                      \
  Vreg tmp2 = v.makeReg(), tmp;                                         \
  v << vasm_load{p, tmp2};                                              \
  if (patchImm(inst.attr_data.q(), v, tmp))                             \
    v << vasm_dst_reg{tmp, tmp2, inst.sf};                              \
  else v << vasm_dst_imm{inst.attr_data, tmp2, inst.sf};                \
}

X(cmpbim,  cmpl,  cmpli,  loadb, s1, s0)
X(cmplim,  cmpl,  cmpli,  loadl, s1, s0)
X(cmpqim,  cmpq,  cmpqi,  load,  s1, s0)
X(testbim, testq, testqi, loadb, s1, s0)
X(testwim, testq, testqi, loadw, s1, s0)
X(testlim, testq, testqi, loadl, s1, s0)
X(testqim, testq, testqi, load,  s1, s0)

#undef X

// Very similar with the above case: handles MemoryRef and Immed, but also
// stores the result in the memory.
#define X(vasm_src, vasm_dst_reg, vasm_dst_imm, vasm_load, vasm_store,  \
                  attr_addr, attr_data)                                 \
void lowerForPPC64(Vout& v, vasm_src& inst) {                           \
  Vreg tmp = v.makeReg(), tmp3 = v.makeReg(), tmp2;                     \
  Vptr p = inst.attr_addr;                                              \
  patchVptr(p, v);                                                      \
  v << vasm_load{p, tmp};                                               \
  if (patchImm(inst.attr_data.q(), v, tmp2))                            \
    v << vasm_dst_reg{tmp2, tmp, tmp3, inst.sf};                        \
  else v << vasm_dst_imm{inst.attr_data, tmp, tmp3, inst.sf};           \
  v << vasm_store{tmp3, p};                                             \
}

X(orwim,   orq,  orqi,  loadw, storew, m, s0)
X(orqim,   orq,  orqi,  load,  store,  m, s0)
X(addqim,  addq, addqi, load,  store,  m, s0)

#undef X

// Handles MemoryRef arguments and load the data input from memory, but these
// ones have no output other than the sign flag register update (SF)
#define X(vasm_src, vasm_dst, vasm_load, attr_addr, attr)               \
void lowerForPPC64(Vout& v, vasm_src& inst) {                           \
  Vptr p = inst.attr_addr;                                              \
  patchVptr(p, v);                                                      \
  Vreg tmp = v.makeReg();                                               \
  v << vasm_load{p, tmp} << vasm_dst{inst.attr, tmp, inst.sf};          \
}

X(testqm, testq, load,  s1, s0)
X(cmplm,  cmpl,  loadl, s1, s0)
X(cmpqm,  cmpq,  load,  s1, s0)

#undef X

// Other lowers that didn't fit the macros above or are not so numerous.
void lowerForPPC64(Vout& v, jmpm& inst) {
  Vptr p = inst.target;
  patchVptr(p, v);
  Vreg tmp = v.makeReg();
  v << load{p, tmp};
  v << jmpr{tmp, inst.args};
}

void lowerForPPC64(Vout& v, callm& inst) {
  Vptr p = inst.target;
  patchVptr(p, v);
  auto d = v.makeReg();
  v << load{p, d};
  v << callr{d, inst.args};
}

void lowerForPPC64(Vout& v, lea& inst) {
  // could do this in a simplify pass
  if (inst.s.disp == 0 && inst.s.base.isValid() && !inst.s.index.isValid()) {
    v << copy{inst.s.base, inst.d};
  } else {
    Vptr p = inst.s;
    patchVptr(p, v);

    if (p.index.isValid()) {
      v << addq{p.base, p.index, inst.d, VregSF(RegSF{0})};
    } else {
      v << addqi{p.disp, p.base, inst.d, VregSF(RegSF{0})};
    }
  }
}

void lowerForPPC64(Vout& v, loadqp& inst) {
  // in PPC we don't have anything like a RIP register
  // RIP register uses a absolute address so we can perform a baseless load in
  // this case
  Vptr p = baseless(inst.s.r.disp);
  patchVptr(p, v);
  v << load{p, inst.d};
}

void lowerForPPC64(Vout& v, popm& inst) {
  auto tmp = v.makeReg();
  patchVptr(inst.d, v);
  v << pop{tmp};
  v << store{tmp, inst.d};
}

void lowerForPPC64(Vout& v, countbytecode& inst) {
  v << incqm{inst.base[g_bytecodesVasm.handle()], inst.sf};
}

/////////////////////////////////////////////////////////////////////////////
/*
 * PHP function ABI
 */
void lowerForPPC64(Vout& v, phplogue& inst) {
  Vreg ret_address = v.makeReg();
  v << mflr{ret_address};
  v << store{ret_address, inst.fp[AROFF(m_savedRip)]};
}

void lowerForPPC64(Vout& v, phpret& inst) {
  Vreg tmp = v.makeReg();
  v << load{inst.fp[AROFF(m_savedRip)], tmp};
  if (!inst.noframe) {
    v << load{inst.fp[AROFF(m_sfp)], inst.d};
  }
  v << jmpr{tmp};
}

/*
 * Tail call elimination on ppc64: call without creating a stack and keep LR
 * contents as prior to the call.
 */
void lowerForPPC64(Vout& v, tailcallphp& inst) {
  Vreg new_return = v.makeReg();
  v << load{inst.fp[AROFF(m_savedRip)], new_return};
  v << mtlr{new_return};
  v << jmpr{inst.target, inst.args};
}

/////////////////////////////////////////////////////////////////////////////

// Lower movs to copy
void lowerForPPC64(Vout& v, movtqb& inst) { v << copy{inst.s, inst.d}; }
void lowerForPPC64(Vout& v, movtql& inst) { v << copy{inst.s, inst.d}; }

// Lower comparison to cmpq
void lowerForPPC64(Vout& v, cmpb& inst) {
  v << cmpq{Reg64(inst.s0), Reg64(inst.s1), inst.sf};
}
void lowerForPPC64(Vout& v, cmpl& inst) {
  v << cmpq{Reg64(inst.s0), Reg64(inst.s1), inst.sf};
}

// Lower comparison with immediate to cmpqi
void lowerForPPC64(Vout& v, cmpbi& inst) {
  v << cmpqi{inst.s0, Reg64(inst.s1), inst.sf};
}
void lowerForPPC64(Vout& v, cmpli& inst) {
  // convert cmpli to cmpqi or ldimmq + cmpq by cmpqi's lowering
  auto lowered = cmpqi{inst.s0, Reg64(inst.s1), inst.sf};
  lowerForPPC64(v, lowered);
  if (v.empty()) v << lowered;
}

void lowerForPPC64(Vout& v, cvttsd2siq& inst) {
  auto tmp = v.makeReg(); //to be used as a 128-bit register

  //round double-precision scalar to signed integer
  v << xscvdpsxds{inst.s, tmp};

  //move from VSR (128-bit) to GPR (64-bit)
  v << mfvsrd{tmp, inst.d};
}

void lowerForPPC64(Vout& v, cvtsi2sd& inst) {

  // 128-bit scratch registers
  auto tmp0 = v.makeReg();
  auto tmp1 = v.makeReg();
  auto tmp2 = v.makeReg();

  // Move integer from GPR (64-bit) to VSR (128-bit).
  v << mtvsrd{inst.s, tmp0};

  // Convert integer to double-precision FP. High doubleword change
  // to undefined state, low doubleword contains DP FP.
  v << xscvsxddp{tmp0, tmp1};

  // Zero register just to use its high doubleword element in
  // permutation (see next instruction), as the convertion from integer
  // set high doubleword element to undefined state.
  // Use tmp0 value to zero tmp2 as we can not use something
  // similar to pure asm in lowering, like 'xxlxor r1,r1,r1'
  // to zero a register.
  v << xxlxor{tmp0,tmp0,tmp2};

  // Permute. Get low doubleword (double-precision DP) from tmp1, and
  // high doubleword (zero) from tmp2.
  v << xxpermdi{tmp2,tmp1,inst.d};
}

// Lower subtraction to subq
void lowerForPPC64(Vout& v, subl& inst) {
  v << subq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
}
void lowerForPPC64(Vout& v, subbi& inst) {
  v << subqi{inst.s0, Reg64(inst.s1), Reg64(inst.d), inst.sf};
}
void lowerForPPC64(Vout& v, subli& inst) {
  v << subqi{inst.s0, Reg64(inst.s1), Reg64(inst.d), inst.sf};
}

// Lower test to testq
void lowerForPPC64(Vout& v, testb& inst) {
  v << testq{Reg64(inst.s0), Reg64(inst.s1), inst.sf};
}
void lowerForPPC64(Vout& v, testl& inst) {
  v << testq{Reg64(inst.s0), Reg64(inst.s1), inst.sf};
}
void lowerForPPC64(Vout& v, testbi& inst) {
  // convert testbi to testqi or ldimmq + testq by testqi's lowering
  auto lowered = testqi{inst.s0, Reg64(inst.s1), inst.sf};
  lowerForPPC64(v, lowered);
  if (v.empty()) v << lowered;
}
void lowerForPPC64(Vout& v, testli& inst) {
  // convert testli to testqi or ldimmq + testq by testqi's lowering
  auto lowered = testqi{inst.s0, Reg64(inst.s1), inst.sf};
  lowerForPPC64(v, lowered);
  if (v.empty()) v << lowered;
}

// Lower xor to xorq
void lowerForPPC64(Vout& v, xorb& inst) {
  v << xorq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
}
void lowerForPPC64(Vout& v, xorl& inst) {
  v << xorq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
}

// Lower xor with immediate to xorqi
void lowerForPPC64(Vout& v, xorbi& inst) {
  v << xorqi{inst.s0, Reg64(inst.s1), Reg64(inst.d), inst.sf};
}

// Lower and to andq
void lowerForPPC64(Vout& v, andb& inst) {
  v << andq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
}
void lowerForPPC64(Vout& v, andl& inst) {
  v << andq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
}
void lowerForPPC64(Vout& v, andbi& inst) {
  // patchImm doesn't need to be called as it should be < 8 bits
  v << andqi{inst.s0, Reg64(inst.s1), Reg64(inst.d), inst.sf};
}

void lowerForPPC64(Vout& v, cloadq& inst) {
  auto m = inst.t;
  patchVptr(m, v);
  auto tmp = v.makeReg();
  v << load{m, tmp};
  v << cmovq{inst.cc, inst.sf, inst.f, tmp, inst.d};
}

void lower_vcallarray(Vunit& unit, Vlabel b) {
  auto& code = unit.blocks[b].code;
  // vcallarray can only appear at the end of a block.
  auto const inst = code.back().get<vcallarray>();
  auto const origin = code.back().origin;

  auto argRegs = inst.args;
  auto const& srcs = unit.tuples[inst.extraArgs];
  jit::vector<Vreg> dsts;
  for (int i = 0; i < srcs.size(); ++i) {
    dsts.emplace_back(rarg(i));
    argRegs |= rarg(i);
  }

  code.back() = copyargs{unit.makeTuple(srcs), unit.makeTuple(std::move(dsts))};
  code.emplace_back(callarray{inst.target, argRegs});
  code.back().origin = origin;
  code.emplace_back(unwind{{inst.targets[0], inst.targets[1]}});
  code.back().origin = origin;
}


/*
 * Lower a few abstractions to facilitate straightforward PPC64 codegen.
 * PPC64 doesn't have instructions for operating on less than 64 bits data
 * (except the memory related load/store), therefore all arithmetic vasms
 * that intend to deal with smaller data will actually operate on 64bits
 */
void lowerForPPC64(Vunit& unit) {
  Timer _t(Timer::vasm_lower);

  // This pass relies on having no critical edges in the unit.
  splitCriticalEdges(unit);

  // Scratch block can change blocks allocation, hence cannot use regular
  // iterators.
  auto& blocks = unit.blocks;

  PostorderWalker{unit}.dfs([&] (Vlabel ib) {
    assertx(!blocks[ib].code.empty());
    auto& back = blocks[ib].code.back();
    if (back.op == Vinstr::vcallarray) {
      lower_vcallarray(unit, Vlabel{ib});
    }

    for (size_t ii = 0; ii < blocks[ib].code.size(); ++ii) {

      vlower(unit, ib, ii);

      auto scratch = unit.makeScratchBlock();
      auto& inst = blocks[ib].code[ii];
      SCOPE_EXIT {unit.freeScratchBlock(scratch);};
      Vout v(unit, scratch, inst.origin);

      switch (inst.op) {

        /*
         * Call every lowering and provide only what is necessary:
         * Vout& v : the Vout instance so vasms can be emitted
         * <Type> inst : the current vasm to be lowered
         *
         * If any vasm is emitted inside of the lower, then the current vasm
         * will be replaced by the vector_splice call below.
         */

#define O(name, imms, uses, defs)                         \
        case Vinstr::name:                                \
          lowerForPPC64(v, inst.name##_);                 \
          if (!v.empty()) {                               \
            vector_splice(unit.blocks[ib].code, ii, 1,    \
                          unit.blocks[scratch].code);     \
          }                                               \
          break;

          VASM_OPCODES

#undef O

      }
    }
  });

  printUnit(kVasmLowerLevel, "after lower for PPC64", unit);
}

///////////////////////////////////////////////////////////////////////////////
} // anonymous namespace

void optimizePPC64(Vunit& unit, const Abi& abi) {
  Timer timer(Timer::vasm_optimize);

  removeTrivialNops(unit);
  optimizePhis(unit);
  fuseBranches(unit);
  optimizeJmps(unit);
  optimizeExits(unit);

  lowerForPPC64(unit);

  simplify(unit);

#if 0 // TODO(gut): not needed?
  if (!unit.constToReg.empty()) {
    foldImms<x64::ImmFolder>(unit);
  }
#endif
  {
    Timer timer(Timer::vasm_copy);
    optimizeCopies(unit, abi);
  }
  if (unit.needsRegAlloc()) {
    Timer timer(Timer::vasm_xls);
    removeDeadCode(unit);
    allocateRegisters(unit, abi);
  }
  if (unit.blocks.size() > 1) {
    Timer timer(Timer::vasm_jumps);
    optimizeJmps(unit);
  }
}

void emitPPC64(const Vunit& unit, Vtext& text, AsmInfo* asmInfo) {
  Timer timer(Timer::vasm_gen);
  vasm_emit<Vgen>(unit, text, asmInfo);
}

///////////////////////////////////////////////////////////////////////////////
}}

