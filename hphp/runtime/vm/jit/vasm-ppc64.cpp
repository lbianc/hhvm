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
#include "hphp/runtime/vm/jit/back-end-ppc64.h"
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

namespace ppc64 { struct ImmFolder; }

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

  // auxiliary
  /*
   * Check algorithm on of pushMinCallStack and popMinCallStack on:
   * https://gist.github.com/gut/956d6431412aad0fc626
   */
  inline void pushMinCallStack(void) {
    a->mflr(ppc64::rfuncln());
    // LR on parent call frame
    Vptr p(ppc64::rsp(), lr_position_on_callstack);
    a->std(ppc64::rfuncln(), p);
    // minimum call stack
    p.disp = -min_callstack_size;

#if PPC64_HAS_PUSH_POP
    // Store the backchain after the last pushed element
    p.base = ppc64::rstktop();
    a->stdu(ppc64::rsp(), p);
    a->mr(ppc64::rsp(), ppc64::rstktop());
#else
    a->stdu(ppc64::rsp(), p);
#endif
  }

  inline void popMinCallStack(void) {
#if PPC64_HAS_PUSH_POP
    // after the minimum call stack the last pushed elements is found
    a->addi(ppc64::rstktop(), ppc64::rsp(), min_callstack_size);
    // use backchain to restore the stack pointer, as the size is unknown.
    Vptr pBackchain(ppc64::rsp(), 0);
    a->ld(ppc64::rsp(), pBackchain);
#else
    // minimum call stack
    a->addi(ppc64::rsp(), ppc64::rsp(), min_callstack_size);
#endif
    // recover LR from callstack
    Vptr p(ppc64::rsp(), lr_position_on_callstack);
    a->ld(ppc64::rfuncln(), p);
    a->mtlr(ppc64::rfuncln());
  }

  // intrinsics
  void emit(const callarray& i) { not_implemented(); } ;
  void emit(const callfaststub& i) {
    emit(call{i.target, i.args});
    emit(syncpoint{i.fix});
  }
  void emit(const contenter& i) { not_implemented(); }
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
        a->mr(ppc64::rAsm, s1);
        a->mr(d0, s0);
        a->mr(d1, ppc64::rAsm);
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
  void emit(const debugtrap& i) { not_implemented(); }
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
  void emit(const load& i);
  void emit(const mcprep& i) { not_implemented(); }
  void emit(const nothrow& i) {
    mcg->registerCatchBlock(a->frontier(), nullptr);
  }
  void emit(const store& i);
  void emit(const syncpoint& i);
  void emit(const unwind& i) {
    catches.push_back({a->frontier(), i.targets[1]});
    emit(jmp{i.targets[0]});
  }
  void emit(const landingpad& i) {}
  void emit(const leavetc&) { not_implemented(); }

  // instructions
  void emit(addl i) { a->add(Reg64(i.d), Reg64(i.s1), Reg64(i.s0)); }
  void emit(addli i) { a->addi(Reg64(i.d), Reg64(i.s1), i.s0); }
  void emit(addq i) { a->add(i.d, i.s0, i.s1, false); }
  void emit(addqi i) { a->addi(i.d, i.s1, i.s0); }
  void emit(addsd i) { not_implemented(); }
  void emit(const andbim& i) { not_implemented(); }
  void emit(andli i) { a->andi(Reg64(i.d), Reg64(i.s1), i.s0); }
  void emit(andq i) { a->and_(i.d, i.s0, i.s1, false); }
  void emit(andqi i) { a->andi(i.d, i.s1, i.s0); }
  void emit(const call& i) {
    // Need to create a new call stack in order to recover LR in the future
    pushMinCallStack();

    a->branchAuto(i.target, BranchConditions::Always, LinkReg::Save);

    popMinCallStack();
  }
  void emit(const callr& i) {
    // Need to create a new call stack in order to recover LR in the future
    pushMinCallStack();

    a->mtctr(i.target);
    a->bctrl();

    popMinCallStack();
  }
  void emit(const cloadq& i) { not_implemented(); }
  void emit(const cmovq& i) { not_implemented(); }
  void emit(const cmpl& i) { a->cmpw(Reg64(i.s1), Reg64(i.s0)); }
  void emit(const cmpli& i) { a->cmpwi(Reg64(i.s1), i.s0); }
  void emit(const cmpq& i) { a->cmpd(i.s1, i.s0); }
  void emit(const cmpqi& i) { a->cmpdi(i.s1, i.s0); }
  void emit(cmpsd i) { not_implemented(); }
  void emit(const cqo& i) { not_implemented(); }
  void emit(const cvttsd2siq& i) { not_implemented(); }
  void emit(const cvtsi2sd& i) { not_implemented(); }
  void emit(const cvtsi2sdm& i) { not_implemented(); }
  void emit(decl i) { a->addi(Reg64(i.d), Reg64(i.s), -1); }
  void emit(decq i) { a->addi(i.d, i.s, -1); }
  void emit(divsd i) { not_implemented(); }
  void emit(imul i) { a->mullw(i.d, i.s1, i.s0, false); }
  void emit(const srem& i) { a->divd(i.d,  i.s0, i.s1, false); }
  void emit(incw i) { a->addi(Reg64(i.d), Reg64(i.s), 1); }
  void emit(incl i) { a->addi(Reg64(i.d), Reg64(i.s), 1); }
  void emit(incq i) { a->addi(i.d, i.s, 1); }
  void emit(const incqmlock& i) { not_implemented(); }
  void emit(const jcc& i) {
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
  void emit(const jcci& i) {
    a->branchAuto(i.taken, i.cc);
    emit(jmp{i.target});
  }
  void emit(const jmp& i) {
    if (next == i.target) return;
    jmps.push_back({a->frontier(), i.target});

    // offset to be determined by a->patchBctr
    a->branchAuto(a->frontier());
  }
  void emit(const jmpr& i) {
    a->mtctr(i.target);
    a->bctr();
  }
  void emit(const jmpi& i) {
    a->branchAuto(i.target, BranchConditions::Always, LinkReg::DoNotTouch);
  }
  void emit(const leap& i) { a->li64(i.d, i.s.r.disp); }
  void emit(const loadups& i) { a->lxvw4x(i.d,i.s); }
  void emit(const loadtqb& i) { a->lbz(Reg64(i.d),i.s); }
  void emit(const loadb& i) { a->lbz(Reg64(i.d),i.s); }
  void emit(const loadw& i) {
    if(i.s.index.isValid()) {
      a->lhzx(Reg64(i.d), i.s);
    } else {
      a->lhz(Reg64(i.d), i.s);
    }
  }
  void emit(const loadl& i) {
    if(i.s.index.isValid()) {
      a->lwzx(Reg64(i.d), i.s);
    } else {
      a->lwz(Reg64(i.d), i.s);
    }
  }
  void emit(const loadqp& i) { not_implemented(); }
  void emit(const loadsd& i) { not_implemented(); }
  void emit(const loadzbl& i) {
    if(i.s.index.isValid()) {
      a->lbzx(Reg64(i.d), i.s);
    } else {
      a->lbz(Reg64(i.d), i.s);
    }
  }
  void emit(const loadzbq& i) {
    if(i.s.index.isValid()) {
      a->lbzx(i.d, i.s);
    } else {
      a->lbz(i.d, i.s);
    }
  }
  void emit(const loadzlq& i) {
    if(i.s.index.isValid()) {
      a->lwzx(i.d, i.s);
    } else {
      a->lwz(i.d, i.s);
    }
  }
  void emit(movb& i) { a->ori(Reg64(i.d), Reg64(i.s), 0); }
  void emit(movl& i) { a->ori(Reg64(i.d), Reg64(i.s), 0); }
  void emit(movzbl& i) { a->ori(Reg64(i.d), Reg64(i.s), 0); }
  void emit(movzbq& i) { a->ori(i.d, Reg64(i.s), 0); }
  void emit(mulsd i) { not_implemented(); }
  void emit(neg i) { a->neg(i.d, i.s, false); }
  void emit(const nop& i) { a->ori(Reg64(0), Reg64(0), 0); } // no-op form
  void emit(not i) { a->nor(i.d, i.s, i.s, false); }
  void emit(notb i) { not_implemented(); }
  void emit(orq i) { a->or_(i.d, i.s0, i.s1, false); }
  void emit(orqi i) { a->ori(i.d, i.s1, i.s0); }
  void emit(const orqim& i) { not_implemented(); }
  void emit(const pop& i);
  void emit(psllq i) { not_implemented(); }
  void emit(psrlq i) { not_implemented(); }
  void emit(const push& i);
  void emit(const roundsd& i) { a->xsrdpi(i.d, i.s); }
  void emit(const ret& i) {
    // LR on parent call frame
    Vptr p(ppc64::rsp(), lr_position_on_callstack);
    a->ld(ppc64::rfuncln(), p);
    a->mtlr(ppc64::rfuncln());
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
  void emit(const storeb& i) {
    if(i.m.index.isValid()) {
      a->stbx(Reg64(i.s), i.m);
    } else {
      a->stb(Reg64(i.s), i.m);
    }
  }
  void emit(const storel& i) {
    if(i.m.index.isValid()) {
      a->stwx(Reg64(i.s), i.m);
    } else {
      a->stw(Reg64(i.s), i.m);
    }
  }
  void emit(const storesd& i) {
    if(i.m.index.isValid()) {
      a->stfdx(i.s, i.m);
    } else {
      a->stfd(i.s, i.m);
    }
  }
  void emit(const storew& i) {
    if(i.m.index.isValid()) {
      a->sthx(Reg64(i.s), i.m);
    } else {
      a->sth(Reg64(i.s), i.m);
    }
  }
  void emit(subq i) { a->subf(i.d, i.s1, i.s0, false); }
  void emit(subqi i) { a->addi(i.s1, i.d, i.s0); /*addi with negative value*/ }
  void emit(subsd i) { a->fsub(i.d, i.s0, i.s1); /* d = s1 - s0 */ }
  void emit(const testq& i) {
    // More information on:
    // https://www.freelists.org/post/hhvm-ppc/Review-on-testb-vasm-change-aka-how-to-translate-x64s-test-operator-to-ppc64
    if (i.s0 != i.s1)
      a->and_(ppc64::rAsm, i.s0, i.s1, true); // result is not used, only flags
    else
      a->cmpdi(i.s0, Immed(0));
  }
  void emit(const testqi& i) { a->andi(ppc64::rAsm, i.s1, i.s0); }
  void emit(const ucomisd& i) { a->dcmpu(i.s0,i.s1); }
  void emit(const ud2& i) { a->trap(); }
  void emit(unpcklpd i) { not_implemented(); }
  void emit(xorb i) { a->xor_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
  void emit(xorbi i) { a->xori(Reg64(i.d), Reg64(i.s1), i.s0); }
  void emit(xorl i) { a->xor_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
  void emit(xorq i) { a->xor_(i.d, i.s0, i.s1, false); }
  void emit(xorqi i) { a->xori(i.d, i.s1, i.s0); }

private:
  template<class Inst> void unary(Inst& i) { prep(i.s, i.d); }
  template<class Inst> void binary(Inst& i) { prep(i.s1, i.d); }
  template<class Inst> void commuteSF(Inst&);
  template<class Inst> void commute(Inst&);
  template<class Inst> void noncommute(Inst&);

  CodeBlock& frozen() { return text.frozen().code; }

private:
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

void Vgen::emit(const pop& i) {
#if PPC64_HAS_PUSH_POP
  Vptr p(ppc64::rstktop(), 0);
  a->ld(i.d, p);
  a->addi(ppc64::rstktop(), ppc64::rstktop(), push_pop_elem_size);
#else
  not_implemented();
#endif
}

/*
 * Grows call stack downwards where it's not in use at the moment
 */
void Vgen::emit(const push& i) {
#if PPC64_HAS_PUSH_POP
  Vptr p(ppc64::rstktop(), -push_pop_elem_size);
  a->stdu(i.s, p);
#else
  not_implemented();
#endif
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
  ppc64_asm::Assembler a { cb };
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

///////////////////////////////////////////////////////////////////////////////

template <typename typeImm>
bool patchImm(typeImm imm, Vout& v, Vreg& tmpRegister) {
  uint64_t imm64 = static_cast<uint64_t>(imm);
  if (!(imm64 >> 16)) {
    // Immediate value sizes less than 16 bits
    return false;
  } else {
    tmpRegister  = v.makeReg();
    v << ldimmq{ imm64, tmpRegister };
    return true;
  }
}

/*
 * Vptr struct supports fancy x64 addressing modes.
 * So we need to patch it to avoid ppc64el unsuported address modes.
 *
 * Returns true if anything was patched, false otherwise.
 */
bool patchVptr(Vptr& p, Vout& v) {
  bool modified = false;
  // Convert scaled*index to index
  if(p.scale > 1) {
    Vreg tmp = v.makeReg();
    uint8_t shift = p.scale == 2 ? 1 :
                    p.scale == 4 ? 2 :
                    p.scale == 8 ? 3 : 0;
    v << shlqi{shift, p.index, tmp, VregSF(RegSF{0})};
    p.scale = 1;
    p.index = tmp;
    modified = true;
  }
  Vreg tmp2;
  bool patchedDisp = patchImm(p.disp,v,tmp2);
  // Convert index+displacement to index
  if (p.index.isValid() && p.disp) {
    Vreg tmp  = v.makeReg();
    if(patchedDisp)
      v << addq{tmp2, p.index, tmp, VregSF(RegSF{0})};
    else
      v << addqi{p.disp,p.index,tmp,VregSF(RegSF{0})};
    p.index = tmp;
    p.disp = 0;
    modified = true;
  } else if (patchedDisp) {
    // Convert to index if displacement is greater than 16 bits
    p.index = tmp2;
    p.disp = 0;
    modified = true;
  }

  // Check if base is valid, otherwise set R0 (as zero)
  if (!p.base.isValid()) {
    p.base = Vreg(0);
    modified = true;
  }
  return modified;
}


/*
 * Rules for the lowering of these vasms:
 * 1) All vasms emitted in lowering are already adjusted/patched.
 *   In other words, it will not be lowered afterwards.
 * 2) If a vasm has a Vptr that can be removed by emitting load/store, do it!
 */
/* fallback */
template <typename Inst>
bool lowerForPPC64(Vout& v, Inst& inst) {
  return false;
}

bool lowerForPPC64(Vout& v, storeb& inst) {
  Vptr p = inst.m;
  if (patchVptr(p, v)) {
    v << storeb{ inst.s, p };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, storebi& inst) {
  auto ir = v.makeReg();
  v << ldimmb{ inst.s, ir };

  Vptr p = inst.m;
  (void)patchVptr(p, v);
  v << storeb{ ir, p };
  return true;
}

bool lowerForPPC64(Vout& v, storel& inst) {
  Vptr p = inst.m;
  if (patchVptr(p, v)) {
    v << storel{ inst.s, p };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, storeli& inst) {
  auto ir = v.makeReg();
  v << ldimml{ inst.s, ir };

  Vptr p = inst.m;
  (void)patchVptr(p, v);
  v << storel{ ir, p };
  return true;
}

bool lowerForPPC64(Vout& v, storew& inst) {
  Vptr p = inst.m;
  if (patchVptr(p, v)) {
    v << storew{ inst.s, p };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, storewi& inst) {
  auto ir = v.makeReg();
  v << ldimmw{ inst.s, ir };

  Vptr p = inst.m;
  (void)patchVptr(p, v);
  v << storew{ ir, p };
  return true;
}

bool lowerForPPC64(Vout& v, storeqi& inst) {
  auto ir = v.makeReg();
  v << ldimmq{ Immed64(inst.s.q()), ir };

  Vptr p = inst.m;
  (void)patchVptr(p, v);
  v << store{ ir, p };
  return true;
}

bool lowerForPPC64(Vout& v, store& inst) {
  Vptr p = inst.d;
  if (patchVptr(p, v)) {
    v << store{ inst.s, p };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, storeups& inst) {
  Vptr p = inst.m;
  if (patchVptr(p, v)) {
    v << storeups{ inst.s, p };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, storesd& inst) {
  Vptr p = inst.m;
  if (patchVptr(p, v)) {
    v << storesd{ inst.s, p };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, load& inst) {
  Vptr p = inst.s;
  if (patchVptr(p, v)) {
    v << load{ p, inst.d };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, lea& inst) {
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
  return true;
}

bool lowerForPPC64(Vout& v, loadl& inst) {
  Vptr p = inst.s;
  if (patchVptr(p, v)) {
    v << loadl{ p, inst.d };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, loadzbl& inst) {
  Vptr p = inst.s;
  if (patchVptr(p, v)) {
    v << loadzbl{ p, inst.d };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, loadzbq& inst) {
  Vptr p = inst.s;
  if (patchVptr(p, v)) {
    v << loadzbq{ p, inst.d };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, loadzlq& inst) {
  Vptr p = inst.s;
  if (patchVptr(p, v)) {
    v << loadzlq{ p, inst.d };
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, loadups& inst) {
  Vptr p = inst.s;
  if (patchVptr(p, v)) {
    v << loadups{ p, inst.d};
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, incwm& inst) {
  Vptr p = inst.m;
  (void)patchVptr(p, v);

  Vreg tmp = v.makeReg();
  Vreg tmp2 = v.makeReg();  // needed as VRegs  can only be defined once
  v << loadw{p, tmp};
  v << incw{tmp, tmp2, inst.sf};
  v << storew{tmp2, p};
  return true;
}

bool lowerForPPC64(Vout& v, inclm& inst) {
  Vptr p = inst.m;
  (void)patchVptr(p, v);

  Vreg tmp = v.makeReg();
  Vreg tmp2 = v.makeReg();  // needed as VRegs  can only be defined once
  v << loadl{p, tmp};
  v << incl{tmp, tmp2, inst.sf};
  v << storel{tmp2, p};

  return true;
}

bool lowerForPPC64(Vout& v, incqm& inst) {
  Vptr p = inst.m;
  (void)patchVptr(p, v);

  Vreg tmp = v.makeReg();
  Vreg tmp2 = v.makeReg();  // needed as VRegs  can only be defined once
  v << load{p, tmp};
  v << incq{tmp, tmp2, inst.sf};
  v << store{tmp2, p};

  return true;
}

bool lowerForPPC64(Vout& v, declm& inst) {
  Vptr p = inst.m;
  (void)patchVptr(p, v);

  Vreg tmp = v.makeReg();
  Vreg tmp2 = v.makeReg();  // needed as VRegs  can only be defined once
  v << loadl{p, tmp};
  v << decl{tmp, tmp2, inst.sf};
  v << storel{tmp2, p};

  return true;
}

bool lowerForPPC64(Vout& v, decqm& inst) {
  Vptr p = inst.m;
  patchVptr(p, v);

  Vreg tmp = v.makeReg();
  Vreg tmp2 = v.makeReg();  // needed as VRegs  can only be defined once
  v << load{p, tmp};
  v << decq{tmp, tmp2, inst.sf};
  v << store{tmp2, p};

  return true;
}

bool lowerForPPC64(Vout& v, cmpqim& inst) {
  Vptr p = inst.s1;
  (void)(patchVptr(p, v));
  Vreg tmp2 = v.makeReg();
  v << load{p, tmp2};

  Vreg tmp;
  if (patchImm(inst.s0.q(), v, tmp)) {
    v << cmpq{ tmp, tmp2, inst.sf };
  } else {
    v << cmpqi{ inst.s0, tmp2, inst.sf };
  }
  return true;
}

bool lowerForPPC64(Vout& v, cmpbim& inst) {
  Vptr p = inst.s1;
  (void)patchVptr(p, v);
  Vreg tmp2 = v.makeReg();
  v << loadb{p, tmp2};

  // comparison only up to 8bits. The immediate can't be bigger than that.
  v << cmpli{ inst.s0, tmp2, inst.sf };
  return true;
}

bool lowerForPPC64(Vout& v, cmplim& inst) {
  Vptr p = inst.s1;
  (void)patchVptr(p, v);
  Vreg tmp2 = v.makeReg();
  v << loadl{p, tmp2};

  Vreg tmp;
  if (patchImm(inst.s0.q(), v, tmp)) {
    v << cmpl{ tmp, tmp2, inst.sf };
  } else {
    v << cmpli{ inst.s0, tmp2, inst.sf };
  }
  return true;
}

bool lowerForPPC64(Vout& v, cmplm& inst) {
  Vptr p = inst.s1;
  (void)patchVptr(p, v);
  Vreg tmp = v.makeReg();
  v << loadl{ p, tmp };

  v << cmpl{ inst.s0, tmp, inst.sf };
  return true;
}

bool lowerForPPC64(Vout& v, cmpqm& inst) {
  Vptr p = inst.s1;
  (void)patchVptr(p, v);
  Vreg tmp = v.makeReg();
  v << load { p, tmp };

  v << cmpq{ inst.s0, tmp, inst.sf };
  return true;
}

bool lowerForPPC64(Vout& v, jmpm& inst) {
  Vptr p = inst.target;
  (void)patchVptr(p, v);
  Vreg tmp = v.makeReg();
  v << load { p, tmp };

  v << jmpr { tmp, inst.args };
  return true;
}

bool lowerForPPC64(Vout& v, callm& inst) {
  Vptr p = inst.target;
  (void)patchVptr(p, v);
  auto d = v.makeReg();
  v << load { p, d };

  v << callr { d, inst.args };
  return true;
}

bool lowerForPPC64(Vout& v, absdbl& inst) {
  // clear the high bit
  auto tmp = v.makeReg();
  v << psllq{1, inst.s, tmp};
  v << psrlq{1, tmp, inst.d};

  return true;
}

bool lowerForPPC64(Vout& v, loadqp& inst) {
  // in PPC we don't have anything like a RIP register
  // RIP register uses a absolute address so we can perform a baseless load in
  // this case
  v << load{ baseless(inst.s.r.disp), inst.d };
  return true;
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
 * Avoid Vptr type on pop for ppc64
 */
bool lowerForPPC64(Vout& v, popm& inst) {
  // PPC can only copy mem->mem by using a temporary register
  auto tmp = v.makeReg();
  v << pop{tmp};
  v << store{tmp, inst.d};

  // remove the original popm (count parameter is 1)
  return true;
}

bool lowerForPPC64(Vout& v, orwim& inst) {
  Vptr p = inst.m;
  (void)patchVptr(p, v);
  Vreg tmp = v.makeReg();
  v << loadw{ p, tmp };

  v << orqi {inst.s0, tmp, tmp, inst.sf};
  v << storew{tmp, p};
  return true;
}

bool lowerForPPC64(Vout& v, orqim& inst) {
  Vreg tmp = v.makeReg();
  v << load {inst.m, tmp};

  Vreg tmp2;
  if (patchImm(inst.s0.q(), v, tmp2)) {
    v << orq {tmp2, tmp, tmp, inst.sf};
  } else {
    v << orqi{inst.s0, tmp, tmp, inst.sf};
  }
  v << store{tmp, inst.m};
  return true;
}

bool lowerForPPC64(Vout& v, addqi& inst) {
  Vreg tmp;
  if (patchImm(inst.s0.q(), v, tmp)) {
    v << addq  {tmp, inst.s1, inst.d, inst.sf};
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, addqim& inst) {
  Vptr p = inst.m;
  (void)patchVptr(p, v);
  Vreg tmp = v.makeReg();
  v << load { p, tmp };

  Vreg tmp2;
  if (patchImm(inst.s0.q(), v, tmp2)) {
    v << addq {tmp2, tmp, tmp, inst.sf};
  } else {
    v << addqi{inst.s0, tmp, tmp, inst.sf};
  }
  v << store{tmp, p};
  return true;
}

bool lowerForPPC64(Vout& v, addli& inst) {
  Vreg tmp;
  if (patchImm(inst.s0.l(), v, tmp)) {
    v << addl  {tmp, inst.s1, inst.d, inst.sf};
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, addlm& inst) {
  Vptr p = inst.m;
  (void)patchVptr(p, v);
  Vreg tmp = v.makeReg();
  v << loadw { p, tmp };

  v << addl  {inst.s0, tmp, tmp, inst.sf};
  v << storew{tmp, p};
  return true;
}

bool lowerForPPC64(Vout& v, andli& inst) {
  Vreg tmp;
  if (patchImm(inst.s0.l(), v, tmp)) {
    v << andl  {tmp, inst.s1, inst.d, inst.sf};
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, cmpqi& inst) {
  Vreg tmp;
  if (patchImm(inst.s0.q(), v, tmp)) {
    v << cmpq  {tmp, inst.s1, inst.sf};
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, testqm& inst) {
  Vptr p = inst.s1;
  (void)(patchVptr(p, v));
  Vreg tmp = v.makeReg();
  v << load{p, tmp};

  v << testq{ inst.s0, tmp, inst.sf };
  return true;
}

bool lowerForPPC64(Vout& v, testqim& inst) {
  Vptr p = inst.s1;
  (void)(patchVptr(p, v));
  Vreg tmp2 = v.makeReg();
  v << load{p, tmp2};

  Vreg tmp;
  if (patchImm(inst.s0.q(), v, tmp)) {
    v << testq{ tmp, tmp2, inst.sf };
  } else {
    v << testqi{ inst.s0, tmp2, inst.sf };  // doesn't need lowering
  }
  return true;
}

bool lowerForPPC64(Vout& v, testlim& inst) {
  Vptr p = inst.s1;
  (void)patchVptr(p, v);
  Vreg tmp2 = v.makeReg();
  v << loadl{p, tmp2};

  Vreg tmp;
  if (patchImm(inst.s0.q(), v, tmp)) {
    v << testq{ tmp, tmp2, inst.sf };
  } else {
    v << testqi{ inst.s0, tmp2, inst.sf };  // doesn't need lowering
  }
  return true;
}

bool lowerForPPC64(Vout& v, testwim& inst) {
  Vptr p = inst.s1;
  (void)patchVptr(p, v);
  Vreg tmp2 = v.makeReg();
  v << loadw{p, tmp2};

  // comparison only up to 8bits. The immediate can't be bigger than that.
  v << testqi{ inst.s0, tmp2, inst.sf };  // doesn't need lowering
  return true;
}

bool lowerForPPC64(Vout& v, testbim& inst) {
  Vptr p = inst.s1;
  (void)patchVptr(p, v);
  Vreg tmp2 = v.makeReg();
  v << loadb{p, tmp2};

  // comparison only up to 8bits. The immediate can't be bigger than that.
  v << testqi{ inst.s0, tmp2, inst.sf };  // doesn't need lowering
  return true;
}

bool lowerForPPC64(Vout& v, testqi& inst) {
  Vreg tmp;
  if (patchImm(inst.s0.q(), v, tmp)) {
    v << testq  {tmp, inst.s1, inst.sf};
    return true;
  }
  return false;
}

bool lowerForPPC64(Vout& v, countbytecode& inst) {
  v << incqm{ inst.base[g_bytecodesVasm.handle()], inst.sf };
  return true;
}

// Lower movs to copy
bool lowerForPPC64(Vout& v, movtqb& inst) {
  v << copy{inst.s, inst.d};
  return true;
}
bool lowerForPPC64(Vout& v, movtql& inst) {
  v << copy{inst.s, inst.d};
  return true;
}

// Lower comparison to cmpq
bool lowerForPPC64(Vout& v, cmpb& inst) {
  v << cmpq{Reg64(inst.s0), Reg64(inst.s1), inst.sf};
  return true;
}
bool lowerForPPC64(Vout& v, cmpl& inst) {
  v << cmpq{Reg64(inst.s0), Reg64(inst.s1), inst.sf};
  return true;
}

// Lower comparison with immediate to cmpqi
bool lowerForPPC64(Vout& v, cmpbi& inst) {
  v << cmpqi{inst.s0, Reg64(inst.s1), inst.sf};
  return true;
}
bool lowerForPPC64(Vout& v, cmpli& inst) {
  // convert cmpli to cmpqi or ldimmq + cmpq by cmpqi's lowering
  auto lowered = cmpqi{inst.s0, Reg64(inst.s1), inst.sf};
  if (!lowerForPPC64(v, lowered)) v << lowered;
  return true;
}

// Lower subtraction to subq
bool lowerForPPC64(Vout& v, subl& inst) {
  v << subq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
  return true;
}
bool lowerForPPC64(Vout& v, subbi& inst) {
  v << subqi{inst.s0, Reg64(inst.s1), Reg64(inst.d), inst.sf};
  return true;
}
bool lowerForPPC64(Vout& v, subli& inst) {
  v << subqi{inst.s0, Reg64(inst.s1), Reg64(inst.d), inst.sf};
  return true;
}

// Lower test to testq
bool lowerForPPC64(Vout& v, testb& inst) {
  v << testq{Reg64(inst.s0), Reg64(inst.s1), inst.sf};
  return true;
}
bool lowerForPPC64(Vout& v, testl& inst) {
  v << testq{Reg64(inst.s0), Reg64(inst.s1), inst.sf};
  return true;
}
bool lowerForPPC64(Vout& v, testbi& inst) {
  // convert testbi to testqi or ldimmq + testq by testqi's lowering
  auto lowered = testqi{inst.s0, Reg64(inst.s1), inst.sf};
  if (!lowerForPPC64(v, lowered)) v << lowered;
  return true;
}
bool lowerForPPC64(Vout& v, testli& inst) {
  // convert testli to testqi or ldimmq + testq by testqi's lowering
  auto lowered = testqi{inst.s0, Reg64(inst.s1), inst.sf};
  if (!lowerForPPC64(v, lowered)) v << lowered;
  return true;
}

// Lower xor to xorq
bool lowerForPPC64(Vout& v, xorb& inst) {
  v << xorq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
  return true;
}
bool lowerForPPC64(Vout& v, xorl& inst) {
  v << xorq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
  return true;
}

// Lower xor with immediate to xorqi
bool lowerForPPC64(Vout& v, xorbi& inst) {
  v << xorqi{inst.s0, Reg64(inst.s1), Reg64(inst.d), inst.sf};
  return true;
}

// Lower and to andq
bool lowerForPPC64(Vout& v, andb& inst) {
  v << andq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
  return true;
}
bool lowerForPPC64(Vout& v, andl& inst) {
  v << andq{Reg64(inst.s0), Reg64(inst.s1), Reg64(inst.d), inst.sf};
  return true;
}
bool lowerForPPC64(Vout& v, andbi& inst) {
  // patchImm doesn't need to be called as it should be < 8 bits
  v << andqi{inst.s0, Reg64(inst.s1), Reg64(inst.d), inst.sf};
  return true;
}


#if PPC64_HAS_PUSH_POP
/*
 * Should only be called once per block that push/pop is used in order to
 * initialize it. It'll not remove the original push instruction.
 */
void InitializePushStk(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  // adjust the beginning of the stack to be below the frame pointer
  v << copy{ppc64::rsp(), ppc64::rstktop()};

  // do not remove the original push (count parameter is 0)
  vector_splice(unit.blocks[b].code, iInst, 0, unit.blocks[scratch].code);
}
#endif

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

#if PPC64_HAS_PUSH_POP
  InitializePushStk(unit, Vlabel{0}, 0);
#endif

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

#define O(name, imms, uses, defs)                         \
        case Vinstr::name:                                \
          if (lowerForPPC64(v, inst.name##_)) {           \
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

