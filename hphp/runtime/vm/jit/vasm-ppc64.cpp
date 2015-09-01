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
#include "hphp/runtime/vm/jit/code-gen-helpers-ppc64.h"
#include "hphp/runtime/vm/jit/code-gen.h"
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
  inline void pushMinCallStack(void)
  {
    a->mflr(ppc64_asm::reg::r0);
    // LR on parent call frame
    a->std(ppc64_asm::reg::r0,  ppc64_asm::reg::r1, 16);
    // minimum call stack
    a->stdu(ppc64_asm::reg::r1, ppc64_asm::reg::r1, -32);
  }

  inline void popMinCallStack(void)
  {
    // minimum call stack
    a->addi(ppc64_asm::reg::r1, ppc64_asm::reg::r1, 32);
    // LR on parent call frame
    a->ld(ppc64_asm::reg::r0,   ppc64_asm::reg::r1, 16);
    a->mtlr(ppc64_asm::reg::r0);
  }

  /*
   * Calculates address of s and stores it on d
   *
   * The address of Vptr can be calculated by:
   *    s.base + s.index * s.scale + s.disp
   * the multiplication will be simplified by a shift left,
   * as s.scale is always 1, 2, 4 or 8
   */
  inline void VptrAddressToReg(Vptr s, Vreg d) {
    /* s.index is optional */
    if (s.index.isPhys()) {
      /* calculate index position before adding base and displacement */
      int shift_left = 0;
      int scale = s.scale;
      while (scale >>= 1) {
        ++shift_left;
      }
      assert(shift_left <= 3);

      emit(shlqi{shift_left,  s.index,  d, VregSF(0)});
      emit(addq {s.base,      d,        d, VregSF(0)});
      emit(addqi{s.disp,      d,        d, VregSF(0)});
    } else if (!s.base.isPhys()) {
      /* Baseless Vptr, solve this for ppc64 */
      not_implemented();
    } else {
      /* Only add base with displacement */
      emit(addqi{s.disp,      s.base,   d, VregSF(0)});
    }
  }

  /*
   * Stores in d the value pointed by s
   */
  inline void VptrToReg(Vptr s, Vreg d) {
    VptrAddressToReg(s, d);
    emit(load{*d, d});
  }

  // intrinsics
  void emit(const bindaddr& i) { not_implemented(); }
  void emit(const bindcall& i) { not_implemented(); }
  void emit(const bindjcc1st& i) { not_implemented(); }
  void emit(const bindjcc& i) { not_implemented(); }
  void emit(const bindjmp& i) { not_implemented(); }
//  void emit(const callstub& i) { not_implemented(); }
  void emit(const callfaststub& i) { not_implemented(); }
  void emit(const contenter& i) { not_implemented(); }
  void emit(const copy& i) { not_implemented(); }
  void emit(const copy2& i) { not_implemented(); }
  void emit(const debugtrap& i) { not_implemented(); }
  void emit(const fallthru& i) {}
  void emit(const ldimmb& i) { not_implemented(); }
  void emit(const ldimml& i) { not_implemented(); }
  void emit(const ldimmq& i) { not_implemented(); }
  void emit(const ldimmqs& i) { not_implemented(); }
  void emit(const load& i);
  void emit(const mccall& i) { not_implemented(); }
  void emit(const mcprep& i) { not_implemented(); }
  void emit(const nothrow& i) { not_implemented(); }
  void emit(const store& i);
  void emit(const syncpoint& i) { not_implemented(); }
  void emit(const unwind& i) { not_implemented(); }
  void emit(const landingpad& i) { not_implemented(); }
  void emit(const vret& i);
  void emit(const leavetc&) { not_implemented(); }

  // instructions
  void emit(addli i) { a->addi(Reg64(i.s1), Reg64(i.d), i.s0); }
  void emit(const addlm& i) { not_implemented(); }
  void emit(addq i) { a->add(i.d, i.s0, i.s1, false); }
  void emit(addqi i) { a->addi(i.d, i.s1, i.s0); }
  void emit(const addqim& i) { not_implemented(); }
  void emit(addsd i) { not_implemented(); }
  void emit(andb i) {  a->and_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
  void emit(andbi i) { a->andi(Reg64(i.s1), Reg64(i.d), i.s0); }
  void emit(const andbim& i) { not_implemented(); }
  void emit(andl i) { a->and_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
  void emit(andli i) {a->andi(Reg64(i.s1), Reg64(i.d), i.s0); }
  void emit(andq i) { a->and_(i.d, i.s0, i.s1, false); }
  void emit(andqi i) { a->andi(i.s1, i.d, i.s0); }
  void emit(const call& i) {
    // Need to create a new call stack in order to recover LR in the future
    pushMinCallStack();

    a->branchAuto(i.target, BranchConditions::Always, LinkReg::Save);
  }
  void emit(const callm& i) {
    // uses scratch register
    VptrToReg(i.target, ppc64::rvasmtmp());
    emit(callr{ppc64::rvasmtmp(), i.args});
  }
  void emit(const callr& i) {
    a->mtctr(i.target);
    a->bctrl();
  }
  void emit(const cloadq& i) { not_implemented(); }
  void emit(const cmovq& i) { not_implemented(); }
  void emit(const cmpb& i) { a->cmp(0, 0, Reg64(i.s0), Reg64(i.s1)); }
  void emit(const cmpbi& i) { a->cmpi(0, 0, Reg64(i.s1), i.s0); }
  void emit(const cmpbim& i) { not_implemented(); }
  void emit(const cmpl& i) {  a->cmp(0, 0, Reg64(i.s0), Reg64(i.s1)); }
  void emit(const cmpli& i) { a->cmpi(0, 0, Reg64(i.s1), i.s0); }
  void emit(const cmplim& i) { not_implemented(); }
  void emit(const cmplm& i) { not_implemented(); }
  //TODO(IBM): field 1 indicates cr (cr0) register who holds the bf result
  void emit(const cmpq& i) { a->cmp(0, 0, i.s0, i.s1); }
  //TODO(IBM): field 1 indicates cr (cr0) register who holds the bf result
  void emit(const cmpqi& i) { a->cmpi(0, 0, i.s1, i.s0); }
  void emit(const cmpqim& i) {
    VptrToReg(i.s1, ppc64::rvasmtmp());
    a->cmpdi(ppc64::rvasmtmp(), i.s0);
  }
  void emit(const cmpqm& i) { not_implemented(); }
  void emit(cmpsd i) { not_implemented(); }
  void emit(const cqo& i) { not_implemented(); }
  void emit(const cvttsd2siq& i) { not_implemented(); }
  void emit(const cvtsi2sd& i) { not_implemented(); }
  void emit(const cvtsi2sdm& i) { not_implemented(); }
  void emit(decl i) { a->addi(Reg64(i.d), Reg64(i.s), -1); }
  void emit(const declm& i) { not_implemented(); }
  void emit(decq i) { a->addi(i.d, i.s, -1); }
  void emit(const decqm& i) { not_implemented(); }
  void emit(divsd i) { not_implemented(); }
  void emit(imul i) { a->mullw(i.d, i.s1, i.s0, false); }
  /*TODO(IBM): idiv instruction takes only one paramenter
    because x64 idiv divides eax:edx by i.s. There is no 
    such instruction in PPC64. So maybe we need to create another vasm.*/
  void emit(const idiv& i) { not_implemented(); }
  void emit(incl i) { a->addi(Reg64(i.d), Reg64(i.s), 1); }
  void emit(const inclm& i) { not_implemented(); }
  void emit(incq i) { a->addi(i.d, i.s, 1); }
  void emit(const incqm& i) { not_implemented(); }
  void emit(const incqmlock& i) { not_implemented(); }
  void emit(const incwm& i) { a->addi(i.m.base, i.m.index, i.m.disp); }
  void emit(const jcc& i) { not_implemented(); }
  void emit(const jcci& i) { not_implemented(); }
  void emit(const jmp& i) {
    if (next == i.target) return;
    jmps.push_back({a->frontier(), i.target});

    // offset to be determined by a->patchBc
    a->b(0);
  }
  void emit(const jmpr& i) {
    a->mtctr(i.target);
    a->bctr();
  }
  void emit(const jmpm& i) {
    // uses scratch register
    VptrToReg(i.target, ppc64::rvasmtmp());
    emit(jmpr {ppc64::rvasmtmp(), i.args});
  }
  void emit(const jmpi& i) {
    a->branchAuto(i.target, BranchConditions::Always, LinkReg::DoNotTouch);
  }
  void emit(const lea& i) { a->addi(i.d, i.s.base, i.s.disp); }
  void emit(const leap& i) { not_implemented(); }
  void emit(const loadups& i) { not_implemented(); }
  void emit(const loadtqb& i) { not_implemented(); }
  void emit(const loadl& i) { not_implemented(); }
  void emit(const loadqp& i) { not_implemented(); }
  void emit(const loadsd& i) { not_implemented(); }
  void emit(const loadzbl& i) { a->lbz(Reg64(i.d), i.s);} 
  void emit(const loadzbq& i) { a->lbz(i.d, i.s); }
  void emit(const loadzlq& i) { a->lwz(i.d, i.s); }
  void emit(movb& i) { a->ori(Reg64(i.d), Reg64(i.s), 0); }
  void emit(movl& i) { a->ori(Reg64(i.d), Reg64(i.s), 0); }
  void emit(movzbl& i) { a->ori(Reg64(i.d), Reg64(i.s), 0); }
  void emit(movzbq& i) { a->ori(i.d, Reg64(i.s), 0); }
  void emit(mulsd i) { not_implemented(); }
  void emit(neg i) { a->neg(i.d, i.s, false); }
  void emit(const nop& i) { a->ori(Reg64(0), Reg64(0), 0); } //no-op form
  void emit(not i) { a->nor(i.d, i.s, i.s, false); }
  void emit(notb i) { not_implemented(); }
  void emit(const orwim& i) { not_implemented(); }
  void emit(orq i) { a->or_(i.d, i.s0, i.s1, false); }
  void emit(orqi i) { a->ori(i.d, i.s1, i.s0); }
  void emit(const orqim& i) { not_implemented(); }
  void emit(const pop& i);
  void emit(const popm& i) { not_implemented(); }
  void emit(psllq i) { not_implemented(); }
  void emit(psrlq i) { not_implemented(); }
  void emit(const push& i);
  void emit(const roundsd& i) { not_implemented(); }
  void emit(const ret& i) {
    // recover LR from callstack
    popMinCallStack();
    a->blr();
  }
  /*Immediate-form logical (unsigned) shift operations are
    obtained by specifying appropriate masks and shift values for 
    certain Rotate instructions.
  */
  void emit(const sarq& i) { not_implemented(); }
  void emit(sarqi i) { a->srawi(i.d, i.s1, Reg64(i.s0.w()), false); }
  void emit(const setcc& i) { not_implemented(); }
  void emit(shlli i) { not_implemented(); }
  void emit(shlq i) { not_implemented(); }
  /*TODO Rc=1*/
  void emit(shlqi i) { a->sldi(i.d, i.s1, i.s0.b()); }
  void emit(shrli i) { not_implemented(); }
  void emit(shrqi i) { a->srdi(i.d, i.s1, i.s0.b()); }
  void emit(const sqrtsd& i) { not_implemented(); }
  void emit(const storeups& i) { not_implemented(); }
  void emit(const storeb& i) { a->stb(Reg64(i.s), i.m); }
  void emit(const storebi& i) { not_implemented(); }
  void emit(const storel& i) { a->stw(Reg64(i.s), i.m); }
  void emit(const storeli& i) { not_implemented(); }
  void emit(const storeqi& i) { not_implemented(); }
  void emit(const storesd& i) { not_implemented(); }
  void emit(const storew& i) { a->sth(Reg64(i.s), i.m); }
  void emit(const storewi& i) { not_implemented(); }
  void emit(subbi i) { not_implemented(); }
  void emit(subl i) { a->subf(Reg64(i.d), Reg64(i.s1), Reg64(i.s0), false); }
  void emit(subli i) { a->addi(Reg64(i.s1), Reg64(i.d), i.s0); }
  void emit(subq i) { a->subf(i.d, i.s1, i.s0, false); }
  void emit(subqi i) { a->addi(i.s1, i.d, i.s0); /*addi with negative value*/ }
  void emit(subsd i) { not_implemented(); }
  void emit(const testb& i) {
    // explicit conversion from Reg8 to Reg64
    Reg64 s0(i.s0), s1(i.s1);
    if (s0 != s1)
      a->and_(ppc64::rvasmtmp(), s0, s1, true);
    else
      a->cmpldi(s0, Immed(0));
  }
  void emit(const testbi& i) { /*a->addi(i.d, i.s1, i.s0);*/ }
  void emit(const testbim& i) { not_implemented(); }
  void emit(const testwim& i) { not_implemented(); }
  //TODO(IBM) Depends on CR registers
  void emit(const testl& i) { /*a->and_(i.s0, i.s1, false);*/ }
  void emit(const testli& i) { /*a->addi(i.d, i.s1, i.s0);*/ }
  void emit(const testlim& i) { not_implemented(); }
  void emit(const testq& i) { /*a->and_(i.s0, i.s1, false);*/ }
  void emit(const testqm& i) { not_implemented(); }
  void emit(const testqim& i) { not_implemented(); }
  void emit(const ucomisd& i) { not_implemented(); }
  void emit(const ud2& i) { a->unimplemented(); }
  void emit(unpcklpd i) { not_implemented(); }
  void emit(xorb i) { a->xor_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
  void emit(xorbi i) { a->xori(Reg64(i.d), Reg64(i.s1), i.s0); }
  void emit(xorl i) { a->xor_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
  void emit(xorq i) { a->xor_(i.d, i.s0, i.s1, false); }
  void emit(xorqi i) { a->xori(i.d, i.s1, i.s0); }

private:
  // helpers
  //void prep(Reg8 s, Reg8 d) { if (s != d) a->movb(s, d); }
  //void prep(Reg32 s, Reg32 d) { if (s != d) a->movl(s, d); }
  //void prep(Reg64 s, Reg64 d) { if (s != d) a->movq(s, d); }
  //void prep(RegXMM s, RegXMM d) { if (s != d) a->movdqa(s, d); }

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

void Vgen::emit(const pop& i) {
  not_implemented();
  //TODO(IBM): Instruction pop. Check if this the best way to do this.
  //a->lwz r0 0(rVmSp)
  //a->addi rVmSp, rVmSp +4
}

void Vgen::emit(const push& i) {
  not_implemented();
  //TODO(IBM): Instruction push. Check if this the best way to do this.
  //a->addi rVmSp, rVmSp -4
  //a->stw r0 0(rVmSp)
}

void Vgen::emit(const vret& i) {
  not_implemented();
  //TODO(IBM): Need to be MemoryRef
  a->bclr(20,0,0); /*brl 0x4e800020*/
}
void Vgen::emit(const load& i) {
  if (i.d.isGP()) {
    a->lwz(i.d, i.s);
  } else {
    not_implemented();//TODO(IBM): SIMD Unsupported
  }
}

void Vgen::patch(Venv& env) {
  not_implemented();
}

void Vgen::pad(CodeBlock& cb) {
  not_implemented();
}

void Vgen::emit(const store& i) {
  if (i.s.isGP()) {
    a->stw(i.s, i.d);
  } else {
    assertx(i.s.isSIMD());
    // TODO(rcardoso) : Unsupported
  }
}

/*
 Lower facilitate code generation. In some cases is used because 
 some vasm opcodes doesn't have a 1:1 mapping to machine asm code.
*/
void lowerForPPC64(Vunit& unit, const Abi& abi) {
  //TODO(rcardoso) Implement function
  printUnit(kVasmARMFoldLevel, "after lower for PPC64", unit);
}
///////////////////////////////////////////////////////////////////////////////
} // anonymous namespace

void optimizePPC64(Vunit& unit, const Abi& abi) {
 // TODO(rcardoso) Implement optimizations here
 //                HHVM have some optimizations for vasm:
 //                   removeTrivialNops(unit);
 //                   fuseBranches(unit);
 //                   optimizeJmps(unit);
 //                   optimizeExits(unit);
 //                   optimizeCopies(unit, abi);
 //                   removeDeadCode(unit);
 //                   allocateRegisters(unit, abi);
}

void emitPPC64(const Vunit& unit, Vtext& text, AsmInfo* asmInfo) {
  Timer timer(Timer::vasm_gen);
  vasm_emit<Vgen>(unit, text, asmInfo);
}

///////////////////////////////////////////////////////////////////////////////
}}
