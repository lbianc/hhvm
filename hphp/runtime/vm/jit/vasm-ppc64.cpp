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
        a->mr(ppc64::rvasmtmp(),s1);
        a->mr(d0,s0);
        a->mr(d1,ppc64::rvasmtmp());
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
      // Read as 16 bits and mask to avoid another cast
      a->li(ppc64::rvasmtmp(), (i.s.l() & UINT8_MAX));
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
      } else {
        a->li64(i.d, val);
      }
    } else {
      not_implemented();
    }
  }
  void emit(const ldimmqs& i) { not_implemented(); }
  void emit(const load& i);
  void emit(const mccall& i) { not_implemented(); }
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
  void emit(const vret& i);
  void emit(const leavetc&) { not_implemented(); }

  // instructions
  void emit(addli i) {
    /* add of immediate up to 32bits */
    if (!i.s0.fits(HPHP::sz::word)) {
      // d = (s0@h + s1@h) + (s0@l + s1@l)
      a->li32(ppc64::rvasmtmp(), i.s0.l());
      a->add(Reg64(i.d), Reg64(i.d), ppc64::rvasmtmp());
    } else {
      // d = s0@l + s1@l
      a->addi(Reg64(i.d), Reg64(i.s1), i.s0);
    }
  }
  void emit(const addlm& i) { not_implemented(); }
  void emit(addq i) { a->add(i.d, i.s0, i.s1, false); }
  void emit(addqi i) { a->addi(i.d, i.s1, i.s0); }
  void emit(const addqim& i) { not_implemented(); }
  void emit(addsd i) { not_implemented(); }
  void emit(const andbim& i) { not_implemented(); }
  void emit(andli i) {
    /* and of immediate up to 32bits */
    if (!i.s0.fits(HPHP::sz::word)) {
      // d = (s0@h & s1@h) | (s0@l & s1@l)
      a->li32un(ppc64::rvasmtmp(), i.s0.l());
      a->and_(Reg64(i.d), ppc64::rvasmtmp(), Reg64(i.d));
    } else {
      // d = s0@l & s1@l
      a->andi(Reg64(i.d), Reg64(i.s1), i.s0);
    }
  }
  void emit(andq i) { a->and_(i.d, i.s0, i.s1, false); }
  void emit(andqi i) { a->andi(i.d, i.s1, i.s0); }
  void emit(const call& i) {
    // Need to create a new call stack in order to recover LR in the future
    pushMinCallStack();

    a->branchAuto(i.target, BranchConditions::Always, LinkReg::Save);

    popMinCallStack();
  }
  void emit(const callm& i) {
    emit(load{i.target, ppc64::rvasmtmp()});
    emit(callr{ppc64::rvasmtmp(), i.args});
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
  void emit(const cmpbim& i) {
    emit(load{i.s1, ppc64::rvasmtmp()}); 
    a->cmpi(0, 0, ppc64::rvasmtmp(), i.s0);
  }
  void emit(const cmplim& i) {
    if (i.s0.fits(HPHP::sz::word)) {
      emit(load{i.s1, ppc64::rvasmtmp()});
      a->cmpi(0, 0, ppc64::rvasmtmp(), i.s0);
    } else {
      emit(load{i.s1, ppc64::rvasmtmp()});
      a->li32(ppc64::rvasmtmp2(), i.s0.l());
      a->cmpw(ppc64::rvasmtmp(), ppc64::rvasmtmp2());
    }
  }
  void emit(const cmplm& i) { not_implemented(); }
  void emit(const cmpq& i) { a->cmpd(i.s1, i.s0); }
  void emit(const cmpqi& i) {
    if (i.s0.fits(HPHP::sz::word)) {
      a->cmpdi(i.s1, i.s0);
    } else {
      emit(ldimmq{i.s0.q(), ppc64::rvasmtmp()});
      emit(cmpq{ppc64::rvasmtmp(), i.s1, i.sf});
    }
  }
  void emit(const cmpqim& i) {
    emit(load{i.s1, ppc64::rvasmtmp()});
    a->cmpdi(ppc64::rvasmtmp(), i.s0);
  }
  void emit(const cmpqm& i) {
    emit(load{i.s1, ppc64::rvasmtmp()});
    a->cmp(0, 0, i.s0, ppc64::rvasmtmp());
  }
  void emit(cmpsd i) { not_implemented(); }
  void emit(const cqo& i) { not_implemented(); }
  void emit(const cvttsd2siq& i) { not_implemented(); }
  void emit(const cvtsi2sd& i) { not_implemented(); }
  void emit(const cvtsi2sdm& i) { not_implemented(); }
  void emit(decl i) { a->addi(Reg64(i.d), Reg64(i.s), -1); }
  void emit(const declm& i) {
    a->addi(ppc64::rvasmtmp(), ppc64::rvasmtmp(), -1);
    emit(store{ppc64::rvasmtmp() ,i.m});
  }
  void emit(decq i) { a->addi(i.d, i.s, -1); }
  void emit(const decqm& i) { not_implemented(); }
  void emit(divsd i) { not_implemented(); }
  void emit(imul i) { a->mullw(i.d, i.s1, i.s0, false); }
  void emit(const idiv& i) { not_implemented(); } // should use vasm srem below
  void emit(const srem& i) {
    a->divd(i.d,  i.s0, i.s1, false);
  }
  void emit(incl i) { a->addi(Reg64(i.d), Reg64(i.s), 1); }
  void emit(const inclm& i) { not_implemented(); }
  void emit(incq i) { a->addi(i.d, i.s, 1); }
  void emit(const incqm& i) { not_implemented(); }
  void emit(const incqmlock& i) { not_implemented(); }
  void emit(const incwm& i) {
    if (i.m.index.isValid()) {
      a->ldx(ppc64::rvasmtmp(), i.m);
      a->addi(ppc64::rvasmtmp(), ppc64::rvasmtmp(), 1);
      a->stdx(ppc64::rvasmtmp(), i.m);
    } else {
      a->ld(ppc64::rvasmtmp(), i.m);
      a->addi(ppc64::rvasmtmp(), ppc64::rvasmtmp(), 1);
      a->std(ppc64::rvasmtmp(), i.m);
    }
  }
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
  void emit(const jmpm& i) {
    //VptrToReg(i.target, ppc64::rvasmtmp());
    //emit(jmpr {ppc64::rvasmtmp(), i.args});
    emit(load{i.target, ppc64::rvasmtmp()});
    emit(jmpr {ppc64::rvasmtmp(), i.args});
  }
  void emit(const jmpi& i) {
    a->branchAuto(i.target, BranchConditions::Always, LinkReg::DoNotTouch);
  }
  void emit(const lea& i) { a->addi(i.d, i.s.base, i.s.disp); }
  void emit(const leap& i) { a->li64(i.d, i.s.r.disp); }
  void emit(const loadups& i) { a->lxvw4x(i.d,i.s); }
  void emit(const loadtqb& i) { a->lbz(Reg64(i.d),i.s); }
  void emit(const loadb& i) { a->lbz(Reg64(i.d),i.s); }
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
  void emit(const nop& i) { a->ori(Reg64(0), Reg64(0), 0); } //no-op form
  void emit(not i) { a->nor(i.d, i.s, i.s, false); }
  void emit(notb i) { not_implemented(); }
  void emit(orq i) { a->or_(i.d, i.s0, i.s1, false); }
  void emit(orqi i) { a->ori(i.d, i.s1, i.s0); }
  void emit(const orqim& i) { not_implemented(); }
  void emit(const pop& i);
  void emit(psllq i) { not_implemented(); }
  void emit(psrlq i) { not_implemented(); }
  void emit(const push& i);
  void emit(const roundsd& i) { not_implemented(); }
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
  void emit(const sqrtsd& i) { not_implemented(); }
  void emit(const storeups& i) { a->stxvw4x(i.s,i.m); }
  void emit(const storeb& i) {
    if(i.m.index.isValid()) {
      a->stbx(Reg64(i.s), i.m);
    } else {
      a->stb(Reg64(i.s), i.m);
    }
  }
  void emit(const storebi& i) {
    a->li(ppc64::rvasmtmp(), (i.s.l() & UINT8_MAX));
    if(i.m.index.isValid()) {
      a->stbx(ppc64::rvasmtmp(), i.m);
    } else {
      a->stb(ppc64::rvasmtmp(), i.m);
    }
  }
  void emit(const storel& i) {
    if(i.m.index.isValid()) {
      a->stwx(Reg64(i.s), i.m);
    } else {
      a->stw(Reg64(i.s), i.m);
    }
  }
  void emit(const storeli& i) {
    a->li32(ppc64::rvasmtmp(), i.s.l());
    if (i.m.index.isValid()) {
      a->stwx(ppc64::rvasmtmp(), i.m);
    } else {
      a->stw(ppc64::rvasmtmp(), i.m);
    }
  }
  void emit(const storeqi& i) {
    a->li64(ppc64::rvasmtmp(), i.s.q());
    if (i.m.index.isValid()) {
      a->stdx(ppc64::rvasmtmp(), i.m);
    } else {
      a->std(ppc64::rvasmtmp(), i.m);
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
  void emit(const storewi& i) {
    a->li(ppc64::rvasmtmp(), i.s);
    if (i.m.index.isValid()) {
      a->sthx(ppc64::rvasmtmp(), i.m);
    } else {
      a->sth(ppc64::rvasmtmp(), i.m);
    }
  }
  void emit(subq i) { a->subf(i.d, i.s1, i.s0, false); }
  void emit(subqi i) { a->addi(i.s1, i.d, i.s0); /*addi with negative value*/ }
  void emit(subsd i) { a->fsub(i.d, i.s0, i.s1); /* d = s1 - s0 */ }
  void emit(const testbim& i) {
    a->lbz(ppc64::rvasmtmp(), i.s1.mr());
    emit(testbi{i.s0, ppc64::rvasmtmp(), i.sf});
  }
  void emit(const testwim& i) {
    a->lhz(ppc64::rvasmtmp(), i.s1);
    emit(testli{i.s0, ppc64::rvasmtmp(), i.sf});
  }
  void emit(const testlim& i) {
    a->lwz(ppc64::rvasmtmp(), i.s1);
    emit(testli{i.s0, ppc64::rvasmtmp(), i.sf});
  }
  void emit(const testq& i) { a->and_(ppc64::rvasmtmp(), i.s0, i.s1, true); }
  void emit(const testqi& i) { a->andi(ppc64::rvasmtmp(), i.s1, i.s0); }
  void emit(const testqm& i) {
    a->ld(ppc64::rvasmtmp(), i.s1);
    emit(testq{i.s0, ppc64::rvasmtmp(), i.sf});
  }
  void emit(const testqim& i) {
    a->ld(ppc64::rvasmtmp(), i.s1);
    emit(testqi{i.s0, ppc64::rvasmtmp(), i.sf});
  }
  void emit(const ucomisd& i) { not_implemented(); }
  void emit(const ud2& i) { a->trap(); }
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

void Vgen::emit(const vret& i) {
  emit(load{i.retAddr, ppc64::rvasmtmp()});
  a->mtlr(ppc64::rvasmtmp());
  emit(load{i.prevFP, i.d});
  a->blr();
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
    //TODO(rcardoso): Needs to check if needs to change to vec instruction
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
    //TODO(rcardoso): Needs to check if needs to change to vec instruction
    a->stfs(i.s, i.d);
  }
}

///////////////////////////////////////////////////////////////////////////////

/*
 Vptr struct supports fancy x64 addressing modes.
 So we need to patch it to avoid ppc64el unsuported address modes.
*/
void patchVptr(Vptr& p, Vout& v) {
  Vreg tmp  = v.makeReg();
  Vreg tmp2 = v.makeReg();
  tmp = p.index;
  // Convert scaled*index to index
  if(p.scale > 1) {
    uint8_t shift = p.scale == 2 ? 1 :
                    p.scale == 4 ? 2 :
                    p.scale == 8 ? 3 : 0;
    v << shlqi{shift, p.index, tmp, VregSF(0)};
    p.scale = 1;
    p.index = tmp;
  }
  // Convert index+displacement to index
  if(p.index.isValid() && p.disp > -1) {
    v << ldimmq{ p.disp, tmp2 };
    v << addq{tmp2, tmp, tmp, VregSF(0)};
    p.index = tmp;
  } else if((p.disp > -1) && (p.disp >> 16)){
    // Convert to index if displacement is greater than 16 bits
    v << ldimmq{ p.disp, tmp };
    p.index = tmp2;
  }
}

void lowerStoreb(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& storeb_ = inst.storeb_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = storeb_.m;
  patchVptr(p, v);
  v << storeb{ storeb_.s, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerStorebi(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& storebi_ = inst.storebi_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = storebi_.m;
  patchVptr(p, v);
  auto ir = v.makeReg();
  v << ldimmb{ storebi_.s, ir };
  v << storeb{ ir, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerStorel(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& storel_ = inst.storel_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = storel_.m;
  patchVptr(p, v);
  v << storel{ storel_.s, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerStoreli(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& storeli_ = inst.storeli_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = storeli_.m;
  patchVptr(p, v);
  auto ir = v.makeReg();
  v << ldimml{ storeli_.s, ir };
  v << storel{ ir, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerStorew(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& storew_ = inst.storew_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = storew_.m;
  patchVptr(p, v);
  v << storew{ storew_.s, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerStorewi(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& storewi_ = inst.storewi_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = storewi_.m;
  patchVptr(p, v);
  auto ir = v.makeReg();
  v << ldimml{ storewi_.s, ir };
  v << storew{ ir, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerStoreqi(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& storeqi_ = inst.storeqi_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = storeqi_.m;
  patchVptr(p, v);
  auto ir = v.makeReg();
  v << ldimmq{ Immed64(storeqi_.s.q()), ir };
  v << store{ ir, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerStore(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& store_ = inst.store_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = store_.d;
  patchVptr(p, v);
  v << store{ store_.s, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerStoreups(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& storeups_ = inst.storeups_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = storeups_.m;
  patchVptr(p, v);
  v << storeups{ storeups_.s, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerStoresd(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& storesd_ = inst.storesd_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = storesd_.m;
  patchVptr(p, v);
  v << storesd{ storesd_.s, p };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerLoad(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& load_ = inst.load_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = load_.s;
  patchVptr(p, v);
  v << load{ p, load_.d };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerLoadl(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& loadl_ = inst.loadl_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = loadl_.s;
  patchVptr(p, v);
  v << loadl{ p, loadl_.d };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerLoadzbl(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& loadzbl_ = inst.loadzbl_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = loadzbl_.s;
  patchVptr(p, v);
  v << loadzbl{ p, loadzbl_.d };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerLoadzbq(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& loadzbq_ = inst.loadzbq_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = loadzbq_.s;
  patchVptr(p, v);
  v << loadzbq{ p, loadzbq_.d };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerLoadzlq(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& loadzlq_ = inst.loadzlq_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = loadzlq_.s;
  patchVptr(p, v);
  v << loadzlq{ p, loadzlq_.d };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerLoadups(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& loadups_ = inst.loadups_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = loadups_.s;
  patchVptr(p, v);
  v << loadups{ p, loadups_.d};
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerIncwm(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& incwm_ = inst.incwm_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = incwm_.m;
  patchVptr(p, v);
  v << incwm{ p, incwm_.sf };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerCmpqim(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& cmpqim_ = inst.cmpqim_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = cmpqim_.s1;
  patchVptr(p, v);
  v << cmpqim{ cmpqim_.s0, p, cmpqim_.sf };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerCmpbim(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& cmpbim_ = inst.cmpbim_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = cmpbim_.s1;
  patchVptr(p, v);
  v << cmpbim{ cmpbim_.s0, p, cmpbim_.sf };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerCmplim(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& cmplim_ = inst.cmplim_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = cmplim_.s1;
  patchVptr(p, v);
  v << cmplim{ cmplim_.s0, p, cmplim_.sf };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerCmpqm(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& cmpqm_ = inst.cmpqm_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = cmpqm_.s1;
  patchVptr(p, v);
  v << cmpqm{ cmpqm_.s0, p, cmpqm_.sf };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerJmpm(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& jmpm_ = inst.jmpm_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = jmpm_.target;
  patchVptr(p, v);
  v << jmpm{ p, jmpm_.args };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerCallm(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& callm_ = inst.callm_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = callm_.target;
  auto d = v.makeReg();
  v << load { p, d };
  v << callr { d, callm_.args };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerVret(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& vret_ = inst.vret_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  Vptr p = vret_.retAddr;
  Vptr prevFP = vret_.prevFP;
  patchVptr(p, v);
  patchVptr(prevFP, v);
  v << vret{ p, prevFP, vret_.d, vret_.args };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerAbsdbl(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& absdbl = inst.absdbl_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  // clear the high bit
  auto tmp = v.makeReg();
  v << psllq{1, absdbl.s, tmp};
  v << psrlq{1, tmp, absdbl.d};

  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerLoadqp(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& loadqp = inst.loadqp_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  // in PPC we don't have anything like a RIP register
  // RIP register uses a absolute address so we can perform a baseless load in
  // this case
  v << load{ baseless(loadqp.s.r.disp), loadqp.d };
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
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
void lowerPopm(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& popm = inst.popm_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  // PPC can only copy mem->mem by using a temporary register
  auto tmp = v.makeReg();
  v << pop{tmp};
  v << store{tmp, popm.d};

  // remove the original popm (count parameter is 1)
  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerOrwim(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& orwim = inst.orwim_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT {unit.freeScratchBlock(scratch);};
  Vout v(unit, scratch, inst.origin);

  auto tmp = v.makeReg();

  /*
   * TODO(igor): It would be better if there was a 16 bits load instruction
   * But, after these instructions, only 16 bits will be stored.
   */
  v << load {orwim.m, tmp};
  v << orqi {orwim.s0, tmp, tmp, orwim.sf};
  v << storew{tmp, orwim.m};

  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

void lowerOrqim(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& orqim = inst.orqim_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT {unit.freeScratchBlock(scratch);};
  Vout v(unit, scratch, inst.origin);

  auto tmp = v.makeReg();

  v << load {orqim.m, tmp};
  v << orqi {orqim.s0, tmp, tmp, orqim.sf};
  v << store{tmp, orqim.m};

  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

/*
 * As the immediate is bigger than 16 bits, use a temporary register to load
 * that value and use addq vasm afterwards.
 */
void lowerAddqi(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& addqi = inst.addqi_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT {unit.freeScratchBlock(scratch);};
  Vout v(unit, scratch, inst.origin);

  auto tmp = v.makeReg();

  v << ldimmq{Immed64(addqi.s0.q()), tmp};
  v << addq  {tmp, addqi.s1, addqi.d, addqi.sf};

  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
}

/*
 * As the immediate is bigger than 16 bits, use a temporary register to load
 * that value and use cmpq vasm afterwards.
 */
void lowerCmpqi(Vunit& unit, Vlabel b, size_t iInst) {
  auto const& inst = unit.blocks[b].code[iInst];
  auto const& cmpqi = inst.cmpqi_;
  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT {unit.freeScratchBlock(scratch);};
  Vout v(unit, scratch, inst.origin);

  auto tmp = v.makeReg();

  v << ldimmq{Immed64(cmpqi.s0.q()), tmp};
  v << cmpq  {tmp, cmpqi.s1, cmpqi.sf};

  vector_splice(unit.blocks[b].code, iInst, 1, unit.blocks[scratch].code);
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
      auto& inst = blocks[ib].code[ii];

      vlower(unit, ib, ii);

      switch (inst.op) {

        case Vinstr::storeb:
          lowerStoreb(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::storebi:
          lowerStoreb(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::storel:
          lowerStorel(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::storeli:
          lowerStoreli(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::storew:
          lowerStorew(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::storewi:
          lowerStorewi(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::storeqi:
          lowerStoreqi(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::store:
          lowerStore(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::storeups:
          lowerStoreups(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::storesd:
          lowerStoresd(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::load:
          lowerLoad(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::loadzbl:
          lowerLoadzbl(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::loadzbq:
          lowerLoadzbq(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::loadzlq:
          lowerLoadzlq(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::loadl:
          lowerLoadl(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::loadups:
          lowerLoadups(unit, Vlabel{ib}, ii);
          break;

       case Vinstr::incwm:
          lowerIncwm(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::cmpqim:
          lowerCmpqim(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::cmpbim:
          lowerCmpbim(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::cmplim:
          lowerCmplim(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::cmpqm:
          lowerCmpqm(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::callm:
          lowerCallm(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::jmpm:
          lowerJmpm(unit, Vlabel{ib}, ii);
          break;

      case Vinstr::vret:
          lowerVret(unit, Vlabel{ib}, ii);
        break;

        case Vinstr::absdbl:
          lowerAbsdbl(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::loadqp:
          lowerLoadqp(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::popm:
          lowerPopm(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::orwim:
          lowerOrwim(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::orqim:
          lowerOrqim(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::addqi:
          // only immediate up to 16bits can be used on addi asm instr
          if (!inst.addqi_.s0.fits(HPHP::sz::word))
            lowerAddqi(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::cmpqi:
          // only immediate up to 16bits can be used on cmpdi asm instr
          if (!inst.cmpqi_.s0.fits(HPHP::sz::word))
            lowerCmpqi(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::countbytecode:
          inst = incqm{inst.countbytecode_.base[g_bytecodesVasm.handle()],
                       inst.countbytecode_.sf};
          break;

        // Lower movs to copy
        case Vinstr::movtqb:
          inst = copy{inst.movtqb_.s, inst.movtqb_.d};
          break;
        case Vinstr::movtql:
          inst = copy{inst.movtql_.s, inst.movtql_.d};
          break;

        // Lower comparison to cmpq
        case Vinstr::cmpb:
          inst = cmpq{Reg64(inst.cmpb_.s0), Reg64(inst.cmpb_.s1),
                      inst.cmpb_.sf};
          break;
        case Vinstr::cmpl:
          inst = cmpq{Reg64(inst.cmpl_.s0), Reg64(inst.cmpl_.s1),
                      inst.cmpl_.sf};
          break;

        // Lower comparison with immediate to cmpqi
        case Vinstr::cmpbi:
          inst = cmpqi{inst.cmpbi_.s0, Reg64(inst.cmpbi_.s1), inst.cmpbi_.sf};
          break;
        case Vinstr::cmpli:
          if (inst.cmpli_.s0.fits(HPHP::sz::word))
            // immediate can be used right away
            inst = cmpqi{inst.cmpli_.s0, Reg64(inst.cmpli_.s1), inst.cmpli_.sf};
          else
            // lower it just like cmpqi with immediate size > 16bits
            lowerCmpqi(unit, Vlabel{ib}, ii);
          break;

        // Lower subtraction to subq
        case Vinstr::subl:
          inst = subq{Reg64(inst.subl_.s0), Reg64(inst.subl_.s1),
                      Reg64(inst.subl_.d), inst.subl_.sf};
          break;
        case Vinstr::subbi:
          inst = subqi{inst.subbi_.s0, Reg64(inst.subbi_.s1),
                       Reg64(inst.subbi_.d), inst.subbi_.sf};
          break;
        case Vinstr::subli:
          inst = subqi{inst.subli_.s0, Reg64(inst.subli_.s1),
                       Reg64(inst.subli_.d), inst.subli_.sf};
          break;

        // Lower test to testq
        case Vinstr::testb:
          inst = testq{Reg64(inst.testb_.s0), Reg64(inst.testb_.s1),
                       inst.testb_.sf};
          break;
        case Vinstr::testl:
          inst = testq{Reg64(inst.testl_.s0), Reg64(inst.testl_.s1),
                       inst.testl_.sf};
          break;
        case Vinstr::testbi:
          inst = testqi{inst.testbi_.s0, Reg64(inst.testbi_.s1),
                        inst.testbi_.sf};
          break;
        case Vinstr::testli:
          inst = testqi{inst.testli_.s0, Reg64(inst.testli_.s1),
                        inst.testli_.sf};
          break;

        // Lower xor to xorq
        case Vinstr::xorb:
          inst = xorq{Reg64(inst.xorb_.s0), Reg64(inst.xorb_.s1),
                      Reg64(inst.xorb_.d), inst.xorb_.sf};
          break;
        case Vinstr::xorl:
          inst = xorq{Reg64(inst.xorl_.s0), Reg64(inst.xorl_.s1),
                      Reg64(inst.xorl_.d), inst.xorl_.sf};
          break;

        // Lower xor with immediate to xorqi
        case Vinstr::xorbi:
          inst = xorqi{inst.xorbi_.s0, Reg64(inst.xorbi_.s1),
                       Reg64(inst.xorbi_.d), inst.xorbi_.sf};
          break;

        // Lower and to andq
        case Vinstr::andb:
          inst = andq{Reg64(inst.andb_.s0), Reg64(inst.andb_.s1),
                       Reg64(inst.andb_.d), inst.andb_.sf};
          break;
        case Vinstr::andl:
          inst = andq{Reg64(inst.andl_.s0), Reg64(inst.andl_.s1),
                       Reg64(inst.andl_.d), inst.andl_.sf};
          break;
        case Vinstr::andbi:
          inst = andqi{inst.andbi_.s0, Reg64(inst.andbi_.s1),
                       Reg64(inst.andbi_.d), inst.andbi_.sf};
          break;

        default:
          break;
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

