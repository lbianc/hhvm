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
    Vptr p(ppc64_asm::reg::r1, lr_position_on_callstack);
    a->std(ppc64_asm::reg::r0, p);
    // minimum call stack
    p.disp = -min_callstack_size;
    a->stdu(ppc64_asm::reg::r1, p);
  }

  inline void popMinCallStack(void)
  {
    // minimum call stack
    a->addi(ppc64_asm::reg::r1, ppc64_asm::reg::r1, min_callstack_size);
    // LR on parent call frame
    Vptr p(ppc64_asm::reg::r1, lr_position_on_callstack);
    a->ld(ppc64_asm::reg::r0, p);
    a->mtlr(ppc64_asm::reg::r0);
  }

  /*
   * Calculates the effective address of Vptr s and stores on Register d
   * The parameter ignore_base can be used to ignore base register for 
   * load/store instructions. In this case base cannot be added to index
   * register.
   */
  inline void VptrAddressToReg(Vptr s, Vreg d, bool ignore_base) {
    if (s.index.isValid()) {
      // Calculate index position before adding base and displacement.
      // If scale is 1 we just ignore it.
      if(scale > 1) {
        int n = 0;
        int scale = s.scale;
        while (scale >>= 1) {
          ++n;
        }
        assert(n <= 3);
        // scale factor is always 1, 2, 4 or, 8
        // so we can perform index*scale doing a shift left
        emit(shlqi{n, s.index, d, VregSF(0)});
      }

      if (s.base.isValid() && !ignore_base) {
        emit(addq {s.base, d, d, VregSF(0)});
      }
      // if we have displacement 0 we can avoid this instruction
      if(s.disp != 0) {
        emit(addqi{s.disp, d, d, VregSF(0)});
      }

    } else {
      // Indexless
      if (s.base.isValid()) {
        // Base + Displacement
        emit(addqi{s.disp, s.base, d, VregSF(0)});
      } else {
        // Baseless
        emit(ldimmq{s.disp, d});
      }
    }
    //TODO(rcardoso): We can insert this here and get rid of VptrToReg function?
    //emit(load{*d, d});
  }

  /*
   * Stores in d the value pointed by s
   */
  inline void VptrToReg(Vptr s, Vreg d, bool ignore_base=0) {
    VptrAddressToReg(s, d, ignore_base);
    emit(load{*d, d}); //TODO(rcardoso): ??
  }

  /*
   * We can have the following address modes in X64
   *
   * - Direct Operand: displacement
   * - Indirect Operand: (base)
   * - Base + Displacement: displacement(base)
   * - (Index * Scale) + Displacement: displacement(,index,scale)
   * - Base + Index + Displacement: displacement(base,index)
   * - Base +(Index * Scale) + Displacement: displacement(base, index,scale)
   * 
   * In PPC64 we have:
   * - Direct Operand: displacement (Form-D)
   * - Indirect Operand: (Base with displacement = 0) (Form-D)
   * - Base + Index: Index(Base) (Form-X)
   * 
   *  If we have displacement > 16 bits we have to use Form-X. So if we get
   *  a Vptr with a unsupported address mode (like Index * Scale) we need
   *  to convert (patch) this address mode to a supported address mode.
   */
  inline void PatchMemoryOperands(Vptr s) {
    // we do nothing for supported address modes
    if(s.index.isValid() || ((s.disp >> 16) > 0)) {
      VptrToReg(s, s.index, 1);
    }
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
  void emit(const landingpad& i) { not_implemented(); }
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
  void emit(andb i) {  a->and_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
  void emit(andbi i) { a->andi(Reg64(i.d), Reg64(i.s1), i.s0); }
  void emit(const andbim& i) { not_implemented(); }
  void emit(andl i) { a->and_(Reg64(i.d), Reg64(i.s0), Reg64(i.s1), false); }
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
  void emit(const cmpbim& i) {
    VptrToReg(i.s1, ppc64::rvasmtmp());
    a->cmpi(0, 0, ppc64::rvasmtmp(), i.s0);
  }
  void emit(const cmpl& i) {  a->cmp(0, 0, Reg64(i.s0), Reg64(i.s1)); }
  void emit(const cmpli& i) { a->cmpi(0, 0, Reg64(i.s1), i.s0); }
  void emit(const cmplim& i) {
    VptrToReg(i.s1, ppc64::rvasmtmp());
    a->cmpi(0, 0, ppc64::rvasmtmp(), i.s0);
  }
  void emit(const cmplm& i) { not_implemented(); }
  //TODO(IBM): field 1 indicates cr (cr0) register who holds the bf result
  void emit(const cmpq& i) { a->cmp(0, 0, i.s0, i.s1); }
  //TODO(IBM): field 1 indicates cr (cr0) register who holds the bf result
  void emit(const cmpqi& i) { a->cmpi(0, 0, i.s1, i.s0); }
  void emit(const cmpqim& i) {
    VptrToReg(i.s1, ppc64::rvasmtmp());
    a->cmpdi(ppc64::rvasmtmp(), i.s0);
  }
  void emit(const cmpqm& i) {
    VptrToReg(i.s1, ppc64::rvasmtmp());
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
    VptrToReg(i.m, ppc64::rvasmtmp());
    a->addi(ppc64::rvasmtmp(), ppc64::rvasmtmp(), 1);
    if (i.m.index.isValid()) {
      PatchMemoryOperands(i.m);
      a->stdx(ppc64::rvasmtmp(), i.m);
    } else {
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

      // offset to be determined by a->patchBc
      a->bc(i.cc, 0);
    }
    emit(jmp{i.targets[0]});
  }
  void emit(const jcci& i) { not_implemented(); }
  void emit(const jmp& i) {
    if (next == i.target) return;
    jmps.push_back({a->frontier(), i.target});

    // offset to be determined by a->patchBc
    BranchParams bp(BranchConditions::Always);
    a->bc(bp.bo(), bp.bi(), 0);
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
  void emit(const leap& i) { a->li64(i.d, i.s.r.disp); }
  void emit(const loadups& i) { not_implemented(); }
  void emit(const loadtqb& i) { not_implemented(); }
  void emit(const loadl& i) {
    PatchMemoryOperands(i.s);
    if(i.s.index.isValid()) {
      a->lwzx(Reg64(i.d), i.s);
    } else {
      a->lwz(Reg64(i.d), i.s);
    }
  }
  void emit(const loadqp& i) { not_implemented(); }
  void emit(const loadsd& i) { not_implemented(); }
  void emit(const loadzbl& i) {
    PatchMemoryOperands(i.s);
    if(i.s.index.isValid()) {
      a->lbzx(Reg64(i.d), i.s);
    } else {
      a->lbz(Reg64(i.d), i.s);
    }
  }
  void emit(const loadzbq& i) {
    PatchMemoryOperands(i.s);
    if(i.s.index.isValid()) {
      a->lbzx(i.d, i.s);
    } else {
      a->lbz(i.d, i.s);
    }
  }
  void emit(const loadzlq& i) {
    PatchMemoryOperands(i.s);
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
  void emit(const sarq& i) { not_implemented(); } // should use vasm sar below
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
  void emit(shlq i) { not_implemented(); } // should use vasm shl below
  /*TODO Rc=1*/
  void emit(shl i) { a->sld(i.d, i.s1, i.s0); }
  void emit(shlqi i) { a->sldi(i.d, i.s1, i.s0.b()); }
  void emit(shrli i) { a->srwi(Reg64(i.d), Reg64(i.s1), i.s0.b()); }
  void emit(shrqi i) { a->srdi(i.d, i.s1, i.s0.b()); }
  void emit(const sqrtsd& i) { not_implemented(); }
  void emit(const storeups& i) { not_implemented(); }
  void emit(const storeb& i) {
    if(i.m.index.isValid()) {
      PatchMemoryOperands(i.m);
      a->stbx(Reg64(i.s), i.m);
    } else {
      a->stb(Reg64(i.s), i.m);
    }
  }
  void emit(const storebi& i) {
    a->li(ppc64::rvasmtmp(), (i.s.l() & UINT8_MAX));
    if(i.m.index.isValid()) {
      PatchMemoryOperands(i.m);
      a->stbx(ppc64::rvasmtmp(), i.m);
    } else {
      a->stb(ppc64::rvasmtmp(), i.m);
    }
  }
  void emit(const storel& i) {
    if(i.m.index.isValid()) {
      PatchMemoryOperands(i.m);
      a->stwx(Reg64(i.s), i.m);
    } else {
      a->stw(Reg64(i.s), i.m);
    }
  }
  void emit(const storeli& i) { not_implemented(); }
  void emit(const storeqi& i) {
    a->li64(ppc64::rvasmtmp(), i.s.q());
    if (i.m.index.isValid()) {
      PatchMemoryOperands(i.m);
      a->stdx(ppc64::rvasmtmp(), i.m);
    } else {
      a->std(ppc64::rvasmtmp(), i.m);
    }
  }
  void emit(const storesd& i) { not_implemented(); }
  void emit(const storew& i) {
    if(i.m.index.isValid()) {
      PatchMemoryOperands(i.m);
      a->sthx(Reg64(i.s), i.m);
    } else {
      a->sth(Reg64(i.s), i.m);
    }
  }
  void emit(const storewi& i) { not_implemented(); }
  void emit(subbi i) { not_implemented(); }
  void emit(subl i) { a->subf(Reg64(i.d), Reg64(i.s1), Reg64(i.s0), false); }
  void emit(subli i) { a->addi(Reg64(i.s1), Reg64(i.d), i.s0); }
  void emit(subq i) { a->subf(i.d, i.s1, i.s0, false); }
  void emit(subqi i) { a->addi(i.s1, i.d, i.s0); /*addi with negative value*/ }
  void emit(subsd i) { not_implemented(); }
  void emit(const testb& i) {
    a->and_(ppc64::rvasmtmp(), Reg64(i.s0), Reg64(i.s1), true);
  }
  void emit(const testbi& i) { a->andi(ppc64::rvasmtmp(), Reg64(i.s1), i.s0); }
  void emit(const testbim& i) {
    a->lbz(ppc64::rvasmtmp(), i.s1.mr());
    emit(testbi{i.s0, ppc64::rvasmtmp(), i.sf});
  }
  void emit(const testwim& i) {
    a->lhz(ppc64::rvasmtmp(), i.s1);
    emit(testli{i.s0, ppc64::rvasmtmp(), i.sf});
  }
  void emit(const testl& i) {
    a->and_(ppc64::rvasmtmp(), Reg64(i.s0), Reg64(i.s1), true);
  }
  void emit(const testli& i) { a->andi(ppc64::rvasmtmp(), Reg64(i.s1), i.s0); }
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
  Vreg tmp_lr = ppc64::rvasmtmp();
  VptrToReg(i.retAddr, tmp_lr);
  a->mtlr(tmp_lr);
  a->ldx(i.d, i.prevFP);
  a->blr();
}

void Vgen::emit(const load& i) {
  if (i.d.isGP()) {
    if (i.s.index.isValid()){
      PatchMemoryOperands(i.s);
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
    ppc64_asm::Assembler::patchBc(p.instr, env.addrs[p.target]);
  }
  for (auto& p : env.jccs) {
    assertx(env.addrs[p.target]);
    ppc64_asm::Assembler::patchBc(p.instr, env.addrs[p.target]);
  }
  assertx(env.bccs.empty());
}

void Vgen::pad(CodeBlock& cb) {
  not_implemented();
}

void Vgen::emit(const store& i) {
  if (i.s.isGP()) {
    if (i.d.index.isValid()){
      PatchMemoryOperands(i.d);
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

void lowerVcall(Vunit& unit, Vlabel b, size_t iInst) {
  auto& blocks = unit.blocks;
  auto& inst = blocks[b].code[iInst];
  auto const is_vcall = inst.op == Vinstr::vcall;
  auto const vcall = inst.vcall_;
  auto const vinvoke = inst.vinvoke_;

  // Extract all the relevant information from the appropriate instruction.
  auto const is_smashable = !is_vcall && vinvoke.smashable;
  auto const call = is_vcall ? vcall.call : vinvoke.call;
  auto const& vargs = unit.vcallArgs[is_vcall ? vcall.args : vinvoke.args];
  auto const& stkArgs = vargs.stkArgs;
  auto const dests = unit.tuples[is_vcall ? vcall.d : vinvoke.d];
  auto const fixup = is_vcall ? vcall.fixup : vinvoke.fixup;
  auto const destType = is_vcall ? vcall.destType : vinvoke.destType;

  auto scratch = unit.makeScratchBlock();
  SCOPE_EXIT { unit.freeScratchBlock(scratch); };
  Vout v(unit, scratch, inst.origin);

  int32_t const adjust = (stkArgs.size() & 0x1) ? sizeof(uintptr_t) : 0;
  if (adjust) v << subqi{adjust, reg::rsp, reg::rsp, v.makeReg()};

  // Push stack arguments, in reverse order.
  for (int i = stkArgs.size() - 1; i >= 0; --i) v << push{stkArgs[i]};

  // Get the arguments in the proper registers.
  RegSet argRegs;
  auto doArgs = [&] (const VregList& srcs, PhysReg (*r)(size_t)) {
    VregList argDests;
    for (size_t i = 0, n = srcs.size(); i < n; ++i) {
      auto const reg = r(i);
      argDests.push_back(reg);
      argRegs |= reg;
    }

    if (argDests.size()) {
      v << copyargs{v.makeTuple(srcs),
                    v.makeTuple(std::move(argDests))};
    }
  };
  doArgs(vargs.args, rarg);
  doArgs(vargs.simdArgs, rarg_simd);

  // Emit the call.
  if (is_smashable) v << mccall{(TCA)call.address(), argRegs};
  else              emitCall(v, call, argRegs);

  // Handle fixup and unwind information.
  if (fixup.isValid()) v << syncpoint{fixup};

  if (!is_vcall) {
    auto& targets = vinvoke.targets;
    v << unwind{{targets[0], targets[1]}};

    // Insert an lea fixup for any stack args at the beginning of the catch
    // block.
    if (auto rspOffset = ((stkArgs.size() + 1) & ~1) * sizeof(uintptr_t)) {
      auto& taken = unit.blocks[targets[1]].code;
      assertx(taken.front().op == Vinstr::landingpad ||
             taken.front().op == Vinstr::jmp);
      Vinstr v{lea{reg::rsp[rspOffset], reg::rsp}};
      v.origin = taken.front().origin;
      if (taken.front().op == Vinstr::jmp) {
        taken.insert(taken.begin(), v);
      } else {
        taken.insert(taken.begin() + 1, v);
      }
    }

    // Write out the code so far to the end of b. Remaining code will be
    // emitted to the next block.
    vector_splice(blocks[b].code, iInst, 1, blocks[scratch].code);
  } else if (vcall.nothrow) {
    v << nothrow{};
  }

  // Copy the call result to the destination register(s)
  switch (destType) {
    case DestType::TV: {
      // rax contains m_type and m_aux but we're expecting just the type in
      // the lower bits, so shift the type result register.
      static_assert(offsetof(TypedValue, m_data) == 0, "");
      static_assert(offsetof(TypedValue, m_type) == 8, "");
      if (dests.size() == 2) {
        v << copy2{ppc64_asm::reg::r3, ppc64_asm::reg::r4, dests[0], dests[1]};
      } else {
        // We have cases where we statically know the type but need the value
        // from native call. Even if the type does not really need a register
        // (e.g., InitNull), a Vreg is still allocated in assignRegs(), so the
        // following assertion holds.
        assertx(dests.size() == 1);
        v << copy{ppc64_asm::reg::r3, dests[0]};
      }
      break;
    }
    case DestType::SIMD: {
      // copy the single-register result to dests[0]
      assertx(dests.size() == 1);
      assertx(dests[0].isValid());
      v << copy{ppc64_asm::reg::v2, dests[0]};
      break;
    }
    case DestType::SSA:
    case DestType::Byte:
      // copy the single-register result to dests[0]
      assertx(dests.size() == 1);
      assertx(dests[0].isValid());
      v << copy{ppc64_asm::reg::r3, dests[0]};
      break;
    case DestType::None:
      assertx(dests.empty());
      break;
    case DestType::Dbl:
      // copy the single-register result to dests[0]
      assertx(dests.size() == 1);
      assertx(dests[0].isValid());
      v << copy{ppc64_asm::reg::f1, dests[0]};
      break;
  }

  if (stkArgs.size() > 0) {
    v << addqi{safe_cast<int32_t>(stkArgs.size() * sizeof(uintptr_t)
                                  + adjust),
               reg::rsp,
               reg::rsp,
               v.makeReg()};
  }

  // Insert new instructions to the appropriate block
  if (is_vcall) {
    vector_splice(blocks[b].code, iInst, 1, blocks[scratch].code);
  } else {
    vector_splice(blocks[vinvoke.targets[0]].code, 0, 0,
                  blocks[scratch].code);
  }
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
 */
void lowerForPPC64(Vunit& unit, const Abi& abi) {
  Timer _t(Timer::vasm_lower);

  // This pass relies on having no critical edges in the unit.
  splitCriticalEdges(unit);

  // Scratch block can change blocks allocation, hence cannot use regular
  // iterators.
  auto& blocks = unit.blocks;

  PostorderWalker{unit}.dfs([&](Vlabel ib) {
    assertx(!blocks[ib].code.empty());
    auto& back = blocks[ib].code.back();
    if (back.op == Vinstr::vcallarray) {
      lower_vcallarray(unit, Vlabel{ib});
    }

    for (size_t ii = 0; ii < blocks[ib].code.size(); ++ii) {
      auto& inst = blocks[ib].code[ii];
      switch (inst.op) {
        case Vinstr::vcall:
        case Vinstr::vinvoke:
          lowerVcall(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::absdbl:
          lowerAbsdbl(unit, Vlabel{ib}, ii);
          break;

        case Vinstr::defvmsp:
          inst = copy{rvmsp(), inst.defvmsp_.d};
          break;

        case Vinstr::syncvmsp:
          inst = copy{inst.syncvmsp_.s, rvmsp()};
          break;

        case Vinstr::movtqb:
          inst = copy{inst.movtqb_.s, inst.movtqb_.d};
          break;

        case Vinstr::movtql:
          inst = copy{inst.movtql_.s, inst.movtql_.d};
          break;

        case Vinstr::countbytecode:
          inst = incqm{inst.countbytecode_.base[g_bytecodesVasm.handle()],
                       inst.countbytecode_.sf};
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

  lowerForPPC64(unit, abi);

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
