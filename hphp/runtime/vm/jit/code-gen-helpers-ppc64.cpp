/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | Copyright (c) 2010-2014 Facebook, Inc. (http://www.facebook.com)     |
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

#include "hphp/runtime/vm/jit/code-gen-helpers-ppc64.h"

#include "hphp/runtime/base/arch.h"
#include "hphp/runtime/base/runtime-option.h"
#include "hphp/runtime/base/stats.h"
#include "hphp/runtime/base/types.h"
#include "hphp/runtime/vm/jit/back-end.h"
#include "hphp/runtime/vm/jit/code-gen-cf.h"
#include "hphp/runtime/vm/jit/ir-opcode.h"
#include "hphp/runtime/vm/jit/mc-generator-internal.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/translator-inline.h"
#include "hphp/runtime/vm/jit/translator.h"
#include "hphp/runtime/vm/jit/vasm-emit.h"
#include "hphp/runtime/vm/jit/vasm-instr.h"
#include "hphp/runtime/vm/jit/vasm-reg.h"

#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/util/ringbuffer.h"
#include "hphp/util/trace.h"

namespace HPHP { namespace jit { namespace ppc64 {

//////////////////////////////////////////////////////////////////////

using namespace jit::reg;

TRACE_SET_MOD(hhir);

//////////////////////////////////////////////////////////////////////

/*
 * It's not normally ok to directly use tracelet abi registers in
 * codegen, unless you're directly dealing with an instruction that
 * does near-end-of-tracelet glue.  (Or also we sometimes use them
 * just for some static_assertions relating to calls to helpers from
 * mcg that hardcode these registers.)
 */

/*
 * Satisfy an alignment constraint. Bridge the gap with int3's.
 */
void moveToAlign(CodeBlock& cb,
                 const size_t align /* =kJmpTargetAlign */) {
  Asm a { cb };
  assertx(folly::isPowTwo(align));
  size_t leftInBlock = align - ((align - 1) & uintptr_t(cb.frontier()));
  if (leftInBlock == align) return;
  a.emitNop(leftInBlock);
}

void emitEagerSyncPoint(Vout& v, const Op* pc, Vreg rds, Vreg vmfp,
                            Vreg vmsp) { not_implemented(); }

void emitGetGContext(Vout& v, Vreg dest) {
  emitTLSLoad<ExecutionContext>(v, g_context, dest);
}

void emitGetGContext(Asm& as, PhysReg dest) { not_implemented(); }

// IfCountNotStatic --
//   Emits if (%reg->_count < 0) { ... }.
//   This depends on UncountedValue and StaticValue
//   being the only valid negative refCounts and both indicating no
//   ref count is needed.
//   May short-circuit this check if the type is known to be
//   static already.
struct IfCountNotStatic {
  typedef CondBlock<FAST_REFCOUNT_OFFSET,
                    0,
                    CC_S,
                    int32_t> NonStaticCondBlock;
  static_assert(UncountedValue < 0 && StaticValue < 0, "");
  NonStaticCondBlock *m_cb; // might be null
  IfCountNotStatic(Asm& as, PhysReg reg,
                   MaybeDataType t = folly::none) { not_implemented(); }

  ~IfCountNotStatic() {
    delete m_cb;
  }
};

void emitTransCounterInc(Vout& v) { not_implemented(); }

void emitTransCounterInc(Asm& a) { not_implemented(); }

Vreg emitDecRef(Vout& v, Vreg base) { not_implemented(); }

void emitIncRef(Vout& v, Vreg base) { not_implemented(); }

void emitIncRef(Asm& as, PhysReg base) { not_implemented(); }

void emitIncRefCheckNonStatic(Asm& as, PhysReg base, DataType dtype) {
  not_implemented();
}

void emitIncRefGenericRegSafe(Asm& as, PhysReg base, int disp,
                                  PhysReg tmpReg) { not_implemented(); }

void emitAssertFlagsNonNegative(Vout& v, Vreg sf) { not_implemented(); }

void emitAssertRefCount(Vout& v, Vreg base) { not_implemented(); }

// Logical register move: ensures the value in src will be in dest
// after execution, but might do so in strange ways. Do not count on
// being able to smash dest to a different register in the future, e.g.
void emitMovRegReg(Asm& as, PhysReg srcReg, PhysReg dstReg) {
  not_implemented();
}

void emitLea(Asm& as, MemoryRef mr, PhysReg dst) { not_implemented(); }

Vreg emitLdObjClass(Vout& v, Vreg objReg, Vreg dstReg) { not_implemented(); }

Vreg emitLdClsCctx(Vout& v, Vreg src, Vreg dst) { not_implemented(); }

void emitCall(Asm& a, TCA dest, RegSet args) { not_implemented(); }

void emitCall(Asm& a, CppCall call, RegSet args) {
  emitCall(Vauto(a.code()).main(), call, args);
}

void emitCall(Vout& v, CppCall target, RegSet args) {
  switch (target.kind()) {
  case CppCall::Kind::Direct:
    v << call{static_cast<TCA>(target.address()), args};
    return;
  case CppCall::Kind::Virtual:
    // Virtual call.
    // Load method's address from proper offset off of object in 1st arg
    // using rAsm as scratch.
    v << load{*argNumToRegName[0], rAsm};
    v << callm{rAsm[target.vtableOffset()], args};
    return;
  case CppCall::Kind::ArrayVirt: {
    auto const addr = reinterpret_cast<intptr_t>(target.arrayTable());
    always_assert_flog(
      deltaFits(addr, sz::dword),
      "deltaFits on ArrayData vtable calls needs to be checked before "
      "emitting them"
    );
    v << loadzbq{argNumToRegName[0][HeaderKindOffset], rAsm};
    v << callm{baseless(rAsm*8 + addr), args};
    static_assert(sizeof(HeaderKind) == 1, "");
    return;
  }
  case CppCall::Kind::Destructor:
    // this movzbq is only needed because callers aren't required to
    // zero-extend the type.
    auto zextType = v.makeReg();
    v << movzbq{target.reg(), zextType};
    auto dtor_ptr = lookupDestructor(v, zextType);
    v << callm{dtor_ptr, args};
    return;
  }
  not_reached();
}

void emitImmStoreq(Vout& v, Immed64 imm, Vptr ref) { not_implemented(); }

void emitImmStoreq(Asm& a, Immed64 imm, MemoryRef ref) { not_implemented(); }

void emitRB(Vout& v, Trace::RingBufferType t, const char* msg) {
  not_implemented();
}

void emitTestSurpriseFlags(Asm& a, PhysReg rds) { not_implemented(); }

Vreg emitTestSurpriseFlags(Vout& v, Vreg rds) { not_implemented(); }

void emitCheckSurpriseFlagsEnter(CodeBlock& mainCode, CodeBlock& coldCode,
                                 PhysReg rds, Fixup fixup) {
  not_implemented();
}

void emitCheckSurpriseFlagsEnter(Vout& v, Vout& vcold, Vreg fp, Vreg rds,
                                 Fixup fixup, Vlabel catchBlock) {
  not_implemented();
}

void emitLdLowPtr(Vout& v, Vptr mem, Vreg reg, size_t size) {
  not_implemented();
}

void emitCmpClass(Vout& v, Vreg sf, const Class* c, Vptr mem) {
  not_implemented();
}

void emitCmpClass(Vout& v, Vreg sf, Vreg reg, Vptr mem) { not_implemented(); }

void emitCmpClass(Vout& v, Vreg sf, Vreg reg1, Vreg reg2) {
  not_implemented();
}

void copyTV(Vout& v, Vloc src, Vloc dst, Type destType) { not_implemented(); }

// copy 2 64-bit values into one 128-bit value
void pack2(Vout& v, Vreg s0, Vreg s1, Vreg d0) { not_implemented(); }

Vreg zeroExtendIfBool(Vout& v, const SSATmp* src, Vreg reg) {
  not_implemented();
}

}}}
