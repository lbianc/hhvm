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
#include "hphp/runtime/vm/jit/code-gen-ppc64.h"
#include "hphp/runtime/vm/jit/code-gen-cf.h"
#include "hphp/runtime/vm/jit/ir-opcode.h"
#include "hphp/runtime/vm/jit/mc-generator-internal.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/translator-inline.h"
#include "hphp/runtime/vm/jit/translator.h"
#include "hphp/runtime/vm/jit/vasm-emit.h"
#include "hphp/runtime/vm/jit/vasm-instr.h"
#include "hphp/runtime/vm/jit/vasm-reg.h"

#include "hphp/util/asm-ppc64.h"
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
                 const size_t align /* =kJmpTargetAlign */) {}

void emitEagerSyncPoint(Vout& v, const Op* pc, Vreg rds, Vreg vmfp, Vreg vmsp) {}

void emitGetGContext(Vout& v, Vreg dest) {
  emitTLSLoad<ExecutionContext>(v, g_context, dest);
}

void emitGetGContext(Asm& as, PhysReg dest) {}

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
                   MaybeDataType t = folly::none) {}

  ~IfCountNotStatic() {
    delete m_cb;
  }
};

void emitTransCounterInc(Vout& v) {}

void emitTransCounterInc(Asm& a) {}

Vreg emitDecRef(Vout& v, Vreg base) {}

void emitIncRef(Vout& v, Vreg base) {}

void emitIncRef(Asm& as, PhysReg base) {}

void emitIncRefCheckNonStatic(Asm& as, PhysReg base, DataType dtype) {}

void emitIncRefGenericRegSafe(Asm& as, PhysReg base, int disp, PhysReg tmpReg) {}

void emitAssertFlagsNonNegative(Vout& v, Vreg sf) {}

void emitAssertRefCount(Vout& v, Vreg base) {}

// Logical register move: ensures the value in src will be in dest
// after execution, but might do so in strange ways. Do not count on
// being able to smash dest to a different register in the future, e.g.
void emitMovRegReg(Asm& as, PhysReg srcReg, PhysReg dstReg) {}

void emitLea(Asm& as, MemoryRef mr, PhysReg dst) {}

Vreg emitLdObjClass(Vout& v, Vreg objReg, Vreg dstReg) {}

Vreg emitLdClsCctx(Vout& v, Vreg src, Vreg dst) {}

void emitCall(Asm& a, TCA dest, RegSet args) {}

void emitCall(Asm& a, CppCall call, RegSet args) {}

void emitCall(Vout& v, CppCall target, RegSet args) {}

void emitImmStoreq(Vout& v, Immed64 imm, Vptr ref) {}

void emitImmStoreq(Asm& a, Immed64 imm, MemoryRef ref) {}

void emitRB(Vout& v, Trace::RingBufferType t, const char* msg) {}

void emitTestSurpriseFlags(Asm& a, PhysReg rds) {}

Vreg emitTestSurpriseFlags(Vout& v, Vreg rds) {}

void emitCheckSurpriseFlagsEnter(CodeBlock& mainCode, CodeBlock& coldCode,
                                 PhysReg rds, Fixup fixup) {}

void emitCheckSurpriseFlagsEnter(Vout& v, Vout& vcold, Vreg fp, Vreg rds,
                                 Fixup fixup, Vlabel catchBlock) {}

void emitLdLowPtr(Vout& v, Vptr mem, Vreg reg, size_t size) {}

void emitCmpClass(Vout& v, Vreg sf, const Class* c, Vptr mem) {}

void emitCmpClass(Vout& v, Vreg sf, Vreg reg, Vptr mem) {}

void emitCmpClass(Vout& v, Vreg sf, Vreg reg1, Vreg reg2) {}

void copyTV(Vout& v, Vloc src, Vloc dst, Type destType) {}

// copy 2 64-bit values into one 128-bit value
void pack2(Vout& v, Vreg s0, Vreg s1, Vreg d0) {}

Vreg zeroExtendIfBool(Vout& v, const SSATmp* src, Vreg reg) {}

}}}
