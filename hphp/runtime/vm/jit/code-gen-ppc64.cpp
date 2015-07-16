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

#include "hphp/runtime/vm/jit/code-gen-ppc64.h"

#include <cstring>
#include <iostream>
#include <limits>
#include <unwind.h>
#include <vector>

#include <folly/ScopeGuard.h>
#include <folly/Format.h>
#include "hphp/util/trace.h"
#include "hphp/util/text-util.h"
#include "hphp/util/abi-cxx.h"

#include "hphp/runtime/base/comparisons.h"
#include "hphp/runtime/base/mixed-array.h"
#include "hphp/runtime/base/rds-header.h"
#include "hphp/runtime/base/rds-util.h"
#include "hphp/runtime/base/rds.h"
#include "hphp/runtime/base/runtime-option.h"
#include "hphp/runtime/base/shape.h"
#include "hphp/runtime/base/stats.h"
#include "hphp/runtime/base/string-data.h"
#include "hphp/runtime/base/types.h"
#include "hphp/runtime/vm/bytecode.h"
#include "hphp/runtime/vm/runtime.h"

#include "hphp/runtime/vm/jit/arg-group.h"
#include "hphp/runtime/vm/jit/back-end-ppc64.h"
#include "hphp/runtime/vm/jit/cfg.h"
#include "hphp/runtime/vm/jit/code-gen-cf.h"
#include "hphp/runtime/vm/jit/code-gen-helpers.h"
#include "hphp/runtime/vm/jit/code-gen-helpers-ppc64.h"
#include "hphp/runtime/vm/jit/ir-opcode.h"
#include "hphp/runtime/vm/jit/mc-generator-internal.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/native-calls.h"
#include "hphp/runtime/vm/jit/print.h"
#include "hphp/runtime/vm/jit/prof-data.h"
#include "hphp/runtime/vm/jit/punt.h"
#include "hphp/runtime/vm/jit/reg-algorithms.h"
#include "hphp/runtime/vm/jit/service-requests-inline.h"
#include "hphp/runtime/vm/jit/service-requests-ppc64.h"
#include "hphp/runtime/vm/jit/stack-offsets-defs.h"
#include "hphp/runtime/vm/jit/stack-offsets.h"
#include "hphp/runtime/vm/jit/target-cache.h"
#include "hphp/runtime/vm/jit/target-profile.h"
#include "hphp/runtime/vm/jit/timer.h"
#include "hphp/runtime/vm/jit/translator-inline.h"
#include "hphp/runtime/vm/jit/translator.h"
#include "hphp/runtime/vm/jit/types.h"
#include "hphp/runtime/vm/jit/vasm-emit.h"
#include "hphp/runtime/vm/jit/vasm-instr.h"
#include "hphp/runtime/vm/jit/vasm-reg.h"
#include "hphp/runtime/vm/jit/vasm.h"

#include "hphp/runtime/ext/asio/asio-blockable.h"
#include "hphp/runtime/ext/asio/async-function-wait-handle.h"
#include "hphp/runtime/ext/asio/wait-handle.h"
#include "hphp/runtime/ext/ext_closure.h"
#include "hphp/runtime/ext/ext_collections.h"
#include "hphp/runtime/ext/ext_generator.h"

#define rVmSp DontUseRVmSpInThisFile

namespace HPHP { namespace jit { namespace ppc64 {

///////////////////////////////////////////////////////////////////////////////

TRACE_SET_MOD(hhir);

namespace {

///////////////////////////////////////////////////////////////////////////////

/*
 * It's not normally ok to directly use tracelet abi registers in
 * codegen, unless you're directly dealing with an instruction that
 * does near-end-of-tracelet glue.  (Or also we sometimes use them
 * just for some static_assertions relating to calls to helpers from
 * mcg that hardcode these registers.)
 */
using namespace jit::reg;

///////////////////////////////////////////////////////////////////////////////

void cgPunt(const char* file, int line, const char* func, uint32_t bcOff,
            const Func* vmFunc, bool resumed,
            TransID profTransId) ATTRIBUTE_NORETURN;

void cgPunt(const char* file, int line, const char* func, uint32_t bcOff,
            const Func* vmFunc, bool resumed, TransID profTransId) {
  if (dumpIREnabled()) {
    auto const phpFile = vmFunc->filename()->data();
    auto const phpLine = vmFunc->unit()->getLineNumber(bcOff);
    HPHP::Trace::trace("--------- CG_PUNT %s at %s:%d from %s:%d (bcOff %d)\n",
                       func, file, line, phpFile, phpLine, bcOff);
  }
  throw FailedCodeGen(file, line, func, bcOff, vmFunc, resumed, profTransId);
}

#define CG_PUNT(marker, instr)                                    \
  cgPunt(__FILE__, __LINE__, #instr, marker.bcOff(),              \
         getFunc(marker), resumed(marker), marker.profTransID())

///////////////////////////////////////////////////////////////////////////////

const char* getContextName(const Class* ctx) {
  return ctx ? ctx->name()->data() : ":anonymous:";
}

///////////////////////////////////////////////////////////////////////////////

template<class Then>
void ifNonStatic(Vout& v, Type ty, Vloc loc, Then then) {
  if (!ty.maybe(TStatic)) {
    then(v);
    return;
  }

  auto const sf = v.makeReg();
  v << cmplim{0, loc.reg()[FAST_REFCOUNT_OFFSET], sf};
  static_assert(UncountedValue < 0 && StaticValue < 0, "");
  ifThen(v, CC_GE, sf, then);
}

template<class Then>
void ifRefCountedType(Vout& v, Vout& vtaken, Type ty, Vloc loc, Then then) {
  if (!ty.maybe(TCounted)) return;
  if (ty.isKnownDataType()) {
    if (IS_REFCOUNTED_TYPE(ty.toDataType())) then(v);
    return;
  }
  auto const sf = v.makeReg();
  emitCmpTVType(v, sf, KindOfRefCountThreshold, loc.reg(1));
  unlikelyIfThen(v, vtaken, CC_NLE, sf, then);
}

template<class Then>
void ifRefCountedNonStatic(Vout& v, Type ty, Vloc loc, Then then) {
  ifRefCountedType(v, v, ty, loc, [&] (Vout& v) {
    ifNonStatic(v, ty, loc, then);
  });
}

///////////////////////////////////////////////////////////////////////////////

/*
 * Emit code to store `loc', the registers representing `src', to `dst'.
 */
void emitStoreTV(Vout& v, Vptr dst, Vloc loc, const SSATmp* src) {}

///////////////////////////////////////////////////////////////////////////////

void debug_trashsp(Vout& v) {
  if (RuntimeOption::EvalHHIRGenerateAsserts) {}
}

void maybe_syncsp(Vout& v, BCMarker marker, Vreg irSP, IRSPOffset off) {}

RegSet leave_trace_args(BCMarker marker) {}

//////////////////////////////////////////////////////////////////////

} // unnamed namespace

//////////////////////////////////////////////////////////////////////

Vloc CodeGenerator::srcLoc(const IRInstruction* inst, unsigned i) const {}

Vloc CodeGenerator::dstLoc(const IRInstruction* inst, unsigned i) const {}

ArgGroup CodeGenerator::argGroup(const IRInstruction* inst) const {}

void CodeGenerator::cgInst(IRInstruction* inst) {}

#define NOOP_OPCODE(opcode) \
  void CodeGenerator::cg##opcode(IRInstruction*) {}

#define CALL_OPCODE(opcode) \
  void CodeGenerator::cg##opcode(IRInstruction* i) { cgCallNative(vmain(), i); }

#define CALL_STK_OPCODE(opcode) \
  CALL_OPCODE(opcode)           \
  CALL_OPCODE(opcode ## Stk)

NOOP_OPCODE(DefConst)
NOOP_OPCODE(DefFP)
NOOP_OPCODE(AssertLoc)
NOOP_OPCODE(Nop)
NOOP_OPCODE(EndGuards)
NOOP_OPCODE(ExitPlaceholder);
NOOP_OPCODE(HintLocInner)
NOOP_OPCODE(HintStkInner)
NOOP_OPCODE(AssertStk)
NOOP_OPCODE(PredictLoc);
NOOP_OPCODE(PredictStk);

CALL_OPCODE(AddElemStrKey)
CALL_OPCODE(AddElemIntKey)
CALL_OPCODE(AddNewElem)
CALL_OPCODE(ArrayAdd)
CALL_OPCODE(Box)
CALL_OPCODE(MapAddElemC)
CALL_OPCODE(ColAddNewElemC)

CALL_OPCODE(CoerceCellToBool);
CALL_OPCODE(CoerceCellToInt);
CALL_OPCODE(CoerceCellToDbl);
CALL_OPCODE(CoerceStrToDbl);
CALL_OPCODE(CoerceStrToInt);

CALL_OPCODE(ConvBoolToArr);
CALL_OPCODE(ConvDblToArr);
CALL_OPCODE(ConvIntToArr);
CALL_OPCODE(ConvObjToArr);
CALL_OPCODE(ConvStrToArr);
CALL_OPCODE(ConvCellToArr);

CALL_OPCODE(ConvStrToBool);
CALL_OPCODE(ConvCellToBool);

CALL_OPCODE(ConvArrToDbl);
CALL_OPCODE(ConvObjToDbl);
CALL_OPCODE(ConvStrToDbl);
CALL_OPCODE(ConvCellToDbl);

CALL_OPCODE(ConvArrToInt);
CALL_OPCODE(ConvObjToInt);
CALL_OPCODE(ConvStrToInt);
CALL_OPCODE(ConvCellToInt);

CALL_OPCODE(ConvCellToObj);

CALL_OPCODE(ConvDblToStr);
CALL_OPCODE(ConvIntToStr);
CALL_OPCODE(ConvObjToStr);
CALL_OPCODE(ConvResToStr);
CALL_OPCODE(ConvCellToStr);

CALL_OPCODE(ConcatStrStr);
CALL_OPCODE(ConcatStrInt);
CALL_OPCODE(ConcatIntStr);
CALL_OPCODE(ConcatStr3);
CALL_OPCODE(ConcatStr4);

CALL_OPCODE(CreateCont)
CALL_OPCODE(CreateAFWH)
CALL_OPCODE(CreateSSWH)
CALL_OPCODE(AFWHPrepareChild)
CALL_OPCODE(ABCUnblock)
CALL_OPCODE(NewArray)
CALL_OPCODE(NewMixedArray)
CALL_OPCODE(NewLikeArray)
CALL_OPCODE(AllocPackedArray)
CALL_OPCODE(Clone)
CALL_OPCODE(AllocObj)
CALL_OPCODE(InitProps)
CALL_OPCODE(InitSProps)
CALL_OPCODE(RegisterLiveObj)
CALL_OPCODE(LdClsCtor)
CALL_OPCODE(LookupClsRDSHandle)
CALL_OPCODE(PrintStr)
CALL_OPCODE(PrintInt)
CALL_OPCODE(PrintBool)
CALL_OPCODE(DbgAssertPtr)
CALL_OPCODE(LdSwitchDblIndex)
CALL_OPCODE(LdSwitchStrIndex)
CALL_OPCODE(LdSwitchObjIndex)
CALL_OPCODE(VerifyParamCallable)
CALL_OPCODE(VerifyParamFail)
CALL_OPCODE(VerifyRetCallable)
CALL_OPCODE(VerifyRetFail)
CALL_OPCODE(RaiseUninitLoc)
CALL_OPCODE(RaiseUndefProp)
CALL_OPCODE(RaiseMissingArg)
CALL_OPCODE(RaiseError)
CALL_OPCODE(RaiseWarning)
CALL_OPCODE(RaiseNotice)
CALL_OPCODE(RaiseArrayIndexNotice)
CALL_OPCODE(RaiseArrayKeyNotice)
CALL_OPCODE(IncStatGrouped)
CALL_OPCODE(ClosureStaticLocInit)
CALL_OPCODE(GenericIdx)
CALL_OPCODE(MapIdx)
CALL_OPCODE(LdClsPropAddrOrNull)
CALL_OPCODE(LdClsPropAddrOrRaise)
CALL_OPCODE(LdGblAddrDef)

// Vector instruction helpers
CALL_OPCODE(StringGet)
CALL_OPCODE(BindElem)
CALL_OPCODE(SetWithRefElem)
CALL_OPCODE(SetWithRefNewElem)
CALL_OPCODE(SetOpElem)
CALL_OPCODE(IncDecElem)
CALL_OPCODE(SetNewElem)
CALL_OPCODE(SetNewElemArray)
CALL_OPCODE(BindNewElem)
CALL_OPCODE(VectorIsset)
CALL_OPCODE(PairIsset)
CALL_OPCODE(ThrowOutOfBounds)

CALL_OPCODE(InstanceOfIface)
CALL_OPCODE(InterfaceSupportsArr)
CALL_OPCODE(InterfaceSupportsStr)
CALL_OPCODE(InterfaceSupportsInt)
CALL_OPCODE(InterfaceSupportsDbl)

CALL_OPCODE(ZeroErrorLevel)
CALL_OPCODE(RestoreErrorLevel)

CALL_OPCODE(Count)

CALL_OPCODE(SuspendHookE)
CALL_OPCODE(SuspendHookR)
CALL_OPCODE(ReturnHook)

CALL_OPCODE(OODeclExists)

CALL_OPCODE(GetMemoKey)

#undef NOOP_OPCODE

///////////////////////////////////////////////////////////////////////////////

Vlabel CodeGenerator::label(Block* b) {
  return m_state.labels[b];
}

void CodeGenerator::emitFwdJcc(Vout& v, ConditionCode cc, Vreg sf,
                               Block* target) {}

///////////////////////////////////////////////////////////////////////////////

void CodeGenerator::cgDefSP(IRInstruction* inst) {
  auto const sp = dstLoc(inst, 0).reg();
  auto& v = vmain();

  if (inst->marker().resumed()) {
    v << defvmsp{sp};
    return;
  }

  auto const fp = srcLoc(inst, 0).reg();
  v << lea{fp[-cellsToBytes(inst->extra<DefSP>()->offset.offset)], sp};
}

void CodeGenerator::cgCheckNullptr(IRInstruction* inst) {}

void CodeGenerator::cgCheckNonNull(IRInstruction* inst) {}

void CodeGenerator::cgAssertNonNull(IRInstruction* inst) {}

void CodeGenerator::cgAssertType(IRInstruction* inst) {}

void CodeGenerator::cgLdUnwinderValue(IRInstruction* inst) {}

void CodeGenerator::cgBeginCatch(IRInstruction* inst) {}

void CodeGenerator::cgEndCatch(IRInstruction* inst) {}

void CodeGenerator::cgUnwindCheckSideExit(IRInstruction* inst) {}

//////////////////////////////////////////////////////////////////////

void CodeGenerator::cgHalt(IRInstruction* inst) {}

//////////////////////////////////////////////////////////////////////

void CodeGenerator::cgCallNative(Vout& v, IRInstruction* inst) {}

CallDest CodeGenerator::callDest(Vreg reg0) const {}

CallDest CodeGenerator::callDest(Vreg reg0, Vreg reg1) const {}

CallDest CodeGenerator::callDest(const IRInstruction* inst) const {}

CallDest CodeGenerator::callDestTV(const IRInstruction* inst) const {}

CallDest CodeGenerator::callDestDbl(const IRInstruction* inst) const {}

// We can't really compile using the compact call if the address of the array
// vtable is in high memory (there is only an encoding for 32bit displacement).
// This can happen, for example, if we have address space randomization
// enabled.  For now just punt these cases.
template<class Arg> CppCall
CodeGenerator::arrayCallIfLowMem(const IRInstruction* inst, Arg vtable) const {}

/*
 * Prepare the given ArgDest for a call by shifting or zero-extending as
 * appropriate, then append its Vreg to the given VregList.
 */
static void prepareArg(const ArgDesc& arg, Vout& v, VregList& vargs) {}

void
CodeGenerator::cgCallHelper(Vout& v, CppCall call, const CallDest& dstInfo,
                            SyncOptions sync, const ArgGroup& args) {}

void CodeGenerator::cgMov(IRInstruction* inst) {}

void CodeGenerator::cgAbsDbl(IRInstruction* inst) {}

Vreg CodeGenerator::emitAddInt(Vout& v, IRInstruction* inst) {}

Vreg CodeGenerator::emitSubInt(Vout& v, IRInstruction* inst) {}

Vreg CodeGenerator::emitMulInt(Vout& v, IRInstruction* inst) {}

void CodeGenerator::cgAddIntO(IRInstruction* inst) {}

void CodeGenerator::cgSubIntO(IRInstruction* inst) {}

void CodeGenerator::cgMulIntO(IRInstruction* inst) {}

void CodeGenerator::cgFloor(IRInstruction* inst) {}

void CodeGenerator::cgCeil(IRInstruction* inst) {}

void CodeGenerator::cgAddInt(IRInstruction* inst) {}

void CodeGenerator::cgSubInt(IRInstruction* inst) {}

void CodeGenerator::cgMulInt(IRInstruction* inst) {}

void CodeGenerator::cgAddDbl(IRInstruction* inst) {}

void CodeGenerator::cgSubDbl(IRInstruction* inst) {}

void CodeGenerator::cgMulDbl(IRInstruction* inst) {}

void CodeGenerator::cgDivDbl(IRInstruction* inst) {}

void CodeGenerator::cgAndInt(IRInstruction* inst) {}

void CodeGenerator::cgOrInt(IRInstruction* inst) {}

void CodeGenerator::cgXorInt(IRInstruction* inst) {}

void CodeGenerator::cgXorBool(IRInstruction* inst) {}

void CodeGenerator::cgMod(IRInstruction* inst) {}

void CodeGenerator::cgSqrt(IRInstruction* inst) {}

template<class Op, class Opi>
void CodeGenerator::cgShiftCommon(IRInstruction* inst) {}

void CodeGenerator::cgShl(IRInstruction* inst) {}

void CodeGenerator::cgShr(IRInstruction* inst) {}

///////////////////////////////////////////////////////////////////////////////
// Comparison Operators
///////////////////////////////////////////////////////////////////////////////

#define DISPATCHER(name)\
  int64_t ccmp_ ## name (StringData* a1, StringData* a2)\
  { return name(a1, a2); }\
  int64_t ccmp_ ## name (StringData* a1, int64_t a2)\
  { return name(a1, a2); }\
  int64_t ccmp_ ## name (StringData* a1, ObjectData* a2)\
  { return name(a1, Object(a2)); }\
  int64_t ccmp_ ## name (ObjectData* a1, ObjectData* a2)\
  { return name(Object(a1), Object(a2)); }\
  int64_t ccmp_ ## name (ObjectData* a1, int64_t a2)\
  { return name(Object(a1), a2); }\
  int64_t ccmp_ ## name (ArrayData* a1, ArrayData* a2)\
  { return name(Array(a1), Array(a2)); }

DISPATCHER(same)
DISPATCHER(equal)
DISPATCHER(more)
DISPATCHER(less)

#undef DISPATCHER

template <typename A, typename B>
inline int64_t ccmp_nsame(A a, B b) { return !ccmp_same(a, b); }

template <typename A, typename B>
inline int64_t ccmp_nequal(A a, B b) { return !ccmp_equal(a, b); }

// TODO Task #2661083: We cannot assume that "(a <= b) === !(a > b)" for
// all types. In particular, this assumption does not hold when comparing
// two arrays or comparing two objects. We should fix this.
template <typename A, typename B>
inline int64_t ccmp_lte(A a, B b) { return !ccmp_more(a, b); }

template <typename A, typename B>
inline int64_t ccmp_gte(A a, B b) { return !ccmp_less(a, b); }

#define CG_OP_CMP(inst, cc, name)                                   \
  cgCmpHelper(inst, cc, ccmp_ ## name, ccmp_ ## name,               \
              ccmp_ ## name, ccmp_ ## name, ccmp_ ## name, ccmp_ ## name)

// SON - string, object, or number
static bool typeIsSON(Type t) {
  return t.subtypeOfAny(TStr, TObj, TInt, TDbl);
}

void CodeGenerator::cgCmpHelper(IRInstruction* inst, ConditionCode cc,
          int64_t (*str_cmp_str)(StringData*, StringData*),
          int64_t (*str_cmp_int)(StringData*, int64_t),
          int64_t (*str_cmp_obj)(StringData*, ObjectData*),
          int64_t (*obj_cmp_obj)(ObjectData*, ObjectData*),
          int64_t (*obj_cmp_int)(ObjectData*, int64_t),
          int64_t (*arr_cmp_arr)(ArrayData*,  ArrayData*)
        ) {}

void CodeGenerator::cgEq(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_E, equal);
}

void CodeGenerator::cgEqX(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_E, equal);
}

void CodeGenerator::cgNeq(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_NE, nequal);
}

void CodeGenerator::cgNeqX(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_NE, nequal);
}

void CodeGenerator::cgSame(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_E, same);
}

void CodeGenerator::cgNSame(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_NE, nsame);
}

void CodeGenerator::cgLt(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_L, less);
}

void CodeGenerator::cgLtX(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_L, less);
}

void CodeGenerator::cgGt(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_G, more);
}

void CodeGenerator::cgGtX(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_G, more);
}

void CodeGenerator::cgLte(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_LE, lte);
}

void CodeGenerator::cgLteX(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_LE, lte);
}

void CodeGenerator::cgGte(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_GE, gte);
}

void CodeGenerator::cgGteX(IRInstruction* inst) {
  CG_OP_CMP(inst, CC_GE, gte);
}

void CodeGenerator::emitCmpInt(IRInstruction* inst, ConditionCode cc) {}

void CodeGenerator::cgEqInt(IRInstruction* inst)  {}
void CodeGenerator::cgNeqInt(IRInstruction* inst) {}
void CodeGenerator::cgLtInt(IRInstruction* inst)  {}
void CodeGenerator::cgGtInt(IRInstruction* inst)  {}
void CodeGenerator::cgLteInt(IRInstruction* inst) {}
void CodeGenerator::cgGteInt(IRInstruction* inst) {}

void CodeGenerator::emitCmpEqDbl(IRInstruction* inst, ComparisonPred pred) {}

void CodeGenerator::emitCmpRelDbl(IRInstruction* inst, ConditionCode cc,
                                  bool flipOperands) {}

void CodeGenerator::cgEqDbl(IRInstruction* inst)  {}

void CodeGenerator::cgNeqDbl(IRInstruction* inst) {}

void CodeGenerator::cgLtDbl(IRInstruction* inst)  {}

void CodeGenerator::cgGtDbl(IRInstruction* inst)  {}

void CodeGenerator::cgLteDbl(IRInstruction* inst) {}

void CodeGenerator::cgGteDbl(IRInstruction* inst) {}

///////////////////////////////////////////////////////////////////////////////
// Type check operators
///////////////////////////////////////////////////////////////////////////////

// Overloads to put the {Object,Array}Data* into a register so
// emitTypeTest can cmp to the Class*/ArrayKind expected by the
// specialized Type

// Nothing to do, return the register that contain the ObjectData already
Vreg getDataPtrEnregistered(Vout&, Vreg dataSrc) {
  return dataSrc;
}

// Enregister the memoryRef so it can be used with an offset by the
// cmp instruction
Vreg getDataPtrEnregistered(Vout& v, Vptr dataSrc) {
  auto t = v.makeReg();
  v << load{dataSrc, t};
  return t;
}

template<class Loc1, class Loc2, class JmpFn>
void CodeGenerator::emitTypeTest(Type type, Loc1 typeSrc, Loc2 dataSrc,
                                 Vreg sf, JmpFn doJcc) {}

template<class DataLoc, class JmpFn>
void CodeGenerator::emitSpecializedTypeTest(Type type, DataLoc dataSrc, Vreg sf,
                                            JmpFn doJcc) {}

template<class JmpFn>
void CodeGenerator::emitIsTypeTest(IRInstruction* inst, Vreg sf, JmpFn doJcc) {}

template<class Loc>
void CodeGenerator::emitTypeCheck(Type type, Loc typeSrc, Loc dataSrc,
                                  Block* taken) {}

void CodeGenerator::emitSetCc(IRInstruction* inst, ConditionCode cc, Vreg sf) {}

void CodeGenerator::cgIsTypeMemCommon(IRInstruction* inst, bool negate) {}

void CodeGenerator::cgIsTypeCommon(IRInstruction* inst, bool negate) {}

void CodeGenerator::cgIsType(IRInstruction* inst) {}

void CodeGenerator::cgIsScalarType(IRInstruction* inst) {}

void CodeGenerator::cgIsNType(IRInstruction* inst) {}

void CodeGenerator::cgIsTypeMem(IRInstruction* inst) {}

void CodeGenerator::cgIsNTypeMem(IRInstruction* inst) {}

///////////////////////////////////////////////////////////////////////////////

/*
 * Check instanceof using instance bitmasks.
 *
 * Note it's not necessary to check whether the test class is defined:
 * if it doesn't exist than the candidate can't be an instance of it
 * and will fail this check.
 */
Vreg CodeGenerator::emitInstanceBitmaskCheck(Vout& v, IRInstruction* inst) {}

void CodeGenerator::cgInstanceOfBitmask(IRInstruction* inst) {}

void CodeGenerator::cgNInstanceOfBitmask(IRInstruction* inst) {}

void CodeGenerator::cgInstanceOf(IRInstruction* inst) {}

/*
 * Check instanceof using the superclass vector on the end of the
 * Class entry.
 */
void CodeGenerator::cgExtendsClass(IRInstruction* inst) {}

void CodeGenerator::cgClsNeq(IRInstruction* inst) {}

void CodeGenerator::cgConvDblToInt(IRInstruction* inst) {}

void CodeGenerator::cgConvDblToBool(IRInstruction* inst) {}

void CodeGenerator::cgConvIntToBool(IRInstruction* inst) {}

void CodeGenerator::cgConvArrToBool(IRInstruction* inst) {}

void CodeGenerator::cgColIsEmpty(IRInstruction* inst) {}

void CodeGenerator::cgColIsNEmpty(IRInstruction* inst) {}

void CodeGenerator::cgConvObjToBool(IRInstruction* inst) {}

void CodeGenerator::emitConvBoolOrIntToDbl(IRInstruction* inst) {}

void CodeGenerator::cgConvBoolToDbl(IRInstruction* inst) {}

void CodeGenerator::cgConvIntToDbl(IRInstruction* inst) {}

void CodeGenerator::cgConvBoolToInt(IRInstruction* inst) {}

void CodeGenerator::cgOrdStr(IRInstruction* inst) {}

void CodeGenerator::cgConvBoolToStr(IRInstruction* inst) {}

void CodeGenerator::cgConvClsToCctx(IRInstruction* inst) {}

void CodeGenerator::cgUnboxPtr(IRInstruction* inst) {}

Vreg CodeGenerator::cgLdFuncCachedCommon(IRInstruction* inst, Vreg dst) {}

void CodeGenerator::cgLdFuncCached(IRInstruction* inst) {}

void CodeGenerator::cgLdFuncCachedSafe(IRInstruction* inst) {}

void CodeGenerator::cgLdFuncCachedU(IRInstruction* inst) {}

void CodeGenerator::cgLdFunc(IRInstruction* inst) {}

void CodeGenerator::cgLdObjClass(IRInstruction* inst) {}

void CodeGenerator::cgLdArrFuncCtx(IRInstruction* inst) {}

void CodeGenerator::cgLdArrFPushCuf(IRInstruction* inst) {}

void CodeGenerator::cgLdStrFPushCuf(IRInstruction* inst) {}

void CodeGenerator::cgLookupClsMethod(IRInstruction* inst) {}

void CodeGenerator::cgLdObjMethod(IRInstruction* inst) {}

void CodeGenerator::cgLdObjInvoke(IRInstruction* inst) {}

void CodeGenerator::cgStRetVal(IRInstruction* inst) {}

void traceRet(ActRec* fp, Cell* sp, void* rip) {}

void CodeGenerator::cgRetCtrl(IRInstruction* inst) {}

void CodeGenerator::cgAsyncRetCtrl(IRInstruction* inst) {}

void CodeGenerator::cgLdBindAddr(IRInstruction* inst) {}

void CodeGenerator::cgProfileSwitchDest(IRInstruction* inst) {}

void CodeGenerator::cgJmpSwitchDest(IRInstruction* inst) {}

void CodeGenerator::cgLdSSwitchDestFast(IRInstruction* inst) {}

static TCA sswitchHelperSlow(TypedValue typedVal,
                             const StringData** strs,
                             int numStrs,
                             TCA* jmptab) {}

void CodeGenerator::cgLdSSwitchDestSlow(IRInstruction* inst) {}

/*
 * It'd be nice not to have the cgMov here (and just copy propagate
 * the source or something), but for now we're keeping it allocated to
 * rVmFp so inlined calls to C++ helpers that use the rbp chain to
 * find the caller's ActRec will work correctly.
 *
 * This instruction primarily exists to assist in optimizing away
 * unused activation records, so it's usually not going to happen
 * anyway.
 */
void CodeGenerator::cgDefInlineFP(IRInstruction* inst) {}

void CodeGenerator::cgInlineReturn(IRInstruction* inst) {}

void CodeGenerator::cgFreeActRec(IRInstruction* inst) {}

void CodeGenerator::cgStMem(IRInstruction* inst) {}

void CodeGenerator::cgStRef(IRInstruction* inst) {}

int CodeGenerator::iterOffset(const BCMarker& marker, uint32_t id) {}

void CodeGenerator::cgStLoc(IRInstruction* inst) {}

void CodeGenerator::cgStLocRange(IRInstruction* inst) {}

void CodeGenerator::cgEagerSyncVMRegs(IRInstruction* inst) {}

void CodeGenerator::cgReqBindJmp(IRInstruction* inst) {}

void CodeGenerator::cgReqRetranslateOpt(IRInstruction* inst) {}

void CodeGenerator::cgReqRetranslate(IRInstruction* inst) {}

void CodeGenerator::cgIncRef(IRInstruction* inst) {}

void CodeGenerator::cgIncRefCtx(IRInstruction* inst) {}

void CodeGenerator::cgGenericRetDecRefs(IRInstruction* inst) {}

/*
 * Depending on the current translation kind, do nothing, profile, or collect
 * profiling data for the current DecRef* instruction
 *
 * Returns true iff the release path for this DecRef should be put in cold
 * code.
 */
float CodeGenerator::decRefDestroyRate(const IRInstruction* inst,
                                       OptDecRefProfile& profile,
                                       Type type) {}

/*
 * We've tried a variety of tweaks to this and found the current state of
 * things optimal, at least when measurements of the following factors were
 * made:
 *
 * - whether to load the count into a register
 *
 * - whether to use if (!--count) release(); if we don't need a static check
 *
 * - whether to skip using the register and just emit --count if we know
 *   its not static, and can't hit zero.
 *
 * The current scheme generates if (!--count) release() for types that cannot
 * possibly be static.  For types that might be static, it generates a compare
 * of the m_count field against 1, followed by two conditional branches on the
 * same flags.  We make use of the invariant that count fields are never zero,
 * and use a code sequence that looks like this:
 *
 *    cmpl $1, $FAST_REFCOUNT_OFFSET(%base)
 *    je do_release  // call the destructor, usually in acold
 *    jl skip_dec    // count < 1 implies it's static
 *    decl $FAST_REFCOUNT_OFFSET(%base)
 *  skip_dec:
 *    // ....
 */
void CodeGenerator::decRefImpl(Vout& v, const IRInstruction* inst,
                               const OptDecRefProfile& profile,
                               bool unlikelyDestroy) {}

void CodeGenerator::emitDecRefTypeStat(Vout& v, const IRInstruction* inst) {}

void CodeGenerator::cgDecRef(IRInstruction *inst) {}

void CodeGenerator::cgDecRefNZ(IRInstruction* inst) {}

void CodeGenerator::cgCufIterSpillFrame(IRInstruction* inst) {}

void CodeGenerator::cgSpillFrame(IRInstruction* inst) {}

void CodeGenerator::cgStClosureFunc(IRInstruction* inst) {}

void CodeGenerator::cgStClosureArg(IRInstruction* inst) {}

void CodeGenerator::cgStClosureCtx(IRInstruction* inst) {}

void CodeGenerator::emitInitObjProps(const IRInstruction* inst, Vreg dstReg,
                                     const Class* cls, size_t nProps) {}

void CodeGenerator::cgConstructInstance(IRInstruction* inst) {}

void CodeGenerator::cgCheckInitProps(IRInstruction* inst) {}

void CodeGenerator::cgCheckInitSProps(IRInstruction* inst) {}

void CodeGenerator::cgNewInstanceRaw(IRInstruction* inst) {}

void CodeGenerator::cgInitObjProps(IRInstruction* inst) {}

void CodeGenerator::cgCallArray(IRInstruction* inst) {}

void CodeGenerator::cgCall(IRInstruction* inst) {}

void CodeGenerator::cgCastStk(IRInstruction *inst) {}

void CodeGenerator::cgCoerceStk(IRInstruction *inst) {}

void CodeGenerator::cgCallBuiltin(IRInstruction* inst) {}

void CodeGenerator::cgStStk(IRInstruction* inst) {}

// Fill the entire 16-byte space for a TypedValue with trash.  Note: it will
// clobber the Aux area of a TypedValueAux.
void CodeGenerator::emitTrashTV(Vreg ptr, int32_t offset, char fillByte) {}

void CodeGenerator::cgDbgTrashStk(IRInstruction* inst) {}

void CodeGenerator::cgDbgTrashFrame(IRInstruction* inst) {}

void CodeGenerator::cgDbgTrashMem(IRInstruction* inst) {}

void CodeGenerator::cgNativeImpl(IRInstruction* inst) {}

void CodeGenerator::cgCastCtxThis(IRInstruction* inst) {}

void CodeGenerator::cgCheckCtxThis(IRInstruction* inst) {}

void CodeGenerator::cgLdClsCtx(IRInstruction* inst) {}

void CodeGenerator::cgLdClsCctx(IRInstruction* inst) {}

void CodeGenerator::cgLdCtx(IRInstruction* inst) {}

void CodeGenerator::cgLdCctx(IRInstruction* inst) {}

void CodeGenerator::cgLdClsName(IRInstruction* inst) {}

void CodeGenerator::cgLdARFuncPtr(IRInstruction* inst) {}

void CodeGenerator::cgLdARNumParams(IRInstruction* inst) {}

void CodeGenerator::cgLdStaticLocCached(IRInstruction* inst) {}

void CodeGenerator::cgCheckStaticLocInit(IRInstruction* inst) {}

void CodeGenerator::cgStaticLocInitCached(IRInstruction* inst) {}

void CodeGenerator::emitLoad(SSATmp* dst, Vloc dstLoc, Vptr base) {}

void CodeGenerator::emitLoadTypedValue(SSATmp* dst, Vloc dstLoc, Vptr ref) {}

void CodeGenerator::cgLdContField(IRInstruction* inst) {}

void CodeGenerator::cgLdMem(IRInstruction* inst) {}

void CodeGenerator::cgLdRef(IRInstruction* inst) {}

void CodeGenerator::cgCheckRefInner(IRInstruction* inst) {}

void CodeGenerator::cgStringIsset(IRInstruction* inst) {}

void CodeGenerator::cgProfilePackedArray(IRInstruction* inst) {}

void CodeGenerator::cgProfileStructArray(IRInstruction* inst) {}

void CodeGenerator::cgCheckPackedArrayBounds(IRInstruction* inst) {}

void CodeGenerator::cgLdPackedArrayElemAddr(IRInstruction* inst) {}

void CodeGenerator::cgCheckRange(IRInstruction* inst) {}

void CodeGenerator::cgLdVectorSize(IRInstruction* inst) {}

void CodeGenerator::cgLdVectorBase(IRInstruction* inst) {}

void CodeGenerator::cgLdColArray(IRInstruction* inst) {}

void CodeGenerator::cgVectorHasImmCopy(IRInstruction* inst) {}

/**
 * Given the base of a vector object, pass it to a helper
 * which is responsible for triggering COW.
 */
void CodeGenerator::cgVectorDoCow(IRInstruction* inst) {}

void CodeGenerator::cgLdPairBase(IRInstruction* inst) {}

void CodeGenerator::cgLdElem(IRInstruction* inst) {}

void CodeGenerator::cgStElem(IRInstruction* inst) {}

Fixup CodeGenerator::makeFixup(const BCMarker& marker, SyncOptions sync) {}

void CodeGenerator::cgLdMIStateAddr(IRInstruction* inst) {}

void CodeGenerator::cgLdLoc(IRInstruction* inst) {}

void CodeGenerator::cgLdLocAddr(IRInstruction* inst) {}

void CodeGenerator::cgLdLocPseudoMain(IRInstruction* inst) {}

void CodeGenerator::cgStLocPseudoMain(IRInstruction* inst) {}

void CodeGenerator::cgLdStkAddr(IRInstruction* inst) {}

void CodeGenerator::cgLdStk(IRInstruction* inst) {}

void CodeGenerator::cgCheckStk(IRInstruction* inst) {}

void CodeGenerator::cgCheckLoc(IRInstruction* inst) {}

void CodeGenerator::cgDefMIStateBase(IRInstruction* inst) {}

void CodeGenerator::cgCheckType(IRInstruction* inst) {}

void CodeGenerator::cgCheckTypeMem(IRInstruction* inst) {}

template <class JmpFn>
void CodeGenerator::emitReffinessTest(IRInstruction* inst, Vreg sf,
                                      JmpFn doJcc) {}

void CodeGenerator::cgCheckRefs(IRInstruction* inst)  {}

void CodeGenerator::cgLdPropAddr(IRInstruction* inst) {}

void CodeGenerator::cgLdClsMethod(IRInstruction* inst) {}

void CodeGenerator::cgLookupClsMethodCache(IRInstruction* inst) {}

void CodeGenerator::cgLdClsMethodCacheCommon(IRInstruction* inst, Offset off) {}

void CodeGenerator::cgLdClsMethodCacheFunc(IRInstruction* inst) {}

void CodeGenerator::cgLdClsMethodCacheCls(IRInstruction* inst) {}

/**
 * Helper to emit getting the value for ActRec's m_this/m_cls slot
 * from a This pointer depending on whether the callee method is
 * static or not.
 */
void CodeGenerator::emitGetCtxFwdCallWithThis(Vreg srcCtx, Vreg dstCtx,
                                              bool staticCallee) {}

void CodeGenerator::cgGetCtxFwdCall(IRInstruction* inst) {}

void CodeGenerator::cgLdClsMethodFCacheFunc(IRInstruction* inst) {}

void CodeGenerator::cgLookupClsMethodFCache(IRInstruction* inst) {}

Vreg CodeGenerator::emitGetCtxFwdCallWithThisDyn(Vreg destCtxReg, Vreg thisReg,
                                                 rds::Handle ch) {}

/**
 * This method is similar to emitGetCtxFwdCall above, but whether or not the
 * callee is a static method is unknown at JIT time, and that is determined
 * dynamically by looking up into the StaticMethodFCache.
 */
void CodeGenerator::cgGetCtxFwdCallDyn(IRInstruction* inst) {}

rds::Handle CodeGenerator::cgLdClsCachedCommon(Vout& v, IRInstruction* inst,
                                               Vreg dst, Vreg sf) {}

void CodeGenerator::cgLdClsCached(IRInstruction* inst) {}

void CodeGenerator::cgLdClsCachedSafe(IRInstruction* inst) {}

void CodeGenerator::cgDerefClsRDSHandle(IRInstruction* inst) {}

void CodeGenerator::cgLdCls(IRInstruction* inst) {}

void CodeGenerator::cgLdRDSAddr(IRInstruction* inst) {}

void CodeGenerator::cgLookupClsCns(IRInstruction* inst) {}

void CodeGenerator::cgLdCns(IRInstruction* inst) {}

void CodeGenerator::cgLookupCnsCommon(IRInstruction* inst) {}

void CodeGenerator::cgLookupCns(IRInstruction* inst) {}

void CodeGenerator::cgLookupCnsE(IRInstruction* inst) {}

void CodeGenerator::cgLookupCnsU(IRInstruction* inst) {}

void CodeGenerator::cgAKExistsArr(IRInstruction* inst) {}

void CodeGenerator::cgAKExistsObj(IRInstruction* inst) {}

void CodeGenerator::cgLdGblAddr(IRInstruction* inst) {}

Vreg CodeGenerator::emitTestZero(Vout& v, SSATmp* src, Vloc srcLoc) {}

void CodeGenerator::cgJmpZero(IRInstruction* inst) {}

void CodeGenerator::cgJmpNZero(IRInstruction* inst) {}

void CodeGenerator::cgJmp(IRInstruction* inst) {}

void CodeGenerator::cgDefLabel(IRInstruction* inst) {}

void CodeGenerator::cgJmpSSwitchDest(IRInstruction* inst) {}

void CodeGenerator::cgNewCol(IRInstruction* inst) {}

void CodeGenerator::cgNewColFromArray(IRInstruction* inst) {}

void CodeGenerator::cgCheckInit(IRInstruction* inst) {}

void CodeGenerator::cgCheckInitMem(IRInstruction* inst) {}

void CodeGenerator::cgCheckSurpriseFlags(IRInstruction* inst) {}

void CodeGenerator::cgCheckCold(IRInstruction* inst) {}

static const StringData* s_ReleaseVV = makeStaticString("ReleaseVV");

void CodeGenerator::cgReleaseVVAndSkip(IRInstruction* inst) {}

void CodeGenerator::cgBoxPtr(IRInstruction* inst) {}

void CodeGenerator::cgInterpOne(IRInstruction* inst) {}

void CodeGenerator::cgInterpOneCF(IRInstruction* inst) {}

void CodeGenerator::cgContEnter(IRInstruction* inst) {}

void CodeGenerator::cgContPreNext(IRInstruction* inst) {}

void CodeGenerator::cgContStartedCheck(IRInstruction* inst) {}

void CodeGenerator::cgContValid(IRInstruction* inst) {}

void CodeGenerator::cgContArIncKey(IRInstruction* inst) {}

void CodeGenerator::cgContArUpdateIdx(IRInstruction* inst) {}

void CodeGenerator::cgLdContActRec(IRInstruction* inst) {}

void CodeGenerator::cgLdContArValue(IRInstruction* inst) {}

void CodeGenerator::cgStContArValue(IRInstruction* inst) {}

void CodeGenerator::cgLdContArKey(IRInstruction* inst) {}

void CodeGenerator::cgStContArKey(IRInstruction* inst) {}

void CodeGenerator::cgStAsyncArSucceeded(IRInstruction* inst) {}

void CodeGenerator::resumableStResumeImpl(IRInstruction* inst,
                                          ptrdiff_t offAddr,
                                          ptrdiff_t offOffset) {}

void CodeGenerator::cgStAsyncArResume(IRInstruction* inst) {}

void CodeGenerator::cgStContArResume(IRInstruction* inst) {}

void CodeGenerator::cgLdContResumeAddr(IRInstruction* inst) {}

void CodeGenerator::cgContArIncIdx(IRInstruction* inst) {}

void CodeGenerator::cgStContArState(IRInstruction* inst) {}

void CodeGenerator::cgStAsyncArResult(IRInstruction* inst) {}

void CodeGenerator::cgLdAsyncArParentChain(IRInstruction* inst) {}

void CodeGenerator::cgAFWHBlockOn(IRInstruction* inst) {}

void CodeGenerator::cgIsWaitHandle(IRInstruction* inst) {}

void CodeGenerator::cgLdWHState(IRInstruction* inst) {}

void CodeGenerator::cgLdWHResult(IRInstruction* inst) {}

void CodeGenerator::cgLdAFWHActRec(IRInstruction* inst) {}

void CodeGenerator::cgLdResumableArObj(IRInstruction* inst) {}

void CodeGenerator::cgIterInit(IRInstruction* inst) {}

void CodeGenerator::cgIterInitK(IRInstruction* inst) {}

void CodeGenerator::cgWIterInit(IRInstruction* inst) {}

void CodeGenerator::cgWIterInitK(IRInstruction* inst) {}

void CodeGenerator::cgIterInitCommon(IRInstruction* inst) {}

void CodeGenerator::cgMIterInit(IRInstruction* inst) {}

void CodeGenerator::cgMIterInitK(IRInstruction* inst) {}

void CodeGenerator::cgMIterInitCommon(IRInstruction* inst) {}

void CodeGenerator::cgIterNext(IRInstruction* inst) {}

void CodeGenerator::cgIterNextK(IRInstruction* inst) {}

void CodeGenerator::cgWIterNext(IRInstruction* inst) {}

void CodeGenerator::cgWIterNextK(IRInstruction* inst) {}

void CodeGenerator::cgIterNextCommon(IRInstruction* inst) {}

void CodeGenerator::cgMIterNext(IRInstruction* inst) {}

void CodeGenerator::cgMIterNextK(IRInstruction* inst) {}

void CodeGenerator::cgMIterNextCommon(IRInstruction* inst) {}

void CodeGenerator::cgIterFree(IRInstruction* inst) {}

void CodeGenerator::cgMIterFree(IRInstruction* inst) {}

void CodeGenerator::cgDecodeCufIter(IRInstruction* inst) {}

void CodeGenerator::cgCIterFree(IRInstruction* inst) {}

void CodeGenerator::cgNewStructArray(IRInstruction* inst) {}

void CodeGenerator::cgIncStat(IRInstruction *inst) {}

void CodeGenerator::cgIncTransCounter(IRInstruction* inst) {}

void CodeGenerator::cgIncProfCounter(IRInstruction* inst) {}

void CodeGenerator::cgDbgTraceCall(IRInstruction* inst) {}

void CodeGenerator::cgDbgAssertRefCount(IRInstruction* inst) {}

void CodeGenerator::cgDbgAssertType(IRInstruction* inst) {}

void CodeGenerator::emitVerifyCls(IRInstruction* inst) {}

void CodeGenerator::cgVerifyParamCls(IRInstruction* inst) {}

void CodeGenerator::cgVerifyRetCls(IRInstruction* inst) {}

void CodeGenerator::cgRBTraceEntry(IRInstruction* inst) {}

void CodeGenerator::cgRBTraceMsg(IRInstruction* inst) {}

void CodeGenerator::cgCountBytecode(IRInstruction* inst) {}

void CodeGenerator::cgLdClsInitData(IRInstruction* inst) {}

void CodeGenerator::cgConjure(IRInstruction* inst) {}

void CodeGenerator::cgCountArray(IRInstruction* inst) {}

void CodeGenerator::cgCountArrayFast(IRInstruction* inst) {}

void CodeGenerator::cgCountCollection(IRInstruction* inst) {}

void CodeGenerator::cgLdStrLen(IRInstruction* inst) {}

void CodeGenerator::cgLdFuncNumParams(IRInstruction* inst) {}

void CodeGenerator::cgInitPackedArray(IRInstruction* inst) {}

void CodeGenerator::cgInitPackedArrayLoop(IRInstruction* inst) {}

void CodeGenerator::cgLdStructArrayElem(IRInstruction* inst) {}

void CodeGenerator::cgEnterFrame(IRInstruction* inst) {}

void CodeGenerator::cgCheckStackOverflow(IRInstruction* inst) {}

void CodeGenerator::cgInitExtraArgs(IRInstruction* inst) {}

void CodeGenerator::cgCheckSurpriseFlagsEnter(IRInstruction* inst) {}

void CodeGenerator::print() const {}

void CodeGenerator::cgProfileObjClass(IRInstruction* inst) {}

}}}
