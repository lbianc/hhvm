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

#include "hphp/runtime/vm/jit/func-prologues-ppc64.h"

#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/util/ringbuffer.h"

#include "hphp/runtime/ext/ext_closure.h"
#include "hphp/runtime/vm/func.h"
#include "hphp/runtime/vm/srckey.h"
#include "hphp/runtime/vm/jit/code-gen-helpers-ppc64.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/mc-generator-internal.h"
#include "hphp/runtime/vm/jit/prof-data.h"
#include "hphp/runtime/vm/jit/translator-runtime.h"
#include "hphp/runtime/vm/jit/write-lease.h"
#include "hphp/runtime/vm/jit/relocation.h"

namespace HPHP { namespace jit { namespace ppc64 {

//////////////////////////////////////////////////////////////////////

TRACE_SET_MOD(mcg);

//////////////////////////////////////////////////////////////////////

namespace {

// Generate an if-then block into a.  thenBlock is executed if cc is true.
template <class Then>
void ifThen(jit::X64Assembler& a, ConditionCode cc, Then thenBlock) {
  not_implemented();
}

void emitStackCheck(X64Assembler& a, int funcDepth, Offset pc) {
  not_implemented();
}

/*
 * This will omit overflow checks if it is a leaf function that can't
 * use more than kStackCheckLeafPadding cells.
 */
void maybeEmitStackCheck(X64Assembler& a, const Func* func) {
  not_implemented();
}

#pragma GCC diagnostic ignored "-Wreturn-type"
TCA emitFuncGuard(X64Assembler& a, const Func* func) { not_implemented(); }

// Initialize at most this many locals inline in function body prologue; more
// than this, and emitting a loop is more compact. To be precise, the actual
// crossover point in terms of code size is 6; 9 was determined by experiment to
// be the optimal point in certain benchmarks. #microoptimization
constexpr auto kLocalsToInitializeInline = 9;

// Maximum number of default-value parameter initializations to
// unroll. Beyond this, a loop is generated.
constexpr auto kMaxParamsInitUnroll = 5;

SrcKey emitPrologueWork(TransID transID, Func* func, int nPassed) {
  not_implemented();
}

} // anonymous namespace

//////////////////////////////////////////////////////////////////////

TCA emitCallArrayPrologue(Func* func, DVFuncletsVec& dvs) {
  not_implemented();
}

SrcKey emitFuncPrologue(TransID transID, Func* func, int nPassed, TCA& start) {
  not_implemented();
}

SrcKey emitMagicFuncPrologue(TransID transID, Func* func, int nPassed,
                             TCA& start) { not_implemented(); }
#pragma GCC diagnostic pop

}}}
