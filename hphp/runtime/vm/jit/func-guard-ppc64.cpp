/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | Copyright (c) 2010-2015 Facebook, Inc. (http://www.facebook.com)     |
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

#include "hphp/runtime/vm/jit/func-guard-ppc64.h"

#include "hphp/runtime/vm/jit/abi.h"
#include "hphp/runtime/vm/jit/abi-ppc64.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/smashable-instr-ppc64.h"
#include "hphp/runtime/vm/jit/translator.h"
#include "hphp/runtime/vm/jit/unique-stubs.h"

#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/util/data-block.h"
#include "hphp/util/immed.h"

namespace HPHP { namespace jit { namespace ppc64 {

///////////////////////////////////////////////////////////////////////////////

namespace {

///////////////////////////////////////////////////////////////////////////////

constexpr auto kFuncGuardLen = 20;
constexpr auto kFuncGuardShortLen = 14;

ALWAYS_INLINE bool isSmall(const Func* func) {
  return deltaFits(reinterpret_cast<uintptr_t>(func), sz::dword);
}

ALWAYS_INLINE bool isPrologueStub(TCA addr) {
  return addr == mcg->tx().uniqueStubs.fcallHelperThunk;
}

///////////////////////////////////////////////////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////

void emitFuncGuard(const Func* func, CodeBlock& cb) {
/*  using namespace reg;
  ppc64_asm::Assembler a { cb };

//  assertx(x64::abi(CodeKind::CrossTrace).gpUnreserved.contains(rax));

  auto const funcImm = Immed64(func);

  if (funcImm.fits(sz::dword)) {
    emitSmashableCmpq(a.code(), funcImm.l(), rvmfp(),
                      safe_cast<int8_t>(AROFF(m_func)));
  } else {
    // Although func doesn't fit in a signed 32-bit immediate, it may still fit
    // in an unsigned one.  Rather than deal with yet another case (which only
    // happens when we disable jemalloc), just emit a smashable mov followed by
    // a register cmp.
    emitSmashableMovq(a.code(), uint64_t(func), rax);
    a.  cmpq   (rax, rvmfp()[AROFF(m_func)]);
  }
  a.    jnz    (mcg->tx().uniqueStubs.funcPrologueRedispatch);

  DEBUG_ONLY auto guard = funcGuardFromPrologue(a.frontier(), func);
  assertx(funcGuardMatches(guard, func));*/
  not_implemented();
}

TCA funcGuardFromPrologue(TCA prologue, const Func* func) {
  if (isPrologueStub(prologue)) return prologue;
  return prologue - (isSmall(func) ? kFuncGuardShortLen : kFuncGuardLen);
}

bool funcGuardMatches(TCA guard, const Func* func) {
  if (isPrologueStub(guard)) return false;

  auto const ifunc = reinterpret_cast<uintptr_t>(func);
  return isSmall(func)
    ? smashableCmpqImm(guard) == ifunc
    : smashableMovqImm(guard) == ifunc;
}

void clobberFuncGuard(TCA guard, const Func* func) {
  return isSmall(func) ? smashCmpq(guard, 0)
                       : smashMovq(guard, 0);
}

///////////////////////////////////////////////////////////////////////////////

}}}
