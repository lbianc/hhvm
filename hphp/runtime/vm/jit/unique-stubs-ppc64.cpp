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

#include "hphp/runtime/vm/jit/unique-stubs-ppc64.h"

#include "hphp/runtime/base/header-kind.h"
#include "hphp/runtime/base/rds-header.h"
#include "hphp/runtime/base/runtime-option.h"
#include "hphp/runtime/base/stats.h"
#include "hphp/runtime/vm/event-hook.h"

#include "hphp/runtime/vm/jit/types.h"
#include "hphp/runtime/vm/jit/abi-ppc64.h"
#include "hphp/runtime/vm/jit/align-ppc64.h"
#include "hphp/runtime/vm/jit/code-gen-cf.h"
#include "hphp/runtime/vm/jit/code-gen-helpers.h"
#include "hphp/runtime/vm/jit/code-gen-tls.h"
#include "hphp/runtime/vm/jit/fixup.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/phys-reg.h"
#include "hphp/runtime/vm/jit/service-requests.h"
#include "hphp/runtime/vm/jit/translator-inline.h"
#include "hphp/runtime/vm/jit/unique-stubs.h"
#include "hphp/runtime/vm/jit/unwind-ppc64.h"
#include "hphp/runtime/vm/jit/vasm-gen.h"
#include "hphp/runtime/vm/jit/vasm-instr.h"

#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/util/data-block.h"

namespace HPHP { namespace jit { namespace ppc64 {

///////////////////////////////////////////////////////////////////////////////

TRACE_SET_MOD(ustubs);

///////////////////////////////////////////////////////////////////////////////

static void alignJmpTarget(CodeBlock& cb) {
  align(cb, Alignment::JmpTarget, AlignContext::Dead);
}

///////////////////////////////////////////////////////////////////////////////

TCA emitFunctionEnterHelper(CodeBlock& cb, UniqueStubs& us) {
  alignJmpTarget(cb);

  auto const start = vwrap(cb, [&] (Vout& v) {
  });

  return start;
}

///////////////////////////////////////////////////////////////////////////////

/*
 * Helper for the freeLocalsHelpers which does the actual work of decrementing
 * a value's refcount or releasing it.
 *
 * This helper is reached via call from the various freeLocalHelpers.  It
 * expects `tv' to be the address of a TypedValue with refcounted type `type'
 * (though it may be static, and we will do nothing in that case).
 *
 * The `saved' register should be a callee-saved GP register that the helper
 * can use to preserve `tv' across native calls.
 */
static TCA emitDecRefHelper(CodeBlock& cb, PhysReg tv, PhysReg type,
                            RegSet live) {
  return vwrap(cb, [&] (Vout& v) {
  });
}

TCA emitFreeLocalsHelpers(CodeBlock& cb, UniqueStubs& us) {
  // The address of the first local is passed in the second argument register.
  // We use the third and fourth as scratch registers.
  auto const local = rarg(1);
  auto const last = rarg(2);
  auto const type = rarg(3);

  // This stub is very hot; keep it cache-aligned.
  align(cb, Alignment::CacheLine, AlignContext::Dead);
  auto const release = emitDecRefHelper(cb, local, type, local | last);

  us.freeManyLocalsHelper = vwrap(cb, [&] (Vout& v) {
  });

  return release;
}

///////////////////////////////////////////////////////////////////////////////

extern "C" void enterTCExit();

TCA emitCallToExit(CodeBlock& cb) {
  ppc64_asm::Assembler a { cb };
  auto const start = a.frontier();

  // Simply go to enterTCExit, no worries about the stack because it's balanced
  a.branchAuto(TCA(enterTCExit));
  return start;
}

TCA emitEndCatchHelper(CodeBlock& cb, UniqueStubs& us) {
  auto const udrspo = rvmtl()[unwinderDebuggerReturnSPOff()];

  auto const debuggerReturn = vwrap(cb, [&] (Vout& v) {
    v << load{udrspo, rvmsp()};
    v << storeqi{0, udrspo};
  });
  svcreq::emit_persistent(cb, folly::none, REQ_POST_DEBUGGER_RET);

  auto const resumeCPPUnwind = vwrap(cb, [] (Vout& v) {
    static_assert(sizeof(tl_regState) == 1,
                  "The following store must match the size of tl_regState.");
    auto const regstate = emitTLSAddr(v, tls_datum(tl_regState));
    v << storebi{static_cast<int32_t>(VMRegState::CLEAN), regstate};

    v << load{rvmtl()[unwinderExnOff()], rarg(0)};
    v << call{TCA(_Unwind_Resume), arg_regs(1)};
  });
  us.endCatchHelperPast = cb.frontier();
  vwrap(cb, [] (Vout& v) { v << ud2{}; });

  alignJmpTarget(cb);

  return vwrap(cb, [&] (Vout& v) {
    auto const done1 = v.makeBlock();
    auto const sf1 = v.makeReg();

    v << cmpqim{0, udrspo, sf1};
    v << jcci{CC_NE, sf1, done1, debuggerReturn};
    v = done1;

    // Normal end catch situation: call back to tc_unwind_resume, which returns
    // the catch trace (or null) in %r3, and the new vmfp in %r4.
    v << copy{rvmfp(), rarg(0)};
    v << call{TCA(tc_unwind_resume)};
    v << copy{ppc64_asm::reg::r4, rvmfp()};

    auto const done2 = v.makeBlock();
    auto const sf2 = v.makeReg();

    v << testq{ppc64_asm::reg::r3, ppc64_asm::reg::r3, sf2};
    v << jcci{CC_Z, sf2, done2, resumeCPPUnwind};
    v = done2;

    // We need to do a syncForLLVMCatch(), but vmfp is already in rdx.
    v << jmpr{ppc64_asm::reg::r3};
  });
}

///////////////////////////////////////////////////////////////////////////////

}}}
