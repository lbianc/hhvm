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
#include "hphp/runtime/vm/jit/unique-stubs.h"

#include <boost/implicit_cast.hpp>
#include <sstream>

#include "hphp/util/abi-cxx.h"
#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/util/disasm.h"

#include "hphp/runtime/vm/bytecode.h"
#include "hphp/runtime/vm/jit/abi-ppc64.h"
#include "hphp/runtime/vm/jit/back-end-ppc64.h"
#include "hphp/runtime/vm/jit/code-gen-cf.h"
#include "hphp/runtime/vm/jit/code-gen-helpers-ppc64.h"
#include "hphp/runtime/vm/jit/mc-generator-internal.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/service-requests-inline.h"
#include "hphp/runtime/vm/jit/types.h"
#include "hphp/runtime/vm/runtime.h"

#pragma GCC diagnostic ignored "-Wreturn-type"

namespace HPHP { namespace jit { namespace ppc64 {

//////////////////////////////////////////////////////////////////////

using namespace jit::reg;
using boost::implicit_cast;

TRACE_SET_MOD(ustubs);

//////////////////////////////////////////////////////////////////////

namespace {

TCA emitRetFromInterpretedFrame() {}

TCA emitRetFromInterpretedGeneratorFrame() {}

TCA emitDebuggerRetFromInterpretedFrame() {}

TCA emitDebuggerRetFromInterpretedGenFrame() {}

//////////////////////////////////////////////////////////////////////

extern "C" void enterTCExit();

void emitCallToExit(UniqueStubs& uniqueStubs) {}

void emitReturnHelpers(UniqueStubs& us) {}

void emitResumeInterpHelpers(UniqueStubs& uniqueStubs) {}

void emitThrowSwitchMode(UniqueStubs& uniqueStubs) {}

void emitCatchHelper(UniqueStubs& uniqueStubs) {}

void emitStackOverflowHelper(UniqueStubs& uniqueStubs) {}

void emitFreeLocalsHelpers(UniqueStubs& uniqueStubs) {}

void emitDecRefHelper(UniqueStubs& us) {}

void emitFuncPrologueRedispatch(UniqueStubs& uniqueStubs) {}

void emitFCallArrayHelper(UniqueStubs& uniqueStubs) {}

//////////////////////////////////////////////////////////////////////

void emitFCallHelperThunk(UniqueStubs& uniqueStubs) {}

//TODO PPC64 start here
void emitFuncBodyHelperThunk(UniqueStubs& uniqueStubs) {
	  TCA (*helper)(ActRec*) = &funcBodyHelper;
	  Asm a { mcg->code.main() };

	  moveToAlign(mcg->code.main());
	  uniqueStubs.funcBodyHelperThunk = a.frontier();

	  // This helper is called via a direct jump from the TC (from
	  // fcallArrayHelper). So the stack parity is already correct.
	  a.    or_(rVmFp, argNumToRegName[0], argNumToRegName[0], false);
//	  a.    movq   (rVmFp, argNumToRegName[0]);
//	  emitCall(a, CppCall::direct(helper), argSet(1));
//	  a.    jmp    (rax);
//	  a.    ud2    ();

	  uniqueStubs.add("funcBodyHelperThunk", uniqueStubs.funcBodyHelperThunk);
}

void emitFunctionEnterHelper(UniqueStubs& uniqueStubs) {}

void emitBindCallStubs(UniqueStubs& uniqueStubs) {}

}

//////////////////////////////////////////////////////////////////////

UniqueStubs emitUniqueStubs() {
  UniqueStubs us;
  auto functions = {
    emitCallToExit,
    emitReturnHelpers,
    emitResumeInterpHelpers,
    emitThrowSwitchMode,
    emitCatchHelper,
    emitStackOverflowHelper,
    emitFreeLocalsHelpers,
    emitDecRefHelper,
    emitFuncPrologueRedispatch,
    emitFCallArrayHelper,
    emitFCallHelperThunk,
    emitFuncBodyHelperThunk,
    emitFunctionEnterHelper,
    emitBindCallStubs,
  };
  for (auto& f : functions) f(us);
  return us;
}

//////////////////////////////////////////////////////////////////////

}}}

#pragma GCC diagnostic pop
