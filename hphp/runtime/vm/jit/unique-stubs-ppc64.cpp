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
#include "hphp/runtime/vm/jit/types.h"
#include "hphp/runtime/vm/runtime.h"

namespace HPHP { namespace jit { namespace ppc64 {

//////////////////////////////////////////////////////////////////////

TRACE_SET_MOD(ustubs);

//////////////////////////////////////////////////////////////////////

namespace {

void moveToAlign(CodeBlock& cb) {
  align(cb, Alignment::JmpTarget, AlignContext::Dead);
}

//////////////////////////////////////////////////////////////////////

extern "C" void enterTCExit();


//////////////////////////////////////////////////////////////////////

} // end of anonymous namespace

//////////////////////////////////////////////////////////////////////

UniqueStubs emitUniqueStubs() {
  UniqueStubs us;
/*  auto functions = {
      emitCallToExit,
      emitThrowSwitchMode,
      emitCatchHelper,
      emitFreeLocalsHelpers,
      emitDecRefHelper,
      emitFCallArrayHelper,
      emitFunctionEnterHelper,
      emitFunctionSurprisedOrStackOverflow,
  };
  for (auto& f : functions) f(us);*/
  return us;
}

//////////////////////////////////////////////////////////////////////

}}}

