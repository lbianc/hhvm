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
#include "hphp/runtime/vm/jit/service-requests-inline.h"
#include "hphp/runtime/vm/jit/types.h"
#include "hphp/runtime/vm/runtime.h"

namespace HPHP { namespace jit { namespace ppc64 {

//////////////////////////////////////////////////////////////////////

TRACE_SET_MOD(ustubs);

//////////////////////////////////////////////////////////////////////

namespace {

//////////////////////////////////////////////////////////////////////

extern "C" void enterTCExit();


//////////////////////////////////////////////////////////////////////

//TODO PPC64 start here
void emitFuncBodyHelperThunk(UniqueStubs& uniqueStubs) {
  TCA (*helper)(ActRec*) = &funcBodyHelper;
  Asm a { mcg->code.main() };

  moveToAlign(mcg->code.main());
  uniqueStubs.funcBodyHelperThunk = a.frontier();

  a.    mflr(ppc64_asm::reg::r0);
  // LR on parent call frame
  a.    std(ppc64_asm::reg::r0, ppc64_asm::reg::r1, 16);
  // minimum call stack
  a.    stdu(ppc64_asm::reg::r1, ppc64_asm::reg::r1, -32);

  // This helper is called via a direct jump from the TC (from
  // fcallArrayHelper). So the stack parity is already correct.
  emitCall(a, CppCall::direct(helper), argSet(1));

  // minimum call stack
  a.    addi(ppc64_asm::reg::r1, ppc64_asm::reg::r1, 32);
  // LR on parent call frame
  a.    ld(ppc64_asm::reg::r0, ppc64_asm::reg::r1, 16);
  a.    mtlr(ppc64_asm::reg::r0);
  a.    blr();
  a.    trap();

  uniqueStubs.add("funcBodyHelperThunk", uniqueStubs.funcBodyHelperThunk);
}

} // end of anonymous namespace

//////////////////////////////////////////////////////////////////////

UniqueStubs emitUniqueStubs() {
  UniqueStubs us;
  auto functions = {
    emitFuncBodyHelperThunk,
  };
  for (auto& f : functions) f(us);
  return us;
}

//////////////////////////////////////////////////////////////////////

}}}

