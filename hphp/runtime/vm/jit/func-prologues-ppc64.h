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


#ifndef incl_HPHP_JIT_FUNC_PROLOGUES_PPC64_H
#define incl_HPHP_JIT_FUNC_PROLOGUES_PPC64_H

#include "hphp/util/data-block.h"
#include "hphp/runtime/base/arch.h"
#include "hphp/runtime/vm/srckey.h"
#include "hphp/runtime/vm/jit/translator-inline.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/types.h"

namespace HPHP {

class Func;

namespace jit { namespace ppc64 {

template<typename T>
T* funcPrologueToGuardImm(jit::TCA prologue) {
  assertx(arch() == Arch::PPC64);
  return nullptr;
}

inline bool funcPrologueHasGuard(jit::TCA prologue, const Func* func) {
  assertx(arch() == Arch::PPC64);
  return false;
}

inline TCA funcPrologueToGuard(TCA prologue, const Func* func) {
  assertx(arch() == Arch::PPC64);
	return nullptr;
}

inline void funcPrologueSmashGuard(jit::TCA prologue, const Func* func) {}

//////////////////////////////////////////////////////////////////////

jit::TCA emitCallArrayPrologue(Func* func, DVFuncletsVec& dvs);
SrcKey emitFuncPrologue(TransID transID, Func* func, int argc, TCA& start);

}}}

#endif
