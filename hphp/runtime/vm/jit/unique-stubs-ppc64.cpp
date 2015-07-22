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

namespace HPHP { namespace jit { namespace ppc64 {

namespace {

//TODO(IBM): Needs to implement all emit stubs here
void emitCallToExit(UniqueStubs& us) {
  not_implemented();
}

void emitReturnHelpers(UniqueStubs& us) {
  not_implemented();
}

void emitResumeHelpers(UniqueStubs& us) {
  not_implemented();
}

void emitStackOverflowHelper(UniqueStubs& us) {
  not_implemented();
}

void emitFreeLocalsHelpers(UniqueStubs& us) {
  not_implemented();
}

void emitFuncPrologueRedispatch(UniqueStubs& us) {
  not_implemented();
}

void emitFCallArrayHelper(UniqueStubs& us) {
  not_implemented();
}

void emitFCallHelperThunk(UniqueStubs& us) {
  not_implemented();
}

void emitFuncBodyHelperThunk(UniqueStubs& us) {
  not_implemented();
}

void emitFunctionEnterHelper(UniqueStubs& us) {
  not_implemented();
}

void emitBindCallStubs(UniqueStubs& uniqueStubs) {
  not_implemented();
}

} // anonymous namespace


//////////////////////////////////////////////////////////////////////

UniqueStubs emitUniqueStubs() {
  UniqueStubs us;
  //TODO(IBM): 
  auto functions {    
    emitCallToExit,
    emitReturnHelpers,
    emitResumeHelpers,
    emitStackOverflowHelper,
    emitFreeLocalsHelpers,
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
