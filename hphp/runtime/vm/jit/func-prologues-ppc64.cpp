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
   
#include "hphp/runtime/base/array-init.h"
#include "hphp/runtime/ext/ext_closure.h"
#include "hphp/runtime/vm/jit/code-gen-helpers-ppc64.h"
#include "hphp/runtime/vm/jit/back-end.h"
#include "hphp/runtime/vm/jit/service-requests-ppc64.h"
#include "hphp/runtime/vm/jit/mc-generator.h"

namespace HPHP { namespace jit { namespace ppc64 {


//////////////////////////////////////////////////////////////////////

namespace {

SrcKey emitPrologueWork(Func* func, int nPassed) {
  not_implemented();
}
//////////////////////////////////////////////////////////////////////

} // anonymous namespace

TCA emitCallArrayPrologue(Func* func, DVFuncletsVec& dvs) {
  not_implemented();
}

SrcKey emitFuncPrologue(TransID transID, Func* func, int argc, TCA& start) {
  not_implemented();
}

}}}