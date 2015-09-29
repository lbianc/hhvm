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

#ifndef incl_HPHP_VM_CODE_GEN_TLS_PPC64_H_
#define incl_HPHP_VM_CODE_GEN_TLS_PPC64_H_

#include "hphp/runtime/vm/jit/vasm-gen.h"
#include "hphp/runtime/vm/jit/vasm-instr.h"
#include "hphp/runtime/vm/jit/vasm-reg.h"

#include "hphp/util/asm-ppc64.h"
#include "hphp/util/thread-local.h"

namespace HPHP { namespace jit { namespace ppc64 { namespace detail {

///////////////////////////////////////////////////////////////////////////////

template<typename T>
Vptr emitTLSAddr(Vout& v, TLSDatum<T> datum) {
  // Same implementation as x64. Might be incorrect.
  // TODO: Do the correct porting for POWER.
  uintptr_t vaddr = uintptr_t(datum.tls) - tlsBase();
  return Vptr{baseless(vaddr), Vptr::FS};
}

}}}}

#endif
