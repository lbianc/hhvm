/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | Copyright (c) 2010-2016 Facebook, Inc. (http://www.facebook.com)     |
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

#include "hphp/runtime/vm/jit/align.h"

#include "hphp/runtime/base/arch.h"

#include "hphp/runtime/vm/jit/align-arm.h"
#include "hphp/runtime/vm/jit/align-x64.h"
#include "hphp/runtime/vm/jit/align-ppc64.h"

namespace HPHP { namespace jit {

///////////////////////////////////////////////////////////////////////////////

bool is_aligned(TCA frontier, Alignment alignment) {
  return ARCH_SWITCH_CALL(is_aligned, frontier, alignment);
}

void align(CodeBlock& cb, Alignment alignment, AlignContext context,
           bool fixups /* = true */) {
  return ARCH_SWITCH_CALL(align, cb, alignment, context, fixups);
}

size_t cache_line_size() {
  return ARCH_SWITCH_CALL(cache_line_size);
}

///////////////////////////////////////////////////////////////////////////////

}}
