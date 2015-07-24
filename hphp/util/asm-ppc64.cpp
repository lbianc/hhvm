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

#include "hphp/util/asm-ppc64.h"

#include <folly/Format.h>

#include "hphp/util/safe-cast.h"

namespace HPHP { namespace jit {

// These are in order according to the binary encoding of the X64
// condition codes.

const char* cc_names[] = {
  "O", "NO", "B", "AE", "E", "NE", "BE", "A",
  "S", "NS", "P", "NP", "L", "GE", "LE", "G"
};

const char* show(RoundDirection rd) {
  switch (rd) {
    case RoundDirection::nearest:  return "nearest";
    case RoundDirection::floor:    return "floor";
    case RoundDirection::ceil:     return "ceil";
    case RoundDirection::truncate: return "truncate";
  }
  not_reached();
}



} }
*/
