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
#ifndef incl_HPHP_ARCH_H
#define incl_HPHP_ARCH_H

#include "hphp/runtime/base/runtime-option.h"

namespace HPHP {

enum class Arch {
  X64,
  ARM,
  PPC64,
};

inline Arch arch() {
#if defined(__powerpc64__)
       return Arch::PPC64;
#else
    if (RuntimeOption::EvalSimulateARM) return Arch::ARM;
      return Arch::X64;
#endif
}

}

#endif
