/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | (c) Copyright IBM Corporation 2015                                   |
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

#include <folly/Format.h>
#include "hphp/ppc64-asm/dasm-ppc64.h"

namespace ppc64_asm {

void Dissasembler::dissasembly(std::ostream& out, PPC64Instr* instr) {
  for(int i=0; i < indent_level_; i++) {
    out << ' ';
  }

  if(print_encoding_) {
   out << folly::format(
      "{:#16x}  {:08x}\t\t{}\n",
      reinterpret_cast<uint64_t>(instr));
  } else {

  }

}

} // namespace ppc64_asm
