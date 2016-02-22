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

#include "hphp/runtime/vm/jit/align-ppc64.h"
#include "hphp/runtime/vm/jit/asm-info.h"
#include "hphp/runtime/vm/jit/ir-opcode.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/relocation.h"
#include "hphp/runtime/vm/jit/smashable-instr.h"

namespace HPHP { namespace jit { namespace ppc64 {

namespace {

TRACE_SET_MOD(hhir);

//////////////////////////////////////////////////////////////////////

using WideJmpSet = hphp_hash_set<void*>;

TcaRange fixupRange(const RelocationInfo& rel, const TcaRange& rng) {
  auto s = rel.adjustedAddressAfter(rng.begin());
  auto e = rel.adjustedAddressBefore(rng.end());
  if (s && e) {
    return TcaRange(s, e);
  }
  if (s && !e) {
    return TcaRange(s, s + rng.size());
  }
  if (!s && e) {
    return TcaRange(e - rng.size(), e);
  }
  return rng;
}

void fixupRanges(AsmInfo* asmInfo, AreaIndex area, RelocationInfo& rel) {
  not_implemented();
}

size_t relocateImpl(RelocationInfo& rel,
                    CodeBlock& destBlock,
                    TCA start, TCA end,
                    CodeGenFixups& fixups,
                    TCA* exitAddr,
                    WideJmpSet& wideJmps) {
  not_implemented();
  return size_t{0};
}

//////////////////////////////////////////////////////////////////////

}

void adjustForRelocation(RelocationInfo& rel) {
  not_implemented();
}

void adjustForRelocation(RelocationInfo& rel, TCA srcStart, TCA srcEnd) {
  not_implemented();
}

void adjustMetaDataForRelocation(RelocationInfo& rel,
                                 AsmInfo* asmInfo,
                                 CodeGenFixups& fixups) {
  not_implemented();
}

void adjustCodeForRelocation(RelocationInfo& rel, CodeGenFixups& fixups) {
  not_implemented();
}

void findFixups(TCA start, TCA end, CodeGenFixups& fixups) {
  not_implemented();
}


size_t relocate(RelocationInfo& rel,
                CodeBlock& destBlock,
                TCA start, TCA end,
                CodeGenFixups& fixups,
                TCA* exitAddr) {
  return size_t{0};
}

//////////////////////////////////////////////////////////////////////

}}}
