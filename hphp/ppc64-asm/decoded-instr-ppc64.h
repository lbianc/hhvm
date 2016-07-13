/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | (c) Copyright IBM Corporation 2016                                   |
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

#ifndef incl_HPHP_PPC64_ASM_DECODED_INSTR_PPC64_H_
#define incl_HPHP_PPC64_ASM_DECODED_INSTR_PPC64_H_

#include <folly/Format.h>

#include "hphp/util/asm-x64.h"

#include "hphp/ppc64-asm/decoder-ppc64.h"
#include "hphp/ppc64-asm/isa-ppc64.h"

namespace ppc64_asm {

struct DecodedInstruction {
  explicit DecodedInstruction(uint8_t* ip)
    : m_ip(ip)
    , m_size(instr_size_in_bytes)
  {
    dinfo = Decoder::GetDecoder().decode(m_ip);
  }

  DecodedInstruction() = delete;

  size_t size() const           { return size_t{m_size}; }

  int32_t offset() const        { return dinfo->offset(); }
  int32_t offsetDS() const        { return dinfo->offsetDS(); }
  bool isNop() const            { return dinfo->isNop(); }
  bool isLdTOC() const          {return dinfo->isLdTOC(); }
  // if it's conditional branch, it's not a jmp
  bool isJmp() const            { return isBranch(false); }
  bool isSpOffsetInstr() const  { return dinfo->isSpOffsetInstr(); }
  bool isClearSignBit() const   { return dinfo->isClearSignBit(); }

  HPHP::jit::ConditionCode jccCondCode() const;

  // True if it's Near or Far type.
  bool isBranch(bool allowCond = true) const;
  bool isNearBranch(bool allowCond = true) const;
  bool isFarBranch(bool allowCond = true) const;

  bool isCall() const;

private:
  uint8_t* m_ip;
  DecoderInfo* dinfo;
  uint8_t m_size;
};

}

#endif
