/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | (c) Copyright IBM Corporation 2015-2016                              |
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

#include "hphp/ppc64-asm/decoded-instr-ppc64.h"

#include <folly/Format.h>

#include "hphp/ppc64-asm/decoder-ppc64.h"
#include "hphp/ppc64-asm/asm-ppc64.h"

namespace ppc64_asm {

bool DecodedInstruction::isBranch(bool allowCond /* = true */) const {
  return isFarBranch(allowCond) || isNearBranch(allowCond);
}
bool DecodedInstruction::isNearBranch(bool allowCond /* = true */) const {
  return dinfo->isBranch(allowCond);
}
bool DecodedInstruction::isFarBranch(bool allowCond /* = true */) const {
  auto size = (allowCond) ? Assembler::kJccLen : Assembler::kCallLen;

  // skip the preparation instructions that are not actually the branch.
  auto far_branch_instr = m_ip + size - instr_size_in_bytes;
  return Decoder::GetDecoder().decode(far_branch_instr)->isBranch(allowCond);
}

bool DecodedInstruction::isCall() const {
  return isFarBranch(false);
}

HPHP::jit::ConditionCode DecodedInstruction::jccCondCode() const {
  not_implemented();
  return HPHP::jit::CC_None;
}

}
