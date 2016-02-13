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

#include "hphp/runtime/vm/jit/smashable-instr-ppc64.h"

#include "hphp/ppc64-asm/decoded-instr-ppc64.h"
#include "hphp/ppc64-asm/decoder-ppc64.h"

#include "hphp/ppc64-asm/asm-ppc64.h"

#include <folly/Format.h>

#include "hphp/util/safe-cast.h"

namespace ppc64_asm {

void DecodedInstruction::decode(uint8_t* ip) {
  m_ip = ip;
  m_flagsVal = 0;
  m_map_select = 0;
  m_xtra_op = 0;
  m_immSz = HPHP::sz::nosize;
  m_offSz = HPHP::sz::nosize;

  ip += m_offSz + m_immSz;
  m_size = ip - m_ip;
}

static int64_t readValue(uint8_t* ip, int size) {
  not_implemented();
  return 0;
}

static bool writeValue(uint8_t* ip, int size, int64_t v) {
  not_implemented();
  return true;
}

std::string DecodedInstruction::toString() {

  auto str = folly::format("{:08x} {:02x}",
                           (uint64_t)m_ip,
                           m_opcode).str();
  not_implemented();
  return str;
}

int32_t DecodedInstruction::offset() const {
  not_implemented();
  return 0;
}

uint8_t* DecodedInstruction::picAddress() const {
  not_implemented();
  return nullptr;
}

bool DecodedInstruction::setPicAddress(uint8_t* target) {
  not_implemented();
  return true;
}

int64_t DecodedInstruction::immediate() const {
  not_implemented();
  return 0;
}

bool DecodedInstruction::setImmediate(int64_t value) {
  not_implemented();
  return true;
}

bool DecodedInstruction::isNop() const {
  return Decoder::GetDecoder().decode(m_ip)->isNop();
}

bool DecodedInstruction::isBranch(bool allowCond /* = true */) const {
  // skip the preparation instructions that are not actually the branch.
  auto branch_instr = m_ip + HPHP::jit::ppc64::smashableJccSkip();
  return Decoder::GetDecoder().decode(branch_instr)->isBranch(allowCond);
}

bool DecodedInstruction::isCall() const {
  // skip the preparation instructions that are not actually the branch.
  auto branch_instr = m_ip + HPHP::jit::ppc64::smashableCallSkip();
  return Decoder::GetDecoder().decode(branch_instr)->isBranch(false);
}

bool DecodedInstruction::isJmp() const {
  // if it's conditional branch, it's not a jmp
  return isBranch(false);
}

bool DecodedInstruction::isLea() const {
  not_implemented();
  return false;
}

bool DecodedInstruction::isClearSignBit() const {
  return Decoder::GetDecoder().decode(m_ip)->isClearSignBit();
}

HPHP::jit::ConditionCode DecodedInstruction::jccCondCode() const {
  not_implemented();
  return HPHP::jit::CC_None;
}

bool DecodedInstruction::shrinkBranch() {
  not_implemented();
  return true;
}

void DecodedInstruction::widenBranch() {
  not_implemented();
}
uint8_t DecodedInstruction::getModRm() const {
  not_implemented();
  return 0;
}

}
