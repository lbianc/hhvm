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

#include "hphp/runtime/vm/jit/decoded-instr-ppc64.h"

#include "hphp/ppc64-asm/asm-ppc64.h"

#include <folly/Format.h>

#include "hphp/util/safe-cast.h"

namespace HPHP { namespace jit { namespace ppc64 {

using namespace ppc64_asm;

void DecodedInstruction::decode(uint8_t* ip) {
  m_ip = ip;
  m_flagsVal = 0;
  m_map_select = 0;
  m_xtra_op = 0;
  m_immSz = sz::nosize;
  m_offSz = sz::nosize;

  ip += m_offSz + m_immSz;
  m_size = ip - m_ip;
}

static int64_t readValue(uint8_t* ip, int size) {
#if 0 // TODO(gut)
  int64_t value = 0;
  value = (signed char)ip[--size];
  while (size--) {
    value <<= 8;
    value += ip[size];
  }
  return value;
#else
  return 0;
#endif
}

static bool writeValue(uint8_t* ip, int size, int64_t v) {
#if 0 // TODO(gut)
  auto value = uint64_t(v);
  if (size * CHAR_BIT < 64) {
    auto topBit = uint64_t(1) << (size * CHAR_BIT - 1);
    if (value + topBit >= topBit * 2) return false;
  }

  while (size--) {
    *ip++ = (uint8_t)value;
    value >>= CHAR_BIT;
  }
  return true;

#else
  return true;
#endif
}

std::string DecodedInstruction::toString() {

  auto str = folly::format("{:08x} {:02x}",
                           (uint64_t)m_ip,
                           m_opcode).str();
#if 0 // TODO(gut)
  if (m_flags.hasModRm) {
    auto modRm = getModRm();
    str += folly::format(" ModRM({:02b} {} {})",
                         modRm >> 6,
                         (modRm >> 3) & 7,
                         modRm & 7).str();
    if (m_flags.hasSib) {
      auto sib = m_ip[m_size - m_immSz - m_offSz - 1];
      str += folly::format(" SIB({:02b} {} {})",
                           sib >> 6,
                           (sib >> 3) & 7,
                           sib & 7).str();
    }
  }

  auto ip = m_ip + m_size - m_immSz - m_offSz;
  if (m_offSz) {
    int64_t value = readValue(ip, m_offSz);
    ip += m_offSz;
    str += folly::format(" {}{:+x}",
                         m_flags.picOff ? "rip" : "",
                         value).str();
    if (m_flags.picOff) {
      str += folly::format("({:08x})", uintptr_t(m_ip + m_size + value)).str();
    }
  }

  if (m_immSz) {
    int64_t value = readValue(ip, m_immSz);
    ip += m_immSz;
    str += folly::format(" #{}", value).str();
  }
#endif
  return str;
}

int32_t DecodedInstruction::offset() const {
#if 0 // TODO(gut)
  assert(hasOffset());
  auto const addr = m_ip + m_size;
  return safe_cast<int32_t>(readValue(addr - m_offSz, m_offSz));
#else
  return 0;
#endif
}

uint8_t* DecodedInstruction::picAddress() const {
#if 0 // TODO(gut)
  assert(hasPicOffset());
  uint8_t* addr = m_ip + m_size;
  return addr + readValue(addr - m_immSz - m_offSz, m_offSz);
#else
  return nullptr;
#endif
}

bool DecodedInstruction::setPicAddress(uint8_t* target) {
#if 0 // TODO(gut)
  assert(hasPicOffset());
  uint8_t* addr = m_ip + m_size;
  ptrdiff_t diff = target - addr;

  return writeValue(addr - m_offSz - m_immSz, m_offSz, diff);
#else
  return true;
#endif
}

int64_t DecodedInstruction::immediate() const {
#if 0 // TODO(gut)
  assert(hasImmediate());
  return readValue(m_ip + m_size - m_immSz, m_immSz);
#else
  return 0;
#endif
}

bool DecodedInstruction::setImmediate(int64_t value) {
#if 0 // TODO(gut)
  assert(hasImmediate());
  return writeValue(m_ip + m_size - m_immSz, m_immSz, value);
#else
  return true;
#endif
}

bool DecodedInstruction::isNop() const {
#if 0 // TODO(gut)
  PPC64Instr instr = *reinterpret_cast<PPC64Instr*>(m_ip);
  D_form_t d_formater {0, 0, 0, 24 }; // check Assembler::ori
  return instr == d_formater.instruction;
#else
  return true;
#endif
}

bool DecodedInstruction::isBranch(bool allowCond /* = true */) const {
  // from patchBctr:
  // It has to skip 6 instructions: li64 (5 instructions) and mtctr
  CodeAddress bctr_addr = m_ip + Assembler::kBytesPerInstr * 6;
  // Opcode located at the 6 most significant bits
  if (((bctr_addr[3] >> 2) & 0x3F) != 19) return false; // from bctr
  return true;
}

bool DecodedInstruction::isCall() const {
#if 0 // TODO(gut)
  if (m_map_select != 0) return false;
  if (m_opcode == 0xe8) return true;
  if (m_opcode != 0xff) return false;
  return ((getModRm() >> 3) & 0x6) == 2;
#else
  return false;
#endif
}

bool DecodedInstruction::isJmp() const {
  // if it's conditional, it's not a jmp

  // from patchBctr:
  // It has to skip 6 instructions: li64 (5 instructions) and mtctr
  PPC64Instr bctr_instr = *(m_ip + Assembler::kBytesPerInstr * 6);

  // grabs binary code for bctr and bctrl
  BranchParams bp(BranchConditions::Always);
  XL_form_t bctr = {0, 528, 0, bp.bi(), bp.bo(), 19};
  XL_form_t bctrl = {1, 528, 0, bp.bi(), bp.bo(), 19};

  return bctr_instr == bctr.instruction || bctr_instr == bctrl.instruction;
}

bool DecodedInstruction::isLea() const {
#if 0 // TODO(gut)
  if (m_map_select != 0) return false;
  return m_opcode == 0x8d;
#else
  return false;
#endif
}

ConditionCode DecodedInstruction::jccCondCode() const {
#if 0 // TODO(gut)
  if (m_map_select == 0) {
    assert((m_opcode & 0xf0) == 0x70); // 8-bit jcc
  } else {
    assert(m_map_select == 1);
    assert((m_opcode & 0xf0) == 0x80); // 32-bit jcc
  }
  return static_cast<ConditionCode>(m_opcode & 0x0f);
#else
  return CC_None;
#endif
}

bool DecodedInstruction::shrinkBranch() {
#if 0 // TODO(gut)
  assert(isBranch());
  if (m_offSz != sz::dword) return false;
  auto addr = m_ip + m_size - m_offSz;
  auto delta = readValue(addr, m_offSz);
  if (m_map_select == 1) {
    if (m_flags.vex) return false;
    assert((m_opcode & 0xf0) == 0x80); // must be a 32-bit conditional branch
    /*
      The pc-relative offset is from the end of the instruction, and the
      instruction is shrinking by 4 bytes (opcode goes from 2 bytes to 1,
      and offset goes from 4 to 1), so we need to adjust delta by 4.
    */
    delta += 4;
    if (-128 > delta || delta > 127) return false;
    addr[-2] = 0x70 | (m_opcode & 0x0f); // make it an 8 bit conditional branch
    addr[-1] = delta;
  } else {
    assert(m_opcode == 0xe9); // must be a 32-bit unconditional branch
    /*
      As above, but opcode was already 1 byte, so the reduction is only 3
      bytes this time.
    */
    delta += 3;
    if (-128 > delta || delta > 127) return false;
    addr[-1] = 0xeb;
    addr[0] = delta;
  }
  decode(m_ip);
  assert(isBranch() && m_offSz == 1);
  return true;
#else
  return true;
#endif
}

void DecodedInstruction::widenBranch() {
#if 0 // TODO(gut)
  assert(m_offSz == 1 && isBranch());
  auto addr = m_ip + m_size - m_offSz;
  auto delta = readValue(addr, 1);
  if (m_opcode == 0xeb) {
    addr[-1] = 0xe9;
    writeValue(addr, 4, delta + 3);
  } else {
    addr[-1] = 0x0f;
    addr[0] = 0x80 | (m_opcode & 0xf);
    writeValue(addr + 1, 4, delta + 4);
  }
  decode(m_ip);
  assert(isBranch() && m_offSz == 4);
#endif
}

uint8_t DecodedInstruction::getModRm() const {
#if 0 // TODO(gut)
  assert(m_flags.hasModRm);
  return m_ip[m_size - m_immSz - m_offSz - m_flags.hasSib - 1];
#else
  return 0;
#endif
}

}}}
