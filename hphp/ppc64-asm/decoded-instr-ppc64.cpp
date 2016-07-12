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

#include "hphp/util/data-block.h"

namespace ppc64_asm {

bool DecodedInstruction::couldBeNearBranch() {
  assert(isFarBranch());
  ptrdiff_t diff = farBranchTarget() - m_ip;
  bool uncond = isCall();
  return fitsOnNearBranch(diff, uncond);
}

uint8_t* DecodedInstruction::nearBranchTarget() const {
  assert(isNearBranch());
  auto address = reinterpret_cast<uint64_t>(m_ip) + m_dinfo.branchOffset();
  return reinterpret_cast<uint8_t*>(address);
}

bool DecodedInstruction::setNearBranchTarget(uint8_t* target) {
  if (!isNearBranch()) return false;
  ptrdiff_t diff = target - m_ip;
  bool uncond = m_dinfo.isOffsetBranch(false);
  if (fitsOnNearBranch(diff, uncond)) {
    auto pinstr = reinterpret_cast<PPC64Instr*>(m_ip);
    *pinstr = m_dinfo.setBranchOffset(int32_t(diff));
    return true;
  } else {
    return false;
  }
}

bool DecodedInstruction::isImmediate() const {
  // if destination register is r12, then it's preparing to branch
  return isLi64Possible() && (reg::r12 != getLi64Reg());
}

bool DecodedInstruction::setImmediate(int64_t value) {
  if (!isImmediate()) return false;

  // Initialize code block cb pointing to li64
  HPHP::CodeBlock cb;
  cb.init(m_ip, Assembler::kLi64Len, "setImmediate relocation");
  HPHP::CodeCursor cursor { cb, m_ip };
  Assembler a{ cb };

  a.li64(getLi64Reg(), ssize_t(value));

  // refresh m_imm and other parameters
  decode();
  return true;
}

bool DecodedInstruction::shrinkBranch() {
  // It should be a Far branch, otherwise don't do anything if it's Near.
  assertx(isBranch() && "Can't shrink instruction that is not a branch.");

  auto uncondBranch = isFarBranch(false);
  auto call = isCall();
  auto condBranch = isFarBranch(true);

  if (uncondBranch || call || condBranch) {
    HPHP::CodeBlock cb;
    cb.init(m_ip, instr_size_in_bytes, "shrinkBranch relocation");
    HPHP::CodeCursor cursor { cb, m_ip };
    Assembler a { cb };

    if (uncondBranch || call) {     // unconditional will be set as b
      // offset will be patched later
      if (call) a.bl(0);
      else      a.b(0);
    } else {                        // conditional will be bc
      // grab conditional parameters
      auto branch_instr = m_ip + Assembler::kJccLen - instr_size_in_bytes;
      BranchParams bp(branch_instr);

      // offset will be patched later
      a.bc(bp.bo(), bp.bi(), 0);
    }
    // refresh m_size and other parameters
    decode();
    return true;
  }
  return false;
}

void DecodedInstruction::widenBranch(uint8_t* target) {
  // currently, it should be a Near branch, else don't do anything if it's Far.
  assertx(isBranch() && "Can't widen instruction that is not a branch.");

  if (isNearBranch()) {
    // grab conditional parameters
    BranchParams bp(m_ip);

    HPHP::CodeBlock cb;
    assertx(Assembler::kJccLen > Assembler::kCallLen);
    auto max_branch_size = Assembler::kJccLen;
    cb.init(m_ip, max_branch_size, "widenBranch relocation");
    HPHP::CodeCursor cursor { cb, m_ip };
    Assembler a { cb };
    a.branchFar(target, bp, false);

    // refresh m_size and other parameters
    decode();
  }
}

bool DecodedInstruction::isBranch(bool allowCond /* = true */) const {
  return isFarBranch(allowCond) || isNearBranch(allowCond);
}

bool DecodedInstruction::isNearBranch(bool allowCond /* = true */) const {
  return m_dinfo.isOffsetBranch(allowCond);
}

bool DecodedInstruction::isFarBranch(bool allowCond /* = true */) const {
  return !getFarBranch(allowCond).isInvalid();
}

DecoderInfo DecodedInstruction::getFarBranch(bool allowCond) const {
  return getFarBranchLength(allowCond).m_di;
}

// Returns -1 if not found, otherwise the offset from m_ip that a register
// branch instruction is found
DecInfoOffset DecodedInstruction::getFarBranchLength(bool allowCond) const {
  DecInfoOffset ret;
  if (!isLi64Possible() || (reg::r12 != getLi64Reg())) return ret;

  // only read bytes up to the smallest of @max_read or @bytes.
  auto canRead = [](uint8_t n, uint8_t max_read, uint8_t bytes) -> bool {
    if (max_read)
#define MIN(a, b)    (((a) < (b)) ? (a) : (b))
      return n < MIN(max_read, bytes);
#undef MIN
    else
      return n < bytes;
  };

  // guarantee that the worst case is being analysed
  assertx(Assembler::kJccLen > Assembler::kCallLen);

  // Search for a register branch instruction like bctr. Return when found.
  for (ret.m_offset = 0;
      canRead(ret.m_offset, m_max_size, Assembler::kJccLen);
      ret.m_offset += instr_size_in_bytes) {
    // skip the preparation instructions that are not actually the branch.
    auto far_branch_instr = m_ip + ret.m_offset;
    ret.m_di = Decoder::GetDecoder().decode(far_branch_instr);
    if (ret.m_di.isRegisterBranch(allowCond)) return ret;
  }
  return DecInfoOffset();
}

bool DecodedInstruction::setFarBranchTarget(uint8_t* target) {
  DecoderInfo di = getFarBranch();
  if (di.isInvalid()) return false;

  ppc64_asm::BranchParams bp(di.ip());
  bool uncond = (bp.bo() == uint8_t(BranchParams::BO::Always));
  auto block_size = uncond ? Assembler::kCallLen : Assembler::kJccLen;

  HPHP::CodeBlock cb;
  cb.init(m_ip, block_size, "setFarBranchTarget");
  Assembler a{ cb };
  // avoid nops
  a.branchFar(target, bp, false);

  // refresh m_imm and other parameters
  decode();
  return true;
}

bool DecodedInstruction::isCall() const {
  return isFarBranch(false);
}

Reg64 DecodedInstruction::getLi64Reg() const {
  // First instruction is always either li or lis, both are D-form
  assertx(isLi64Possible());
  D_form_t d_instr;
  d_instr.instruction = m_dinfo.instruction_image();
  return Reg64(d_instr.RT);
}

///////////////////////////////////////////////////////////////////////////////
// Private Interface
///////////////////////////////////////////////////////////////////////////////

void DecodedInstruction::decode() {
  m_dinfo = Decoder::GetDecoder().decode(m_ip);

  if (isLi64Possible() && (reg::r12 == getLi64Reg())) {
    // Compute the whole branch on the m_size. Used on relocation to skip
    // instructions
    DecInfoOffset dio = getFarBranchLength();
    assertx(dio.m_offset > 0 && "Expected to find a Far branch");
    m_size = dio.m_offset + instr_size_in_bytes;
    decodeImm();            // sets m_imm for farBranchTarget()
  } else if (isImmediate()) {
    m_size = decodeImm();
  } else {
    m_size = instr_size_in_bytes;
  }
}

/*
 * Reads back the immediate when emmited with (without nops) and return the
 * number of bytes reaad for this immediate decoding.
 */
uint8_t DecodedInstruction::decodeImm() {
  // Functions that detect if @dinfo is exactly the instructions flavor that
  // our li64 uses. Also the target register to be modified has to be the same.
  auto isLi = [](DecoderInfo* dinfo, const Reg64& dest, int16_t* imm) {
    D_form_t dform;
    dform.instruction = dinfo->instruction_image();
    *imm = dform.D;
    return (OpcodeNames::op_addi == dinfo->opcode_name()) &&
      (dest == Reg64(dform.RT)) &&
      (!dform.RA);
  };
  auto isLis = [](DecoderInfo* dinfo, const Reg64& dest, int16_t* imm) {
    D_form_t dform;
    dform.instruction = dinfo->instruction_image();
    *imm = dform.D;
    return (OpcodeNames::op_addis == dinfo->opcode_name()) &&
      (dest == Reg64(dform.RT)) &&
      (!dform.RA);
  };
  auto isSldi = [](DecoderInfo* dinfo, const Reg64& dest, uint16_t* bits) {
    MD_form_t mdform;
    mdform.instruction = dinfo->instruction_image();

    // Assembling these crazy encodings
    *bits = (mdform.sh << 5) | mdform.SH;
    auto mask = ((mdform.MB & 0x1) << 5) | (mdform.MB >> 1);

    // It is only interesting when it's shifting 16bits multiples
    return (OpcodeNames::op_rldicr == dinfo->opcode_name()) &&
      (dest == Reg64(mdform.RS)) &&
      (1 == mdform.XO) &&
      ((16 == *bits) || (32 == *bits) || (48 == *bits)) &&
      ((63 - *bits) == mask);
  };
  auto isOri = [](DecoderInfo* dinfo, const Reg64& dest, uint16_t* bits) {
    D_form_t dform;
    dform.instruction = dinfo->instruction_image();
    *bits = dform.D;
    return (OpcodeNames::op_ori == dinfo->opcode_name()) &&
      (dest == Reg64(dform.RT)) &&
      (dest == Reg64(dform.RA));
  };
  auto isOris = [](DecoderInfo* dinfo, const Reg64& dest, uint16_t* bits) {
    D_form_t dform;
    dform.instruction = dinfo->instruction_image();
    *bits = dform.D;
    return (OpcodeNames::op_oris == dinfo->opcode_name()) &&
      (dest == Reg64(dform.RT)) &&
      (dest == Reg64(dform.RA));
  };

  const auto dest_reg = getLi64Reg();
  PPC64Instr* base = reinterpret_cast<PPC64Instr*>(m_ip);
  PPC64Instr* last_instr = base +
    (Assembler::kLi64Len / instr_size_in_bytes);

  // If m_max_size is 0, it can always read more, otherwise it's limited
  auto canReadMore = [&](uint8_t bytes_read) -> bool {
    if (m_max_size) return bytes_read < m_max_size;
    else            return true;
  };

  // Analyze at maximum kLi64Len instructions (and limited by m_max_size, if
  // not 0) but stop when some other instruction appears (or any that doesn't
  // have the dest_reg as a target).
  PPC64Instr* pinstr = base;
  uint8_t bytes_read = 0;
  while ((pinstr < last_instr) && canReadMore(bytes_read)) {

    auto dinfo = Decoder::GetDecoder().decode(pinstr);
    int16_t tmp_imm = 0;
    uint16_t tmp_bits = 0;

    if (isLi(&dinfo, dest_reg, &tmp_imm)) {
      m_imm = tmp_imm;
    } else if (isLis(&dinfo, dest_reg, &tmp_imm)) {
      m_imm = tmp_imm << 16;
    } else if (isSldi(&dinfo, dest_reg, &tmp_bits)) {
      m_imm = m_imm << tmp_bits;
    } else if (isOri(&dinfo, dest_reg, &tmp_bits)) {
      m_imm = m_imm | tmp_bits;
    } else if (isOris(&dinfo, dest_reg, &tmp_bits)) {
      m_imm = m_imm | (tmp_bits << 16);
    } else { break; }

    pinstr++;
    bytes_read += instr_size_in_bytes;
  }

  // amount of bytes read by this immediate reading
  return bytes_read;
}

bool DecodedInstruction::fitsOnNearBranch(ptrdiff_t diff, bool uncond) const {
  // is it b or bc? b can use offsets up to 26bits and bc only 16bits
  auto bitsize = uncond ? 26 : 16;
  return HPHP::jit::deltaFitsBits(diff, bitsize);
}

bool DecodedInstruction::isLi64Possible() const {
  // The beginning of a li64 starts with either li or lis.
  auto opn = m_dinfo.opcode_name();
  if ((OpcodeNames::op_addi == opn) || (OpcodeNames::op_addis == opn)) {
    D_form_t dform;
    dform.instruction = m_dinfo.instruction_image();
    if (!dform.RA) {
      // li or lis
      return true;
    }
  }
  return false;
}

}
