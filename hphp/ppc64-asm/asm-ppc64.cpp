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

#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/ppc64-asm/decoded-instr-ppc64.h"
#include "hphp/ppc64-asm/decoder-ppc64.h"
#include "hphp/runtime/base/runtime-option.h"

namespace ppc64_asm {
int64_t VMTOC::pushElem(int64_t elem) {
  if (m_map.find(elem) != m_map.end()) {
    return m_map[elem];
  }
  auto offset = allocTOC(static_cast<int32_t>(elem & 0xffffffff), true);
  m_map.insert( { elem, offset });
  allocTOC(static_cast<int32_t>((elem & 0xffffffff00000000) >> 32));
  m_last_elem_pos += 2;
  return offset;
}

int64_t VMTOC::pushElem(int32_t elem) {
  if (m_map.find(elem) != m_map.end()) {
    return m_map[elem];
  }
  auto offset = allocTOC(elem);
  m_map.insert( { elem, offset });
  m_last_elem_pos++;
  return offset;
}

VMTOC& VMTOC::getInstance() {
  static VMTOC instance;
  return instance;
}

intptr_t VMTOC::getPtrVector() {
  always_assert(m_tocvector != nullptr);
  return reinterpret_cast<intptr_t>(m_tocvector->base() + INT16_MAX + 1);
}

int64_t VMTOC::allocTOC (int32_t target, bool align) {
  if (!m_tocvector->canEmit(sizeof(int32_t))) {
    return 0x0;
  }

  HPHP::Address addr = m_tocvector->frontier();
  if (align && reinterpret_cast<uintptr_t>(addr) % 8 != 0) {
    m_tocvector->dword(reinterpret_cast<int32_t>(0xF0F0));
    addr = m_tocvector->frontier();
  }

  m_tocvector->dword(reinterpret_cast<int32_t>(target));
  return addr - (m_tocvector->base() + INT16_MAX + 1);
}

void BranchParams::decodeInstr(const PPC64Instr* const pinstr) {
  const DecoderInfo dinfo = Decoder::GetDecoder().decode(pinstr);
  switch (dinfo.opcode_name()) {
    case OpcodeNames::op_b:
    case OpcodeNames::op_bl:
      assert(dinfo.form() == Form::kI);
      defineBoBi(BranchConditions::Always);
      break;
    case OpcodeNames::op_bc:
      assert(dinfo.form() == Form::kB);
      B_form_t bform;
      bform.instruction = dinfo.instruction_image();
      m_bo = BranchParams::BO(bform.BO);
      m_bi = BranchParams::BI(bform.BI);
      break;
    case OpcodeNames::op_bcctr:
    case OpcodeNames::op_bcctrl:
      assert(dinfo.form() == Form::kXL);
      XL_form_t xlform;
      xlform.instruction = dinfo.instruction_image();
      m_bo = BranchParams::BO(xlform.BT);
      m_bi = BranchParams::BI(xlform.BA);
      break;
    default:
      assert(false && "Not a valid conditional branch instruction");
      // also possible: defineBoBi(BranchConditions::Always);
      break;
  }

  // Set m_lr accordingly for all 'call' flavors used
  switch (dinfo.opcode_name()) {
    case OpcodeNames::op_bl:
    case OpcodeNames::op_bcctrl:
      m_lr = true;
      break;
    default:
      m_lr = false;
      break;
  }
}

/*
 * Macro definition for EmitXOForm functions
 * Format:
 *  X(name,   arg3,  oe,  xop)
 *    name: function name
 *    arg3: ARG if needed, otherwise NONE to skip
 *    oe:   parameter value
 *    xop:  parameter value
 */

#define ADDS\
  X(add,     ARG,   0,  266)  \
  X(addme,   NONE,  0,  234)  \
  X(addc,    ARG,   0,  10)   \
  X(addco,   ARG,   1,  10)   \
  X(adde,    ARG,   0,  138)  \
  X(addeo,   ARG,   1,  138)  \
  X(addmeo,  NONE,  1,  234)  \
  X(addo,    ARG,   1,  266)  \
  X(addze,   NONE,  0,  202)  \
  X(addzeo,  NONE,  1,  202)  \
  X(divd,    ARG,   0,  489)  \
  X(divde,   ARG,   0,  425)  \
  X(divdeo,  ARG,   1,  425)  \
  X(divdeu,  ARG,   0,  393)  \
  X(divdeuo, ARG,   1,  393)  \
  X(divdo,   ARG,   1,  489)  \
  X(divdu,   ARG,   0,  457)  \
  X(divduo,  ARG,   1,  457)  \
  X(divw,    ARG,   0,  491)  \
  X(divwe,   ARG,   0,  427)  \
  X(divweo,  ARG,   1,  427)  \
  X(divweu,  ARG,   0,  395)  \
  X(divweuo, ARG,   1,  395)  \
  X(divwo,   ARG,   1,  491)  \
  X(divwu,   ARG,   0,  459)  \
  X(divwuo,  ARG,   1,  459)  \
  X(mulhd,   ARG,   0,  73)   \
  X(mulhdu,  ARG,   0,  9)    \
  X(mulhw,   ARG,   0,  75)   \
  X(mulhwu,  ARG,   0,  11)   \
  X(mulld,   ARG,   0,  233)  \
  X(mulldo,  ARG,   1,  233)  \
  X(mullw,   ARG,   0,  235)  \
  X(mullwo,  ARG,   1,  235)  \
  X(neg,     NONE,  0,  104)  \
  X(nego,    NONE,  1,  104)  \
  X(subf,    ARG,   0,  40)   \
  X(subfo,   ARG,   1,  40)   \
  X(subfc,   ARG,   0,  8)    \
  X(subfco,  ARG,   1,  8)    \
  X(subfe,   ARG,   0,  136)  \
  X(subfeo,  ARG,   1,  136)  \
  X(subfme,  NONE,  0,  232)  \
  X(subfmeo, NONE,  1,  232)  \
  X(subfze,  NONE,  0,  200)  \
  X(subfzeo, NONE,  1,  200)

/* Function header: XO1 */
#define HEADER_ARG  const Reg64& rb,
#define HEADER_NONE

#define XO1(name, arg3, oe, xop)                                          \
void Assembler::name(const Reg64& rt, const Reg64& ra, arg3 bool rc) {

/* Function body: XO2 */
#define BODY_ARG  rb
#define BODY_NONE 0

#define XO2(name, arg3, oe, xop)                                          \
    EmitXOForm(31, rn(rt), rn(ra), rn(arg3), oe, xop, rc);                \
}

/* Macro expansion for function parts */
#define X(name, arg3, oe, xop)  \
      XO1(name, HEADER_##arg3,  oe, xop)  \
      XO2(name, BODY_##arg3,    oe, xop)
ADDS
#undef X

#undef HEADER_ARG
#undef HEADER_NONE

#undef BODY_ARG
#undef BODY_NONE

#undef ADDS

void Assembler::addi(const Reg64& rt, const Reg64& ra, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(14, rn(rt), rn(ra), imm.w());
}

void Assembler::addic(const Reg64& rt, const Reg64& ra, uint16_t imm, bool rc) {
  EmitDForm(12 + (uint8_t) rc, rn(rt), rn(ra), imm);
}

void Assembler::addis(const Reg64& rt, const Reg64& ra, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(15, rn(rt), rn(ra), imm.w());
}

void Assembler::and(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                     bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 28, rc);
}

void Assembler::andc(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                     bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 60, rc);
}

void Assembler::andi(const Reg64& ra, const Reg64& rs, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(28, rn(rs), rn(ra), imm.w());
}

void Assembler::andis(const Reg64& ra, const Reg64& rs, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(29, rn(rs), rn(ra), imm.w());
}

void Assembler::b(int32_t offset) {
  EmitIForm(18, uint32_t(offset));
}

void Assembler::bl(int32_t offset) {
  EmitIForm(18, uint32_t(offset), 0, 1);
}

void Assembler::bc(uint8_t bo, uint8_t bi, int16_t offset) {
  EmitBForm(16, bo, bi, uint32_t(offset), 0, 0);
}

void Assembler::bcctr(uint8_t bo, uint8_t bi, uint16_t bh) {
  EmitXLForm(19, bo, bi, (bh & 0x3), 528);
}

void Assembler::bctrl() {
  // The concept of a conditional call is not existent for upper layers.
  // Therefore no bcctrl is defined despite being possible.
  // Only bctrl is defined.
  BranchParams bp(BranchConditions::Always);
  EmitXLForm(19, bp.bo(), bp.bi(), (0 /*bh*/ & 0x3), 528, 1);
}

void Assembler::blr() {
  // The concept of a conditional return is not existent for upper layers.
  // Therefore no bclr is defined despite being possible.
  // Only blr is defined.
  BranchParams bp(BranchConditions::Always);
  EmitXLForm(19, bp.bo(), bp.bi(), (0 /*bh*/ & 0x3), 16, 0);
}

void Assembler::bpermd(const Reg64& ra, const Reg64& rs, const Reg64& rv) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 252);
}

void Assembler::cmp(uint16_t bf, bool l, const Reg64& ra, const Reg64& rb) {
  EmitXForm(31, rn((bf << 2) | (uint16_t)l), rn(ra), rn(rb), 0);
}

void Assembler::cmpi(uint16_t bf, bool l, const Reg64& ra, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(11, rn((bf << 2) | (uint16_t)l), rn(ra), imm.w());
}

void Assembler::cmpb(const Reg64& rs, const Reg64& ra, const Reg64& rb) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 508);
}

void Assembler::cmpl(uint16_t bf, bool l, const Reg64& ra, const Reg64& rb) {
  EmitXForm(31, rn((bf << 2) | (uint16_t)l), rn(ra), rn(rb), 32);
}

void Assembler::cmpli(uint16_t bf, bool l, const Reg64& ra, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(10, rn((bf << 2) | (uint16_t)l), rn(ra), imm.w());
}

void Assembler::cntlzd(const Reg64& ra, const Reg64& rs, bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 26, rc);
}
void Assembler::cntlzw(const Reg64& ra, const Reg64& rs, bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 58, rc);
}

void Assembler::crand(uint16_t bt, uint16_t ba, uint16_t bb) {
  EmitXLForm(19, bt, ba, bb, 257);
}

void Assembler::crandc(uint16_t bt, uint16_t ba, uint16_t bb) {
  EmitXLForm(19, bt, ba, bb, 129);
}

void Assembler::creqv(uint16_t bt, uint16_t ba, uint16_t bb) {
  EmitXLForm(19, bt, ba, bb, 289);
}

void Assembler::crnand(uint16_t bt, uint16_t ba, uint16_t bb) {
  EmitXLForm(19, bt, ba, bb, 225);
}

void Assembler::crnor(uint16_t bt, uint16_t ba, uint16_t bb) {
  EmitXLForm(19, bt, ba, bb, 33);
}

void Assembler::cror(uint16_t bt, uint16_t ba, uint16_t bb) {
  EmitXLForm(19, bt, ba, bb, 449);
}

void Assembler::crorc(uint16_t bt, uint16_t ba, uint16_t bb) {
  EmitXLForm(19, bt, ba, bb, 417);
}

void Assembler::crxor(uint16_t bt, uint16_t ba, uint16_t bb) {
  EmitXLForm(19, bt, ba, bb, 193);
}

void Assembler::eqv(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                    bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 284, rc);
}

void Assembler::extsb(const Reg64& ra, const Reg64& rs, bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 954, rc);
}

void Assembler::extsh(const Reg64& ra, const Reg64& rs, bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 922);
}

void Assembler::extsw(const Reg64& ra, const Reg64& rs, bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 986);
}

void Assembler::isel(const Reg64& rt, const Reg64& ra, const Reg64& rb,
                     uint8_t bc) {
  EmitAForm(31, rn(rt), rn(ra), rn(rb), rn(bc), 15);
}

void Assembler::lbz(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(34, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::lbzu(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(35, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::lbzux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 119);
}

void Assembler::lbzx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 87);
}

void Assembler::ld(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDSForm(58, rn(rt), rn(m.r.base), m.r.disp, 0);
}

void Assembler::ldbrx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 532);
}

void Assembler::ldu(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDSForm(58, rn(rt), rn(m.r.base), m.r.disp, 1);
}

void Assembler::ldux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 53);
}

void Assembler::ldx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 21);
}

void Assembler::lhbrx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 790);
}

void Assembler::lhz(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(40, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::lhzu(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(41, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::lhzux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 331);
}

void Assembler::lhzx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 279);
}

void Assembler::lha(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(42, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::lhau(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(43, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::lhaux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 375);
}

void Assembler::lhax(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 343);
}

void Assembler::lmw(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(46, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::lq(const Reg64& rtp, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  assertx(rn(rtp) == rn(m.r.base)); // assert invalid instruction form
  EmitDQForm(56, rn(rtp), rn(m.r.base), m.r.disp);
}

void Assembler::lswi(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 597);
}

void Assembler::lswx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 533);
}

void Assembler::lwz(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(32, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::lwzu(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(33, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::lwzux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 55);
}

void Assembler::lwzx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 23);
}

void Assembler::lwa(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDSForm(58, rn(rt), rn(m.r.base), m.r.disp, 2);
}

void Assembler::lwaux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 373);
}

void Assembler::lwax(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 341);
}

void Assembler::lwbrx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 534);
}

void Assembler::mcrf(uint16_t bf, uint16_t bfa) {
  EmitXLForm(19, (bf & 0x1c), (bfa & 0x1c), 0, 0);
}

void Assembler::mfspr(const SpecialReg spr, const Reg64& rs) {
  EmitXFXForm(31, rn(rs), spr, 339);
}

void Assembler::mtspr(const SpecialReg spr, const Reg64& rs) {
  EmitXFXForm(31, rn(rs), spr, 467);
}

void Assembler::mulli(const Reg64& rt, const Reg64& ra, uint16_t imm) {
  EmitDForm(7, rn(rt), rn(ra), imm);
}

void Assembler::nand(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                     bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 476, rc);
}

void Assembler::nor(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                    bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 124, rc);
}

void Assembler::or(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                    bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 444, rc);
}

void Assembler::orc(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                    bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 412, rc);
}

void Assembler::ori(const Reg64& ra, const Reg64& rs, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(24, rn(rs), rn(ra), imm.w());
}

void Assembler::oris(const Reg64& ra, const Reg64& rs, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(25, rn(rs), rn(ra), imm.w());
}

void Assembler::popcntb(const Reg64& ra, const Reg64& rs) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 122);
}

void Assembler::popcntd(const Reg64& ra, const Reg64& rs) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 439);
}

void Assembler::popcntw(const Reg64& ra, const Reg64& rs) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 356);
}

void Assembler::prtyd(const Reg64& ra, const Reg64& rs) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 186);
}

void Assembler::prtyw(const Reg64& ra, const Reg64& rs) {
  EmitXForm(31, rn(rs), rn(ra), rn(0), 154);
}

void Assembler::rldcl(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                      uint8_t mb, bool rc) {
  EmitMDSForm(30, rn(rs), rn(ra), rn(rb), mb, 8, rc);
}

void Assembler::rldcr(const Reg64& ra, const Reg64& rs,  const Reg64& rb,
                      uint8_t mb, bool rc) {
  EmitMDSForm(30, rn(rs), rn(ra), rn(rb), mb, 9, rc);
}

void Assembler::rldic(const Reg64& ra, const Reg64& rs, uint8_t sh,
                      uint8_t mb, bool rc) {
  EmitMDForm(30, rn(rs), rn(ra), sh, mb, 2, rc);
}

void Assembler::rldicl(const Reg64& ra, const Reg64& rs, uint8_t sh,
                       uint8_t mb, bool rc) {
  EmitMDForm(30, rn(rs), rn(ra), sh, mb, 0, rc);
}

void Assembler::rldicr(const Reg64& ra, const Reg64& rs, uint8_t sh,
                       uint8_t mb, bool rc) {
  EmitMDForm(30, rn(rs), rn(ra), sh, mb, 1, rc);
}

void Assembler::rldimi(const Reg64& ra, const Reg64& rs, uint8_t sh,
                       uint8_t mb, bool rc) {
  EmitMDForm(30, rn(rs), rn(ra), sh, mb, 3, rc);
}

void Assembler::rlwimi(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb,
                       uint16_t me, bool rc) {
  EmitMForm(20, rn(rs), rn(ra), rn(sh), mb, me, rc);
}

void Assembler::rlwinm(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb,
                       uint16_t me, bool rc) {
  EmitMForm(21, rn(rs), rn(ra), rn(sh), mb, me, rc);
}

void Assembler::rlwnm(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb,
                      uint16_t me, bool rc) {
  EmitMForm(23, rn(rs), rn(ra), rn(sh), mb, me, rc);
}

void Assembler::sc(uint16_t lev) {
  EmitSCForm(17, (lev & 0x1));
}

void Assembler::sld(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                    bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 27, rc);
}

void Assembler::slw(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                    bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 24, rc);
}

void Assembler::srad(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                     bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 794, rc);
}

void Assembler::sradi(const Reg64& ra, const Reg64& rs, uint8_t sh, bool rc) {
  EmitXSForm(31, rn(rs), rn(ra), sh, 413, rc);
}

void Assembler::sraw(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                     bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 792, rc);
}

void Assembler::srawi(const Reg64& ra, const Reg64& rs, uint8_t sh, bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(sh), 824, rc);
}

void Assembler::srd(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                    bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 539, rc);
}

void Assembler::srw(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                    bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 536, rc);
}

  void Assembler::stb(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(38, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::stbu(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(39, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::stbux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 247);
}

void Assembler::stbx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 215);
}

void Assembler::sth(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(44, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::sthu(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(45, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::sthux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 439);
}

void Assembler::sthx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 407);
}

void Assembler::stw(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(36, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::stwu(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(37, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::stwux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 183);
}

void Assembler::stwx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 151);
}

void Assembler::std(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDSForm(62, rn(rt), rn(m.r.base), m.r.disp, 0);
}

void Assembler::stdu(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDSForm(62, rn(rt), rn(m.r.base), m.r.disp, 1);
}

void Assembler::stdux(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 181);
}

void Assembler::stdx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 149);
}

void Assembler::stq(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDSForm(62, rn(rt), rn(m.r.base), m.r.disp, 2);
}

void Assembler::sthbrx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 918);
}

void Assembler::stwbrx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 662);
}

void Assembler::stdbrx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 660);
}

void Assembler::stmw(const Reg64& rt, MemoryRef m) {
  assertx(Reg64(-1) == m.r.index);  // doesn't support base+index
  EmitDForm(47, rn(rt), rn(m.r.base), m.r.disp);
}

void Assembler::stswi(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 725);
}

void Assembler::stswx(const Reg64& rt, MemoryRef m) {
  assertx(!m.r.disp);  // doesn't support immediate displacement
  EmitXForm(31, rn(rt), rn(m.r.base), rn(m.r.index), 661);
}
void Assembler::subfic(const Reg64& rt, const Reg64& ra,  uint16_t imm) {
  EmitDForm(8, rn(rt), rn(ra), imm);
}

void Assembler::td(uint16_t to, const Reg64& ra, const Reg64& rb) {
  EmitXForm(31, rn(to), rn(ra), rn(rb), 68);
}

void Assembler::tdi(uint16_t to, const Reg64& ra, uint16_t imm) {
  EmitDForm(2, rn(to), rn(ra), imm);
}

void Assembler::tw(uint16_t to, const Reg64& ra, const Reg64& rb) {
  EmitXForm(31, rn(to), rn(ra), rn(rb), 4);
}

void Assembler::twi(uint16_t to, const Reg64& ra, uint16_t imm) {
  EmitDForm(3, rn(to), rn(ra), imm);
}

void Assembler::xor(const Reg64& ra, const Reg64& rs, const Reg64& rb,
                     bool rc) {
  EmitXForm(31, rn(rs), rn(ra), rn(rb), 316, rc);
}

void Assembler::xori(const Reg64& ra, const Reg64& rs, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(26, rn(rs), rn(ra), imm.w());
}

void Assembler::xoris(const Reg64& ra, const Reg64& rs, Immed imm) {
  assert(imm.fits(HPHP::sz::word) && "Immediate is too big");
  EmitDForm(27, rn(rs), rn(ra), imm.w());
}

/* Floating point operations */
void Assembler::fadd(const RegXMM& frt, const RegXMM& fra, const RegXMM& frb,
                     bool rc) {
  EmitAForm(63, rn(frt), rn(fra), rn(frb), rn(0), 21, rc);
}

void Assembler::fsub(const RegXMM& frt, const RegXMM& fra, const RegXMM& frb,
                     bool rc) {
  EmitAForm(63, rn(frt), rn(fra), rn(frb), rn(0), 20, rc);
}

void Assembler::fmul(const RegXMM& frt, const RegXMM& fra, const RegXMM& frc,
                     bool rc) {
  EmitAForm(63, rn(frt), rn(fra), rn(0), rn(frc), 25, rc);
}

void Assembler::fdiv(const RegXMM& frt, const RegXMM& fra, const RegXMM& frb,
                     bool rc) {
  EmitAForm(63, rn(frt), rn(fra), rn(frb), rn(0), 18, rc);
}


void Assembler::unimplemented(){
  //Emit a instruction with invalid opcode 0x0
  EmitDForm(0, rn(0), rn(0), 0);
}

//////////////////////////////////////////////////////////////////////

void Assembler::patchAbsolute(CodeAddress jmp,
                              CodeAddress dest,
                              bool useTOC) {
  // Initialize code block cb pointing to li64
  HPHP::CodeBlock cb;
  cb.init(jmp, Assembler::kLi64Len, "patched bctr");
  Assembler a{ cb };
  if (!useTOC) {
    a.li64(reg::r12, ssize_t(dest), true);
  }
  else {
    a.limmediate(reg::r12, ssize_t(dest), true);
  }
}

void Assembler::patchBranch(CodeAddress jmp,
                            CodeAddress dest,
                            bool useTOC) {
  auto di = DecodedInstruction(jmp);

  // Detect Far branch
  if (di.isFarBranch()) {
    patchAbsolute(jmp, dest, useTOC);
    return;
  }

  // Regular patch for branch by offset type
  if (!di.setNearBranchTarget(dest))
    assert(false && "Can't patch a branch with such a big offset");
}

//////////////////////////////////////////////////////////////////////

void Assembler::li64 (const Reg64& rt, int64_t imm64, bool fixedSize) {
  // li64 always emits 5 instructions i.e. 20 bytes of instructions.
  // Assumes that 0 bytes will be missing in the end.
  uint8_t missing = 0;

  // for assert purposes
  DEBUG_ONLY CodeAddress li64StartPos = frontier();

  if (HPHP::jit::deltaFits(imm64, HPHP::sz::word)) {
    // immediate has only low 16 bits set, use simple load immediate
    li(rt, static_cast<int16_t>(imm64));
    if (imm64 & (1ULL << 15) && !(imm64 & (1ULL << 16))) {
      // clear extended sign that should not be set
      // (32bits number. Sets the 16th bit but not the 17th, it's not negative!)
      clrldi(rt, rt, 48);
      missing = kLi64Len - 2 * instr_size_in_bytes;
    } else {
      missing = kLi64Len - 1 * instr_size_in_bytes;
    }
  } else if (HPHP::jit::deltaFits(imm64, HPHP::sz::dword)) {
    // immediate has only low 32 bits set
    lis(rt, static_cast<int16_t>(imm64 >> 16));
    ori(rt, rt, static_cast<int16_t>(imm64 & UINT16_MAX));
    if (imm64 & (1ULL << 31) && !(imm64 & (1ULL << 32))) {
      // clear extended sign
      // (64bits number. Sets the 32th bit but not the 33th, it's not negative!)
      clrldi(rt, rt, 32);
      missing = kLi64Len - 3 * instr_size_in_bytes;
    } else {
      missing = kLi64Len - 2 * instr_size_in_bytes;
    }
  } else if (imm64 >> 48 == 0) {
    // immediate has only low 48 bits set
    lis(rt, static_cast<int16_t>(imm64 >> 32));
    ori(rt, rt, static_cast<int16_t>((imm64 >> 16) & UINT16_MAX));
    sldi(rt,rt,16);
    ori(rt, rt, static_cast<int16_t>(imm64 & UINT16_MAX));
    if (imm64 & (1ULL << 47)) {
      // clear extended sign
      clrldi(rt, rt, 16);
    } else {
      missing = kLi64Len - 4 * instr_size_in_bytes;
    }
  } else {
    // load all 64 bits
    lis(rt, static_cast<int16_t>(imm64 >> 48));
    ori(rt, rt, static_cast<int16_t>((imm64 >> 32) & UINT16_MAX));
    sldi(rt,rt,32);
    oris(rt, rt, static_cast<int16_t>((imm64 >> 16) & UINT16_MAX));
    ori(rt, rt, static_cast<int16_t>(imm64 & UINT16_MAX));
  }

  if (fixedSize) {
    emitNop(missing);
    // guarantee our math with kLi64Len is working
    assert(kLi64Len == frontier() - li64StartPos);
  }
}

/*int64_t Assembler::getLimmediate(PPC64Instr* pinstr) {
  auto di = Decoder::GetDecoder().decode(pinstr);
  if (di.isLdTOC()) {
    auto indexTOC = di.offsetDS() >> 2;
    return VMTOC::getInstance().getValue(indexTOC);
  }
  else if (di.isLwzTOC()) {
    auto indexTOC = di.offsetD() >> 2;
    return VMTOC::getInstance().getValue(indexTOC);
  }

  DecodedInstruction dinstr(pinstr);
  return dinstr.immediate();
}

Reg64 Assembler::getLimmediateReg(PPC64Instr* pinstr) {
  auto di = Decoder::GetDecoder().decode(pinstr);
  if (di.isLdTOC()) {
    DS_form_t ds_instr;
    ds_instr.instruction = *pinstr;
    return Reg64(ds_instr.RT);
  }
  else if (di.isLwzTOC()) {
    D_form_t d_instr;
    d_instr.instruction = *pinstr;
    return Reg64(d_instr.RT);
  }
  DecodedInstruction dinstr(pinstr);
  return dinstr.getLi64Reg();
}*/

void Assembler::li32(const Reg64& rt, int32_t imm32) {

  if (HPHP::jit::deltaFits(imm32, HPHP::sz::word)) {
    // immediate has only low 16 bits set, use simple load immediate
    li(rt, static_cast<int16_t>(imm32));
    if (imm32 & (1ULL << 15) && !(imm32 & (1ULL << 16))) {
      // clear extended sign that should not be set
      // (32bits number. Sets the 16th bit but not the 17th, it's not negative!)
      clrldi(rt, rt, 48);
    } else {
      emitNop(instr_size_in_bytes); // emit nop for a balanced li32 with 2 instr
    }
  } else {
    // immediate has 32 bits set
    lis(rt, static_cast<int16_t>(imm32 >> 16));
    ori(rt, rt, static_cast<int16_t>(imm32 & UINT16_MAX));
  }
}

void Assembler::limmediate (const Reg64& rt, int64_t imm64, bool fixedSize) {
  always_assert(HPHP::RuntimeOption::Evalppc64minTOCImmSize >= 0 &&
    HPHP::RuntimeOption::Evalppc64minTOCImmSize <= 64);

  auto fits = [](int64_t imm64, uint16_t shift_n) {
     return (static_cast<uint64_t>(imm64) >> shift_n) == 0 ? true : false;
  };

  if (fits(imm64, HPHP::RuntimeOption::Evalppc64minTOCImmSize)) {
    li64(rt, imm64, fixedSize);
    return;
  }

  bool fits32 = fits(imm64, 32);
  int64_t TOCoffset;
  if (HPHP::RuntimeOption::Evalppc64useTOCLwz && fits32) {
    TOCoffset = VMTOC::getInstance().pushElem(
        static_cast<int32_t>(0xffffffff & imm64));
  }
  else {
    TOCoffset = VMTOC::getInstance().pushElem(imm64);
  }

  auto loadTOC = [&](const Reg64& rt, const Reg64& rttoc,  int64_t imm64,
      uint64_t offset, bool fixedSize, bool fits32) {

    if (HPHP::RuntimeOption::Evalppc64useTOCLwz && fits32) {
      lwz(rt,rttoc[offset]);
    }
    else {
      ld(rt, rttoc[offset]);
    }
    if (fixedSize) {
      emitNop(3 * instr_size_in_bytes);
    }
  };

  if (TOCoffset > INT16_MAX) {
    int16_t complement = 0;
    // If last four bytes is still bigger than a signed 16bits, uses as two complement.
    if ((TOCoffset & UINT16_MAX) > INT16_MAX) complement = 1;
    addis(Reg64(26), Reg64(2), static_cast<int16_t>((TOCoffset >> 16) + complement));
    loadTOC(rt, Reg64(26), imm64, TOCoffset & UINT16_MAX, fixedSize, fits32);
  }
  else {
    loadTOC(rt, Reg64(2), imm64, TOCoffset, fixedSize, fits32);
    emitNop(1 * instr_size_in_bytes);
  }

  return;
}

//////////////////////////////////////////////////////////////////////
// Label
//////////////////////////////////////////////////////////////////////

Label::~Label() {
  for (auto& ji : m_toPatch) {
    ji.a->patchBranch(ji.addr, m_address, false);
  }
}

void Label::branch(Assembler& a, BranchConditions bc, LinkReg lr) {
  // Only optimize jump if it'll unlikely going to be patched.
  if (m_address) {
    // if diff is 0, then this is for sure going to be patched.
    ssize_t diff = ssize_t(m_address - a.frontier());
    if (diff) {
      // check if an unconditional branch with b can be used
      if (BranchConditions::Always == bc) {
        // unconditional branch
        if (HPHP::jit::deltaFitsBits(diff, 26)) {
          addJump(&a);
          if (LinkReg::Save == lr) a.bl(diff);
          else                     a.b (diff);
          return;
        }
      } else {
        // conditional branch
        if (HPHP::jit::deltaFits(diff, HPHP::sz::word)) {
          BranchParams bp(bc);
          addJump(&a);
          assert(LinkReg::DoNotTouch == lr &&
              "Conditional call is NOT supported.");

          // Special code for overflow handling
          if (bc == BranchConditions::Overflow ||
              bc == BranchConditions::NoOverflow) {
            a.xor(reg::r0, reg::r0, reg::r0,false);
            a.mtspr(Assembler::SpecialReg::XER, reg::r0);
          }
          a.bc (bp.bo(), bp.bi(), diff);
          return;
        }
      }
    }
  }
  // fallback: use CTR to perform absolute branch up to 64 bits
  branchFar(a, bc, lr);
}

void Label::branchFar(Assembler& a,
                  BranchConditions bc,
                  LinkReg lr,
                  bool fixedSize /* = true */) {
  // Marking current address for patchAbsolute
  addJump(&a);

  // Use reserved function linkage register
  const ssize_t address = ssize_t(m_address);
  a.li64(reg::r12, address, fixedSize);

  // When branching to another context, r12 need to keep the target address
  // to correctly set r2 (TOC reference).
  a.mtctr(reg::r12);

  // Special code for overflow handling
  bool cond = (BranchConditions::Always != bc);
  if (bc == BranchConditions::Overflow || bc == BranchConditions::NoOverflow) {
    a.xor(reg::r0, reg::r0, reg::r0,false);
    a.mtspr(Assembler::SpecialReg::XER, reg::r0);
  } else if (cond && fixedSize) {
    // Unconditional branch (jmp or call) doesn't need this reserve bytes
    a.emitNop(2 * instr_size_in_bytes);
  }

  BranchParams bp(bc);
  if (LinkReg::Save == lr) {
    // call
    a.bctrl();
  } else {
    // jcc
    a.bcctr(bp.bo(), bp.bi(), 0);
  }
}

void Label::asm_label(Assembler& a) {
  assert(!m_address && !m_a && "Label was already set");
  m_a = &a;
  m_address = a.frontier();
}

void Label::addJump(Assembler* a) {
  if (m_address) return;
  JumpInfo info;
  info.a = a;
  info.addr = a->codeBlock.frontier();
  m_toPatch.push_back(info);
}

} // namespace ppc64_asm
