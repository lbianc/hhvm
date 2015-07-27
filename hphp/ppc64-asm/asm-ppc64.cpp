/**
 * Copyright 2015, IBM
 * All Rights Reserved
 *
 * @author: Rogerio Alves
 *
 * This is a experimental macro assembler for PPC64. 
 * Don't expect to find all instructions here. 
 *
 * If you're looking for something more fully baked, here are some options
 * to consider use Nanojit or LLVM, both of which translate abstract virtual machine
 * instructions to the native target architecture.
 *
 */

#include "hphp/ppc64-asm/asm-ppc64.h"

namespace ppc64_asm {

void Assembler::add(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 0, 266, rc);
}

void Assembler::addc(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 0, 10, rc);
}

void Assembler::addco(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 1, 10, rc);
}

void Assembler::adde(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 0, 138, rc);
}

void Assembler::addeo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 1, 138, rc);
}

void Assembler::addi(const Reg64& rt, const Reg64& ra, uint16_t imm) {
    EmitDForm(14, rn(rt), rn(ra), imm);
}

void Assembler::addic(const Reg64& rt, const Reg64& ra, uint16_t imm, bool rc) {
    EmitDForm(12 + (uint8_t) rc, rn(rt), rn(ra), imm);
}

void Assembler::addis(const Reg64& rt, const Reg64& ra, uint16_t imm) {
    EmitDForm(15, rn(rt), rn(ra), imm);
}

void Assembler::addme(const Reg64& rt, const Reg64& ra, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 234, rc);
}

void Assembler::addmeo(const Reg64& rt, const Reg64& ra, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 234, rc);
}

void Assembler::addo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 1, 266, rc);
}

void Assembler::addze(const Reg64& rt, const Reg64& ra, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 202, rc);
}

void Assembler::addzeo(const Reg64& rt, const Reg64& ra, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 202, rc);
}

void Assembler::and_(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 28, rc);
}

void Assembler::andc(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 60, rc);
}

void Assembler::andi(const Reg64& rs, const Reg64& ra, uint16_t imm) {
    EmitDForm(28, rn(rs), rn(ra), imm);
}

void Assembler::andis(const Reg64& rs, const Reg64& ra, uint16_t imm) {
    EmitDForm(29, rn(rs), rn(ra), imm);
}

void Assembler::b(uint32_t target_addr) {
    EmitIForm(18, target_addr);
}

void Assembler::ba(uint32_t target_addr) {
    EmitIForm(18, target_addr, 1, 0);
}

void Assembler::bl(uint32_t target_addr) {
    EmitIForm(18, target_addr, 0, 1);
}

void Assembler::bla(uint32_t target_addr) {
    EmitIForm(18, target_addr, 1, 1);
}

void Assembler::bc(uint8_t bo, uint8_t bi, uint16_t target_addr) {
    EmitBForm(18, bo, bi, target_addr, 0, 0);
}

void Assembler::bca(uint8_t bo, uint8_t bi, uint16_t target_addr) {
    EmitBForm(18, bo, bi, target_addr, 1, 0);
}

void Assembler::bcctr(uint8_t bo, uint8_t bi, uint16_t bh) {
    EmitXLForm(19, bo, bi, (bh & 0x3), 528);
}

void Assembler::bcctrl(uint8_t bo, uint8_t bi, uint16_t bh) {
    EmitXLForm(19, bo, bi, (bh & 0x3), 528, 1);
}

void Assembler::bcl(uint8_t bo, uint8_t bi, uint16_t target_addr) {
    EmitBForm(18, bo, bi, target_addr, 0, 1);
}

void Assembler::bcla(uint8_t bo, uint8_t bi, uint16_t target_addr) {
    EmitBForm(18, bo, bi, target_addr, 1, 1);
}

void Assembler::bclr(uint8_t bo, uint8_t bi, uint16_t bh) {
    EmitXLForm(19, bo, bi, (bh & 0x3), 16, 0);
}

void Assembler::bclrl(uint8_t bo, uint8_t bi, uint16_t bh) {
    EmitXLForm(19, bo, bi, (bh & 0x3), 16, 1);
}

void Assembler::bctar(uint8_t bo, uint8_t bi, uint16_t bh) {
    EmitXLForm(19, bo, bi, (bh & 0x3), 560, 0);
}

void Assembler::bctarl(uint8_t bo, uint8_t bi, uint16_t bh) {
    EmitXLForm(19, bo, bi, (bh & 0x3), 560, 1);
}

void Assembler::bpermd(const Reg64& rs, const Reg64& ra, const Reg64& rv) {
    EmitXForm(31, rn(rs), rn(ra), rn(0), 252);    
}

void Assembler::cmp(uint16_t bf, bool l, Reg64& ra, Reg64& rb) {
    EmitXForm(31, rn((bf+(uint16_t)l) & 0x1d), rn(ra), rn(rb), 0);
}

void Assembler::cmpi(uint16_t bf, bool l, Reg64& ra, uint16_t imm) {
    EmitDForm(11, rn((bf+(uint16_t)l) & 0x1d), rn(ra), imm);
}

void Assembler::cmpb(const Reg64& rs, const Reg64& ra, const Reg64& rb) {
     EmitXForm(31, rn(rs), rn(ra), rn(rb), 508);
}

void Assembler::cmpl(uint16_t bf, bool l, Reg64& ra, Reg64& rb) {
    EmitXForm(31, rn((bf+(uint16_t)l) & 0x1d), rn(ra), rn(rb), 32);
}

void Assembler::cmpli(uint16_t bf, bool l, Reg64& ra, uint16_t imm) {
    EmitDForm(10, rn((bf+(uint16_t)l) & 0x1d), rn(ra), imm);
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

void Assembler::divd(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 489, rc);
}

void Assembler::divde(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 425, rc);
}

void Assembler::divdeo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 425, rc); 
}

void Assembler::divdeu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 393, rc);
}

void Assembler::divdeuo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 393, rc);  
}

void Assembler::divdo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 489, rc);
}

void Assembler::divdu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 457, rc);
}

void Assembler::divduo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 457, rc);
}

void Assembler::divw(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 491, rc); 
} 

void Assembler::divwe(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 427, rc);  
} 

void Assembler::divweo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 427, rc);
} 

void Assembler::divweu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 395, rc); 
} 

void Assembler::divweuo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 395, rc);
} 

void Assembler::divwo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 491, rc);
} 

void Assembler::divwu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 459, rc); 
} 

void Assembler::divwuo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 459, rc); 
} 

void Assembler::eqv(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc) {
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

void Assembler::isel(const Reg64& rt, const Reg64& ra, const Reg64& rb, uint16_t bc) {
    EmitAForm(31, rn(rt), rn(ra), rn(rb), bc, 15);
}

void Assembler::lbz(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(34, rn(rt), rn(rb), imm);
}

void Assembler::lbzu(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(35, rn(rt), rn(rb), imm);
}

void Assembler::lbzux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 87);
}

void Assembler::lbzx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 119);
}

void Assembler::ld(const Reg64& rt, const Reg64& rb, uint16_t imm) {
     EmitDSForm(58, rn(rt), rn(rb), imm, 0);
}

void Assembler::ldbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 532);
}

void Assembler::ldu(const Reg64& rt, const Reg64& rb, uint16_t imm) {
     EmitDSForm(58, rn(rt), rn(rb), imm, 1);
}

void Assembler::ldux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 21);
}

void Assembler::ldx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 53);
}

void Assembler::lhbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 790);
}

void Assembler::lhz(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(40, rn(rt), rn(rb), imm);
}

void Assembler::lhzu(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(41, rn(rt), rn(rb), imm);
}

void Assembler::lhzux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 279);
}

void Assembler::lhzx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 331);
}

void Assembler::lha(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(42, rn(rt), rn(rb), imm);
}

void Assembler::lhau(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(43, rn(rt), rn(rb), imm);
}

void Assembler::lhaux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 343);
}

void Assembler::lhax(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 375);
}

void Assembler::lmw(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(46, rn(rt), rn(rb), imm);
}

void Assembler::lq(const Reg64& rtp, const Reg64& ra, uint16_t imm) {
    //assert invalid instruction form
    assert(rn(rtp) != rn(ra));
    EmitDQForm(56, rn(rtp), rn(ra), imm);
}

void Assembler::lswi(const Reg64& rt, const Reg64& ra, uint16_t nb) {
    EmitXForm(31, rn(rt), rn(ra), rn(nb), 597);
}

void Assembler::lswx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 533);
}

void Assembler::lwz(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(32, rn(rt), rn(rb), imm);
}

void Assembler::lwzu(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(33, rn(rt), rn(rb), imm);
}

void Assembler::lwzux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 23);
}

void Assembler::lwzx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 55);
}

void Assembler::lwa(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDSForm(58, rn(rt), rn(rb), imm, 2);
}

void Assembler::lwaux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 341);
}

void Assembler::lwax(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 373);
}

void Assembler::lwbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 534);
}

void Assembler::mcrf(uint16_t bf, uint16_t bfa) {
    EmitXLForm(19, (bf & 0x1c), (bfa & 0x1c), 0, 0);
}

void Assembler::mulhd(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 73, rc);
}

void Assembler::mulhdu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 9, rc);
}

void Assembler::mulhw(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 0, 75, rc);
}

void Assembler::mulhwu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 0, 11, rc);
}

void Assembler::mulld(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 233, rc);
}

void Assembler::mulldo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 233, rc);
}

void Assembler::mulli(const Reg64& rt, const Reg64& ra, uint16_t imm) {
    EmitDForm(7, rn(rt), rn(ra), imm);
}

void Assembler::mullw(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 0, 235, rc);
}

void Assembler::mullwo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 1, 235, rc);
}

void Assembler::nand(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc) {
     EmitXForm(31, rn(rs), rn(ra), rn(rb), 476, rc);
}

void Assembler::neg(const Reg64& rt, const Reg64& ra, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 104, rc);
}

void Assembler::nego(const Reg64& rt, const Reg64& ra, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 104, rc);
}

void Assembler::nor(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 124, rc);
}

void Assembler::or_(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 444, rc);
}

void Assembler::orc(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 412, rc);
}

void Assembler::ori(const Reg64& rs, const Reg64& ra, uint16_t imm) {
    EmitDForm(24, rn(rs), rn(ra), imm);
}

void Assembler::oris(const Reg64& rs, const Reg64& ra, uint16_t imm) {
    EmitDForm(25, rn(rs), rn(ra), imm);
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

void Assembler::rldcl(const Reg64& ra, const Reg64& rs, const Reg64& rb, uint8_t mb, bool rc) { 
    EmitMDSForm(30, rn(rs), rn(ra), rn(rb), mb, 8, rc);
}

void Assembler::rldcr(const Reg64& ra, const Reg64& rs,  const Reg64& rb, uint8_t mb, bool rc) { 
    EmitMDSForm(30, rn(rs), rn(ra), rn(rb), mb, 9, rc);
}

void Assembler::rldic(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, bool rc) { 
    EmitMDForm(30, rn(rs), rn(ra), sh, mb, 2, (sh >> 6), rc);
}

void Assembler::rldicl(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, bool rc) { 
    EmitMDForm(30, rn(rs), rn(ra), sh, mb, 0, (sh >> 6), rc);
}

void Assembler::rldicr(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, bool rc) { 
    EmitMDForm(30, rn(rs), rn(ra), sh, mb, 1, (sh >> 6), rc);
}

void Assembler::rldimi(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, bool rc) { 
    EmitMDForm(30, rn(rs), rn(ra), sh, mb, 3, (sh >> 6), rc);
}

void Assembler::rlwimi(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, uint16_t me, bool rc) { 
    EmitMForm(20, rn(rs), rn(ra), rn(sh), mb, me, rc);
}

void Assembler::rlwinm(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, uint16_t me, bool rc) { 
    EmitMForm(21, rn(rs), rn(ra), rn(sh), mb, me, rc);
}

void Assembler::rlwnm(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, uint16_t me, bool rc) { 
    EmitMForm(23, rn(rs), rn(ra), rn(sh), mb, me, rc);
}

void Assembler::sc(uint16_t lev) {
    EmitSCForm(17, (lev & 0x1));
}

void Assembler::sld(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 27, rc);
}

void Assembler::slw(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 24, rc);
}

void Assembler::srad(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 794, rc);
}

void Assembler::sraw(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 792, rc);
}

void Assembler::srawi(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 824, rc);
}

void Assembler::srd(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 539, rc);
}

void Assembler::srw(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 536, rc);
}

void Assembler::stb(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(38, rn(rt), rn(rb), imm);
}

void Assembler::stbu(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(39, rn(rt), rn(rb), imm);
}

void Assembler::stbux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 215);
}

void Assembler::stbx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 247);
}

void Assembler::sth(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(44, rn(rt), rn(rb), imm);
}

void Assembler::sthu(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(45, rn(rt), rn(rb), imm);
}

void Assembler::sthux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 407);
}

void Assembler::sthx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 439);
}

void Assembler::stw(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(36, rn(rt), rn(rb), imm);
}

void Assembler::stwu(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(37, rn(rt), rn(rb), imm);
}

void Assembler::stwux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 151);
}

void Assembler::stwx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 183);
}

void Assembler::std(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDSForm(62, rn(rt), rn(rb), imm, 0);
}

void Assembler::stdu(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDSForm(62, rn(rt), rn(rb), imm, 1);
}

void Assembler::stdux(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 149);
}

void Assembler::stdx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 181);
}

void Assembler::stq(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDSForm(62, rn(rt), rn(rb), imm, 2);
}

void Assembler::sthbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 918);
}

void Assembler::stwbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 662);
}

void Assembler::stdbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 660);
}

void Assembler::stmw(const Reg64& rt, const Reg64& rb, uint16_t imm) {
    EmitDForm(47, rn(rt), rn(rb), imm);
}

void Assembler::stswi(const Reg64& rt, const Reg64& ra, uint16_t nb) {
    EmitXForm(31, rn(rt), rn(ra), rn(nb), 725);
}

void Assembler::stswx(const Reg64& rt, const Reg64& ra, const Reg64& rb) {
    EmitXForm(31, rn(rt), rn(ra), rn(rb), 661);
}

void Assembler::subf(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 0, 40, rc);
}

void Assembler::subfo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 1, 40, rc);
}

void Assembler::subfc(const Reg64& rt, const Reg64& ra, const Reg64& rb,  bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 0, 8, rc);
}

void Assembler::subfco(const Reg64& rt, const Reg64& ra, const Reg64& rb,  bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 1, 8, rc);
}

void Assembler::subfe(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) { 
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 0, 136, rc);
}

void Assembler::subfeo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(rb), 1, 136, rc); 
}   

void Assembler::subfic(const Reg64& rt, const Reg64& ra,  uint16_t imm) {
    EmitDForm(8, rn(rt), rn(ra), imm);
}

void Assembler::subfme(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 232, rc); 
}

void Assembler::subfmeo(const Reg64& rt,  const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 232, rc);
}

void Assembler::subfze(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 0, 200, rc); 
}

void Assembler::subfzeo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXOForm(31, rn(rt), rn(ra), rn(0), 1, 200, rc);
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

void Assembler::xor_(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc) {
    EmitXForm(31, rn(rs), rn(ra), rn(rb), 316, rc);
}

void Assembler::xori(const Reg64& rs, const Reg64& ra, uint16_t imm) {
    EmitDForm(26, rn(rs), rn(ra), imm);
}

void Assembler::xoris(const Reg64& rs, const Reg64& ra, uint16_t imm) {
    EmitDForm(27, rn(rs), rn(ra), imm);
}

} // namespace ppc64_asm
