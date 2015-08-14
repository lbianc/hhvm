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
 * to consider use Nanojit or LLVM, both of which translate abstract virtual 
 * machine instructions to the native target architecture.
 *
 */

#ifndef INCLUDE_ASM_PPC64_H_
#define INCLUDE_ASM_PPC64_H_

#include <cstdint>
#include <cassert>
#include <vector>
#include <iostream>

#include "hphp/util/data-block.h"
#include "hphp/runtime/vm/jit/types.h"

#include "hphp/ppc64-asm/isa-ppc64.h"
#include "hphp/util/asm-x64.h"
#include "hphp/util/immed.h"
#include "hphp/util/safe-cast.h"

namespace ppc64_asm {

using HPHP::jit::Reg64;
using HPHP::jit::MemoryRef;
using HPHP::jit::Immed;
using HPHP::CodeAddress;

#define BRANCHES(cr) \
  CR##cr##_LessThan,         \
  CR##cr##_LessThanEqual,    \
  CR##cr##_GreaterThan,      \
  CR##cr##_GreaterThanEqual, \
  CR##cr##_Equal,            \
  CR##cr##_NotEqual

enum class BranchConditions {
  BRANCHES(0),
  BRANCHES(1),
  BRANCHES(2),
  BRANCHES(3),
  BRANCHES(4),
  BRANCHES(5),
  BRANCHES(6),
  BRANCHES(7),
  Always,

  // mnemonics for the common case by using CR0:
  LessThan          = CR0_LessThan,
  LessThanEqual     = CR0_LessThanEqual,
  GreaterThan       = CR0_GreaterThan,
  GreaterThanEqual  = CR0_GreaterThanEqual,
  Equal             = CR0_Equal,
  NotEqual          = CR0_NotEqual
};

#undef BRANCHES

enum class RegNumber : uint32_t {};

namespace reg {
  constexpr Reg64 r0(0);    /* volatile, used in function prologue / linkage */
  constexpr Reg64 r1(1);    /* nonvolatile, stack pointer */
  constexpr Reg64 r2(2);    /* nonvolatile, TOC */
  /* volatile, argument passing registers */
  constexpr Reg64 r3(3);
  constexpr Reg64 r4(4);
  constexpr Reg64 r5(5);
  constexpr Reg64 r6(6);
  constexpr Reg64 r7(7);
  constexpr Reg64 r8(8);
  constexpr Reg64 r9(9);
  constexpr Reg64 r10(10);

  constexpr Reg64 r11(11);  /* volatile, environment pointer */
  constexpr Reg64 r12(12);  /* volatile, function entry address */
  constexpr Reg64 r13(13);  /* reserved, thread pointer */
  /* nonvolatile, local variables */
  constexpr Reg64 r14(14);
  constexpr Reg64 r15(15);
  constexpr Reg64 r16(16);
  constexpr Reg64 r17(17);
  constexpr Reg64 r18(18);
  constexpr Reg64 r19(19);
  constexpr Reg64 r20(20);
  constexpr Reg64 r21(21);
  constexpr Reg64 r22(22);
  constexpr Reg64 r23(23);
  constexpr Reg64 r24(24);
  constexpr Reg64 r25(25);
  constexpr Reg64 r26(26);
  constexpr Reg64 r27(27);
  constexpr Reg64 r28(28);
  constexpr Reg64 r29(29);
  constexpr Reg64 r30(30);
  constexpr Reg64 r31(31);

  constexpr Reg64 f0(0);   /* volatile scratch register */
  /* volatile, argument passing floating point registers */
  constexpr Reg64 f1(1);
  constexpr Reg64 f2(2);
  constexpr Reg64 f3(3);
  constexpr Reg64 f4(4);
  constexpr Reg64 f5(5);
  constexpr Reg64 f6(6);
  constexpr Reg64 f7(7);
  constexpr Reg64 f8(8);
  constexpr Reg64 f9(9);
  constexpr Reg64 f10(10);
  constexpr Reg64 f11(11);
  constexpr Reg64 f12(12);
  constexpr Reg64 f13(13);
  /* nonvolatile, local variables */
  constexpr Reg64 f14(14);
  constexpr Reg64 f15(15);
  constexpr Reg64 f16(16);
  constexpr Reg64 f17(17);
  constexpr Reg64 f18(18);
  constexpr Reg64 f19(19);
  constexpr Reg64 f20(20);
  constexpr Reg64 f21(21);
  constexpr Reg64 f22(22);
  constexpr Reg64 f23(23);
  constexpr Reg64 f24(24);
  constexpr Reg64 f25(25);
  constexpr Reg64 f26(26);
  constexpr Reg64 f27(27);
  constexpr Reg64 f28(28);
  constexpr Reg64 f29(29);
  constexpr Reg64 f30(30);
  constexpr Reg64 f31(31);

  /* volatile, local variables */
  constexpr Reg64 v0(0);
  constexpr Reg64 v1(1);
  /* volatile, argument passing vector registers */
  constexpr Reg64 v2(2);
  constexpr Reg64 v3(3);
  constexpr Reg64 v4(4);
  constexpr Reg64 v5(5);
  constexpr Reg64 v6(6);
  constexpr Reg64 v7(7);
  constexpr Reg64 v8(8);
  constexpr Reg64 v9(9);
  constexpr Reg64 v10(10);
  constexpr Reg64 v11(11);
  constexpr Reg64 v12(12);
  constexpr Reg64 v13(13);
  /* volatile, local variables */
  constexpr Reg64 v14(14);
  constexpr Reg64 v15(15);
  constexpr Reg64 v16(16);
  constexpr Reg64 v17(17);
  constexpr Reg64 v18(18);
  constexpr Reg64 v19(19);
  /* nonvolatile, local variables */
  constexpr Reg64 v20(20);
  constexpr Reg64 v21(21);
  constexpr Reg64 v22(22);
  constexpr Reg64 v23(23);
  constexpr Reg64 v24(24);
  constexpr Reg64 v25(25);
  constexpr Reg64 v26(26);
  constexpr Reg64 v27(27);
  constexpr Reg64 v28(28);
  constexpr Reg64 v29(29);
  constexpr Reg64 v30(30);
  constexpr Reg64 v31(31);
}

//////////////////////////////////////////////////////////////////////

/*
  This is extracted from X64 assembler. For PPC64 we maybe need to 
  implement a TOC as decribed in PPC64 ABI. To futher information see
  http://physinfo.ulb.ac.be/divers_html/powerpc_programming_info/intro_to_ppc/
  ppc4_runtime4.html
*/

/*
 * Select the assembler which contains a given address.
 *
 * Asm& a = codeBlockChoose(toPatch, a, acold);
 *   a.patchJmp(...);
 */
// inline HPHP::CodeBlock& codeBlockChoose(CodeAddress addr) {
//   always_assert_flog(false,
//                     "address {} was not part of any known code block", addr);
// }
// template<class... Blocks>
// HPHP::CodeBlock& codeBlockChoose(CodeAddress addr, HPHP::CodeBlock& a,
// Blocks&... as) {
//   if (a.contains(addr)) return a;
//   return codeBlockChoose(addr, as...);
// }

//////////////////////////////////////////////////////////////////////

class Label;

class Assembler {
public:

  friend struct Label;

   explicit Assembler(HPHP::CodeBlock& cb) : codeBlock(cb) {}
   ~Assembler(){}

   HPHP::CodeBlock& code() const { return codeBlock; }

   CodeAddress base() const {
     return codeBlock.base();
   }

   CodeAddress frontier() const {
     return codeBlock.frontier();
   }

   void setFrontier(CodeAddress newFrontier) {
     codeBlock.setFrontier(newFrontier);
   }

   size_t capacity() const {
     return codeBlock.capacity();
   }

   size_t used() const {
     return codeBlock.used();
   }

   size_t available() const {
     return codeBlock.available();
   }

   bool contains(CodeAddress addr) const {
     return codeBlock.contains(addr);
   }

   bool empty() const {
     return codeBlock.empty();
   }

   void clear() {
     codeBlock.clear();
   }

   bool canEmit(size_t nBytes) const {
     assert(capacity() >= used());
     return nBytes < (capacity() - used());
   }

  /*
    TODO(IBM): Must create a macro for these similar instructions.
               This will make code more clean.

    #define CC_ARITH_REG_OP(name, opcode, x_opcode)
    void name##c
    void name##co
    ...
    #define CC_ARITH_IMM_OP(name, opcode)

    #define LOAD_STORE_OP(name, opcode)
    void name#w
    void name#b
    void name#h
    void name#d
  */

  enum class SpecialReg {
    XER      = 1,
    DSCR     = 3,
    LR       = 8,
    CTR      = 9,
    AMR      = 13,
    TFHAR    = 128,
    TFIAR    = 129,
    TEXASR   = 130,
    TEXASRU  = 131,
    VRSAVE   = 256,
    SPEFSCR  = 512,
    MMCR2    = 769,
    MMCRA    = 770,
    PMC1     = 771,
    PMC2     = 772,
    PMC3     = 773,
    PMC4     = 774,
    PMC5     = 775,
    PMC6     = 776,
    MMCR0    = 779,
    BESCRS   = 800,
    BESCRSU  = 801,
    BESCRR   = 802,
    BESCRRU  = 803,
    EBBHR    = 804,
    EBBRR    = 805,
    BESCR    = 806,
    TAR      = 815,
    PPR      = 896,
    PPR32    = 898
  };

  //PPC64 ISA - Only Fixed Point instructions have been implemented
  void add(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addc(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addco(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void adde(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addeo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addi(const Reg64& rt, const Reg64& ra, Immed imm);
  void addic(const Reg64& rt, const Reg64& ra, uint16_t imm, bool rc);
  void addis(const Reg64& rt, const Reg64& ra, uint16_t imm);
  void addme(const Reg64& rt, const Reg64& ra, bool rc);
  void addmeo(const Reg64& rt, const Reg64& ra, bool rc);
  void addo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addze(const Reg64& rt, const Reg64& ra, bool rc);
  void addzeo(const Reg64& rt, const Reg64& ra, bool rc);
  void and_(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void andc(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void andi(const Reg64& rs, const Reg64& ra, Immed imm);
  void andis(const Reg64& rs, const Reg64& ra, uint16_t imm);
  void b(uint32_t target_addr);
  void ba(uint32_t target_addr);
  void bl(CodeAddress target_addr);
  void bla(uint32_t target_addr);
  void bc(uint8_t bo, uint8_t bi, CodeAddress target_addr);
  void bca(uint8_t bo, uint8_t bi, CodeAddress target_addr);
  void bcctr(uint8_t bo, uint8_t bi, uint16_t bh);
  void bcctrl(uint8_t bo, uint8_t bi, uint16_t bh);
  void bcl(uint8_t bo, uint8_t bi, uint16_t target_addr);
  void bcla(uint8_t bo, uint8_t bi, uint16_t target_addr);
  void bclr(uint8_t bo, uint8_t bi, uint16_t bh);
  void bclrl(uint8_t bo, uint8_t bi, uint16_t bh);
  void bctar(uint8_t bo, uint8_t bi, uint16_t bh);
  void bctarl(uint8_t bo, uint8_t bi, uint16_t bh);
  void bpermd(const Reg64& rs, const Reg64& ra, const Reg64& rv);
  void cmp(uint16_t bf, bool l, const Reg64& ra, const Reg64& rb);
  void cmpi(uint16_t bf, bool l, const Reg64& ra, Immed imm);
  void cmpb(const Reg64& rs, const Reg64& ra, const Reg64& rb);
  void cmpl(uint16_t bf, bool l, Reg64& ra, Reg64& rb);
  void cmpli(uint16_t bf, bool l, Reg64& ra, uint16_t imm);
  void cntlzd(const Reg64& ra, const Reg64& rs, bool rc);
  void cntlzw(const Reg64& ra, const Reg64& rs, bool rc);
  void crand(uint16_t bt, uint16_t ba, uint16_t bb);
  void crandc(uint16_t bt, uint16_t ba, uint16_t bb);
  void creqv(uint16_t bt, uint16_t ba, uint16_t bb);
  void crnand(uint16_t bt, uint16_t ba, uint16_t bb);
  void crnor(uint16_t bt, uint16_t ba, uint16_t bb);
  void cror(uint16_t bt, uint16_t ba, uint16_t bb);
  void crorc(uint16_t bt, uint16_t ba, uint16_t bb);
  void crxor(uint16_t bt, uint16_t ba, uint16_t bb);
  void divd(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divde(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divdeo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divdeu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divdeuo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divdo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divdu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divduo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divw(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divwe(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divweo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divweu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divweuo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divwo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divwu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void divwuo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void eqv(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void extsb(const Reg64& ra, const Reg64& rs, bool rc);
  void extsh(const Reg64& ra, const Reg64& rs, bool rc);
  void extsw(const Reg64& ra, const Reg64& rs, bool rc);
  void isel(const Reg64& rt, const Reg64& ra, const Reg64& rb, uint16_t bc);
  void lbz(const Reg64& rt, MemoryRef m);
  void lbzu(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lbzux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lbzx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void ld(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void ldbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void ldu(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void ldux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void ldx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lhbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lhz(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lhzu(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lhzux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lhzx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lha(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lhau(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lhaux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lhax(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lmw(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lq(const Reg64& rtp, const Reg64& ra, uint16_t imm);
  void lswi(const Reg64& rt, const Reg64& ra, uint16_t nb);
  void lswx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lwz(const Reg64& rt, MemoryRef m);
  void lwzu(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lwzux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lwzx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lwa(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lwaux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lwax(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lwbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void mcrf(uint16_t bf, uint16_t bfa);
  void mtspr(const SpecialReg spr, const Reg64& rs);
  void mulhd(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void mulhdu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void mulhw(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void mulhwu(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void mulld(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void mulldo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void mulli(const Reg64& rt, const Reg64& ra, uint16_t imm);
  void mullw(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void mullwo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void nand(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void neg(const Reg64& rt, const Reg64& ra, bool rc);
  void nego(const Reg64& rt, const Reg64& ra, bool rc);
  void nor(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void or_(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void orc(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void ori(const Reg64& rs, const Reg64& ra, Immed imm);
  void oris(const Reg64& rs, const Reg64& ra, uint16_t imm);
  void popcntb(const Reg64& ra, const Reg64& rs);
  void popcntd(const Reg64& ra, const Reg64& rs);
  void popcntw(const Reg64& ra, const Reg64& rs);
  void prtyd(const Reg64& ra, const Reg64& rs);
  void prtyw(const Reg64& ra, const Reg64& rs);
  void rldcl(const Reg64& ra, const Reg64& rs, const Reg64& rb, 
             uint8_t mb, bool rc);
  void rldcr(const Reg64& ra, const Reg64& rs,  const Reg64& rb, 
             uint8_t mb, bool rc);
  void rldic(const Reg64& ra, const Reg64& rs, uint8_t sh, 
             uint8_t mb, bool rc);
  void rldicl(const Reg64& ra, const Reg64& rs, uint8_t sh, 
              uint8_t mb, bool rc);
  void rldicr(const Reg64& ra, const Reg64& rs, uint8_t sh, 
              uint8_t mb, bool rc);
  void rldimi(const Reg64& ra, const Reg64& rs, uint8_t sh, 
              uint8_t mb, bool rc);
  void rlwimi(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, 
              uint16_t me, bool rc);
  void rlwinm(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, 
              uint16_t me, bool rc);
  void rlwnm(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, 
             uint16_t me, bool rc);
  void sc(uint16_t lev);
  void sld(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void slw(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void srad(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void sraw(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void srawi(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void srd(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void srw(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void stb(const Reg64& rt, MemoryRef m); //TODO(IBM): first parameter is rs
  void stbu(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void stbux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void stbx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void sth(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void sthu(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void sthux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void sthx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void stw(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void stwu(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void stwux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void stwx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void std(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void stdu(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void stdux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void stdx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void stq(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void sthbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void stwbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void stdbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void stmw(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void stswi(const Reg64& rt, const Reg64& ra, uint16_t nb);
  void stswx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void subf(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void subfo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void subfc(const Reg64& rt, const Reg64& ra, const Reg64& rb,  bool rc);
  void subfco(const Reg64& rt, const Reg64& ra, const Reg64& rb,  bool rc);
  void subfe(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void subfeo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void subfic(const Reg64& rt, const Reg64& ra,  uint16_t imm);
  void subfme(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void subfmeo(const Reg64& rt,  const Reg64& ra, const Reg64& rb, bool rc);
  void subfze(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void subfzeo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void td(uint16_t to, const Reg64& ra, const Reg64& rb);
  void tdi(uint16_t to, const Reg64& ra, uint16_t imm);
  void tw(uint16_t to, const Reg64& ra, const Reg64& rb);
  void twi(uint16_t to, const Reg64& ra, uint16_t imm);
  void xor_(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void xori(const Reg64& rs, const Reg64& ra, Immed imm);
  void xoris(const Reg64& rs, const Reg64& ra, uint16_t imm);

  //Unimplemented Instructions
  void addg6s()         { not_implemented(); }
  void bcdadd()         { not_implemented(); }
  void bcdsub()         { not_implemented(); }
  void brinc()          { not_implemented(); }
  void cbcdtd()         { not_implemented(); }
  void cdtbcd()         { not_implemented(); }
  void clrbhrb()        { not_implemented(); }
  void dadd()           { not_implemented(); }
  void daddq()          { not_implemented(); }
  void dcba()           { not_implemented(); }
  void dcbf()           { not_implemented(); }
  void dcbfep()         { not_implemented(); }
  void dcbi()           { not_implemented(); }
  void dcblc()          { not_implemented(); }
  void dcblq()          { not_implemented(); }
  void dcbst()          { not_implemented(); }
  void dcbstep()        { not_implemented(); }
  void dcbt()           { not_implemented(); }
  void dcbtep()         { not_implemented(); }
  void dcbtls()         { not_implemented(); }
  void dcbtst()         { not_implemented(); }
  void dcbtstep()       { not_implemented(); }
  void dcbtstls()       { not_implemented(); }
  void dcbz()           { not_implemented(); }
  void dcbzep()         { not_implemented(); }
  void dcffix()         { not_implemented(); }
  void dcffixq()        { not_implemented(); }
  void dci()            { not_implemented(); }
  void dcmpo()          { not_implemented(); }
  void dcmpoq()         { not_implemented(); }
  void dcmpu()          { not_implemented(); }
  void dcmpuq()         { not_implemented(); }
  void dcread()         { not_implemented(); }
  void dctdp()          { not_implemented(); }
  void dctfix()         { not_implemented(); }
  void dctfixq()        { not_implemented(); }
  void dctqpq()         { not_implemented(); }
  void ddedpd()         { not_implemented(); }
  void ddedpdq()        { not_implemented(); }
  void ddiv()           { not_implemented(); }
  void ddivq()          { not_implemented(); }
  void denbcd()         { not_implemented(); }
  void denbcdq()        { not_implemented(); }
  void diex()           { not_implemented(); }
  void diexq()          { not_implemented(); }
  void dlmzb()          { not_implemented(); }
  void dmul()           { not_implemented(); }
  void dmulq()          { not_implemented(); }
  void dnh()            { not_implemented(); }
  void doze()           { not_implemented(); }
  void dqua()           { not_implemented(); }
  void dquai()          { not_implemented(); }
  void dquaiq()         { not_implemented(); }
  void dquaq()          { not_implemented(); }
  void drdpq()          { not_implemented(); }
  void drintn()         { not_implemented(); }
  void drintnq()        { not_implemented(); }
  void drintx()         { not_implemented(); }
  void drintxq()        { not_implemented(); }
  void drrnd()          { not_implemented(); }
  void drrndq()         { not_implemented(); }
  void drsp()           { not_implemented(); }
  void dscli()          { not_implemented(); }
  void dscliq()         { not_implemented(); }
  void dscri()          { not_implemented(); }
  void dscriq()         { not_implemented(); }
  void dsn()            { not_implemented(); }
  void dsub()           { not_implemented(); }
  void dsubq()          { not_implemented(); }
  void dtstdc()         { not_implemented(); }
  void dtstdcq()        { not_implemented(); }
  void dtstdg()         { not_implemented(); }
  void dtstdgq()        { not_implemented(); }
  void dtstex()         { not_implemented(); }
  void dtstexq()        { not_implemented(); }
  void dtstsf()         { not_implemented(); }
  void dtstsfq()        { not_implemented(); }
  void dxex()           { not_implemented(); }
  void dxexq()          { not_implemented(); }
  void eciwx()          { not_implemented(); }
  void ecowx()          { not_implemented(); }
  void efdabs()         { not_implemented(); }
  void efdadd()         { not_implemented(); }
  void efdcfs()         { not_implemented(); }
  void efdcfsf()        { not_implemented(); }
  void efdcfsi()        { not_implemented(); }
  void efdcfsid()       { not_implemented(); }
  void efdcfuf()        { not_implemented(); }
  void efdcfui()        { not_implemented(); }
  void efdcfuid()       { not_implemented(); }
  void efdcmpeq()       { not_implemented(); }
  void efdcmpgt()       { not_implemented(); }
  void efdcmplt()       { not_implemented(); }
  void efdctsf()        { not_implemented(); }
  void efdctsi()        { not_implemented(); }
  void efdctsidz()      { not_implemented(); }
  void efdctsiz()       { not_implemented(); }
  void efdctuf()        { not_implemented(); }
  void efdctui()        { not_implemented(); }
  void efdctuidz()      { not_implemented(); }
  void efdctuiz()       { not_implemented(); }
  void efddiv()         { not_implemented(); }
  void efdmul()         { not_implemented(); }
  void efdnabs()        { not_implemented(); }
  void efdneg()         { not_implemented(); }
  void efdsub()         { not_implemented(); }
  void efdtsteq()       { not_implemented(); }
  void efdtstgt()       { not_implemented(); }
  void efdtstlt()       { not_implemented(); }
  void efsabs()         { not_implemented(); }
  void efsadd()         { not_implemented(); }
  void efscfd()         { not_implemented(); }
  void efscfsf()        { not_implemented(); }
  void efscfsi()        { not_implemented(); }
  void efscfuf()        { not_implemented(); }
  void efscfui()        { not_implemented(); }
  void efscmpeq()       { not_implemented(); }
  void efscmpgt()       { not_implemented(); }
  void efscmplt()       { not_implemented(); }
  void efsctsf()        { not_implemented(); }
  void efsctsi()        { not_implemented(); }
  void efsctsiz()       { not_implemented(); }
  void efsctuf()        { not_implemented(); }
  void efsctui()        { not_implemented(); }
  void efsctuiz()       { not_implemented(); }
  void efsdiv()         { not_implemented(); }
  void efsmul()         { not_implemented(); }
  void efsnabs()        { not_implemented(); }
  void efsneg()         { not_implemented(); }
  void efssub()         { not_implemented(); }
  void efststeq()       { not_implemented(); }
  void efststgt()       { not_implemented(); }
  void efststlt()       { not_implemented(); }
  void ehpriv()         { not_implemented(); }
  void eieio()          { not_implemented(); }
  void evabs()          { not_implemented(); }
  void evaddiw()        { not_implemented(); }
  void evaddsmiaaw()    { not_implemented(); }
  void evaddssiaaw()    { not_implemented(); }
  void evaddumiaaw()    { not_implemented(); }
  void evaddusiaaw()    { not_implemented(); }
  void evaddw()         { not_implemented(); }
  void evand()          { not_implemented(); }
  void evandc()         { not_implemented(); }
  void evcmpeq()        { not_implemented(); }
  void evcmpgts()       { not_implemented(); }
  void evcmpgtu()       { not_implemented(); }
  void evcmplts()       { not_implemented(); }
  void evcmpltu()       { not_implemented(); }
  void evcntlsw()       { not_implemented(); }
  void evcntlzw()       { not_implemented(); }
  void evdivws()        { not_implemented(); }
  void evdivwu()        { not_implemented(); }
  void eveqv()          { not_implemented(); }
  void evextsb()        { not_implemented(); }
  void evextsh()        { not_implemented(); }
  void evfsabs()        { not_implemented(); }
  void evfsadd()        { not_implemented(); }
  void evfscfsf()       { not_implemented(); }
  void evfscfsi()       { not_implemented(); }
  void evfscfuf()       { not_implemented(); }
  void evfscfui()       { not_implemented(); }
  void evfscmpeq()      { not_implemented(); }
  void evfscmpgt()      { not_implemented(); }
  void evfscmplt()      { not_implemented(); }
  void evfsctsf()       { not_implemented(); }
  void evfsctsi()       { not_implemented(); }
  void evfsctsiz()      { not_implemented(); }
  void evfsctuf()       { not_implemented(); }
  void evfsctui()       { not_implemented(); }
  void evfsctuiz()      { not_implemented(); }
  void evfsdiv()        { not_implemented(); }
  void evfsmul()        { not_implemented(); }
  void evfsnabs()       { not_implemented(); }
  void evfsneg()        { not_implemented(); }
  void evfssub()        { not_implemented(); }
  void evfststeq()      { not_implemented(); }
  void evfststgt()      { not_implemented(); }
  void evfststlt()      { not_implemented(); }
  void evldd()          { not_implemented(); }
  void evlddepx()       { not_implemented(); }
  void evlddx()         { not_implemented(); }
  void evldh()          { not_implemented(); }
  void evldhx()         { not_implemented(); }
  void evldw()          { not_implemented(); }
  void evldwx()         { not_implemented(); }
  void evlhhesplat()    { not_implemented(); }
  void evlhhesplatx()   { not_implemented(); }
  void evlhhossplat()   { not_implemented(); }
  void evlhhossplatx()  { not_implemented(); }
  void evlhhousplat()   { not_implemented(); }
  void evlhhousplatx()  { not_implemented(); }
  void evlwhe()         { not_implemented(); }
  void evlwhex()        { not_implemented(); }
  void evlwhos()        { not_implemented(); }
  void evlwhosx()       { not_implemented(); }
  void evlwhou()        { not_implemented(); }
  void evlwhoux()       { not_implemented(); }
  void evlwhsplat()     { not_implemented(); }
  void evlwhsplatx()    { not_implemented(); }
  void evlwwsplat()     { not_implemented(); }
  void evlwwsplatx()    { not_implemented(); }
  void evmergehi()      { not_implemented(); }
  void evmergehilo()    { not_implemented(); }
  void evmergelo()      { not_implemented(); }
  void evmergelohi()    { not_implemented(); }
  void evmhegsmfaa()    { not_implemented(); }
  void evmhegsmfan()    { not_implemented(); }
  void evmhegsmiaa()    { not_implemented(); }
  void evmhegsmian()    { not_implemented(); }
  void evmhegumiaa()    { not_implemented(); }
  void evmhegumian()    { not_implemented(); }
  void evmhesmf()       { not_implemented(); }
  void evmhesmfa()      { not_implemented(); }
  void evmhesmfaaw()    { not_implemented(); }
  void evmhesmfanw()    { not_implemented(); }
  void evmhesmi()       { not_implemented(); }
  void evmhesmia()      { not_implemented(); }
  void evmhesmiaaw()    { not_implemented(); }
  void evmhesmianw()    { not_implemented(); }
  void evmhessf()       { not_implemented(); }
  void evmhessfa()      { not_implemented(); }
  void evmhessfaaw()    { not_implemented(); }
  void evmhessfanw()    { not_implemented(); }
  void evmhessiaaw()    { not_implemented(); }
  void evmhessianw()    { not_implemented(); }
  void evmheumi()       { not_implemented(); }
  void evmheumia()      { not_implemented(); }
  void evmheumiaaw()    { not_implemented(); }
  void evmheumianw()    { not_implemented(); }
  void evmheusiaaw()    { not_implemented(); }
  void evmheusianw()    { not_implemented(); }
  void evmhogsmfaa()    { not_implemented(); }
  void evmhogsmfan()    { not_implemented(); }
  void evmhogsmiaa()    { not_implemented(); }
  void evmhogsmian()    { not_implemented(); }
  void evmhogumiaa()    { not_implemented(); }
  void evmhogumian()    { not_implemented(); }
  void evmhosmf()       { not_implemented(); }
  void evmhosmfa()      { not_implemented(); }
  void evmhosmfaaw()    { not_implemented(); }
  void evmhosmfanw()    { not_implemented(); }
  void evmhosmi()       { not_implemented(); }
  void evmhosmia()      { not_implemented(); }
  void evmhosmiaaw()    { not_implemented(); }
  void evmhosmianw()    { not_implemented(); }
  void evmhossf()       { not_implemented(); }
  void evmhossfa()      { not_implemented(); }
  void evmhossfaaw()    { not_implemented(); }
  void evmhossfanw()    { not_implemented(); }
  void evmhossiaaw()    { not_implemented(); }
  void evmhossianw()    { not_implemented(); }
  void evmhoumi()       { not_implemented(); }
  void evmhoumia()      { not_implemented(); }
  void evmhoumiaaw()    { not_implemented(); }
  void evmhoumianw()    { not_implemented(); }
  void evmhousiaaw()    { not_implemented(); }
  void evmhousianw()    { not_implemented(); }
  void evmra()          { not_implemented(); }
  void evmwhsmf()       { not_implemented(); }
  void evmwhsmfa()      { not_implemented(); }
  void evmwhsmi()       { not_implemented(); }
  void evmwhsmia()      { not_implemented(); }
  void evmwhssf()       { not_implemented(); }
  void evmwhssfa()      { not_implemented(); }
  void evmwhumi()       { not_implemented(); }
  void evmwhumia()      { not_implemented(); }
  void evmwlsmiaaw()    { not_implemented(); }
  void evmwlsmianw()    { not_implemented(); }
  void evmwlssiaaw()    { not_implemented(); }
  void evmwlssianw()    { not_implemented(); }
  void evmwlumi()       { not_implemented(); }
  void evmwlumia()      { not_implemented(); }
  void evmwlumiaaw()    { not_implemented(); }
  void evmwlumianw()    { not_implemented(); }
  void evmwlusiaaw()    { not_implemented(); }
  void evmwlusianw()    { not_implemented(); }
  void evmwsmf()        { not_implemented(); }
  void evmwsmfa()       { not_implemented(); }
  void evmwsmfaa()      { not_implemented(); }
  void evmwsmfan()      { not_implemented(); }
  void evmwsmi()        { not_implemented(); }
  void evmwsmia()       { not_implemented(); }
  void evmwsmiaa()      { not_implemented(); }
  void evmwsmian()      { not_implemented(); }
  void evmwssf()        { not_implemented(); }
  void evmwssfa()       { not_implemented(); }
  void evmwssfaa()      { not_implemented(); }
  void evmwssfan()      { not_implemented(); }
  void evmwumi()        { not_implemented(); }
  void evmwumia()       { not_implemented(); }
  void evmwumiaa()      { not_implemented(); }
  void evmwumian()      { not_implemented(); }
  void evnand()         { not_implemented(); }
  void evneg()          { not_implemented(); }
  void evnor()          { not_implemented(); }
  void evor()           { not_implemented(); }
  void evorc()          { not_implemented(); }
  void evrlw()          { not_implemented(); }
  void evrlwi()         { not_implemented(); }
  void evrndw()         { not_implemented(); }
  void evsel()          { not_implemented(); }
  void evslw()          { not_implemented(); }
  void evslwi()         { not_implemented(); }
  void evsplatfi()      { not_implemented(); }
  void evsplati()       { not_implemented(); }
  void evsrwis()        { not_implemented(); }
  void evsrwiu()        { not_implemented(); }
  void evsrws()         { not_implemented(); }
  void evsrwu()         { not_implemented(); }
  void evstdd()         { not_implemented(); }
  void evstddepx()      { not_implemented(); }
  void evstddx()        { not_implemented(); }
  void evstdh()         { not_implemented(); }
  void evstdhx()        { not_implemented(); }
  void evstdw()         { not_implemented(); }
  void evstdwx()        { not_implemented(); }
  void evstwhe()        { not_implemented(); }
  void evstwhex()       { not_implemented(); }
  void evstwho()        { not_implemented(); }
  void evstwhox()       { not_implemented(); }
  void evstwwe()        { not_implemented(); }
  void evstwwex()       { not_implemented(); }
  void evstwwo()        { not_implemented(); }
  void evstwwox()       { not_implemented(); }
  void evsubfsmiaaw()   { not_implemented(); }
  void evsubfssiaaw()   { not_implemented(); }
  void evsubfumiaaw()   { not_implemented(); }
  void evsubfusiaaw()   { not_implemented(); }
  void evsubfw()        { not_implemented(); }
  void evsubifw()       { not_implemented(); }
  void evxor()          { not_implemented(); }
  void fabs()           { not_implemented(); }
  void fadd(const Reg64& frt, const Reg64& fra, const Reg64& frb, bool rc);
  void fadds()          { not_implemented(); }
  void fcfid()          { not_implemented(); }
  void fcfids()         { not_implemented(); }
  void fcfidu()         { not_implemented(); }
  void fcfidus()        { not_implemented(); }
  void fcmpo()          { not_implemented(); }
  void fcmpu()          { not_implemented(); }
  void fcpsgn()         { not_implemented(); }
  void fctid()          { not_implemented(); }
  void fctidu()         { not_implemented(); }
  void fctiduz()        { not_implemented(); }
  void fctidz()         { not_implemented(); }
  void fctiw()          { not_implemented(); }
  void fctiwu()         { not_implemented(); }
  void fctiwuz()        { not_implemented(); }
  void fctiwz()         { not_implemented(); }
  void fdiv()           { not_implemented(); }
  void fdivs()          { not_implemented(); }
  void fmadd()          { not_implemented(); }
  void fmadds()         { not_implemented(); }
  void fmr()            { not_implemented(); }
  void fmrgew()         { not_implemented(); }
  void fmrgow()         { not_implemented(); }
  void fmsub()          { not_implemented(); }
  void fmsubs()         { not_implemented(); }
  void fmul()           { not_implemented(); }
  void fmuls()          { not_implemented(); }
  void fnabs()          { not_implemented(); }
  void fneg()           { not_implemented(); }
  void fnmadd()         { not_implemented(); }
  void fnmadds()        { not_implemented(); }
  void fnmsub()         { not_implemented(); }
  void fnmsubs()        { not_implemented(); }
  void fre()            { not_implemented(); }
  void fres()           { not_implemented(); }
  void frim()           { not_implemented(); }
  void frin()           { not_implemented(); }
  void frip()           { not_implemented(); }
  void friz()           { not_implemented(); }
  void frsp()           { not_implemented(); }
  void frsqrte()        { not_implemented(); }
  void fsel()           { not_implemented(); }
  void fsqrt()          { not_implemented(); }
  void fsqrts()         { not_implemented(); }
  void fsub()           { not_implemented(); }
  void fsubs()          { not_implemented(); }
  void ftdiv()          { not_implemented(); }
  void ftsqrt()         { not_implemented(); }
  void hrfid()          { not_implemented(); }
  void icbi()           { not_implemented(); }
  void icbiep()         { not_implemented(); }
  void icblc()          { not_implemented(); }
  void icblq()          { not_implemented(); }
  void icbt()           { not_implemented(); }
  void icbtls()         { not_implemented(); }
  void ici()            { not_implemented(); }
  void icread()         { not_implemented(); }
  void isync()          { not_implemented(); }
  void lbarx()          { not_implemented(); }
  void lbdx()           { not_implemented(); }
  void lbepx()          { not_implemented(); }
  void lbzcix()         { not_implemented(); }
  void ldarx()          { not_implemented(); }
  void ldcix()          { not_implemented(); }
  void lddx()           { not_implemented(); }
  void ldepx()          { not_implemented(); }
  void lfd()            { not_implemented(); }
  void lfddx()          { not_implemented(); }
  void lfdepx()         { not_implemented(); }
  void lfdp()           { not_implemented(); }
  void lfdpx()          { not_implemented(); }
  void lfdu()           { not_implemented(); }
  void lfdux()          { not_implemented(); }
  void lfdx()           { not_implemented(); }
  void lfiwax()         { not_implemented(); }
  void lfiwzx()         { not_implemented(); }
  void lfs()            { not_implemented(); }
  void lfsu()           { not_implemented(); }
  void lfsux()          { not_implemented(); }
  void lfsx()           { not_implemented(); }
  void lharx()          { not_implemented(); }
  void lvebx()          { not_implemented(); }
  void lvehx()          { not_implemented(); }
  void lvepx()          { not_implemented(); }
  void lvepxl()         { not_implemented(); }
  void lvewx()          { not_implemented(); }
  void lvsl()           { not_implemented(); }
  void lvsr()           { not_implemented(); }
  void lvx()            { not_implemented(); }
  void lvxl()           { not_implemented(); }
  void lwarx()          { not_implemented(); }
  void lwdx()           { not_implemented(); }
  void lwepx()          { not_implemented(); }
  void lwzcix()         { not_implemented(); }
  void lhdx()           { not_implemented(); }
  void lhepx()          { not_implemented(); }
  void lhzcix()         { not_implemented(); }
  void lqarx()          { not_implemented(); }
  void lxsdx()          { not_implemented(); }
  void lxsiwax()        { not_implemented(); }
  void lxsiwzx()        { not_implemented(); }
  void lxsspx()         { not_implemented(); }
  void lxvdsx()         { not_implemented(); }
  void lxvdx()          { not_implemented(); }
  void lxvwx()          { not_implemented(); }
  void macchw()         { not_implemented(); }
  void macchwo()        { not_implemented(); }
  void macchws()        { not_implemented(); }
  void macchwso()       { not_implemented(); }
  void macchwsu()       { not_implemented(); }
  void macchwsuo()      { not_implemented(); }
  void macchwu()        { not_implemented(); }
  void macchwuo()       { not_implemented(); }
  void machhw()         { not_implemented(); }
  void machhwo()        { not_implemented(); }
  void machhws()        { not_implemented(); }
  void machhwso()       { not_implemented(); }
  void machhwsu()       { not_implemented(); }
  void machhwsuo()      { not_implemented(); }
  void machhwu()        { not_implemented(); }
  void machhwuo()       { not_implemented(); }
  void maclhw()         { not_implemented(); }
  void maclhwo()        { not_implemented(); }
  void maclhws()        { not_implemented(); }
  void maclhwso()       { not_implemented(); }
  void maclhwsu()       { not_implemented(); }
  void maclhwsuo()      { not_implemented(); }
  void maclhwu()        { not_implemented(); }
  void maclhwuo()       { not_implemented(); }
  void mbar()           { not_implemented(); }
  void mcrfs()          { not_implemented(); }
  void mcrxr()          { not_implemented(); }
  void mfbhrbe()        { not_implemented(); }
  void mfcr()           { not_implemented(); }
  void mfdcr()          { not_implemented(); }
  void mfdcrux ()       { not_implemented(); }
  void mfdcrx()         { not_implemented(); }
  void mffs()           { not_implemented(); }
  void mfocrf()         { not_implemented(); }
  void mfpmr ()         { not_implemented(); }
  void mfsr()           { not_implemented(); }
  void mfsrin()         { not_implemented(); }
  void mftbmfvscr()     { not_implemented(); }
  void mfvsrd()         { not_implemented(); }
  void mfvsrwz()        { not_implemented(); }
  void msgclr()         { not_implemented(); }
  void msgclrp()        { not_implemented(); }
  void msgsnd()         { not_implemented(); }
  void msgsndp()        { not_implemented(); }
  void mtcrf()          { not_implemented(); }
  void mtdcr()          { not_implemented(); }
  void mtdcrux()        { not_implemented(); }
  void mtdcrx ()        { not_implemented(); }
  void mtfsb0()         { not_implemented(); }
  void mtfsb1()         { not_implemented(); }
  void mtfsf()          { not_implemented(); }
  void mtfsfi()         { not_implemented(); }
  void mtsr()           { not_implemented(); }
  void mtsrin()         { not_implemented(); }
  void mtvscr()         { not_implemented(); }
  void mtvsrd()         { not_implemented(); }
  void mtvsrwa()        { not_implemented(); }
  void mtvsrwz()        { not_implemented(); }
  void mulchw()         { not_implemented(); }
  void mulchwu()        { not_implemented(); }
  void mulhhw()         { not_implemented(); }
  void mulhhwu()        { not_implemented(); }
  void mullhw()         { not_implemented(); }
  void mullhwu()        { not_implemented(); }
  void nap()            { not_implemented(); }
  void nmacchw()        { not_implemented(); }
  void nmacchwo()       { not_implemented(); }
  void nmacchws()       { not_implemented(); }
  void nmacchwso()      { not_implemented(); }
  void nmachhw()        { not_implemented(); }
  void nmachhwo()       { not_implemented(); }
  void nmachhws()       { not_implemented(); }
  void nmachhwso()      { not_implemented(); }
  void nmaclhw()        { not_implemented(); }
  void nmaclhwo()       { not_implemented(); }
  void nmaclhws()       { not_implemented(); }
  void nmaclhwso()      { not_implemented(); }
  void rfci()           { not_implemented(); }
  void rfdi()           { not_implemented(); }
  void rfebb()          { not_implemented(); }
  void rfgi()           { not_implemented(); }
  void rfi()            { not_implemented(); }
  void rfid()           { not_implemented(); }
  void rfmci()          { not_implemented(); }
  void rvwinkle()       { not_implemented(); }
  void rvwinklesc()     { not_implemented(); }
  void slbfee()         { not_implemented(); }
  void slbia()          { not_implemented(); }
  void slbie()          { not_implemented(); }
  void slbmfee()        { not_implemented(); }
  void slbmfev()        { not_implemented(); }
  void slbmte()         { not_implemented(); }
  void sleep()          { not_implemented(); }
  void sradi()          { not_implemented(); }
  void stbcix()         { not_implemented(); }
  void stbcx()          { not_implemented(); }
  void stbdx()          { not_implemented(); }
  void stbepx()         { not_implemented(); }
  void stdcix()         { not_implemented(); }
  void stdcx()          { not_implemented(); }
  void stddx()          { not_implemented(); }
  void stdepx()         { not_implemented(); }
  void stfd()           { not_implemented(); }
  void stfddx()         { not_implemented(); }
  void stfdepx()        { not_implemented(); }
  void stfdp()          { not_implemented(); }
  void stfdpx()         { not_implemented(); }
  void stfdu()          { not_implemented(); }
  void stfdux()         { not_implemented(); }
  void stfdx()          { not_implemented(); }
  void stfiwx()         { not_implemented(); }
  void stfs()           { not_implemented(); }
  void stfsu()          { not_implemented(); }
  void stfsux()         { not_implemented(); }
  void stfsx()          { not_implemented(); }
  void sthcix()         { not_implemented(); }
  void sthcx()          { not_implemented(); }
  void sthdx()          { not_implemented(); }
  void sthepx()         { not_implemented(); }
  void stqcx()          { not_implemented(); }
  void stvebx()         { not_implemented(); }
  void stvehx()         { not_implemented(); }
  void stvepx()         { not_implemented(); }
  void stvepxl()        { not_implemented(); }
  void stvewx()         { not_implemented(); }
  void stvx()           { not_implemented(); }
  void stvxl()          { not_implemented(); }
  void stwcix()         { not_implemented(); }
  void stwcx()          { not_implemented(); }
  void stwdx()          { not_implemented(); }
  void stwepx()         { not_implemented(); }
  void stxsdx()         { not_implemented(); }
  void stxsiwx()        { not_implemented(); }
  void stxsspx()        { not_implemented(); }
  void stxvd2x()        { not_implemented(); }
  void stxvw4x()        { not_implemented(); }
  void sync()           { not_implemented(); }
  void tabort()         { not_implemented(); }
  void tabortdc()       { not_implemented(); }
  void tabortdci()      { not_implemented(); }
  void tabortwc()       { not_implemented(); }
  void tabortwci()      { not_implemented(); }
  void tbegin()         { not_implemented(); }
  void tcheck()         { not_implemented(); }
  void tend()           { not_implemented(); }
  void tlbia()          { not_implemented(); }
  void tlbie()          { not_implemented(); }
  void tlbiel()         { not_implemented(); }
  void tlbilx()         { not_implemented(); }
  void tlbivax()        { not_implemented(); }
  void tlbre()          { not_implemented(); }
  void tlbsrx()         { not_implemented(); }
  void tlbsx()          { not_implemented(); }
  void tlbsync()        { not_implemented(); }
  void tlbwe()          { not_implemented(); }
  void trechkpt()       { not_implemented(); }
  void treclaim()       { not_implemented(); }
  void tsr()            { not_implemented(); }
  void vaddcuq()        { not_implemented(); }
  void vaddcuw()        { not_implemented(); }
  void vaddecuq()       { not_implemented(); }
  void vaddeuqm()       { not_implemented(); }
  void vaddfp()         { not_implemented(); }
  void vaddsbs()        { not_implemented(); }
  void vaddshs()        { not_implemented(); }
  void vaddsws()        { not_implemented(); }
  void vaddubm()        { not_implemented(); }
  void vaddubs()        { not_implemented(); }
  void vaddudm()        { not_implemented(); }
  void vadduhm()        { not_implemented(); }
  void vadduhs()        { not_implemented(); }
  void vadduqm()        { not_implemented(); }
  void vadduwm()        { not_implemented(); }
  void vadduws()        { not_implemented(); }
  void vand()           { not_implemented(); }
  void vandc()          { not_implemented(); }
  void vavgsb()         { not_implemented(); }
  void vavgsh()         { not_implemented(); }
  void vavgsw()         { not_implemented(); }
  void vavgub()         { not_implemented(); }
  void vavguh()         { not_implemented(); }
  void vavguw()         { not_implemented(); }
  void vbpermq()        { not_implemented(); }
  void vcfsx()          { not_implemented(); }
  void vcfux()          { not_implemented(); }
  void vcipher()        { not_implemented(); }
  void vcipherlast()    { not_implemented(); }
  void vclzb()          { not_implemented(); }
  void vclzd()          { not_implemented(); }
  void vclzh()          { not_implemented(); }
  void vclzw()          { not_implemented(); }
  void vcmpbfp()        { not_implemented(); }
  void vcmpeqfp()       { not_implemented(); }
  void vcmpequb()       { not_implemented(); }
  void vcmpequd()       { not_implemented(); }
  void vcmpequh()       { not_implemented(); }
  void vcmpequw()       { not_implemented(); }
  void vcmpgefp()       { not_implemented(); }
  void vcmpgtfp()       { not_implemented(); }
  void vcmpgtsb()       { not_implemented(); }
  void vcmpgtsd()       { not_implemented(); }
  void vcmpgtsh()       { not_implemented(); }
  void vcmpgtsw()       { not_implemented(); }
  void vcmpgtub()       { not_implemented(); }
  void vcmpgtud()       { not_implemented(); }
  void vcmpgtuh()       { not_implemented(); }
  void vcmpgtuw()       { not_implemented(); }
  void vctsxs()         { not_implemented(); }
  void vctuxs()         { not_implemented(); }
  void veqv()           { not_implemented(); }
  void vexptefp()       { not_implemented(); }
  void vgbbd()          { not_implemented(); }
  void vlogefp()        { not_implemented(); }
  void vmaddfp()        { not_implemented(); }
  void vmaxfp()         { not_implemented(); }
  void vmaxsb()         { not_implemented(); }
  void vmaxsd()         { not_implemented(); }
  void vmaxsh()         { not_implemented(); }
  void vmaxsw()         { not_implemented(); }
  void vmaxub()         { not_implemented(); }
  void vmaxud()         { not_implemented(); }
  void vmaxuh()         { not_implemented(); }
  void vmaxuw()         { not_implemented(); }
  void vmhaddshs()      { not_implemented(); }
  void vmhraddshs()     { not_implemented(); }
  void vminfp()         { not_implemented(); }
  void vminsb()         { not_implemented(); }
  void vminsd()         { not_implemented(); }
  void vminsh()         { not_implemented(); }
  void vminsw()         { not_implemented(); }
  void vminub()         { not_implemented(); }
  void vminud()         { not_implemented(); }
  void vminuh()         { not_implemented(); }
  void vminuw()         { not_implemented(); }
  void vmladduhm()      { not_implemented(); }
  void vmrgew()         { not_implemented(); }
  void vmrghb()         { not_implemented(); }
  void vmrghh()         { not_implemented(); }
  void vmrghw()         { not_implemented(); }
  void vmrglb()         { not_implemented(); }
  void vmrglh()         { not_implemented(); }
  void vmrglw()         { not_implemented(); }
  void vmrgow()         { not_implemented(); }
  void vmsummbm()       { not_implemented(); }
  void vmsumshm()       { not_implemented(); }
  void vmsumshs()       { not_implemented(); }
  void vmsumubm()       { not_implemented(); }
  void vmsumuhm()       { not_implemented(); }
  void vmsumuhs()       { not_implemented(); }
  void vmulesb()        { not_implemented(); }
  void vmulesh()        { not_implemented(); }
  void vmulesw()        { not_implemented(); }
  void vmuleub()        { not_implemented(); }
  void vmuleuh()        { not_implemented(); }
  void vmuleuw()        { not_implemented(); }
  void vmulosb()        { not_implemented(); }
  void vmulosh()        { not_implemented(); }
  void vmulosw()        { not_implemented(); }
  void vmuloub()        { not_implemented(); }
  void vmulouh()        { not_implemented(); }
  void vmulouw()        { not_implemented(); }
  void vmuluwm()        { not_implemented(); }
  void vnand()          { not_implemented(); }
  void vncipher()       { not_implemented(); }
  void vncipherlast()   { not_implemented(); }
  void vnmsubfp()       { not_implemented(); }
  void vnor()           { not_implemented(); }
  void vor()            { not_implemented(); }
  void vorc()           { not_implemented(); }
  void vperm()          { not_implemented(); }
  void vpermxor()       { not_implemented(); }
  void vpkpx()          { not_implemented(); }
  void vpksdss()        { not_implemented(); }
  void vpksdus()        { not_implemented(); }
  void vpkshss()        { not_implemented(); }
  void vpkshus()        { not_implemented(); }
  void vpkswss()        { not_implemented(); }
  void vpkswus()        { not_implemented(); }
  void vpkudum()        { not_implemented(); }
  void vpkudus()        { not_implemented(); }
  void vpkuhum()        { not_implemented(); }
  void vpkuhus()        { not_implemented(); }
  void vpkuwum()        { not_implemented(); }
  void vpkuwus()        { not_implemented(); }
  void vpmsumb()        { not_implemented(); }
  void vpmsumd()        { not_implemented(); }
  void vpmsumh()        { not_implemented(); }
  void vpmsumw()        { not_implemented(); }
  void vpopcntb()       { not_implemented(); }
  void vpopcntd()       { not_implemented(); }
  void vpopcnth()       { not_implemented(); }
  void vpopcntw()       { not_implemented(); }
  void vrefp()          { not_implemented(); }
  void vrfim()          { not_implemented(); }
  void vrfin()          { not_implemented(); }
  void vrfip()          { not_implemented(); }
  void vrfiz()          { not_implemented(); }
  void vrlb()           { not_implemented(); }
  void vrld()           { not_implemented(); }
  void vrlh()           { not_implemented(); }
  void vrlw()           { not_implemented(); }
  void vrsqrtefp()      { not_implemented(); }
  void vsbox()          { not_implemented(); }
  void vsel()           { not_implemented(); }
  void vshasigmad()     { not_implemented(); }
  void vshasigmaw()     { not_implemented(); }
  void vsl()            { not_implemented(); }
  void vslb()           { not_implemented(); }
  void vsld()           { not_implemented(); }
  void vsldoi()         { not_implemented(); }
  void vslh()           { not_implemented(); }
  void vslo()           { not_implemented(); }
  void vslw()           { not_implemented(); }
  void vspltb()         { not_implemented(); }
  void vsplth()         { not_implemented(); }
  void vspltisb()       { not_implemented(); }
  void vspltish()       { not_implemented(); }
  void vspltisw()       { not_implemented(); }
  void vspltw()         { not_implemented(); }
  void vsr()            { not_implemented(); }
  void vsrab()          { not_implemented(); }
  void vsrad()          { not_implemented(); }
  void vsrah()          { not_implemented(); }
  void vsraw()          { not_implemented(); }
  void vsrb()           { not_implemented(); }
  void vsrd()           { not_implemented(); }
  void vsrh()           { not_implemented(); }
  void vsro()           { not_implemented(); }
  void vsrw()           { not_implemented(); }
  void vsubcuq()        { not_implemented(); }
  void vsubcuw()        { not_implemented(); }
  void vsubecuq()       { not_implemented(); }
  void vsubeuqm()       { not_implemented(); }
  void vsubfp()         { not_implemented(); }
  void vsubsbs()        { not_implemented(); }
  void vsubshs()        { not_implemented(); }
  void vsubsws()        { not_implemented(); }
  void vsububm()        { not_implemented(); }
  void vsububs()        { not_implemented(); }
  void vsubudm()        { not_implemented(); }
  void vsubuhm()        { not_implemented(); }
  void vsubuhs()        { not_implemented(); }
  void vsubuqm()        { not_implemented(); }
  void vsubuwm()        { not_implemented(); }
  void vsubuws()        { not_implemented(); }
  void vsum2sws()       { not_implemented(); }
  void vsum4sbs()       { not_implemented(); }
  void vsum4shs()       { not_implemented(); }
  void vsum4ubs()       { not_implemented(); }
  void vsumsws()        { not_implemented(); }
  void vupkhpx()        { not_implemented(); }
  void vupkhsb()        { not_implemented(); }
  void vupkhsh()        { not_implemented(); }
  void vupkhsw()        { not_implemented(); }
  void vupklpx()        { not_implemented(); }
  void vupklsb()        { not_implemented(); }
  void vupklsh()        { not_implemented(); }
  void vupklsw()        { not_implemented(); }
  void vxor()           { not_implemented(); }
  void wait()           { not_implemented(); }
  void wrtee()          { not_implemented(); }
  void wrteei()         { not_implemented(); }
  void xsabsdp()        { not_implemented(); }
  void xsadddp()        { not_implemented(); }
  void xsaddsp()        { not_implemented(); }
  void xscmpodp()       { not_implemented(); }
  void xscmpudp()       { not_implemented(); }
  void xscpsgndp()      { not_implemented(); }
  void xscvdpsp()       { not_implemented(); }
  void xscvdpspn()      { not_implemented(); }
  void xscvdpsxds()     { not_implemented(); }
  void xscvdpsxws()     { not_implemented(); }
  void xscvdpuxds()     { not_implemented(); }
  void xscvdpuxws()     { not_implemented(); }
  void xscvspdp()       { not_implemented(); }
  void xscvspdpn()      { not_implemented(); }
  void xscvsxddp()      { not_implemented(); }
  void xscvsxdsp()      { not_implemented(); }
  void xscvuxddp()      { not_implemented(); }
  void xscvuxdsp()      { not_implemented(); }
  void xsdivdp()        { not_implemented(); }
  void xsdivsp()        { not_implemented(); }
  void xsmaddadp()      { not_implemented(); }
  void xsmaddasp()      { not_implemented(); }
  void xsmaddmdp()      { not_implemented(); }
  void xsmaddmsp()      { not_implemented(); }
  void xsmaxdp()        { not_implemented(); }
  void xsmindp()        { not_implemented(); }
  void xsmsubadp()      { not_implemented(); }
  void xsmsubasp()      { not_implemented(); }
  void xsmsubmdp()      { not_implemented(); }
  void xsmsubmsp()      { not_implemented(); }
  void xsmuldp()        { not_implemented(); }
  void xsmulsp()        { not_implemented(); }
  void xsnabsdp()       { not_implemented(); }
  void xsnegdp()        { not_implemented(); }
  void xsnmaddadp()     { not_implemented(); }
  void xsnmaddasp()     { not_implemented(); }
  void xsnmaddmdp()     { not_implemented(); }
  void xsnmaddmsp()     { not_implemented(); }
  void xsnmsubadp()     { not_implemented(); }
  void xsnmsubasp()     { not_implemented(); }
  void xsnmsubmdp()     { not_implemented(); }
  void xsnmsubmsp()     { not_implemented(); }
  void xsrdpi()         { not_implemented(); }
  void xsrdpic()        { not_implemented(); }
  void xsrdpim()        { not_implemented(); }
  void xsrdpip()        { not_implemented(); }
  void xsrdpiz()        { not_implemented(); }
  void xsredp()         { not_implemented(); }
  void xsresp()         { not_implemented(); }
  void xsrsp()          { not_implemented(); }
  void xsrsqrtedp()     { not_implemented(); }
  void xsrsqrtesp()     { not_implemented(); }
  void xssqrtdp()       { not_implemented(); }
  void xssqrtsp()       { not_implemented(); }
  void xssubdp()        { not_implemented(); }
  void xssubsp()        { not_implemented(); }
  void xstdivdp()       { not_implemented(); }
  void xstsqrtdp()      { not_implemented(); }
  void xvabsdp()        { not_implemented(); }
  void xvabssp()        { not_implemented(); }
  void xvadddp()        { not_implemented(); }
  void xvaddsp()        { not_implemented(); }
  void xvcmpeqdp()      { not_implemented(); }
  void xvcmpeqsp()      { not_implemented(); }
  void xvcmpgedp()      { not_implemented(); }
  void xvcmpgesp()      { not_implemented(); }
  void xvcmpgtdp()      { not_implemented(); }
  void xvcmpgtsp()      { not_implemented(); }
  void xvcpsgndp()      { not_implemented(); }
  void xvcpsgnsp()      { not_implemented(); }
  void xvcvdpsp()       { not_implemented(); }
  void xvcvdpsxds()     { not_implemented(); }
  void xvcvdpsxws()     { not_implemented(); }
  void xvcvdpuxds()     { not_implemented(); }
  void xvcvdpuxws()     { not_implemented(); }
  void xvcvspdp()       { not_implemented(); }
  void xvcvspsxds()     { not_implemented(); }
  void xvcvspsxws()     { not_implemented(); }
  void xvcvspuxds()     { not_implemented(); }
  void xvcvspuxws()     { not_implemented(); }
  void xvcvsxddp()      { not_implemented(); }
  void xvcvsxdsp()      { not_implemented(); }
  void xvcvsxwdp()      { not_implemented(); }
  void xvcvsxwsp()      { not_implemented(); }
  void xvcvuxddp()      { not_implemented(); }
  void xvcvuxdsp()      { not_implemented(); }
  void xvcvuxwdp()      { not_implemented(); }
  void xvcvuxwsp()      { not_implemented(); }
  void xvdivdp()        { not_implemented(); }
  void xvdivsp()        { not_implemented(); }
  void xvmaddadp()      { not_implemented(); }
  void xvmaddasp()      { not_implemented(); }
  void xvmaddmdp()      { not_implemented(); }
  void xvmaddmsp()      { not_implemented(); }
  void xvmaxdp()        { not_implemented(); }
  void xvmaxsp()        { not_implemented(); }
  void xvmindp()        { not_implemented(); }
  void xvminsp()        { not_implemented(); }
  void xvmsubadp()      { not_implemented(); }
  void xvmsubasp()      { not_implemented(); }
  void xvmsubmdp()      { not_implemented(); }
  void xvmsubmsp()      { not_implemented(); }
  void xvmuldp()        { not_implemented(); }
  void xvmulsp()        { not_implemented(); }
  void xvnabsdp()       { not_implemented(); }
  void xvnabssp()       { not_implemented(); }
  void xvnegdp()        { not_implemented(); }
  void xvnegsp()        { not_implemented(); }
  void xvnmaddadp()     { not_implemented(); }
  void xvnmaddasp()     { not_implemented(); }
  void xvnmaddmdp()     { not_implemented(); }
  void xvnmaddmsp()     { not_implemented(); }
  void xvnmsubadp()     { not_implemented(); }
  void xvnmsubasp()     { not_implemented(); }
  void xvnmsubmdp()     { not_implemented(); }
  void xvnmsubmsp()     { not_implemented(); }
  void xvrdpi()         { not_implemented(); }
  void xvrdpic()        { not_implemented(); }
  void xvrdpim()        { not_implemented(); }
  void xvrdpip()        { not_implemented(); }
  void xvrdpiz()        { not_implemented(); }
  void xvredp()         { not_implemented(); }
  void xvresp()         { not_implemented(); }
  void xvrspi()         { not_implemented(); }
  void xvrspic()        { not_implemented(); }
  void xvrspim()        { not_implemented(); }
  void xvrspip()        { not_implemented(); }
  void xvrspiz()        { not_implemented(); }
  void xvrsqrtedp()     { not_implemented(); }
  void xvrsqrtesp()     { not_implemented(); }
  void xvsqrtdp()       { not_implemented(); }
  void xvsqrtsp()       { not_implemented(); }
  void xvsubdp()        { not_implemented(); }
  void xvsubsp()        { not_implemented(); }
  void xvtdivdp()       { not_implemented(); }
  void xvtdivsp()       { not_implemented(); }
  void xvtsqrtdp()      { not_implemented(); }
  void xvtsqrtsp()      { not_implemented(); }
  void xxland()         { not_implemented(); }
  void xxlandc()        { not_implemented(); }
  void xxleqv()         { not_implemented(); }
  void xxlnand()        { not_implemented(); }
  void xxlnor()         { not_implemented(); }
  void xxlor()          { not_implemented(); }
  void xxlorc()         { not_implemented(); }
  void xxlxor()         { not_implemented(); }
  void xxmrghw()        { not_implemented(); }
  void xxmrglw()        { not_implemented(); }
  void xxpermdi()       { not_implemented(); }
  void xxsel()          { not_implemented(); }
  void xxsldwi()        { not_implemented(); }
  void xxspltw()        { not_implemented(); }

  //Extended/Synthetic PPC64 Instructions
  void blt()            { not_implemented(); }  //Extended bc 12,0,target
  void bne()            { not_implemented(); }  //Extended bc 4,10,target
  void bdnz()           { not_implemented(); }  //Extended bc 16,0,target
  void bltlr()          { not_implemented(); }  //Extended bclr 12,0,0
  void bnelr()          { not_implemented(); }  //Extended bclr 4,10,0
  void bdnzlr()         { not_implemented(); }  //Extended bclr 16,0,0
  void bltctr()         { not_implemented(); }  //Extended bcctr 12,0,0
  void bnectr()         { not_implemented(); }  //Extended bcctr 4,10,0
  void crmov()          { not_implemented(); }  //Extended cror Bx,By,By
  void crclr()          { not_implemented(); }  //Extended crxor Bx,Bx,BX
  void crnot()          { not_implemented(); }  //Extended crnor Bx,By,By
  void crset()          { not_implemented(); }  //Extended creqv Bx,Bx,Bx
  void li(const Reg64& rt, Immed imm) {
    addi(rt, Reg64(0), imm);
  }
  void la()             { not_implemented(); }  //Extended addi Rx,Ry,disp
  void subi()           { not_implemented(); }  //Extended addi Rx,Ry,-value
  void lis()            { not_implemented(); }  //Extended addis Rx,0,value
  void subis()          { not_implemented(); }  //Extended addis Rx,Ry,-value
  void sub()            { not_implemented(); }  //Extended subf Rx,Rz,Ry
  void subic()          { not_implemented(); }  //Extended addic Rx,Ry,-value
  void subc()           { not_implemented(); }  //Extended subfc Rx,Rz,Ry
  void cmpdi()          { not_implemented(); }  //Extended cmpi 0,1,Rx,value
  void cmpwi()          { not_implemented(); }  //Extended cmpi 3,0,Rx,value
  void cmpd(const Reg64& ra, const Reg64& rb) {
    cmp(0, 1, ra, rb);
  }
  void cmpw ()          { not_implemented(); }  //Extended cmp 3,0,Rx,Ry
  void cmpldi()         { not_implemented(); }  //Extended cmpli 0,1,Rx,value
  void cmplwi()         { not_implemented(); }  //Extended cmpli 3,0,Rx,value
  void cmpld()          { not_implemented(); }  //Extended cmpl 0,1,Rx,Ry
  void cmplw()          { not_implemented(); }  //Extended cmpl 3,0,Rx,Ry
  void twgti()          { not_implemented(); }  //Extended twi 8,Rx,value
  void twllei()         { not_implemented(); }  //Extended twi 6,Rx,value
  void tweq()           { not_implemented(); }  //Extended tw 4,Rx,Ry
  void twlge()          { not_implemented(); }  //Extended tw 5,Rx,Ry
  void trap()           { not_implemented(); }  //Extended tw 31,0,0
  void tdlti()          { not_implemented(); }  //Extended tdi 16,Rx,value
  void tdnei()          { not_implemented(); }  //Extended tdi 24,Rx,value
  void tdeg()           { not_implemented(); }  //Extended td 12,Rx,Ry
  void isellt()         { not_implemented(); }  //Extended isel Rx,Ry,Rz,0
  void iselgt()         { not_implemented(); }  //Extended isel Rx,Ry,Rz,1
  void iseleq()         { not_implemented(); }  //Extended isel Rx,Ry,Rz,1
  void no_op()          { not_implemented(); }  //Extended
  void andis()          { not_implemented(); }  //Extended
  void xnop()           { not_implemented(); }  //Extended
  void mr()             { not_implemented(); }  //Extended
  void not_()           { not_implemented(); }  //Extended
  void srwi()           { not_implemented(); }  //Extended
  void clrwi()          { not_implemented(); }  //Extended
  void extwi()          { not_implemented(); }  //Extended
  void rotlw()          { not_implemented(); }  //Extended
  void inslwi()         { not_implemented(); }  //Extended
  void extrdi()         { not_implemented(); }  //Extended
  void srdi()           { not_implemented(); }  //Extended
  void clrldi()         { not_implemented(); }  //Extended
  void extldi()         { not_implemented(); }  //Extended
  inline void sldi(const Reg64& ra, const Reg64& rs, uint8_t sh) {
    rldicr(ra, rs, sh, 63-sh);
  }
  void clrrdi()         { not_implemented(); }  //Extended
  void clrlsldi()       { not_implemented(); }  //Extended
  void rotld()          { not_implemented(); }  //Extended
  void insrdi()         { not_implemented(); }  //Extended
  void mtcr()           { not_implemented(); }  //Extended
  void mtctr(const Reg64& rx) {
    mtspr(SpecialReg::CTR, rx);
  }
  
  void b(Label&);
  void ba(Label&);
  void bc (Label&, BranchConditions);
  void bca(Label&, BranchConditions);
  void branchAuto(Label& l, Reg64 tmp, BranchConditions bc);

  //Can be used to generate or force a unimplemented opcode exception
  void unimplemented();

  static void patchBc(CodeAddress jmp, CodeAddress dest) {
    // Opcode located at the first 6 bits
    assert(((jmp[3] >> 2) & 0x3F) == 16);
    ssize_t diff = dest - (jmp + 4);  // skip instruction
    int16_t* BD = (int16_t*)(jmp + 2);

    // Keep AA and LK values
    *BD = HPHP::safe_cast<int16_t>(diff & 0xFFFC) | ((*BD) & 0x3);
  }

  static void patchBctr(CodeAddress jmp, CodeAddress dest) {
    // Check Label::branchAuto for details
    ssize_t diff = dest - (jmp + 6*4);  // skip instructions

    int16_t *imm = (int16_t*)(jmp + 2); // immediate field of addi
    *imm = HPHP::safe_cast<int16_t>((diff & (ssize_t(UINT16_MAX) << 32)) >> 32);

    imm += 2*4;                         // skip sldi instruction
    *imm = HPHP::safe_cast<int16_t>((diff & (ssize_t(UINT16_MAX) << 16)) >> 16);

    imm += 4;                           // next instruction
    *imm = HPHP::safe_cast<int16_t>(diff & ssize_t(UINT16_MAX));
  }

  // Secure high level instruction emitter
  void Emit(PPC64Instr instruction){
    assert(codeBlock.canEmit(sizeof(instruction)));
    assert(sizeof(instruction) == sizeof(uint32_t));
    dword(instruction);
  }

protected:

  // type instruction emitters
  // TODO(IBM): try remove cast for uint32_t
  // TODO(IBM): make those functions inline
  void EmitXOForm(const uint8_t op,
                  const RegNumber rt,
                  const RegNumber ra,
                  const RegNumber rb,
                  const bool oe,
                  const uint16_t xop,
                  const bool rc = 0) {

    XO_form_t xo_formater {
                            rc,
                            xop,
                            oe,
                            static_cast<uint32_t>(rb),
                            static_cast<uint32_t>(ra),
                            static_cast<uint32_t>(rt),
                            op
                          };

    dword(xo_formater.instruction);
  }

  void EmitDForm(const uint8_t op,
                 const RegNumber rt,
                 const RegNumber ra,
                 const int16_t imm) {

    D_form_t d_formater {
                          static_cast<uint32_t>(imm),
                          static_cast<uint32_t>(ra),
                          static_cast<uint32_t>(rt),
                          op
                         };

    dword(d_formater.instruction);
  }

  void EmitIForm(const uint8_t op,
                 const uint32_t imm,
                 const bool aa = 0,
                 const bool lk = 0) {

      I_form_t i_formater {
                            lk,
                            aa,
                            imm,
                            op
                          };

      dword(i_formater.instruction);
  }

   void EmitBForm(const uint8_t op,
                  const uint8_t bo,
                  const uint8_t bi,
                  const uint16_t bd,
                  const bool aa = 0,
                  const bool lk = 0) {
      B_form_t b_formater {
                            lk,
                            aa,
                            bd,
                            bi,
                            bo, //TODO(IBM): extend by 0b00 ??
                            op
                          };

       dword(b_formater.instruction);
   }

   void EmitSCForm(const uint8_t op,
                   const uint16_t lev) {
      SC_form_t sc_formater {
                              1,
                              lev,
                              op
                            };

      dword(sc_formater.instruction);
   }

   void EmitXForm(const uint8_t op,
                  const RegNumber rt,
                  const RegNumber ra,
                  const RegNumber rb,
                  const uint16_t xop,
                  const bool rc = 0){

      X_form_t x_formater {
                            rc,
                            xop,
                            static_cast<uint32_t>(rb),
                            static_cast<uint32_t>(ra),
                            static_cast<uint32_t>(rt),
                            op
                          };

      dword(x_formater.instruction);
   }

   void EmitDSForm(const uint8_t op,
                   const RegNumber rt,
                   const RegNumber ra,
                   const uint16_t imm,
                   const uint16_t xop) {

      DS_form_t ds_formater {
                             xop,
                             imm,
                             static_cast<uint32_t>(ra),
                             static_cast<uint32_t>(rt),
                             op
                            };

      dword(ds_formater.instruction);
   }

   void EmitDQForm(const uint8_t op,
                   const RegNumber rtp,
                   const RegNumber ra,
                   uint16_t imm){

      DQ_form_t dq_formater {
                             0x0, //Reserved
                             static_cast<uint32_t>(rtp),
                             static_cast<uint32_t>(ra),
                             imm,
                             op
                            };

      dword(dq_formater.instruction);
   }


   void EmitXLForm(const uint8_t op,
                   const uint8_t bt,
                   const uint8_t ba,
                   const uint8_t bb,
                   const uint16_t xop,
                   const bool lk = 0) {

      XL_form_t xl_formater {
                             lk,
                             xop,
                             bb,
                             ba,
                             bt,
                             op
                            };

      dword(xl_formater.instruction);
   }

   void EmitAForm(const uint8_t op,
                  const RegNumber rt,
                  const RegNumber ra,
                  const RegNumber rb,
                  const uint16_t bc,
                  const uint16_t xop,
                  const bool rc = 0) {

      A_form_t a_formater {
                           rc,
                           xop,
                           bc,
                           static_cast<uint32_t>(rb),
                           static_cast<uint32_t>(ra),
                           static_cast<uint32_t>(rt),
                           op
                          };

      dword(a_formater.instruction);
   }

   void EmitMForm(const uint8_t op,
                  const RegNumber rs,
                  const RegNumber ra,
                  const RegNumber rb,
                  const uint8_t mb,
                  const uint8_t me,
                  const bool rc = 0) {

      M_form_t m_formater {
                           rc,
                           me,
                           mb,
                           static_cast<uint32_t>(rb),
                           static_cast<uint32_t>(ra),
                           static_cast<uint32_t>(rs),
                           op
                          };

      dword(m_formater.instruction);
   }

   void EmitMDForm(const uint8_t op,
                   const RegNumber rs,
                   const RegNumber ra,
                   const uint8_t sh,
                   const uint8_t mb,
                   const uint8_t xop,
                   const bool rc = 0) {

      MD_form_t md_formater {
        rc,
        static_cast<uint32_t>(sh >> 5),                         // sh5
        xop,
        static_cast<uint32_t>(((mb >> 5) | (mb << 1)) & 0x3F),  // me5 || me0:4
        static_cast<uint32_t>(sh & 0x1F),                       // sh0:4
        static_cast<uint32_t>(ra),
        static_cast<uint32_t>(rs),
        op
      };

      dword(md_formater.instruction);
   }

   void EmitMDSForm(const uint8_t op,
                    const RegNumber rs,
                    const RegNumber ra,
                    const RegNumber rb,
                    const uint8_t mb,
                    const uint8_t xop,
                    const bool rc = 0) {

      MDS_form_t mds_formater {
                               rc,
                               xop,
                               mb,
                               static_cast<uint32_t>(rb),
                               static_cast<uint32_t>(ra),
                               static_cast<uint32_t>(rs),
                               op
                              };

      dword(mds_formater.instruction);
   }

  void EmitXFXForm(const uint8_t op,
                   const RegNumber rs,
                   const SpecialReg spr,
                   const uint16_t xo,
                   const uint8_t rsv = 0) {
    XFX_form_t xfx_formater {
      rsv,
      xo,
      (static_cast<uint32_t>(spr) >> 5) & 0x1F,
      static_cast<uint32_t>(spr) & 0x1F,
      static_cast<uint32_t>(rs),
      op
    };
    dword(xfx_formater.instruction);
  }

  //TODO(IBM): Unimplemented instruction formaters
  void EmitXFLForm()  { not_implemented(); }
  void EmitXX1Form()  { not_implemented(); }
  void EmitXX2Form()  { not_implemented(); }
  void EmitXX3Form()  { not_implemented(); }
  void EmitXX4Form()  { not_implemented(); }
  void EmitXSForm()   { not_implemented(); }
  void EmitVAForm()   { not_implemented(); }
  void EmitVCForm()   { not_implemented(); }
  void EmitVXForm()   { not_implemented(); }
  void EmitZ23Form()  { not_implemented(); }
  void EmitEVXForm()  { not_implemented(); }
  void EmitEVSForm()  { not_implemented(); }
  void EmitZ22Form()  { not_implemented(); }

private:
  //Low-level emitter functions.
  void byte(uint8_t b) {
    codeBlock.byte(b);
  }
  void word(uint16_t w) {
    codeBlock.word(w);
  }
  void dword(uint32_t dw) {
    codeBlock.dword(dw);
  }
  void qword(uint64_t qw) {
    codeBlock.qword(qw);
  }
  void bytes(size_t n, const uint8_t* bs) {
    codeBlock.bytes(n, bs);
  }

  HPHP::CodeBlock& codeBlock;

  RegNumber rn(Reg64 r) {
    return RegNumber(int(r));
  }
  RegNumber rn(int n) {
    return RegNumber(int(n));
  }

};

//////////////////////////////////////////////////////////////////////
// Branches
//////////////////////////////////////////////////////////////////////

class BranchParams {
  /* BO and BI parameter mapping related to BranchConditions */
  public:
    BranchParams() { assert("BranchParams created without parameter"); }

    enum class BO {
      CRNotSet              = 4,
      CRSet                 = 12,
      Always                = 20
    };

#define CR_CONDITIONS(cr) \
      CR##cr##_LessThan          = 0x80000000ULL >> (32 - (0 + (cr * 4))), \
      CR##cr##_GreaterThan       = 0x80000000ULL >> (32 - (1 + (cr * 4))), \
      CR##cr##_Equal             = 0x80000000ULL >> (32 - (2 + (cr * 4))), \
      CR##cr##_SummaryOverflow   = 0x80000000ULL >> (32 - (3 + (cr * 4)))

    enum class BI {
      CR_CONDITIONS(0),
      CR_CONDITIONS(1),
      CR_CONDITIONS(2),
      CR_CONDITIONS(3),
      CR_CONDITIONS(4),
      CR_CONDITIONS(5),
      CR_CONDITIONS(6),
      CR_CONDITIONS(7)
    };

#undef CR_CONDITIONS

    enum class BH {
      CTR_Loop              = 0,
      LR_Loop               = 1,
      Reserved              = 2,
      NoBranchPrediction    = 3
    };

#define SWITCHES(cr)                                              \
  case BranchConditions::CR##cr##_LessThan:                       \
    m_bo = BO::CRSet;    m_bi = BI::CR##cr##_LessThan;    break;  \
  case BranchConditions::CR##cr##_LessThanEqual:                  \
    m_bo = BO::CRNotSet; m_bi = BI::CR##cr##_GreaterThan; break;  \
  case BranchConditions::CR##cr##_GreaterThan:                    \
    m_bo = BO::CRSet;    m_bi = BI::CR##cr##_GreaterThan; break;  \
  case BranchConditions::CR##cr##_GreaterThanEqual:               \
    m_bo = BO::CRNotSet; m_bi = BI::CR##cr##_LessThan;    break;  \
  case BranchConditions::CR##cr##_Equal:                          \
    m_bo = BO::CRSet;    m_bi = BI::CR##cr##_Equal;       break;  \
  case BranchConditions::CR##cr##_NotEqual:                       \
    m_bo = BO::CRNotSet; m_bi = BI::CR##cr##_Equal;       break

    BranchParams(BranchConditions bc) {
      /* TODO(gut): implement other CRs */
      switch (bc) {
        SWITCHES(0);
        SWITCHES(1);
        SWITCHES(2);
        SWITCHES(3);
        SWITCHES(4);
        SWITCHES(5);
        SWITCHES(6);
        SWITCHES(7);
        case BranchConditions::Always:
          m_bo = BO::Always; m_bi = BI(0); break;
      }
    }

#undef SWITCHES

    ~BranchParams() {}

    uint8_t bo() { return (uint8_t)m_bo; }
    uint8_t bi() { return (uint8_t)m_bi; }

  private:
    BranchParams::BO m_bo;
    BranchParams::BI m_bi;
};

/*
 TODO(IBM) We need to check if we can have a 1:1 mapping between vasm opcodes: jccs, jmps, call
 if don't we need to implement this like ARM calling a backend to squash the N instructions to perform
 vasm operations into one.
*/
class Label : private boost::noncopyable {
public:
  Label() : m_a(nullptr) , m_address(nullptr) {}

  ~Label() {
    if (!m_toPatch.empty()) {
      assert(m_a && m_address && "Label had jumps but was never set");
    }
    for (auto& ji : m_toPatch) {
      switch (ji.type) {
      case BranchType::bc:
        ji.a->patchBc(ji.addr, m_address);
        break;
      case BranchType::bctr:
        ji.a->patchBctr(ji.addr, m_address);
        break;
      }
    }
  }

  void branchOffset(Assembler& a, BranchConditions bc) {
    BranchParams bp(bc);
    addJump(&a, BranchType::bc);

    /* Address is going to be redefined on patchBc() */
    a.bc(bp.bo(), bp.bi(), m_address ? m_address : a.frontier());
  }

  void branchAbsolute(Assembler& a, BranchConditions bc) {
    BranchParams bp(bc);
    addJump(&a, BranchType::bc);

    /* Address is going to be redefined on patchBc() */
    a.bca(bp.bo(), bp.bi(), m_address ? m_address : a.frontier());
  }

  void branchAuto(Assembler& a, Reg64 tmp, BranchConditions bc) {
    assert(m_address);
    auto delta = m_address - (a.frontier() + 4); // jumps the branch instr
    // offset is 14 bits long, a bit shorter than HPHP::sz::word
    if (HPHP::jit::deltaFits(delta, HPHP::sz::word) && (delta < (1 << 14))) {
      /* Branch by offset */
      a.bc(*this, bc);
    } else {
      // use CTR to perform absolute branch
      BranchParams bp(bc);
      addJump(&a, BranchType::bctr);

      // TODO: is this really the best way? If only there was a register that
      // was already filled with the address...
      const ssize_t address = ssize_t(m_address ? m_address : a.frontier());

      // Optimization: the highest 48th up to 63rd bits are never used to
      // address RAM data so we can assume it's zero
      a.li   (tmp, HPHP::safe_cast<int16_t>(
                   (address & (ssize_t(UINT16_MAX) << 32)) >> 32));
      a.sldi (tmp, tmp, 32);
      a.addis(tmp, tmp, HPHP::safe_cast<int16_t>(
                   (address & (ssize_t(UINT16_MAX) << 16)) >> 16));
      a.addi (tmp, tmp, HPHP::safe_cast<int16_t>(
                   address & ssize_t(UINT16_MAX)));
      a.mtctr(tmp);
      a.bcctr(bp.bo(), bp.bi(), 0);
    }
  }

  friend void asm_label(Assembler& a, Label& l) {
    assert(!l.m_address && !l.m_a && "Label was already set");
    l.m_a = &a;
    l.m_address = a.frontier();
  }

private:
  enum class BranchType {
    bc,
    bctr
  };

  struct JumpInfo {
    BranchType type;
    Assembler* a;
    CodeAddress addr;
  };

private:
  void addJump(Assembler* a, BranchType type) {
    if (m_address) return;
    JumpInfo info;
    info.type = type;
    info.a = a;
    info.addr = a->codeBlock.frontier();
    m_toPatch.push_back(info);
  }

private:
  Assembler* m_a;
  CodeAddress m_address;
  std::vector<JumpInfo> m_toPatch;
};

/* Simplify to conditional branch that always branch */
// TODO: implement also a patchB if b is not a mnemonic to bc
inline void Assembler::b(Label& l)  { bc (l, BranchConditions::Always); }
inline void Assembler::ba(Label& l) { bca(l, BranchConditions::Always); }

inline void Assembler::bc (Label& l, BranchConditions bc) {
  l.branchOffset(*this, bc);
}
inline void Assembler::bca(Label& l, BranchConditions bc) {
  l.branchAbsolute(*this, bc);
}

inline void Assembler::branchAuto(Label& l,
                              Reg64 tmp,
                              BranchConditions bc = BranchConditions::Always) {
  l.branchAuto(*this, tmp, bc);
}

class Decoder {
public:
  explicit Decoder(uint32_t* ip) { decode(ip); }
  std::string toString();
private:
  void decode(uint32_t* ip);
  uint8_t op_;
};

} // namespace ppc64_asm

#endif
