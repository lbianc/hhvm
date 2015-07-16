/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | Copyright (c) 2010-2014 Facebook, Inc. (http://www.facebook.com)     |
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
#ifndef incl_HPHP_UTIL_ASM_PPC64_H_
#define incl_HPHP_UTIL_ASM_PPC64_H_

#include <boost/noncopyable.hpp>
#include <type_traits>

#include "hphp/util/asm-x64.h"
#include "hphp/util/data-block.h"
#include "hphp/util/atomic.h"
#include "hphp/util/immed.h"
#include "hphp/util/trace.h"
#include "hphp/util/safe-cast.h"

/*
 * An experimental macro assembler for x64, that strives for low coupling to
 * the runtime environment.
 *
 * There are more complete assemblers out there; if you use this one
 * yourself, expect not to find all the instructions you wanted to use. You
 * may have to go spelunking in the Intel manuals:
 *
 *   http://www.intel.com/products/processor/manuals/
 *
 * If you're looking for something more fully baked, here are some options
 * to consider:
 *
 *   1. Nanojit or llvm, both of which translate abstract virtual machine
 *      instructions to the native target architecture, or
 *   2. The embedded assemblers from v8, the Sun JVM, etc.
 */

/*
 * Some members cannot be const because their values aren't known in
 * an initialization list. Like the opposite of the "mutable" keyword.
 * This declares this property to readers.
 */
#define logical_const /* nothing */

namespace HPHP { namespace jit { namespace ppc64 {

#define TRACEMOD ::HPHP::Trace::asmx64

namespace reg {
  constexpr HPHP::jit::Reg64 r0(0);
  constexpr Reg64 r1(1);
  constexpr Reg64 r2(2);
  constexpr Reg64 r3(3);
  constexpr Reg64 r4(4);
  constexpr Reg64 r5(5);
  constexpr Reg64 r6(6);
  constexpr Reg64 r7(7);
  constexpr Reg64 r8(8);
  constexpr Reg64 r9(9);
  constexpr Reg64 r10(10);

  constexpr Reg64 r11(11);
  constexpr Reg64 r12(12);
  constexpr Reg64 r13(13);
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

  constexpr RegXMM v0(0);
  constexpr RegXMM v1(1);
  constexpr RegXMM v2(2);
  constexpr RegXMM v3(3);
  constexpr RegXMM v4(4);
  constexpr RegXMM v5(5);
  constexpr RegXMM v6(6);
  constexpr RegXMM v7(7);
  constexpr RegXMM v8(8);
  constexpr RegXMM v9(9);
  constexpr RegXMM v10(10);
  constexpr RegXMM v11(11);
  constexpr RegXMM v12(12);
  constexpr RegXMM v13(13);
  constexpr RegXMM v14(14);
  constexpr RegXMM v15(15);
  constexpr RegXMM v16(16);
  constexpr RegXMM v17(17);
  constexpr RegXMM v18(18);
  constexpr RegXMM v19(19);
  constexpr RegXMM v20(20);
  constexpr RegXMM v21(21);
  constexpr RegXMM v22(22);
  constexpr RegXMM v23(23);
  constexpr RegXMM v24(24);
  constexpr RegXMM v25(25);
  constexpr RegXMM v26(26);
  constexpr RegXMM v27(27);
  constexpr RegXMM v28(28);
  constexpr RegXMM v29(29);
  constexpr RegXMM v30(30);
  constexpr RegXMM v31(31);

#undef X

}

//////////////////////////////////////////////////////////////////////

enum X64InstrFlags {
//TODO - create flags for PPC64
	  IF_REVERSE    = 0x0001, // The operand encoding for some instructions are
	                          // "backwards" in x64; these instructions are
	                          // called "reverse" instructions. There are a few
	                          // details about emitting "reverse" instructions:
	                          // (1) for the R_M address mode, we use the MR
	                          // opcode, (2) for M_R and R address modes, we use
	                          // the RM opcode, and (3) for the R_R address mode,
	                          // we still use MR opcode, but we have to swap the
	                          // first argument and the second argument.

	  IF_TWOBYTEOP  = 0x0002, // Some instructions have two byte opcodes. For
	                          // these instructions, an additional byte (0x0F) is
	                          // emitted before the standard opcode byte.

	  IF_JCC        = 0x0004, // instruction is jcc
	  IF_IMUL       = 0x0008, // instruction is imul
	  IF_HAS_IMM8   = 0x0010, // instruction has an encoding that takes an 8-bit
	                          // immediate
	  IF_SHIFT      = 0x0020, // instruction is rol, ror, rcl, rcr, shl, shr, sar
	  IF_RET        = 0x0040, // instruction is ret
	  IF_SHIFTD     = 0x0080, // instruction is shld, shrd
	  IF_NO_REXW    = 0x0100, // rexW prefix is not needed
	  IF_MOV        = 0x0200, // instruction is mov
	  IF_COMPACTR   = 0x0400, // instruction supports compact-R encoding
	  IF_RAX        = 0x0800, // instruction supports special rax encoding
	  IF_XCHG       = 0x1000, // instruction is xchg (not xchgb)
	  IF_BYTEREG    = 0x2000, // instruction is movzbq, movsbq
	  IF_66PREFIXED = 0x4000, // instruction requires a manditory 0x66 prefix
	  IF_F3PREFIXED = 0x8000, // instruction requires a manditory 0xf3 prefix
	  IF_F2PREFIXED = 0x10000, // instruction requires a manditory 0xf2 prefix
	  IF_THREEBYTEOP = 0x20000, // instruction requires a 0x0F 0x3A prefix
	  IF_ROUND       = 0x40000, // instruction is round(sp)d
};

/*
  Address mode to table index map:
      Table index 0 <- R_R / M_R(n) / R_M(r) / R(n)
      Table index 1 <- R_M(n) / M_R(r) / R(r)
      Table index 2 <- I / R_I / M_I / R_R_I / M_R_I / R_M_I
      Table index 3 <- "/digit" value used by the above address modes
      Table index 4 <- special R_I (for rax)
      Table index 5 <- compact-R / none

  (n) - for normal instructions only (IF_REVERSE flag is not set)
  (r) - for reverse instructions only (IF_REVERSE flag is set)

  0xF1 is used to indicate invalid opcodes.
*/

struct X64Instr {
  unsigned char table[6];
  unsigned long flags;
};

struct PPC64Instr {
  unsigned char table[6];
  unsigned long flags;
};

enum class RoundDirection : ssize_t {
  nearest  = 0,
  floor    = 1,
  ceil     = 2,
  truncate = 3,
};

const char* show(RoundDirection);

enum class ComparisonPred : uint8_t {
  // True if...
  eq_ord = 0,    // ...operands are ordered AND equal
  ne_unord = 4,  // ...operands are unordered OR unequal
};

/*enum ConditionCode {
  CC_None = -1,
  CC_O    = 0x00,
  CC_NO   = 0x01,

  CC_B    = 0x02,
  CC_NAE  = 0x02,
  CC_AE   = 0x03,
  CC_NB   = 0x03,
  CC_NC   = 0x03,

  CC_E    = 0x04,
  CC_Z    = 0x04,
  CC_NE   = 0x05,
  CC_NZ   = 0x05,

  CC_BE   = 0x06,
  CC_NA   = 0x06,
  CC_A    = 0x07,
  CC_NBE  = 0x07,

  CC_S    = 0x08,
  CC_NS   = 0x09,

  CC_P    = 0x0A,
  CC_NP   = 0x0B,

  CC_L    = 0x0C,
  CC_NGE  = 0x0C,
  CC_GE   = 0x0D,
  CC_NL   = 0x0D,

  CC_LE   = 0x0E,
  CC_NG   = 0x0E,
  CC_G    = 0x0F,
  CC_NLE  = 0x0F,
};*/

// names of condition codes, indexable by the ConditionCode enum value.
extern const char* cc_names[];

inline ConditionCode ccNegate(ConditionCode c) {
  return ConditionCode(int(c) ^ 1); // And you thought x86 was irregular!
}

///////////////////////////////////////////////////////////////////////////////

struct Label;


class PPC64Assembler : private boost::noncopyable {
  friend struct Label;

  /*
   * Type for register numbers, independent of the size we're going to
   * be using it as. Also, the same register number may mean different
   * physical registers for different instructions (e.g. xmm0 and rax
   * are both 0). Only for internal use in X64Assembler.
   */
  enum class RegNumber : int {};
  static const RegNumber noreg = RegNumber(-1);

public:
  explicit PPC64Assembler(CodeBlock& cb) : codeBlock(cb) {}

  CodeBlock& code() const { return codeBlock; }

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

  void tdi () {}
  void twi () {}
  void mulli () {}
  void subfic () {}
  void cmplwi () {}
  void cmplwi_ () {}
  void cmpldi () {}
  void cmpldi_ () {}
  void cmpwi () {}
  void cmpwi_ () {}
  void cmpdi () {}
  void cmpdi_ () {}
  void addic () {}
  void addic_ () {}
  void addi () {}
  void li () {}
  void la () {}
  void addis () {}
  void lis () {}
  void lus () {}
  void bc () {}
  void bcl () {}
  void bdnz () {}
  void bdz () {}
  void sc_0 () {}
  void b () {}
  void bl () {}
  void rlwimi () {}
  void rlwinm () {}
  void rlwnm () {}
  void ori () {}
  void nop_0 () {}
  void oris () {}
  void xori () {}
  void xoris () {}
  void andi_ () {}
  void andis_ () {}
  void lwz () {}
  void lwzu () {}
  void lbz () {}
  void lbzu () {}
  void stw () {}
  void stwu () {}
  void stb () {}
  void stbu () {}
  void lhz () {}
  void lhzu () {}
  void lha () {}
  void lhau () {}
  void sth () {}
  void sthu () {}
  void lmw () {}
  void stmw () {}
  void lfs () {}
  void lfsu () {}
  void lfd () {}
  void lfdu () {}
  void stfs () {}
  void stfsu () {}
  void stfd () {}
  void stfdu () {}
  void ld () {}
  void ldu () {}
  void lwa () {}
  void std () {}
  void stdu () {}
  void mulhhwu () {}
  void machhwu () {}
  void mulhhw () {}
  void nmachhw () {}
  void machhwsu () {}
  void machhws () {}
  void nmachhws () {}
  void mulchwu () {}
  void macchwu () {}
  void mulchw () {}
  void macchw () {}
  void nmacchw () {}
  void macchwsu () {}
  void macchws () {}
  void nmacchws () {}
  void mullhw () {}
  void maclhw () {}
  void nmaclhw () {}
  void maclhwsu () {}
  void maclhws () {}
  void nmaclhws () {}
  void machhwuo () {}
  void nmachhwo () {}
  void machhwsuo () {}
  void machhwso () {}
  void nmachhwso () {}
  void macchwuo () {}
  void macchwo () {}
  void nmacchwo () {}
  void macchwsuo () {}
  void macchwso () {}
  void nmacchwso () {}
  void maclhwo () {}
  void nmaclhwo () {}
  void maclhwsuo () {}
  void maclhwso () {}
  void nmaclhwso () {}
  void vaddubm () {}
  void vmaxub () {}
  void vrlb () {}
  void vcmpequb () {}
  void vmuloub () {}
  void vaddfp () {}
  void vmrghb () {}
  void vpkuhum () {}
  void vmhaddshs () {}
  void vmhraddshs () {}
  void vmladduhm () {}
  void vmsumubm () {}
  void vmsummbm () {}
  void vmsumuhm () {}
  void vmsumuhs () {}
  void vmsumshm () {}
  void vmsumshs () {}
  void vsel () {}
  void vperm () {}
  void vsldoi () {}
  void vpermxor () {}
  void vmaddfp () {}
  void vnmsubfp () {}
  void vaddeuqm () {}
  void vaddecuq () {}
  void vsubeuqm () {}
  void vsubecuq () {}
  void vadduhm () {}
  void vmaxuh () {}
  void vrlh () {}
  void vcmpequh () {}
  void vmulouh () {}
  void vsubfp () {}
  void vmrghh () {}
  void vpkuwum () {}
  void vadduwm () {}
  void vmaxuw () {}
  void vrlw () {}
  void vcmpequw () {}
  void vmulouw () {}
  void vmuluwm () {}
  void vmrghw () {}
  void vpkuhus () {}
  void vaddudm () {}
  void vmaxud () {}
  void vrld () {}
  void vcmpeqfp () {}
  void vcmpequd () {}
  void vpkuwus () {}
  void vadduqm () {}
  void vmaxsb () {}
  void vslb () {}
  void vmulosb () {}
  void vrefp () {}
  void vmrglb () {}
  void vpkshus () {}
  void vaddcuq () {}
  void vmaxsh () {}
  void vslh () {}
  void vmulosh () {}
  void vrsqrtefp () {}
  void vmrglh () {}
  void vpkswus () {}
  void vaddcuw () {}
  void vmaxsw () {}
  void vslw () {}
  void vmulosw () {}
  void vexptefp () {}
  void vmrglw () {}
  void vpkshss () {}
  void vmaxsd () {}
  void vsl () {}
  void vcmpgefp () {}
  void vlogefp () {}
  void vpkswss () {}
  void vadduhs () {}
  void vminuh () {}
  void vsrh () {}
  void vcmpgtuh () {}
  void vmuleuh () {}
  void vrfiz () {}
  void vsplth () {}
  void vupkhsh () {}
  void vminuw () {}
  void vminud () {}
  void vcmpgtud () {}
  void vrfim () {}
  void vcmpgtsb () {}
  void vcfux () {}
  void vaddshs () {}
  void vminsh () {}
  void vsrah () {}
  void vcmpgtsh () {}
  void vmulesh () {}
  void vcfsx () {}
  void vspltish () {}
  void vupkhpx () {}
  void vaddsws () {}
  void vminsw () {}
  void vsraw () {}
  void vcmpgtsw () {}
  void vmulesw () {}
  void vctuxs () {}
  void vspltisw () {}
  void vminsd () {}
  void vsrad () {}
  void vcmpbfp () {}
  void vcmpgtsd () {}
  void vctsxs () {}
  void vupklpx () {}
  void vsububm () {}
  void bcdadd_ () {}
  void vavgub () {}
  void vand () {}
  void vcmpequb_ () {}
  void vmaxfp () {}
  void vsubuhm () {}
  void bcdsub_ () {}
  void vavguh () {}
  void vandc () {}
  void vcmpequh_ () {}
  void vminfp () {}
  void vpkudum () {}
  void vsubuwm () {}
  void vavguw () {}
  void vor () {}
  void vcmpequw_ () {}
  void vpmsumw () {}
  void vcmpeqfp_ () {}
  void vcmpequd_ () {}
  void vpkudus () {}
  void vavgsb () {}
  void vavgsh () {}
  void vorc () {}
  void vbpermq () {}
  void vpksdus () {}
  void vavgsw () {}
  void vsld () {}
  void vcmpgefp_ () {}
  void vpksdss () {}
  void vsububs () {}
  void mfvscr () {}
  void vsum4ubs () {}
  void vsubuhs () {}
  void mtvscr () {}
  void vcmpgtuh_ () {}
  void vsum4shs () {}
  void vupkhsw () {}
  void vsubuws () {}
  void vshasigmaw () {}
  void veqv () {}
  void vsum2sws () {}
  void vmrgow () {}
  void vshasigmad () {}
  void vsrd () {}
  void vcmpgtud_ () {}
  void vupklsw () {}
  void vupkslw () {}
  void vsubsbs () {}
  void vclzb () {}
  void vpopcntb () {}
  void vcmpgtsb_ () {}
  void vsum4sbs () {}
  void vsubshs () {}
  void vclzh () {}
  void vpopcnth () {}
  void vcmpgtsh_ () {}
  void vsubsws () {}
  void vclzw () {}
  void vpopcntw () {}
  void vcmpgtsw_ () {}
  void vsumsws () {}
  void vmrgew () {}
  void vclzd () {}
  void vpopcntd () {}
  void vcmpbfp_ () {}
  void vcmpgtsd_ () {}
  void mcrf () {}
  void isync_0 () {}
  void crnor () {}
  void crnot () {}
  void crandc () {}
  void crxor () {}
  void crclr () {}
  void crnand () {}
  void crand () {}
  void creqv () {}
  void crset () {}
  void crorc () {}
  void cror () {}
  void crmove () {}
  void bclr () {}
  void bclrl () {}
  void bcctr () {}
  void bcctrl () {}
  void bctar () {}
  void bctarl () {}
  void blr_0 () {}
  void blrl_0 () {}
  void bctr_0 () {}
  void bctrl_0 () {}
  void cmpw () {}
  void cmpw_ () {}
  void cmpd () {}
  void cmpd_ () {}
  void tw () {}
  void lvsl () {}
  void subfc () {}
  void subc () {}
  void mulhdu () {}
  void addc () {}
  void mulhwu () {}
  void isel () {}
  void isellt () {}
  void iselgt () {}
  void iseleq () {}
  void mfcr () {}
  void mfocrf () {}
  void mtcrf () {}
  void mtocrf () {}
  void lwarx () {}
  void ldx () {}
  void lwzx () {}
  void slw () {}
  void cntlzw () {}
  void sld () {}
  void and () {}
  void cmplw () {}
  void cmplw_ () {}
  void cmpld () {}
  void cmpld_ () {}
  void lvsr () {}
  void subf () {}
  void sub () {}
  void lbarx () {}
  void ldux () {}
  void dcbst () {}
  void lwzux () {}
  void cntlzd () {}
  void andc () {}
  void td () {}
  void lvewx () {}
  void mulhd () {}
  void addg6s () {}
  void mulhw () {}
  void dlmzb () {}
  void ldarx () {}
  void dcbf () {}
  void lbzx () {}
  void lvx () {}
  void neg () {}
  void lharx () {}
  void lbzux () {}
  void popcntb () {}
  void not () {}
  void nor () {}
  void stvebx () {}
  void subfe () {}
  void sube () {}
  void adde () {}
  void stdx () {}
  void stwcx_ () {}
  void stwx () {}
  void prtyw () {}
  void stvehx () {}
  void stdux () {}
  void stqcx_ () {}
  void stwux () {}
  void prtyd () {}
  void stvewx () {}
  void subfze () {}
  void addze () {}
  void stdcx_ () {}
  void stbx () {}
  void stvx () {}
  void subfme () {}
  void mulld () {}
  void addme () {}
  void mullw () {}
  void dcbtst () {}
  void stbux () {}
  void bpermd () {}
  void lvepxl () {}
  void add () {}
  void lqarx () {}
  void dcbt () {}
  void lhzx () {}
  void cdtbcd () {}
  void eqv () {}
  void lvepx () {}
  void eciwx () {}
  void lhzux () {}
  void cbcdtd () {}
  void xor () {}
  void mfspefscr () {}
  void mfxer () {}
  void mflr () {}
  void mfctr () {}
  void lwax () {}
  void lhax () {}
  void mftb () {}
  void mftbu () {}
  void lvxl () {}
  void lwaux () {}
  void lhaux () {}
  void popcntw () {}
  void divdeu () {}
  void divweu () {}
  void sthx () {}
  void orc () {}
  void ecowx () {}
  void sthux () {}
  void or_ (Reg64 rt, Reg64 ra, Reg64 rb) { emitRRR(instr_or, rn(rt), rn(ra), rn(rb)); } //primeira instrucao
  void mr () {}
  void divdu () {}
  void divwu () {}
  void mtspefscr () {}
  void mtxer () {}
  void mtlr () {}
  void mtctr () {}
  void dcbi () {}
  void nand () {}
  void dsn () {}
  void stvxl () {}
  void divd () {}
  void divw () {}
  void popcntd () {}
  void cmpb () {}
  void mcrxr () {}
  void lbdx () {}
  void subfco () {}
  void subco () {}
  void addco () {}
  void ldbrx () {}
  void lswx () {}
  void lwbrx () {}
  void lfsx () {}
  void srw () {}
  void srd () {}
  void lhdx () {}
  void subfo () {}
  void subo () {}
  void lfsux () {}
  void lwdx () {}
  void lswi () {}
  void sync_0 () {}
  void lwsync_0 () {}
  void ptesync_0 () {}
  void lfdx () {}
  void lddx () {}
  void nego () {}
  void lfdux () {}
  void stbdx () {}
  void subfeo () {}
  void subeo () {}
  void addeo () {}
  void stdbrx () {}
  void stswx () {}
  void stwbrx () {}
  void stfsx () {}
  void sthdx () {}
  void stbcx_ () {}
  void stfsux () {}
  void stwdx () {}
  void subfzeo () {}
  void addzeo () {}
  void stswi () {}
  void sthcx_ () {}
  void stfdx () {}
  void stddx () {}
  void subfmeo () {}
  void mulldo () {}
  void addmeo () {}
  void mullwo () {}
  void dcba () {}
  void stfdux () {}
  void stvepxl () {}
  void addo () {}
  void lhbrx () {}
  void lfdpx () {}
  void sraw () {}
  void srad () {}
  void lfddx () {}
  void stvepx () {}
  void srawi () {}
  void sradi () {}
  void eieio_0 () {}
  void lfiwax () {}
  void divdeuo () {}
  void divweuo () {}
  void sthbrx () {}
  void stfdpx () {}
  void extsh () {}
  void stfddx () {}
  void divdeo () {}
  void divweo () {}
  void extsb () {}
  void divduo () {}
  void divwou () {}
  void icbi () {}
  void stfiwx () {}
  void extsw () {}
  void divdo () {}
  void divwo () {}
  void dcbz () {}
  void tbegin_ () {}
  void tbegin_0 () {}
  void tend_ () {}
  void tend_0 () {}
  void tendall_0 () {}
  void tcheck () {}
  void tsr_ () {}
  void tsuspend_0 () {}
  void tresume_0 () {}
  void tabortwc_ () {}
  void tabortdc_ () {}
  void tabortwci_ () {}
  void tabortdci_ () {}
  void tabort_ () {}
  void treclaim_ () {}
  void trechkpt_0 () {}
  void lxsiwzx () {}
  void lxsiwax () {}
  void mfvsrd () {}
  void mfvsrwz () {}
  void stxsiwx () {}
  void mtvsrd () {}
  void mtvsrwa () {}
  void lxvdsx () {}
  void lxsspx () {}
  void lxsdx () {}
  void stxsspx () {}
  void stxsdx () {}
  void lxvw4x () {}
  void lxvd2x () {}
  void stxvw4x () {}
  void stxvd2x () {}
  void rldicl () {}
  void rldicr () {}
  void rldic () {}
  void rldimi () {}
  void rldcl () {}
  void rldcr () {}
  void sldi () {}
  void srdi () {}
  void lq () {}
  void lfdp () {}
  void fdivs () {}
  void fsubs () {}
  void fadds () {}
  void fsqrts () {}
  void fres () {}
  void fmuls () {}
  void frsqrtes () {}
  void fmsubs () {}
  void fmadds () {}
  void fnmsubs () {}
  void fnmadds () {}
  void fcfids () {}
  void fcfidus () {}
  void dadd () {}
  void dqua () {}
  void dmul () {}
  void drrnd () {}
  void dscli () {}
  void dquai () {}
  void dscri () {}
  void drintx () {}
  void dcmpo () {}
  void dtstex () {}
  void dtstdc () {}
  void dtstdg () {}
  void drintn () {}
  void dctdp () {}
  void dctfix () {}
  void ddedpd () {}
  void dxex () {}
  void dsub () {}
  void ddiv () {}
  void dcmpu () {}
  void dtstsf () {}
  void drsp () {}
  void dcffix () {}
  void denbcd () {}
  void diex () {}
  void xsaddsp () {}
  void xsmaddasp () {}
  void xxsldwi () {}
  void xsrsqrtesp () {}
  void xssqrtsp () {}
  void xxsel () {}
  void xssubsp () {}
  void xsmaddmsp () {}
  void xxpermdi () {}
  void xsresp () {}
  void xsmulsp () {}
  void xsmsubasp () {}
  void xxmrghw () {}
  void xsdivsp () {}
  void xsmsubmsp () {}
  void xsadddp () {}
  void xsmaddadp () {}
  void xscmpudp () {}
  void xscvdpuxws () {}
  void xsrdpi () {}
  void xsrsqrtedp () {}
  void xssqrtdp () {}
  void xssubdp () {}
  void xsmaddmdp () {}
  void xscmpodp () {}
  void xscvdpsxws () {}
  void xsrdpiz () {}
  void xsredp () {}
  void xsmuldp () {}
  void xsmsubadp () {}
  void xxmrglw () {}
  void xsrdpip () {}
  void xstsqrtdp () {}
  void xsrdpic () {}
  void xsdivdp () {}
  void xsmsubmdp () {}
  void xsrdpim () {}
  void xstdivdp () {}
  void xvaddsp () {}
  void xvmaddasp () {}
  void xvcmpeqsp () {}
  void xvcvspuxws () {}
  void xvrspi () {}
  void xvrsqrtesp () {}
  void xvsqrtsp () {}
  void xvsubsp () {}
  void xvmaddmsp () {}
  void xvcmpgtsp () {}
  void xvcvspsxws () {}
  void xvrspiz () {}
  void xvresp () {}
  void xvmulsp () {}
  void xvmsubasp () {}
  void xxspltw () {}
  void xvcmpgesp () {}
  void xvcvuxwsp () {}
  void xvrspip () {}
  void xvtsqrtsp () {}
  void xvrspic () {}
  void xvdivsp () {}
  void xvmsubmsp () {}
  void xvcvsxwsp () {}
  void xvrspim () {}
  void xvtdivsp () {}
  void xvadddp () {}
  void xvmaddadp () {}
  void xvcmpeqdp () {}
  void xvcvdpuxws () {}
  void xvrdpi () {}
  void xvrsqrtedp () {}
  void xvsqrtdp () {}
  void xvsubdp () {}
  void xvmaddmdp () {}
  void xvcmpgtdp () {}
  void xvcvdpsxws () {}
  void xvrdpiz () {}
  void xvredp () {}
  void xvmuldp () {}
  void xvmsubadp () {}
  void xvcmpgedp () {}
  void xvcvuxwdp () {}
  void xvrdpip () {}
  void xvtsqrtdp () {}
  void xvrdpic () {}
  void xvdivdp () {}
  void xvmsubmdp () {}
  void xvcvsxwdp () {}
  void xvrdpim () {}
  void xvtdivdp () {}
  void xsnmaddasp () {}
  void xxland () {}
  void xscvdpsp () {}
  void xscvdpspn () {}
  void xsnmaddmsp () {}
  void xxlandc () {}
  void xsrsp () {}
  void xsnmsubasp () {}
  void xxlor () {}
  void xscvuxdsp () {}
  void xsnmsubmsp () {}
  void xxlxor () {}
  void xscvsxdsp () {}
  void xsmaxdp () {}
  void xsnmaddadp () {}
  void xxlnor () {}
  void xscvdpuxds () {}
  void xscvspdp () {}
  void xscvspdpn () {}
  void xsmindp () {}
  void xsnmaddmdp () {}
  void xxlorc () {}
  void xscvdpsxds () {}
  void xsabsdp () {}
  void xscpsgndp () {}
  void xsnmsubadp () {}
  void xxlnand () {}
  void xscvuxddp () {}
  void xsnabsdp () {}
  void xsnmsubmdp () {}
  void xxleqv () {}
  void xscvsxddp () {}
  void xsnegdp () {}
  void xvmaxsp () {}
  void xvnmaddasp () {}
  void xvcmpeqsp_ () {}
  void xvcvspuxds () {}
  void xvcvdpsp () {}
  void xvminsp () {}
  void xvnmaddmsp () {}
  void xvcmpgtsp_ () {}
  void xvcvspsxds () {}
  void xvabssp () {}
  void xvcpsgnsp () {}
  void xvnmsubasp () {}
  void xvcmpgesp_ () {}
  void xvcvuxdsp () {}
  void xvnabssp () {}
  void xvnmsubmsp () {}
  void xvcvsxdsp () {}
  void xvnegsp () {}
  void xvmaxdp () {}
  void xvnmaddadp () {}
  void xvcmpeqdp_ () {}
  void xvcvdpuxds () {}
  void xvcvspdp () {}
  void xvmindp () {}
  void xvnmaddmdp () {}
  void xvcmpgtdp_ () {}
  void xvcvdpsxds () {}
  void xvabsdp () {}
  void xvcpsgndp () {}
  void xvnmsubadp () {}
  void xvcmpgedp_ () {}
  void xvcvuxddp () {}
  void xvnabsdp () {}
  void xvnmsubmdp () {}
  void xvcvsxddp () {}
  void xvnegdp () {}
  void stfdp () {}
  void stq () {}
  void fdiv () {}
  void fsub () {}
  void fadd () {}
  void fsqrt () {}
  void fsel () {}
  void fre () {}
  void fmul () {}
  void frsqrte () {}
  void fmsub () {}
  void fmadd () {}
  void fnmsub () {}
  void fnmadd () {}
  void fcmpu () {}
  void fcpsgn () {}
  void fcmpo () {}
  void mtfsb1 () {}
  void fneg () {}
  void mcrfs () {}
  void mtfsb0 () {}
  void fmr () {}
  void frsp () {}
  void fctiw () {}
  void fctiwz () {}
  void ftdiv () {}
  void fctiwu () {}
  void fctiwuz () {}
  void mtfsfi () {}
  void fnabs () {}
  void ftsqrt () {}
  void fabs () {}
  void frin () {}
  void friz () {}
  void frip () {}
  void frim () {}
  void mffs () {}
  void fctid () {}
  void fctidz () {}
  void fmrgow () {}
  void fcfid () {}
  void fctidu () {}
  void fctiduz () {}
  void fmrgew () {}
  void fcfidu () {}
  void daddq () {}
  void dquaq () {}
  void dmulq () {}
  void drrndq () {}
  void dscliq () {}
  void dquaiq () {}
  void dscriq () {}
  void drintxq () {}
  void dcmpoq () {}
  void dtstexq () {}
  void dtstdcq () {}
  void dtstdgq () {}
  void drintnq () {}
  void dctqpq () {}
  void dctfixq () {}
  void ddedpdq () {}
  void dxexq () {}
  void dsubq () {}
  void ddivq () {}
  void dcmpuq () {}
  void dtstsfq () {}
  void drdpq () {}
  void dcffixq () {}
  void denbcdq () {}
  void diexq () {}
  void evaddw () {}
  void evaddiw () {}
  void evsubw () {}
  void evsubiw () {}
  void evabs () {}
  void evneg () {}
  void evextsb () {}
  void evextsh () {}
  void evrndw () {}
  void evcntlzw () {}
  void evcntlsw () {}
  void brinc () {}
  void evand () {}
  void evandc () {}
  void evxor () {}
  void evor () {}
  void evmr () {}
  void evnor () {}
  void evnot () {}
  void eveqv () {}
  void evorc () {}
  void evnand () {}
  void evsrwu () {}
  void evsrws () {}
  void evsrwiu () {}
  void evsrwis () {}
  void evslw () {}
  void evslwi () {}
  void evrlw () {}
  void evsplati () {}
  void evrlwi () {}
  void evsplatfi () {}
  void evmergehi () {}
  void evmergelo () {}
  void evcmpgtu () {}
  void evcmpgtu_ () {}
  void evcmpgts () {}
  void evcmpgts_ () {}
  void evcmpltu () {}
  void evcmpltu_ () {}
  void evcmplts () {}
  void evcmplts_ () {}
  void evcmpeq () {}
  void evcmpeq_ () {}
  void evsel () {}
  void evsel_ () {}
  void evfsadd () {}
  void evfssub () {}
  void evfsabs () {}
  void evfsnabs () {}
  void evfsneg () {}
  void evfsmul () {}
  void evfsdiv () {}
  void evfscmpgt () {}
  void evfscmpgt_ () {}
  void evfscmplt () {}
  void evfscmplt_ () {}
  void evfscmpeq () {}
  void evfscmpeq_ () {}
  void evfscfui () {}
  void evfscfsi () {}
  void evfscfuf () {}
  void evfscfsf () {}
  void evfsctui () {}
  void evfsctsi () {}
  void evfsctuf () {}
  void evfsctsf () {}
  void evfsctuiz () {}
  void evfsctsiz_ () {}
  void evfststgt () {}
  void evfststgt_ () {}
  void evfststlt () {}
  void evfststlt_ () {}
  void evfststeq () {}
  void evfststeq_ () {}
  void efsadd () {}
  void efssub () {}
  void efsabs () {}
  void efsnabs () {}
  void efsneg () {}
  void efsmul () {}
  void efsdiv () {}
  void efscmpgt () {}
  void efscmpgt_ () {}
  void efscmplt () {}
  void efscmplt_ () {}
  void efscmpeq () {}
  void efscmpeq_ () {}
  void efscfd () {}
  void efscfui () {}
  void efscfsi () {}
  void efscfuf () {}
  void efscfsf () {}
  void efsctui () {}
  void efsctsi () {}
  void efsctuf () {}
  void efsctsf () {}
  void efsctuiz () {}
  void efsctsiz () {}
  void efststgt () {}
  void efststgt_ () {}
  void efststlt () {}
  void efststlt_ () {}
  void efststeq () {}
  void efststeq_ () {}
  void efdadd () {}
  void efdsub () {}
  void efdcfuid () {}
  void efdcfsid () {}
  void efdabs () {}
  void efdnabs () {}
  void efdneg () {}
  void efdmul () {}
  void efddiv () {}
  void efdctuidz () {}
  void efdctsidz_ () {}
  void efdcmpgt () {}
  void efdcmpgt_ () {}
  void efdcmplt () {}
  void efdcmplt_ () {}
  void efdcmpeq () {}
  void efdcmpeq_ () {}
  void efdcfs () {}
  void efdcfui () {}
  void efdcfsi () {}
  void efdcfuf () {}
  void efdcfsf () {}
  void efdctui () {}
  void efdctsi () {}
  void efdctuf () {}
  void efdctsf () {}
  void efdctuiz () {}
  void efdctsiz () {}
  void efdtstgt () {}
  void efdtstgt_ () {}
  void efdtstlt () {}
  void efdtstlt_ () {}
  void efdtsteq () {}
  void efdtsteq_ () {}
  void evlddx () {}
  void evldd () {}
  void evldwx () {}
  void evldw () {}
  void evldhx () {}
  void evldh () {}
  void evlwhex () {}
  void evlwhe () {}
  void evlwhoux () {}
  void evlwhou () {}
  void evlwhosx () {}
  void evlwhos () {}
  void evstddx () {}
  void evstdd () {}
  void evstdwx () {}
  void evstdw () {}
  void evstdhx () {}
  void evstdh () {}
  void evstwhex () {}
  void evstwhe () {}
  void evstwhox () {}
  void evstwho () {}
  void evstwwex () {}
  void evstwwe () {}
  void evstwwox () {}
  void evstwwo () {}
  void evmhessf () {}
  void evmhossf () {}
  void evmheumi () {}
  void evmhesmi () {}
  void evmhesmf () {}
  void evmhoumi () {}
  void evmhosmi () {}
  void evmhosmf () {}
  void evmhessfa () {}
  void evmhossfa () {}
  void evmheumia () {}
  void evmhesmia () {}
  void evmhesmfa () {}
  void evmhoumia () {}
  void evmhosmia () {}
  void evmhosmfa () {}
  void evmwhssf () {}
  void evmwlumi () {}
  void evmwhumi () {}
  void evmwhsmi () {}
  void evmwhsmf () {}
  void evmwssf () {}
  void evmwumi () {}
  void evmwsmi () {}
  void evmwsmf () {}
  void evmwhssfa () {}
  void evmwlumia () {}
  void evmwhumia () {}
  void evmwhsmia () {}
  void evmwhsmfa () {}
  void evmwssfa () {}
  void evmwumia () {}
  void evmwsmia () {}
  void evmwsmfa () {}
  void evmra () {}
  void evdivws () {}
  void evdivwu () {}
  void evmwssfaa () {}
  void evmwumiaa () {}
  void evmwsmiaa () {}
  void evmwsmfaa () {}
  void evmwssfan () {}
  void evmwumian () {}
  void evmwsmian () {}
  void evmwsmfan () {}
  void evmergehilo () {}
  void evmergelohi () {}
  void evlhhesplatx () {}
  void evlhhesplat () {}
  void evlhhousplatx () {}
  void evlhhousplat () {}
  void evlhhossplatx () {}
  void evlhhossplat () {}
  void evlwwsplatx () {}
  void evlwwsplat () {}
  void evlwhsplatx () {}
  void evlwhsplat () {}
  void evaddusiaaw () {}
  void evaddssiaaw () {}
  void evsubfusiaaw () {}
  void evsubfssiaaw () {}
  void evaddumiaaw () {}
  void evaddsmiaaw () {}
  void evsubfumiaaw () {}
  void evsubfsmiaaw () {}
  void evmheusiaaw () {}
  void evmhessiaaw () {}
  void evmhessfaaw () {}
  void evmhousiaaw () {}
  void evmhossiaaw () {}
  void evmhossfaaw () {}
  void evmheumiaaw () {}
  void evmhesmiaaw () {}
  void evmhesmfaaw () {}
  void evmhoumiaaw () {}
  void evmhosmiaaw () {}
  void evmhosmfaaw () {}
  void evmhegumiaa () {}
  void evmhegsmiaa () {}
  void evmhegsmfaa () {}
  void evmhogumiaa () {}
  void evmhogsmiaa () {}
  void evmhogsmfaa () {}
  void evmwlusiaaw () {}
  void evmwlssiaaw () {}
  void evmwlumiaaw () {}
  void evmwlsmiaaw () {}
  void evmheusianw () {}
  void evmhessianw () {}
  void evmhessfanw () {}
  void evmhousianw () {}
  void evmhossianw () {}
  void evmhossfanw () {}
  void evmheumianw () {}
  void evmhesmianw () {}
  void evmhesmfanw () {}
  void evmhoumianw () {}
  void evmhosmianw () {}
  void evmhosmfanw () {}
  void evmhegumian () {}
  void evmhegsmian () {}
  void evmhegsmfan () {}
  void evmhogumian () {}
  void evmhogsmian () {}
  void evmhogsmfan () {}
  void evmwlusianw () {}
  void evmwlssianw () {}
  void evmwlumianw () {}
  void evmwlsmianw () {}

  /*
   * Low-level emitter functions.
   *
   * These functions are the core of the assembler, and can also be
   * used directly.
   */

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

  // Segment register prefixes.
  PPC64Assembler& fs()  { byte(0x64); return *this; }
  PPC64Assembler& gs()  { byte(0x65); return *this; }


  const PPC64Instr instr_tdi =  { {0x08, 0x00, 0x00, 0x00/*ARI*/}, 0x00000 };
  const PPC64Instr instr_twi =  { {0x0C, 0x00, 0x00, 0x00/*ARI*/}, 0x00000 };
  const PPC64Instr instr_mulli =  { {0x1C, 0x00, 0x00, 0x00/*RRI*/}, 0x00000 };
  const PPC64Instr instr_subfic =  { {0x20, 0x00, 0x00, 0x00/*RRI*/}, 0x00000 };
  const PPC64Instr instr_cmplwi =  { {0x28, 0x00, 0x00, 0x00/*XRU*/}, 0x00000 };
  const PPC64Instr instr_cmplwi_ =  { {0x28, 0x00, 0x00, 0x00/*-RU*/}, 0x00000 };
  const PPC64Instr instr_cmpldi =  { {0x28, 0x20, 0x00, 0x00/*XRU*/}, 0x00000 };
  const PPC64Instr instr_cmpldi_ =  { {0x28, 0x20, 0x00, 0x00/*-RU*/}, 0x00000 };
  const PPC64Instr instr_cmpwi =  { {0x2C, 0x00, 0x00, 0x00/*XRI*/}, 0x00000 };
  const PPC64Instr instr_cmpwi_ =  { {0x2C, 0x00, 0x00, 0x00/*-RI*/}, 0x00000 };
  const PPC64Instr instr_cmpdi =  { {0x2C, 0x20, 0x00, 0x00/*XRI*/}, 0x00000 };
  const PPC64Instr instr_cmpdi_ =  { {0x2C, 0x20, 0x00, 0x00/*-RI*/}, 0x00000 };
  const PPC64Instr instr_addic =  { {0x30, 0x00, 0x00, 0x00/*RRI*/}, 0x00000 };
  const PPC64Instr instr_addic_ = { {0x34, 0x00, 0x00, 0x00/*RRI*/}, 0x00000 };
  const PPC64Instr instr_addi =  { {0x38, 0x00, 0x00, 0x00/*RR0I*/}, 0x00000 };
  const PPC64Instr instr_li =  { {0x38, 0x00, 0x00, 0x00/*RI*/}, 0x00000 };
  const PPC64Instr instr_la =  { {0x38, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_addis =  { {0x3C, 0x00, 0x00, 0x00/*RR0I*/}, 0x00000 };
  const PPC64Instr instr_lis =  { {0x3C, 0x00, 0x00, 0x00/*RI*/}, 0x00000 };
  const PPC64Instr instr_lus =  { {0x3C, 0x00, 0x00, 0x00/*RU*/}, 0x00000 };
  const PPC64Instr instr_bc =  { {0x40, 0x00, 0x00, 0x00/*AAK*/}, 0x00000 };
  const PPC64Instr instr_bcl =  { {0x40, 0x00, 0x00, 0x01/*AAK*/}, 0x00000 };
  const PPC64Instr instr_bdnz =  { {0x42, 0x00, 0x00, 0x00/*K*/}, 0x00000 };
  const PPC64Instr instr_bdz =  { {0x42, 0x40, 0x00, 0x00/*K*/}, 0x00000 };
  const PPC64Instr instr_sc_0 =  { {0x44, 0x00, 0x00, 0x00/**/}, 0x00000 };
  const PPC64Instr instr_b =  { {0x48, 0x00, 0x00, 0x00/*J*/}, 0x00000 };
  const PPC64Instr instr_bl =  { {0x48, 0x00, 0x00, 0x01/*J*/}, 0x00000 };
  const PPC64Instr instr_rlwimi =  { {0x50, 0x00, 0x00, 0x00/*RR~AAA.*/}, 0x00000 };
  const PPC64Instr instr_rlwinm =  { {0x54, 0x00, 0x00, 0x00/*RR~AAA.*/}, 0x00000 };
  const PPC64Instr instr_rlwnm =  { {0x5C, 0x00, 0x00, 0x00/*RR~RAA.*/}, 0x00000 };
  const PPC64Instr instr_ori =  { {0x60, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
  const PPC64Instr instr_nop_0 =  { {0x60, 0x00, 0x00, 0x00/**/}, 0x00000 };
  const PPC64Instr instr_oris =  { {0x64, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
  const PPC64Instr instr_xori =  { {0x68, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
  const PPC64Instr instr_xoris =  { {0x6C, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
  const PPC64Instr instr_andi_ =  { {0x70, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
  const PPC64Instr instr_andis_ = { {0x74, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
  const PPC64Instr instr_lwz =  { {0x80, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_lwzu =  { {0x84, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_lbz =  { {0x88, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_lbzu =  { {0x8C, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_stw =  { {0x90, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_stwu =  { {0x94, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_stb =  { {0x98, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_stbu =  { {0x9C, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_lhz =  { {0xA0, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_lhzu =  { {0xA4, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_lha =  { {0xA8, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_lhau =  { {0xAC, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_sth =  { {0xB0, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_sthu =  { {0xB4, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_lmw =  { {0xB8, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_stmw =  { {0xBC, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_lfs =  { {0xC0, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
  const PPC64Instr instr_lfsu =  { {0xC4, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
  const PPC64Instr instr_lfd =  { {0xC8, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
  const PPC64Instr instr_lfdu =  { {0xCC, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
  const PPC64Instr instr_stfs =  { {0xD0, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
  const PPC64Instr instr_stfsu =  { {0xD4, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
  const PPC64Instr instr_stfd =  { {0xD8, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
  const PPC64Instr instr_stfdu =  { {0xDC, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
  const PPC64Instr instr_ld =  { {0xE8, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_ldu =  { {0xE8, 0x00, 0x00, 0x01/*RD*/}, 0x00000 };
  const PPC64Instr instr_lwa =  { {0xE8, 0x00, 0x00, 0x02/*RD*/}, 0x00000 };
  const PPC64Instr instr_std =  { {0xF8, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
  const PPC64Instr instr_stdu =  { {0xF8, 0x00, 0x00, 0x01/*RD*/}, 0x00000 };
  const PPC64Instr instr_mulhhwu =  { {0x10, 0x00, 0x00, 0x10/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_machhwu =  { {0x10, 0x00, 0x00, 0x18/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_mulhhw =  { {0x10, 0x00, 0x00, 0x50/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmachhw =  { {0x10, 0x00, 0x00, 0x5C/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_machhwsu =  { {0x10, 0x00, 0x00, 0x98/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_machhws =  { {0x10, 0x00, 0x00, 0xD8/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmachhws =  { {0x10, 0x00, 0x00, 0xDC/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_mulchwu =  { {0x10, 0x00, 0x01, 0x10/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_macchwu =  { {0x10, 0x00, 0x01, 0x18/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_mulchw =  { {0x10, 0x00, 0x01, 0x50/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_macchw =  { {0x10, 0x00, 0x01, 0x58/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmacchw =  { {0x10, 0x00, 0x01, 0x5C/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_macchwsu =  { {0x10, 0x00, 0x01, 0x98/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_macchws =  { {0x10, 0x00, 0x01, 0xD8/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmacchws =  { {0x10, 0x00, 0x01, 0xDC/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_mullhw =  { {0x10, 0x00, 0x03, 0x50/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_maclhw =  { {0x10, 0x00, 0x03, 0x58/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmaclhw =  { {0x10, 0x00, 0x03, 0x5C/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_maclhwsu =  { {0x10, 0x00, 0x03, 0x98/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_maclhws =  { {0x10, 0x00, 0x03, 0xD8/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmaclhws =  { {0x10, 0x00, 0x03, 0xDC/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_machhwuo =  { {0x10, 0x00, 0x04, 0x18/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmachhwo =  { {0x10, 0x00, 0x04, 0x5C/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_machhwsuo =  { {0x10, 0x00, 0x04, 0x98/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_machhwso =  { {0x10, 0x00, 0x04, 0xD8/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmachhwso =  { {0x10, 0x00, 0x04, 0xDC/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_macchwuo =  { {0x10, 0x00, 0x05, 0x18/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_macchwo =  { {0x10, 0x00, 0x05, 0x58/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmacchwo =  { {0x10, 0x00, 0x05, 0x5C/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_macchwsuo =  { {0x10, 0x00, 0x05, 0x98/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_macchwso =  { {0x10, 0x00, 0x05, 0xD8/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmacchwso =  { {0x10, 0x00, 0x05, 0xDC/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_maclhwo =  { {0x10, 0x00, 0x07, 0x58/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmaclhwo =  { {0x10, 0x00, 0x07, 0x5C/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_maclhwsuo =  { {0x10, 0x00, 0x07, 0x98/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_maclhwso =  { {0x10, 0x00, 0x07, 0xD8/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_nmaclhwso =  { {0x10, 0x00, 0x07, 0xDC/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_vaddubm =  { {0x10, 0x00, 0x00, 0x00/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmaxub =  { {0x10, 0x00, 0x00, 0x02/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vrlb =  { {0x10, 0x00, 0x00, 0x04/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpequb =  { {0x10, 0x00, 0x00, 0x06/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmuloub =  { {0x10, 0x00, 0x00, 0x08/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vaddfp =  { {0x10, 0x00, 0x00, 0x0A/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmrghb =  { {0x10, 0x00, 0x00, 0x0C/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpkuhum =  { {0x10, 0x00, 0x00, 0x0E/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmhaddshs =  { {0x10, 0x00, 0x00, 0x20/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vmhraddshs =  { {0x10, 0x00, 0x00, 0x21/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vmladduhm =  { {0x10, 0x00, 0x00, 0x22/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vmsumubm =  { {0x10, 0x00, 0x00, 0x24/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vmsummbm =  { {0x10, 0x00, 0x00, 0x25/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vmsumuhm =  { {0x10, 0x00, 0x00, 0x26/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vmsumuhs =  { {0x10, 0x00, 0x00, 0x27/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vmsumshm =  { {0x10, 0x00, 0x00, 0x28/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vmsumshs =  { {0x10, 0x00, 0x00, 0x29/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vsel =  { {0x10, 0x00, 0x00, 0x2A/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vperm =  { {0x10, 0x00, 0x00, 0x2B/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vsldoi =  { {0x10, 0x00, 0x00, 0x2C/*VVVP*/}, 0x00000 };
  const PPC64Instr instr_vpermxor =  { {0x10, 0x00, 0x00, 0x2D/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vmaddfp =  { {0x10, 0x00, 0x00, 0x2E/*VVVV~*/}, 0x00000 };
  const PPC64Instr instr_vnmsubfp =  { {0x10, 0x00, 0x00, 0x2F/*VVVV~*/}, 0x00000 };
  const PPC64Instr instr_vaddeuqm =  { {0x10, 0x00, 0x00, 0x3C/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vaddecuq =  { {0x10, 0x00, 0x00, 0x3D/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vsubeuqm =  { {0x10, 0x00, 0x00, 0x3E/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vsubecuq =  { {0x10, 0x00, 0x00, 0x3F/*VVVV*/}, 0x00000 };
  const PPC64Instr instr_vadduhm =  { {0x10, 0x00, 0x00, 0x40/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmaxuh =  { {0x10, 0x00, 0x00, 0x42/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vrlh =  { {0x10, 0x00, 0x00, 0x44/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpequh =  { {0x10, 0x00, 0x00, 0x46/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmulouh =  { {0x10, 0x00, 0x00, 0x48/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsubfp =  { {0x10, 0x00, 0x00, 0x4A/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmrghh =  { {0x10, 0x00, 0x00, 0x4C/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpkuwum =  { {0x10, 0x00, 0x00, 0x4E/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vadduwm =  { {0x10, 0x00, 0x00, 0x80/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmaxuw =  { {0x10, 0x00, 0x00, 0x82/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vrlw =  { {0x10, 0x00, 0x00, 0x84/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpequw =  { {0x10, 0x00, 0x00, 0x86/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmulouw =  { {0x10, 0x00, 0x00, 0x88/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmuluwm =  { {0x10, 0x00, 0x00, 0x89/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmrghw =  { {0x10, 0x00, 0x00, 0x8C/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpkuhus =  { {0x10, 0x00, 0x00, 0x8E/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vaddudm =  { {0x10, 0x00, 0x00, 0xC0/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmaxud =  { {0x10, 0x00, 0x00, 0xC2/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vrld =  { {0x10, 0x00, 0x00, 0xC4/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpeqfp =  { {0x10, 0x00, 0x00, 0xC6/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpequd =  { {0x10, 0x00, 0x00, 0xC7/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpkuwus =  { {0x10, 0x00, 0x00, 0xCE/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vadduqm =  { {0x10, 0x00, 0x01, 0x00/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmaxsb =  { {0x10, 0x00, 0x01, 0x02/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vslb =  { {0x10, 0x00, 0x01, 0x04/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmulosb =  { {0x10, 0x00, 0x01, 0x08/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vrefp =  { {0x10, 0x00, 0x01, 0x0A/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vmrglb =  { {0x10, 0x00, 0x01, 0x0C/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpkshus =  { {0x10, 0x00, 0x01, 0x0E/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vaddcuq =  { {0x10, 0x00, 0x01, 0x40/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmaxsh =  { {0x10, 0x00, 0x01, 0x42/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vslh =  { {0x10, 0x00, 0x01, 0x44/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmulosh =  { {0x10, 0x00, 0x01, 0x48/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vrsqrtefp =  { {0x10, 0x00, 0x01, 0x4A/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vmrglh =  { {0x10, 0x00, 0x01, 0x4C/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpkswus =  { {0x10, 0x00, 0x01, 0x4E/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vaddcuw =  { {0x10, 0x00, 0x01, 0x80/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmaxsw =  { {0x10, 0x00, 0x01, 0x82/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vslw =  { {0x10, 0x00, 0x01, 0x84/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmulosw =  { {0x10, 0x00, 0x01, 0x88/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vexptefp =  { {0x10, 0x00, 0x01, 0x8A/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vmrglw =  { {0x10, 0x00, 0x01, 0x8C/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpkshss =  { {0x10, 0x00, 0x01, 0x8E/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmaxsd =  { {0x10, 0x00, 0x01, 0xC2/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsl =  { {0x10, 0x00, 0x01, 0xC4/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpgefp =  { {0x10, 0x00, 0x01, 0xC6/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vlogefp =  { {0x10, 0x00, 0x01, 0xCA/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vpkswss =  { {0x10, 0x00, 0x01, 0xCE/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vadduhs =  { {0x10, 0x00, 0x02, 0x40/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vminuh =  { {0x10, 0x00, 0x02, 0x42/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsrh =  { {0x10, 0x00, 0x02, 0x44/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtuh =  { {0x10, 0x00, 0x02, 0x46/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmuleuh =  { {0x10, 0x00, 0x02, 0x48/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vrfiz =  { {0x10, 0x00, 0x02, 0x4A/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vsplth =  { {0x10, 0x00, 0x02, 0x4C/*VV3*/}, 0x00000 };
  const PPC64Instr instr_vupkhsh =  { {0x10, 0x00, 0x02, 0x4E/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vminuw =  { {0x10, 0x00, 0x02, 0x82/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vminud =  { {0x10, 0x00, 0x02, 0xC2/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtud =  { {0x10, 0x00, 0x02, 0xC7/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vrfim =  { {0x10, 0x00, 0x02, 0xCA/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtsb =  { {0x10, 0x00, 0x03, 0x06/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcfux =  { {0x10, 0x00, 0x03, 0x0A/*VVA~*/}, 0x00000 };
  const PPC64Instr instr_vaddshs =  { {0x10, 0x00, 0x03, 0x40/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vminsh =  { {0x10, 0x00, 0x03, 0x42/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsrah =  { {0x10, 0x00, 0x03, 0x44/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtsh =  { {0x10, 0x00, 0x03, 0x46/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmulesh =  { {0x10, 0x00, 0x03, 0x48/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcfsx =  { {0x10, 0x00, 0x03, 0x4A/*VVA~*/}, 0x00000 };
  const PPC64Instr instr_vspltish =  { {0x10, 0x00, 0x03, 0x4C/*VS*/}, 0x00000 };
  const PPC64Instr instr_vupkhpx =  { {0x10, 0x00, 0x03, 0x4E/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vaddsws =  { {0x10, 0x00, 0x03, 0x80/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vminsw =  { {0x10, 0x00, 0x03, 0x82/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsraw =  { {0x10, 0x00, 0x03, 0x84/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtsw =  { {0x10, 0x00, 0x03, 0x86/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmulesw =  { {0x10, 0x00, 0x03, 0x88/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vctuxs =  { {0x10, 0x00, 0x03, 0x8A/*VVA~*/}, 0x00000 };
  const PPC64Instr instr_vspltisw =  { {0x10, 0x00, 0x03, 0x8C/*VS*/}, 0x00000 };
  const PPC64Instr instr_vminsd =  { {0x10, 0x00, 0x03, 0xC2/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsrad =  { {0x10, 0x00, 0x03, 0xC4/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpbfp =  { {0x10, 0x00, 0x03, 0xC6/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtsd =  { {0x10, 0x00, 0x03, 0xC7/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vctsxs =  { {0x10, 0x00, 0x03, 0xCA/*VVA~*/}, 0x00000 };
  const PPC64Instr instr_vupklpx =  { {0x10, 0x00, 0x03, 0xCE/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vsububm =  { {0x10, 0x00, 0x04, 0x00/*VVV*/}, 0x00000 };
  const PPC64Instr instr_bcdadd_ =  { {0x10, 0x00, 0x04, 0x01/*VVVY.*/}, 0x00000 };
  const PPC64Instr instr_vavgub =  { {0x10, 0x00, 0x04, 0x02/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vand =  { {0x10, 0x00, 0x04, 0x04/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpequb_ =  { {0x10, 0x00, 0x04, 0x06/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmaxfp =  { {0x10, 0x00, 0x04, 0x0A/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsubuhm =  { {0x10, 0x00, 0x04, 0x40/*VVV*/}, 0x00000 };
  const PPC64Instr instr_bcdsub_ =  { {0x10, 0x00, 0x04, 0x41/*VVVY.*/}, 0x00000 };
  const PPC64Instr instr_vavguh =  { {0x10, 0x00, 0x04, 0x42/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vandc =  { {0x10, 0x00, 0x04, 0x44/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpequh_ =  { {0x10, 0x00, 0x04, 0x46/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vminfp =  { {0x10, 0x00, 0x04, 0x4A/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpkudum =  { {0x10, 0x00, 0x04, 0x4E/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsubuwm =  { {0x10, 0x00, 0x04, 0x80/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vavguw =  { {0x10, 0x00, 0x04, 0x82/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vor =  { {0x10, 0x00, 0x04, 0x84/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpequw_ =  { {0x10, 0x00, 0x04, 0x86/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpmsumw =  { {0x10, 0x00, 0x04, 0x88/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpeqfp_ =  { {0x10, 0x00, 0x04, 0xC6/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpequd_ =  { {0x10, 0x00, 0x04, 0xC7/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpkudus =  { {0x10, 0x00, 0x04, 0xCE/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vavgsb =  { {0x10, 0x00, 0x05, 0x02/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vavgsh =  { {0x10, 0x00, 0x05, 0x42/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vorc =  { {0x10, 0x00, 0x05, 0x44/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vbpermq =  { {0x10, 0x00, 0x05, 0x4C/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpksdus =  { {0x10, 0x00, 0x05, 0x4E/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vavgsw =  { {0x10, 0x00, 0x05, 0x82/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsld =  { {0x10, 0x00, 0x05, 0xC4/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpgefp_ =  { {0x10, 0x00, 0x05, 0xC6/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vpksdss =  { {0x10, 0x00, 0x05, 0xCE/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsububs =  { {0x10, 0x00, 0x06, 0x00/*VVV*/}, 0x00000 };
  const PPC64Instr instr_mfvscr =  { {0x10, 0x00, 0x06, 0x04/*V--*/}, 0x00000 };
  const PPC64Instr instr_vsum4ubs =  { {0x10, 0x00, 0x06, 0x08/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsubuhs =  { {0x10, 0x00, 0x06, 0x40/*VVV*/}, 0x00000 };
  const PPC64Instr instr_mtvscr =  { {0x10, 0x00, 0x06, 0x44/*--V*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtuh_ =  { {0x10, 0x00, 0x06, 0x46/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsum4shs =  { {0x10, 0x00, 0x06, 0x48/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vupkhsw =  { {0x10, 0x00, 0x06, 0x4E/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vsubuws =  { {0x10, 0x00, 0x06, 0x80/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vshasigmaw =  { {0x10, 0x00, 0x06, 0x82/*VVYP*/}, 0x00000 };
  const PPC64Instr instr_veqv =  { {0x10, 0x00, 0x06, 0x84/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsum2sws =  { {0x10, 0x00, 0x06, 0x88/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmrgow =  { {0x10, 0x00, 0x06, 0x8C/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vshasigmad =  { {0x10, 0x00, 0x06, 0xC2/*VVYP*/}, 0x00000 };
  const PPC64Instr instr_vsrd =  { {0x10, 0x00, 0x06, 0xC4/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtud_ =  { {0x10, 0x00, 0x06, 0xC7/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vupklsw =  { {0x10, 0x00, 0x06, 0xCE/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vupkslw =  { {0x10, 0x00, 0x06, 0xCE/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vsubsbs =  { {0x10, 0x00, 0x07, 0x00/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vclzb =  { {0x10, 0x00, 0x07, 0x02/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vpopcntb =  { {0x10, 0x00, 0x07, 0x03/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtsb_ =  { {0x10, 0x00, 0x07, 0x06/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsum4sbs =  { {0x10, 0x00, 0x07, 0x08/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsubshs =  { {0x10, 0x00, 0x07, 0x40/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vclzh =  { {0x10, 0x00, 0x07, 0x42/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vpopcnth =  { {0x10, 0x00, 0x07, 0x43/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtsh_ =  { {0x10, 0x00, 0x07, 0x46/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsubsws =  { {0x10, 0x00, 0x07, 0x80/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vclzw =  { {0x10, 0x00, 0x07, 0x82/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vpopcntw =  { {0x10, 0x00, 0x07, 0x83/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtsw_ =  { {0x10, 0x00, 0x07, 0x86/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vsumsws =  { {0x10, 0x00, 0x07, 0x88/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vmrgew =  { {0x10, 0x00, 0x07, 0x8C/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vclzd =  { {0x10, 0x00, 0x07, 0xC2/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vpopcntd =  { {0x10, 0x00, 0x07, 0xC3/*V-V*/}, 0x00000 };
  const PPC64Instr instr_vcmpbfp_ =  { {0x10, 0x00, 0x07, 0xC6/*VVV*/}, 0x00000 };
  const PPC64Instr instr_vcmpgtsd_ =  { {0x10, 0x00, 0x07, 0xC7/*VVV*/}, 0x00000 };
  const PPC64Instr instr_mcrf =  { {0x4C, 0x00, 0x00, 0x00/*XX*/}, 0x00000 };
  const PPC64Instr instr_isync_0 =  { {0x4C, 0x00, 0x01, 0x2C/**/}, 0x00000 };
  const PPC64Instr instr_crnor =  { {0x4C, 0x00, 0x00, 0x42/*CCC*/}, 0x00000 };
  const PPC64Instr instr_crnot =  { {0x4C, 0x00, 0x00, 0x42/*CC=*/}, 0x00000 };
  const PPC64Instr instr_crandc =  { {0x4C, 0x00, 0x01, 0x02/*CCC*/}, 0x00000 };
  const PPC64Instr instr_crxor =  { {0x4C, 0x00, 0x01, 0x82/*CCC*/}, 0x00000 };
  const PPC64Instr instr_crclr =  { {0x4C, 0x00, 0x01, 0x82/*C==*/}, 0x00000 };
  const PPC64Instr instr_crnand =  { {0x4C, 0x00, 0x01, 0xC2/*CCC*/}, 0x00000 };
  const PPC64Instr instr_crand =  { {0x4C, 0x00, 0x02, 0x02/*CCC*/}, 0x00000 };
  const PPC64Instr instr_creqv =  { {0x4C, 0x00, 0x02, 0x42/*CCC*/}, 0x00000 };
  const PPC64Instr instr_crset =  { {0x4C, 0x00, 0x02, 0x42/*C==*/}, 0x00000 };
  const PPC64Instr instr_crorc =  { {0x4C, 0x00, 0x03, 0x42/*CCC*/}, 0x00000 };
  const PPC64Instr instr_cror =  { {0x4C, 0x00, 0x03, 0x82/*CCC*/}, 0x00000 };
  const PPC64Instr instr_crmove =  { {0x4C, 0x00, 0x03, 0x82/*CC=*/}, 0x00000 };
  const PPC64Instr instr_bclr =  { {0x4C, 0x00, 0x00, 0x20/*AA*/}, 0x00000 };
  const PPC64Instr instr_bclrl =  { {0x4C, 0x00, 0x00, 0x21/*AA*/}, 0x00000 };
  const PPC64Instr instr_bcctr =  { {0x4C, 0x00, 0x04, 0x20/*AA*/}, 0x00000 };
  const PPC64Instr instr_bcctrl =  { {0x4C, 0x00, 0x04, 0x21/*AA*/}, 0x00000 };
  const PPC64Instr instr_bctar =  { {0x4C, 0x00, 0x04, 0x60/*AA*/}, 0x00000 };
  const PPC64Instr instr_bctarl =  { {0x4C, 0x00, 0x04, 0x61/*AA*/}, 0x00000 };
  const PPC64Instr instr_blr_0 =  { {0x4E, 0x80, 0x00, 0x20/**/}, 0x00000 };
  const PPC64Instr instr_blrl_0 =  { {0x4E, 0x80, 0x00, 0x21/**/}, 0x00000 };
  const PPC64Instr instr_bctr_0 =  { {0x4E, 0x80, 0x04, 0x20/**/}, 0x00000 };
  const PPC64Instr instr_bctrl_0 =  { {0x4E, 0x80, 0x04, 0x21/**/}, 0x00000 };
  const PPC64Instr instr_cmpw =  { {0x7C, 0x00, 0x00, 0x00/*XRR*/}, 0x00000 };
  const PPC64Instr instr_cmpw_ =  { {0x7C, 0x00, 0x00, 0x00/*-RR*/}, 0x00000 };
  const PPC64Instr instr_cmpd =  { {0x7C, 0x20, 0x00, 0x00/*XRR*/}, 0x00000 };
  const PPC64Instr instr_cmpd_ =  { {0x7C, 0x20, 0x00, 0x00/*-RR*/}, 0x00000 };
  const PPC64Instr instr_tw =  { {0x7C, 0x00, 0x00, 0x08/*ARR*/}, 0x00000 };
  const PPC64Instr instr_lvsl =  { {0x7C, 0x00, 0x00, 0x0C/*VRR*/}, 0x00000 };
  const PPC64Instr instr_subfc =  { {0x7C, 0x00, 0x00, 0x10/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_subc =  { {0x7C, 0x00, 0x00, 0x10/*RRR~.*/}, 0x00000 };
  const PPC64Instr instr_mulhdu =  { {0x7C, 0x00, 0x00, 0x12/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_addc =  { {0x7C, 0x00, 0x00, 0x14/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_mulhwu =  { {0x7C, 0x00, 0x00, 0x16/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_isel =  { {0x7C, 0x00, 0x00, 0x1E/*RRRC*/}, 0x00000 };
  const PPC64Instr instr_isellt =  { {0x7C, 0x00, 0x00, 0x1E/*RRR*/}, 0x00000 };
  const PPC64Instr instr_iselgt =  { {0x7C, 0x00, 0x00, 0x5E/*RRR*/}, 0x00000 };
  const PPC64Instr instr_iseleq =  { {0x7C, 0x00, 0x00, 0x9E/*RRR*/}, 0x00000 };
  const PPC64Instr instr_mfcr =  { {0x7C, 0x00, 0x00, 0x26/*R*/}, 0x00000 };
  const PPC64Instr instr_mfocrf =  { {0x7C, 0x10, 0x00, 0x26/*RG*/}, 0x00000 };
  const PPC64Instr instr_mtcrf =  { {0x7C, 0x00, 0x01, 0x20/*GR*/}, 0x00000 };
  const PPC64Instr instr_mtocrf =  { {0x7C, 0x10, 0x01, 0x20/*GR*/}, 0x00000 };
  const PPC64Instr instr_lwarx =  { {0x7C, 0x00, 0x00, 0x28/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_ldx =  { {0x7C, 0x00, 0x00, 0x2A/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lwzx =  { {0x7C, 0x00, 0x00, 0x2E/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_slw =  { {0x7C, 0x00, 0x00, 0x30/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_cntlzw =  { {0x7C, 0x00, 0x00, 0x34/*RR~*/}, 0x00000 };
  const PPC64Instr instr_sld =  { {0x7C, 0x00, 0x00, 0x36/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_and =  { {0x7C, 0x00, 0x00, 0x38/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_cmplw =  { {0x7C, 0x00, 0x00, 0x40/*XRR*/}, 0x00000 };
  const PPC64Instr instr_cmplw_ =  { {0x7C, 0x00, 0x00, 0x40/*-RR*/}, 0x00000 };
  const PPC64Instr instr_cmpld =  { {0x7C, 0x20, 0x00, 0x40/*XRR*/}, 0x00000 };
  const PPC64Instr instr_cmpld_ =  { {0x7C, 0x20, 0x00, 0x40/*-RR*/}, 0x00000 };
  const PPC64Instr instr_lvsr =  { {0x7C, 0x00, 0x00, 0x4C/*VRR*/}, 0x00000 };
  const PPC64Instr instr_subf =  { {0x7C, 0x00, 0x00, 0x50/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_sub =  { {0x7C, 0x00, 0x00, 0x50/*RRR~.*/}, 0x00000 };
  const PPC64Instr instr_lbarx =  { {0x7C, 0x00, 0x00, 0x68/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_ldux =  { {0x7C, 0x00, 0x00, 0x6A/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_dcbst =  { {0x7C, 0x00, 0x00, 0x6C/*-RR*/}, 0x00000 };
  const PPC64Instr instr_lwzux =  { {0x7C, 0x00, 0x00, 0x6E/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_cntlzd =  { {0x7C, 0x00, 0x00, 0x74/*RR~*/}, 0x00000 };
  const PPC64Instr instr_andc =  { {0x7C, 0x00, 0x00, 0x78/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_td =  { {0x7C, 0x00, 0x00, 0x88/*ARR*/}, 0x00000 };
  const PPC64Instr instr_lvewx =  { {0x7C, 0x00, 0x00, 0x8E/*VRR*/}, 0x00000 };
  const PPC64Instr instr_mulhd =  { {0x7C, 0x00, 0x00, 0x92/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_addg6s =  { {0x7C, 0x00, 0x00, 0x94/*RRR*/}, 0x00000 };
  const PPC64Instr instr_mulhw =  { {0x7C, 0x00, 0x00, 0x96/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_dlmzb =  { {0x7C, 0x00, 0x00, 0x9C/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_ldarx =  { {0x7C, 0x00, 0x00, 0xA8/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_dcbf =  { {0x7C, 0x00, 0x00, 0xAC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_lbzx =  { {0x7C, 0x00, 0x00, 0xAE/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lvx =  { {0x7C, 0x00, 0x00, 0xCE/*VRR*/}, 0x00000 };
  const PPC64Instr instr_neg =  { {0x7C, 0x00, 0x00, 0xD0/*RR.*/}, 0x00000 };
  const PPC64Instr instr_lharx =  { {0x7C, 0x00, 0x00, 0xE8/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lbzux =  { {0x7C, 0x00, 0x00, 0xEE/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_popcntb =  { {0x7C, 0x00, 0x00, 0xF4/*RR~*/}, 0x00000 };
  const PPC64Instr instr_not =  { {0x7C, 0x00, 0x00, 0xF8/*RR~%.*/}, 0x00000 };
  const PPC64Instr instr_nor =  { {0x7C, 0x00, 0x00, 0xF8/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_stvebx =  { {0x7C, 0x00, 0x01, 0x0E/*VRR*/}, 0x00000 };
  const PPC64Instr instr_subfe =  { {0x7C, 0x00, 0x01, 0x10/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_sube =  { {0x7C, 0x00, 0x01, 0x10/*RRR~.*/}, 0x00000 };
  const PPC64Instr instr_adde =  { {0x7C, 0x00, 0x01, 0x14/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_stdx =  { {0x7C, 0x00, 0x01, 0x2A/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_stwcx_ =  { {0x7C, 0x00, 0x01, 0x2D/*RR0R.*/}, 0x00000 };
  const PPC64Instr instr_stwx =  { {0x7C, 0x00, 0x01, 0x2E/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_prtyw =  { {0x7C, 0x00, 0x01, 0x34/*RR~*/}, 0x00000 };
  const PPC64Instr instr_stvehx =  { {0x7C, 0x00, 0x01, 0x4E/*VRR*/}, 0x00000 };
  const PPC64Instr instr_stdux =  { {0x7C, 0x00, 0x01, 0x6A/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_stqcx_ =  { {0x7C, 0x00, 0x01, 0x6D/*R:R0R.*/}, 0x00000 };
  const PPC64Instr instr_stwux =  { {0x7C, 0x00, 0x01, 0x6E/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_prtyd =  { {0x7C, 0x00, 0x01, 0x74/*RR~*/}, 0x00000 };
  const PPC64Instr instr_stvewx =  { {0x7C, 0x00, 0x01, 0x8E/*VRR*/}, 0x00000 };
  const PPC64Instr instr_subfze =  { {0x7C, 0x00, 0x01, 0x90/*RR.*/}, 0x00000 };
  const PPC64Instr instr_addze =  { {0x7C, 0x00, 0x01, 0x94/*RR.*/}, 0x00000 };
  const PPC64Instr instr_stdcx_ =  { {0x7C, 0x00, 0x01, 0xAD/*RR0R.*/}, 0x00000 };
  const PPC64Instr instr_stbx =  { {0x7C, 0x00, 0x01, 0xAE/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_stvx =  { {0x7C, 0x00, 0x01, 0xCE/*VRR*/}, 0x00000 };
  const PPC64Instr instr_subfme =  { {0x7C, 0x00, 0x01, 0xD0/*RR.*/}, 0x00000 };
  const PPC64Instr instr_mulld =  { {0x7C, 0x00, 0x01, 0xD2/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_addme =  { {0x7C, 0x00, 0x01, 0xD4/*RR.*/}, 0x00000 };
  const PPC64Instr instr_mullw =  { {0x7C, 0x00, 0x01, 0xD6/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_dcbtst =  { {0x7C, 0x00, 0x01, 0xEC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_stbux =  { {0x7C, 0x00, 0x01, 0xEE/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_bpermd =  { {0x7C, 0x00, 0x01, 0xF8/*RR~R*/}, 0x00000 };
  const PPC64Instr instr_lvepxl =  { {0x7C, 0x00, 0x02, 0x0E/*VRR*/}, 0x00000 };
  const PPC64Instr instr_add =  { {0x7C, 0x00, 0x02, 0x14/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_lqarx =  { {0x7C, 0x00, 0x02, 0x28/*R:R0R*/}, 0x00000 };
  const PPC64Instr instr_dcbt =  { {0x7C, 0x00, 0x02, 0x2C/*-RR*/}, 0x00000 };
  const PPC64Instr instr_lhzx =  { {0x7C, 0x00, 0x02, 0x2E/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_cdtbcd =  { {0x7C, 0x00, 0x02, 0x34/*RR~*/}, 0x00000 };
  const PPC64Instr instr_eqv =  { {0x7C, 0x00, 0x02, 0x38/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_lvepx =  { {0x7C, 0x00, 0x02, 0x4E/*VRR*/}, 0x00000 };
  const PPC64Instr instr_eciwx =  { {0x7C, 0x00, 0x02, 0x6C/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lhzux =  { {0x7C, 0x00, 0x02, 0x6E/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_cbcdtd =  { {0x7C, 0x00, 0x02, 0x74/*RR~*/}, 0x00000 };
  const PPC64Instr instr_xor =  { {0x7C, 0x00, 0x02, 0x78/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_mfspefscr =  { {0x7C, 0x00, 0x82, 0xA6/*R*/}, 0x00000 };
  const PPC64Instr instr_mfxer =  { {0x7C, 0x01, 0x02, 0xA6/*R*/}, 0x00000 };
  const PPC64Instr instr_mflr =  { {0x7C, 0x08, 0x02, 0xA6/*R*/}, 0x00000 };
  const PPC64Instr instr_mfctr =  { {0x7C, 0x09, 0x02, 0xA6/*R*/}, 0x00000 };
  const PPC64Instr instr_lwax =  { {0x7C, 0x00, 0x02, 0xAA/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lhax =  { {0x7C, 0x00, 0x02, 0xAE/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_mftb =  { {0x7C, 0x0C, 0x42, 0xE6/*R*/}, 0x00000 };
  const PPC64Instr instr_mftbu =  { {0x7C, 0x0D, 0x42, 0xE6/*R*/}, 0x00000 };
  const PPC64Instr instr_lvxl =  { {0x7C, 0x00, 0x02, 0xCE/*VRR*/}, 0x00000 };
  const PPC64Instr instr_lwaux =  { {0x7C, 0x00, 0x02, 0xEA/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lhaux =  { {0x7C, 0x00, 0x02, 0xEE/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_popcntw =  { {0x7C, 0x00, 0x02, 0xF4/*RR~*/}, 0x00000 };
  const PPC64Instr instr_divdeu =  { {0x7C, 0x00, 0x03, 0x12/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_divweu =  { {0x7C, 0x00, 0x03, 0x16/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_sthx =  { {0x7C, 0x00, 0x03, 0x2E/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_orc =  { {0x7C, 0x00, 0x03, 0x38/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_ecowx =  { {0x7C, 0x00, 0x03, 0x6C/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_sthux =  { {0x7C, 0x00, 0x03, 0x6E/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_or =  { {0x7C, 0x00, 0x03, 0x78/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_mr =  { {0x7C, 0x00, 0x03, 0x78/*RR~%.*/}, 0x00000 };
  const PPC64Instr instr_divdu =  { {0x7C, 0x00, 0x03, 0x92/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_divwu =  { {0x7C, 0x00, 0x03, 0x96/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_mtspefscr =  { {0x7C, 0x00, 0x83, 0xA6/*R*/}, 0x00000 };
  const PPC64Instr instr_mtxer =  { {0x7C, 0x01, 0x03, 0xA6/*R*/}, 0x00000 };
  const PPC64Instr instr_mtlr =  { {0x7C, 0x08, 0x03, 0xA6/*R*/}, 0x00000 };
  const PPC64Instr instr_mtctr =  { {0x7C, 0x09, 0x03, 0xA6/*R*/}, 0x00000 };
  const PPC64Instr instr_dcbi =  { {0x7C, 0x00, 0x03, 0xAC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_nand =  { {0x7C, 0x00, 0x03, 0xB8/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_dsn =  { {0x7C, 0x00, 0x03, 0xC6/*-RR*/}, 0x00000 };
  const PPC64Instr instr_stvxl =  { {0x7C, 0x00, 0x03, 0xCE/*VRR*/}, 0x00000 };
  const PPC64Instr instr_divd =  { {0x7C, 0x00, 0x03, 0xD2/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_divw =  { {0x7C, 0x00, 0x03, 0xD6/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_popcntd =  { {0x7C, 0x00, 0x03, 0xF4/*RR~*/}, 0x00000 };
  const PPC64Instr instr_cmpb =  { {0x7C, 0x00, 0x03, 0xF8/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_mcrxr =  { {0x7C, 0x00, 0x04, 0x00/*X*/}, 0x00000 };
  const PPC64Instr instr_lbdx =  { {0x7C, 0x00, 0x04, 0x06/*RRR*/}, 0x00000 };
  const PPC64Instr instr_subfco =  { {0x7C, 0x00, 0x04, 0x10/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_subco =  { {0x7C, 0x00, 0x04, 0x10/*RRR~.*/}, 0x00000 };
  const PPC64Instr instr_addco =  { {0x7C, 0x00, 0x04, 0x14/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_ldbrx =  { {0x7C, 0x00, 0x04, 0x28/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lswx =  { {0x7C, 0x00, 0x04, 0x2A/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lwbrx =  { {0x7C, 0x00, 0x04, 0x2C/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lfsx =  { {0x7C, 0x00, 0x04, 0x2E/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_srw =  { {0x7C, 0x00, 0x04, 0x30/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_srd =  { {0x7C, 0x00, 0x04, 0x36/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_lhdx =  { {0x7C, 0x00, 0x04, 0x46/*RRR*/}, 0x00000 };
  const PPC64Instr instr_subfo =  { {0x7C, 0x00, 0x04, 0x50/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_subo =  { {0x7C, 0x00, 0x04, 0x50/*RRR~.*/}, 0x00000 };
  const PPC64Instr instr_lfsux =  { {0x7C, 0x00, 0x04, 0x6E/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_lwdx =  { {0x7C, 0x00, 0x04, 0x86/*RRR*/}, 0x00000 };
  const PPC64Instr instr_lswi =  { {0x7C, 0x00, 0x04, 0xAA/*RR0A*/}, 0x00000 };
  const PPC64Instr instr_sync_0 =  { {0x7C, 0x00, 0x04, 0xAC/**/}, 0x00000 };
  const PPC64Instr instr_lwsync_0 =  { {0x7C, 0x20, 0x04, 0xAC/**/}, 0x00000 };
  const PPC64Instr instr_ptesync_0 =  { {0x7C, 0x40, 0x04, 0xAC/**/}, 0x00000 };
  const PPC64Instr instr_lfdx =  { {0x7C, 0x00, 0x04, 0xAE/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_lddx =  { {0x7C, 0x00, 0x04, 0xC6/*RRR*/}, 0x00000 };
  const PPC64Instr instr_nego =  { {0x7C, 0x00, 0x04, 0xD0/*RR.*/}, 0x00000 };
  const PPC64Instr instr_lfdux =  { {0x7C, 0x00, 0x04, 0xEE/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_stbdx =  { {0x7C, 0x00, 0x05, 0x06/*RRR*/}, 0x00000 };
  const PPC64Instr instr_subfeo =  { {0x7C, 0x00, 0x05, 0x10/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_subeo =  { {0x7C, 0x00, 0x05, 0x10/*RRR~.*/}, 0x00000 };
  const PPC64Instr instr_addeo =  { {0x7C, 0x00, 0x05, 0x14/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_stdbrx =  { {0x7C, 0x00, 0x05, 0x28/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_stswx =  { {0x7C, 0x00, 0x05, 0x2A/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_stwbrx =  { {0x7C, 0x00, 0x05, 0x2C/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_stfsx =  { {0x7C, 0x00, 0x05, 0x2E/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_sthdx =  { {0x7C, 0x00, 0x05, 0x46/*RRR*/}, 0x00000 };
  const PPC64Instr instr_stbcx_ =  { {0x7C, 0x00, 0x05, 0x6D/*RRR*/}, 0x00000 };
  const PPC64Instr instr_stfsux =  { {0x7C, 0x00, 0x05, 0x6E/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_stwdx =  { {0x7C, 0x00, 0x05, 0x86/*RRR*/}, 0x00000 };
  const PPC64Instr instr_subfzeo =  { {0x7C, 0x00, 0x05, 0x90/*RR.*/}, 0x00000 };
  const PPC64Instr instr_addzeo =  { {0x7C, 0x00, 0x05, 0x94/*RR.*/}, 0x00000 };
  const PPC64Instr instr_stswi =  { {0x7C, 0x00, 0x05, 0xAA/*RR0A*/}, 0x00000 };
  const PPC64Instr instr_sthcx_ =  { {0x7C, 0x00, 0x05, 0xAD/*RRR*/}, 0x00000 };
  const PPC64Instr instr_stfdx =  { {0x7C, 0x00, 0x05, 0xAE/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_stddx =  { {0x7C, 0x00, 0x05, 0xC6/*RRR*/}, 0x00000 };
  const PPC64Instr instr_subfmeo =  { {0x7C, 0x00, 0x05, 0xD0/*RR.*/}, 0x00000 };
  const PPC64Instr instr_mulldo =  { {0x7C, 0x00, 0x05, 0xD2/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_addmeo =  { {0x7C, 0x00, 0x05, 0xD4/*RR.*/}, 0x00000 };
  const PPC64Instr instr_mullwo =  { {0x7C, 0x00, 0x05, 0xD6/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_dcba =  { {0x7C, 0x00, 0x05, 0xEC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_stfdux =  { {0x7C, 0x00, 0x05, 0xEE/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_stvepxl =  { {0x7C, 0x00, 0x06, 0x0E/*VRR*/}, 0x00000 };
  const PPC64Instr instr_addo =  { {0x7C, 0x00, 0x06, 0x14/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_lhbrx =  { {0x7C, 0x00, 0x06, 0x2C/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_lfdpx =  { {0x7C, 0x00, 0x06, 0x2E/*F:RR*/}, 0x00000 };
  const PPC64Instr instr_sraw =  { {0x7C, 0x00, 0x06, 0x30/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_srad =  { {0x7C, 0x00, 0x06, 0x34/*RR~R.*/}, 0x00000 };
  const PPC64Instr instr_lfddx =  { {0x7C, 0x00, 0x06, 0x46/*FRR*/}, 0x00000 };
  const PPC64Instr instr_stvepx =  { {0x7C, 0x00, 0x06, 0x4E/*VRR*/}, 0x00000 };
  const PPC64Instr instr_srawi =  { {0x7C, 0x00, 0x06, 0x70/*RR~A.*/}, 0x00000 };
  const PPC64Instr instr_sradi =  { {0x7C, 0x00, 0x06, 0x74/*RR~H.*/}, 0x00000 };
  const PPC64Instr instr_eieio_0 =  { {0x7C, 0x00, 0x06, 0xAC/**/}, 0x00000 };
  const PPC64Instr instr_lfiwax =  { {0x7C, 0x00, 0x06, 0xAE/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_divdeuo =  { {0x7C, 0x00, 0x07, 0x12/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_divweuo =  { {0x7C, 0x00, 0x07, 0x16/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_sthbrx =  { {0x7C, 0x00, 0x07, 0x2C/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_stfdpx =  { {0x7C, 0x00, 0x07, 0x2E/*F:RR*/}, 0x00000 };
  const PPC64Instr instr_extsh =  { {0x7C, 0x00, 0x07, 0x34/*RR~.*/}, 0x00000 };
  const PPC64Instr instr_stfddx =  { {0x7C, 0x00, 0x07, 0x46/*FRR*/}, 0x00000 };
  const PPC64Instr instr_divdeo =  { {0x7C, 0x00, 0x07, 0x52/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_divweo =  { {0x7C, 0x00, 0x07, 0x56/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_extsb =  { {0x7C, 0x00, 0x07, 0x74/*RR~.*/}, 0x00000 };
  const PPC64Instr instr_divduo =  { {0x7C, 0x00, 0x07, 0x92/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_divwou =  { {0x7C, 0x00, 0x07, 0x96/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_icbi =  { {0x7C, 0x00, 0x07, 0xAC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_stfiwx =  { {0x7C, 0x00, 0x07, 0xAE/*FR0R*/}, 0x00000 };
  const PPC64Instr instr_extsw =  { {0x7C, 0x00, 0x07, 0xB4/*RR~.*/}, 0x00000 };
  const PPC64Instr instr_divdo =  { {0x7C, 0x00, 0x07, 0xD2/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_divwo =  { {0x7C, 0x00, 0x07, 0xD6/*RRR.*/}, 0x00000 };
  const PPC64Instr instr_dcbz =  { {0x7C, 0x00, 0x07, 0xEC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_tbegin_ =  { {0x7C, 0x00, 0x05, 0x1D/*1*/}, 0x00000 };
  const PPC64Instr instr_tbegin_0 =  { {0x7C, 0x00, 0x05, 0x1D/**/}, 0x00000 };
  const PPC64Instr instr_tend_ =  { {0x7C, 0x00, 0x05, 0x5D/*Y*/}, 0x00000 };
  const PPC64Instr instr_tend_0 =  { {0x7C, 0x00, 0x05, 0x5D/**/}, 0x00000 };
  const PPC64Instr instr_tendall_0 =  { {0x7E, 0x00, 0x05, 0x5D/**/}, 0x00000 };
  const PPC64Instr instr_tcheck =  { {0x7C, 0x00, 0x05, 0x9C/*X*/}, 0x00000 };
  const PPC64Instr instr_tsr_ =  { {0x7C, 0x00, 0x05, 0xDD/*1*/}, 0x00000 };
  const PPC64Instr instr_tsuspend_0 =  { {0x7C, 0x00, 0x05, 0xDD/**/}, 0x00000 };
  const PPC64Instr instr_tresume_0 =  { {0x7C, 0x20, 0x05, 0xDD/**/}, 0x00000 };
  const PPC64Instr instr_tabortwc_ =  { {0x7C, 0x00, 0x06, 0x1D/*ARR*/}, 0x00000 };
  const PPC64Instr instr_tabortdc_ =  { {0x7C, 0x00, 0x06, 0x5D/*ARR*/}, 0x00000 };
  const PPC64Instr instr_tabortwci_ =  { {0x7C, 0x00, 0x06, 0x9D/*ARS*/}, 0x00000 };
  const PPC64Instr instr_tabortdci_ =  { {0x7C, 0x00, 0x06, 0xDD/*ARS*/}, 0x00000 };
  const PPC64Instr instr_tabort_ =  { {0x7C, 0x00, 0x07, 0x1D/*-R-*/}, 0x00000 };
  const PPC64Instr instr_treclaim_ =  { {0x7C, 0x00, 0x07, 0x5D/*-R*/}, 0x00000 };
  const PPC64Instr instr_trechkpt_0 =  { {0x7C, 0x00, 0x07, 0xDD/**/}, 0x00000 };
  const PPC64Instr instr_lxsiwzx =  { {0x7C, 0x00, 0x00, 0x18/*QRR*/}, 0x00000 };
  const PPC64Instr instr_lxsiwax =  { {0x7C, 0x00, 0x00, 0x98/*QRR*/}, 0x00000 };
  const PPC64Instr instr_mfvsrd =  { {0x7C, 0x00, 0x00, 0x66/*-RQ*/}, 0x00000 };
  const PPC64Instr instr_mfvsrwz =  { {0x7C, 0x00, 0x00, 0xE6/*-RQ*/}, 0x00000 };
  const PPC64Instr instr_stxsiwx =  { {0x7C, 0x00, 0x01, 0x18/*QRR*/}, 0x00000 };
  const PPC64Instr instr_mtvsrd =  { {0x7C, 0x00, 0x01, 0x66/*QR*/}, 0x00000 };
  const PPC64Instr instr_mtvsrwa =  { {0x7C, 0x00, 0x01, 0xA6/*QR*/}, 0x00000 };
  const PPC64Instr instr_lxvdsx =  { {0x7C, 0x00, 0x02, 0x98/*QRR*/}, 0x00000 };
  const PPC64Instr instr_lxsspx =  { {0x7C, 0x00, 0x04, 0x18/*QRR*/}, 0x00000 };
  const PPC64Instr instr_lxsdx =  { {0x7C, 0x00, 0x04, 0x98/*QRR*/}, 0x00000 };
  const PPC64Instr instr_stxsspx =  { {0x7C, 0x00, 0x05, 0x18/*QRR*/}, 0x00000 };
  const PPC64Instr instr_stxsdx =  { {0x7C, 0x00, 0x05, 0x98/*QRR*/}, 0x00000 };
  const PPC64Instr instr_lxvw4x =  { {0x7C, 0x00, 0x06, 0x18/*QRR*/}, 0x00000 };
  const PPC64Instr instr_lxvd2x =  { {0x7C, 0x00, 0x06, 0x98/*QRR*/}, 0x00000 };
  const PPC64Instr instr_stxvw4x =  { {0x7C, 0x00, 0x07, 0x18/*QRR*/}, 0x00000 };
  const PPC64Instr instr_stxvd2x =  { {0x7C, 0x00, 0x07, 0x98/*QRR*/}, 0x00000 };
  const PPC64Instr instr_rldicl =  { {0x78, 0x00, 0x00, 0x00/*RR~HM.*/}, 0x00000 };
  const PPC64Instr instr_rldicr =  { {0x78, 0x00, 0x00, 0x04/*RR~HM.*/}, 0x00000 };
  const PPC64Instr instr_rldic =  { {0x78, 0x00, 0x00, 0x08/*RR~HM.*/}, 0x00000 };
  const PPC64Instr instr_rldimi =  { {0x78, 0x00, 0x00, 0x0C/*RR~HM.*/}, 0x00000 };
  const PPC64Instr instr_rldcl =  { {0x78, 0x00, 0x00, 0x10/*RR~RM.*/}, 0x00000 };
  const PPC64Instr instr_rldcr =  { {0x78, 0x00, 0x00, 0x12/*RR~RM.*/}, 0x00000 };
  const PPC64Instr instr_sldi =  { {0x78, 0x00, 0x00, 0x04/*RR~L*/}, 0x00000 };
  const PPC64Instr instr_srdi =  { {0x78, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
  const PPC64Instr instr_lq =  { {0xE0, 0x00, 0x00, 0x00/*R:D*/}, 0x00000 };
  const PPC64Instr instr_lfdp =  { {0xE4, 0x00, 0x00, 0x00/*F:D*/}, 0x00000 };
  const PPC64Instr instr_fdivs =  { {0xEC, 0x00, 0x00, 0x24/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_fsubs =  { {0xEC, 0x00, 0x00, 0x28/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_fadds =  { {0xEC, 0x00, 0x00, 0x2A/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_fsqrts =  { {0xEC, 0x00, 0x00, 0x2C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fres =  { {0xEC, 0x00, 0x00, 0x30/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fmuls =  { {0xEC, 0x00, 0x00, 0x32/*FF-F.*/}, 0x00000 };
  const PPC64Instr instr_frsqrtes =  { {0xEC, 0x00, 0x00, 0x34/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fmsubs =  { {0xEC, 0x00, 0x00, 0x38/*FFFF~.*/}, 0x00000 };
  const PPC64Instr instr_fmadds =  { {0xEC, 0x00, 0x00, 0x3A/*FFFF~.*/}, 0x00000 };
  const PPC64Instr instr_fnmsubs =  { {0xEC, 0x00, 0x00, 0x3C/*FFFF~.*/}, 0x00000 };
  const PPC64Instr instr_fnmadds =  { {0xEC, 0x00, 0x00, 0x3E/*FFFF~.*/}, 0x00000 };
  const PPC64Instr instr_fcfids =  { {0xEC, 0x00, 0x06, 0x9C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fcfidus =  { {0xEC, 0x00, 0x07, 0x9C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_dadd =  { {0xEC, 0x00, 0x00, 0x04/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_dqua =  { {0xEC, 0x00, 0x00, 0x06/*FFFZ.*/}, 0x00000 };
  const PPC64Instr instr_dmul =  { {0xEC, 0x00, 0x00, 0x44/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_drrnd =  { {0xEC, 0x00, 0x00, 0x46/*FFFZ.*/}, 0x00000 };
  const PPC64Instr instr_dscli =  { {0xEC, 0x00, 0x00, 0x84/*FF6.*/}, 0x00000 };
  const PPC64Instr instr_dquai =  { {0xEC, 0x00, 0x00, 0x86/*SF~FZ.*/}, 0x00000 };
  const PPC64Instr instr_dscri =  { {0xEC, 0x00, 0x00, 0xC4/*FF6.*/}, 0x00000 };
  const PPC64Instr instr_drintx =  { {0xEC, 0x00, 0x00, 0xC6/*1F~FZ.*/}, 0x00000 };
  const PPC64Instr instr_dcmpo =  { {0xEC, 0x00, 0x01, 0x04/*XFF*/}, 0x00000 };
  const PPC64Instr instr_dtstex =  { {0xEC, 0x00, 0x01, 0x44/*XFF*/}, 0x00000 };
  const PPC64Instr instr_dtstdc =  { {0xEC, 0x00, 0x01, 0x84/*XF6*/}, 0x00000 };
  const PPC64Instr instr_dtstdg =  { {0xEC, 0x00, 0x01, 0xC4/*XF6*/}, 0x00000 };
  const PPC64Instr instr_drintn =  { {0xEC, 0x00, 0x01, 0xC6/*1F~FZ.*/}, 0x00000 };
  const PPC64Instr instr_dctdp =  { {0xEC, 0x00, 0x02, 0x04/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_dctfix =  { {0xEC, 0x00, 0x02, 0x44/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_ddedpd =  { {0xEC, 0x00, 0x02, 0x84/*ZF~F.*/}, 0x00000 };
  const PPC64Instr instr_dxex =  { {0xEC, 0x00, 0x02, 0xC4/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_dsub =  { {0xEC, 0x00, 0x04, 0x04/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_ddiv =  { {0xEC, 0x00, 0x04, 0x44/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_dcmpu =  { {0xEC, 0x00, 0x05, 0x04/*XFF*/}, 0x00000 };
  const PPC64Instr instr_dtstsf =  { {0xEC, 0x00, 0x05, 0x44/*XFF*/}, 0x00000 };
  const PPC64Instr instr_drsp =  { {0xEC, 0x00, 0x06, 0x04/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_dcffix =  { {0xEC, 0x00, 0x06, 0x44/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_denbcd =  { {0xEC, 0x00, 0x06, 0x84/*YF~F.*/}, 0x00000 };
  const PPC64Instr instr_diex =  { {0xEC, 0x00, 0x06, 0xC4/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_xsaddsp =  { {0xF0, 0x00, 0x00, 0x00/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsmaddasp =  { {0xF0, 0x00, 0x00, 0x08/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxsldwi =  { {0xF0, 0x00, 0x00, 0x10/*QQQZ*/}, 0x00000 };
  const PPC64Instr instr_xsrsqrtesp =  { {0xF0, 0x00, 0x00, 0x28/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xssqrtsp =  { {0xF0, 0x00, 0x00, 0x2C/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xxsel =  { {0xF0, 0x00, 0x00, 0x30/*QQQQ*/}, 0x00000 };
  const PPC64Instr instr_xssubsp =  { {0xF0, 0x00, 0x00, 0x40/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsmaddmsp =  { {0xF0, 0x00, 0x00, 0x48/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxpermdi =  { {0xF0, 0x00, 0x00, 0x50/*QQQZ*/}, 0x00000 };
  const PPC64Instr instr_xsresp =  { {0xF0, 0x00, 0x00, 0x68/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsmulsp =  { {0xF0, 0x00, 0x00, 0x80/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsmsubasp =  { {0xF0, 0x00, 0x00, 0x88/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxmrghw =  { {0xF0, 0x00, 0x00, 0x90/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsdivsp =  { {0xF0, 0x00, 0x00, 0xC0/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsmsubmsp =  { {0xF0, 0x00, 0x00, 0xC8/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsadddp =  { {0xF0, 0x00, 0x01, 0x00/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsmaddadp =  { {0xF0, 0x00, 0x01, 0x08/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xscmpudp =  { {0xF0, 0x00, 0x01, 0x18/*XQQ*/}, 0x00000 };
  const PPC64Instr instr_xscvdpuxws =  { {0xF0, 0x00, 0x01, 0x20/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsrdpi =  { {0xF0, 0x00, 0x01, 0x24/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsrsqrtedp =  { {0xF0, 0x00, 0x01, 0x28/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xssqrtdp =  { {0xF0, 0x00, 0x01, 0x2C/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xssubdp =  { {0xF0, 0x00, 0x01, 0x40/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsmaddmdp =  { {0xF0, 0x00, 0x01, 0x48/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xscmpodp =  { {0xF0, 0x00, 0x01, 0x58/*XQQ*/}, 0x00000 };
  const PPC64Instr instr_xscvdpsxws =  { {0xF0, 0x00, 0x01, 0x60/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsrdpiz =  { {0xF0, 0x00, 0x01, 0x64/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsredp =  { {0xF0, 0x00, 0x01, 0x68/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsmuldp =  { {0xF0, 0x00, 0x01, 0x80/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsmsubadp =  { {0xF0, 0x00, 0x01, 0x88/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxmrglw =  { {0xF0, 0x00, 0x01, 0x90/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsrdpip =  { {0xF0, 0x00, 0x01, 0xA4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xstsqrtdp =  { {0xF0, 0x00, 0x01, 0xA8/*X-Q*/}, 0x00000 };
  const PPC64Instr instr_xsrdpic =  { {0xF0, 0x00, 0x01, 0xAC/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsdivdp =  { {0xF0, 0x00, 0x01, 0xC0/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsmsubmdp =  { {0xF0, 0x00, 0x01, 0xC8/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsrdpim =  { {0xF0, 0x00, 0x01, 0xE4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xstdivdp =  { {0xF0, 0x00, 0x01, 0xE8/*XQQ*/}, 0x00000 };
  const PPC64Instr instr_xvaddsp =  { {0xF0, 0x00, 0x02, 0x00/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvmaddasp =  { {0xF0, 0x00, 0x02, 0x08/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpeqsp =  { {0xF0, 0x00, 0x02, 0x18/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvspuxws =  { {0xF0, 0x00, 0x02, 0x20/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrspi =  { {0xF0, 0x00, 0x02, 0x24/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrsqrtesp =  { {0xF0, 0x00, 0x02, 0x28/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvsqrtsp =  { {0xF0, 0x00, 0x02, 0x2C/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvsubsp =  { {0xF0, 0x00, 0x02, 0x40/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvmaddmsp =  { {0xF0, 0x00, 0x02, 0x48/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpgtsp =  { {0xF0, 0x00, 0x02, 0x58/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvspsxws =  { {0xF0, 0x00, 0x02, 0x60/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrspiz =  { {0xF0, 0x00, 0x02, 0x64/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvresp =  { {0xF0, 0x00, 0x02, 0x68/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvmulsp =  { {0xF0, 0x00, 0x02, 0x80/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvmsubasp =  { {0xF0, 0x00, 0x02, 0x88/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxspltw =  { {0xF0, 0x00, 0x02, 0x90/*QQG~*/}, 0x00000 };
  const PPC64Instr instr_xvcmpgesp =  { {0xF0, 0x00, 0x02, 0x98/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvuxwsp =  { {0xF0, 0x00, 0x02, 0xA0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrspip =  { {0xF0, 0x00, 0x02, 0xA4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvtsqrtsp =  { {0xF0, 0x00, 0x02, 0xA8/*X-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrspic =  { {0xF0, 0x00, 0x02, 0xAC/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvdivsp =  { {0xF0, 0x00, 0x02, 0xC0/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvmsubmsp =  { {0xF0, 0x00, 0x02, 0xC8/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvsxwsp =  { {0xF0, 0x00, 0x02, 0xE0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrspim =  { {0xF0, 0x00, 0x02, 0xE4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvtdivsp =  { {0xF0, 0x00, 0x02, 0xE8/*XQQ*/}, 0x00000 };
  const PPC64Instr instr_xvadddp =  { {0xF0, 0x00, 0x03, 0x00/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvmaddadp =  { {0xF0, 0x00, 0x03, 0x08/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpeqdp =  { {0xF0, 0x00, 0x03, 0x18/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvdpuxws =  { {0xF0, 0x00, 0x03, 0x20/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrdpi =  { {0xF0, 0x00, 0x03, 0x24/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrsqrtedp =  { {0xF0, 0x00, 0x03, 0x28/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvsqrtdp =  { {0xF0, 0x00, 0x03, 0x2C/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvsubdp =  { {0xF0, 0x00, 0x03, 0x40/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvmaddmdp =  { {0xF0, 0x00, 0x03, 0x48/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpgtdp =  { {0xF0, 0x00, 0x03, 0x58/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvdpsxws =  { {0xF0, 0x00, 0x03, 0x60/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrdpiz =  { {0xF0, 0x00, 0x03, 0x64/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvredp =  { {0xF0, 0x00, 0x03, 0x68/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvmuldp =  { {0xF0, 0x00, 0x03, 0x80/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvmsubadp =  { {0xF0, 0x00, 0x03, 0x88/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpgedp =  { {0xF0, 0x00, 0x03, 0x98/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvuxwdp =  { {0xF0, 0x00, 0x03, 0xA0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrdpip =  { {0xF0, 0x00, 0x03, 0xA4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvtsqrtdp =  { {0xF0, 0x00, 0x03, 0xA8/*X-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrdpic =  { {0xF0, 0x00, 0x03, 0xAC/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvdivdp =  { {0xF0, 0x00, 0x03, 0xC0/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvmsubmdp =  { {0xF0, 0x00, 0x03, 0xC8/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvsxwdp =  { {0xF0, 0x00, 0x03, 0xE0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvrdpim =  { {0xF0, 0x00, 0x03, 0xE4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvtdivdp =  { {0xF0, 0x00, 0x03, 0xE8/*XQQ*/}, 0x00000 };
  const PPC64Instr instr_xsnmaddasp =  { {0xF0, 0x00, 0x04, 0x08/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxland =  { {0xF0, 0x00, 0x04, 0x10/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xscvdpsp =  { {0xF0, 0x00, 0x04, 0x24/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xscvdpspn =  { {0xF0, 0x00, 0x04, 0x2C/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsnmaddmsp =  { {0xF0, 0x00, 0x04, 0x48/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxlandc =  { {0xF0, 0x00, 0x04, 0x50/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsrsp =  { {0xF0, 0x00, 0x04, 0x64/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsnmsubasp =  { {0xF0, 0x00, 0x04, 0x88/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxlor =  { {0xF0, 0x00, 0x04, 0x90/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xscvuxdsp =  { {0xF0, 0x00, 0x04, 0xA0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsnmsubmsp =  { {0xF0, 0x00, 0x04, 0xC8/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxlxor =  { {0xF0, 0x00, 0x04, 0xD0/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xscvsxdsp =  { {0xF0, 0x00, 0x04, 0xE0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsmaxdp =  { {0xF0, 0x00, 0x05, 0x00/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsnmaddadp =  { {0xF0, 0x00, 0x05, 0x08/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxlnor =  { {0xF0, 0x00, 0x05, 0x10/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xscvdpuxds =  { {0xF0, 0x00, 0x05, 0x20/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xscvspdp =  { {0xF0, 0x00, 0x05, 0x24/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xscvspdpn =  { {0xF0, 0x00, 0x05, 0x2C/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsmindp =  { {0xF0, 0x00, 0x05, 0x40/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsnmaddmdp =  { {0xF0, 0x00, 0x05, 0x48/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxlorc =  { {0xF0, 0x00, 0x05, 0x50/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xscvdpsxds =  { {0xF0, 0x00, 0x05, 0x60/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsabsdp =  { {0xF0, 0x00, 0x05, 0x64/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xscpsgndp =  { {0xF0, 0x00, 0x05, 0x80/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xsnmsubadp =  { {0xF0, 0x00, 0x05, 0x88/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxlnand =  { {0xF0, 0x00, 0x05, 0x90/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xscvuxddp =  { {0xF0, 0x00, 0x05, 0xA0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsnabsdp =  { {0xF0, 0x00, 0x05, 0xA4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsnmsubmdp =  { {0xF0, 0x00, 0x05, 0xC8/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xxleqv =  { {0xF0, 0x00, 0x05, 0xD0/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xscvsxddp =  { {0xF0, 0x00, 0x05, 0xE0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xsnegdp =  { {0xF0, 0x00, 0x05, 0xE4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvmaxsp =  { {0xF0, 0x00, 0x06, 0x00/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvnmaddasp =  { {0xF0, 0x00, 0x06, 0x08/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpeqsp_ =  { {0xF0, 0x00, 0x06, 0x18/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvspuxds =  { {0xF0, 0x00, 0x06, 0x20/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvcvdpsp =  { {0xF0, 0x00, 0x06, 0x24/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvminsp =  { {0xF0, 0x00, 0x06, 0x40/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvnmaddmsp =  { {0xF0, 0x00, 0x06, 0x48/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpgtsp_ =  { {0xF0, 0x00, 0x06, 0x58/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvspsxds =  { {0xF0, 0x00, 0x06, 0x60/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvabssp =  { {0xF0, 0x00, 0x06, 0x64/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvcpsgnsp =  { {0xF0, 0x00, 0x06, 0x80/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvnmsubasp =  { {0xF0, 0x00, 0x06, 0x88/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpgesp_ =  { {0xF0, 0x00, 0x06, 0x98/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvuxdsp =  { {0xF0, 0x00, 0x06, 0xA0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvnabssp =  { {0xF0, 0x00, 0x06, 0xA4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvnmsubmsp =  { {0xF0, 0x00, 0x06, 0xC8/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvsxdsp =  { {0xF0, 0x00, 0x06, 0xE0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvnegsp =  { {0xF0, 0x00, 0x06, 0xE4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvmaxdp =  { {0xF0, 0x00, 0x07, 0x00/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvnmaddadp =  { {0xF0, 0x00, 0x07, 0x08/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpeqdp_ =  { {0xF0, 0x00, 0x07, 0x18/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvdpuxds =  { {0xF0, 0x00, 0x07, 0x20/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvcvspdp =  { {0xF0, 0x00, 0x07, 0x24/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvmindp =  { {0xF0, 0x00, 0x07, 0x40/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvnmaddmdp =  { {0xF0, 0x00, 0x07, 0x48/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpgtdp_ =  { {0xF0, 0x00, 0x07, 0x58/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvdpsxds =  { {0xF0, 0x00, 0x07, 0x60/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvabsdp =  { {0xF0, 0x00, 0x07, 0x64/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvcpsgndp =  { {0xF0, 0x00, 0x07, 0x80/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvnmsubadp =  { {0xF0, 0x00, 0x07, 0x88/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcmpgedp_ =  { {0xF0, 0x00, 0x07, 0x98/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvuxddp =  { {0xF0, 0x00, 0x07, 0xA0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvnabsdp =  { {0xF0, 0x00, 0x07, 0xA4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvnmsubmdp =  { {0xF0, 0x00, 0x07, 0xC8/*QQQ*/}, 0x00000 };
  const PPC64Instr instr_xvcvsxddp =  { {0xF0, 0x00, 0x07, 0xE0/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_xvnegdp =  { {0xF0, 0x00, 0x07, 0xE4/*Q-Q*/}, 0x00000 };
  const PPC64Instr instr_stfdp =  { {0xF4, 0x00, 0x00, 0x00/*F:D*/}, 0x00000 };
  const PPC64Instr instr_stq =  { {0xF8, 0x00, 0x00, 0x02/*R:D*/}, 0x00000 };
  const PPC64Instr instr_fdiv =  { {0xFC, 0x00, 0x00, 0x24/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_fsub =  { {0xFC, 0x00, 0x00, 0x28/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_fadd =  { {0xFC, 0x00, 0x00, 0x2A/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_fsqrt =  { {0xFC, 0x00, 0x00, 0x2C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fsel =  { {0xFC, 0x00, 0x00, 0x2E/*FFFF~.*/}, 0x00000 };
  const PPC64Instr instr_fre =  { {0xFC, 0x00, 0x00, 0x30/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fmul =  { {0xFC, 0x00, 0x00, 0x32/*FF-F.*/}, 0x00000 };
  const PPC64Instr instr_frsqrte =  { {0xFC, 0x00, 0x00, 0x34/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fmsub =  { {0xFC, 0x00, 0x00, 0x38/*FFFF~.*/}, 0x00000 };
  const PPC64Instr instr_fmadd =  { {0xFC, 0x00, 0x00, 0x3A/*FFFF~.*/}, 0x00000 };
  const PPC64Instr instr_fnmsub =  { {0xFC, 0x00, 0x00, 0x3C/*FFFF~.*/}, 0x00000 };
  const PPC64Instr instr_fnmadd =  { {0xFC, 0x00, 0x00, 0x3E/*FFFF~.*/}, 0x00000 };
  const PPC64Instr instr_fcmpu =  { {0xFC, 0x00, 0x00, 0x00/*XFF*/}, 0x00000 };
  const PPC64Instr instr_fcpsgn =  { {0xFC, 0x00, 0x00, 0x10/*FFF.*/}, 0x00000 };
  const PPC64Instr instr_fcmpo =  { {0xFC, 0x00, 0x00, 0x40/*XFF*/}, 0x00000 };
  const PPC64Instr instr_mtfsb1 =  { {0xFC, 0x00, 0x00, 0x4C/*A*/}, 0x00000 };
  const PPC64Instr instr_fneg =  { {0xFC, 0x00, 0x00, 0x50/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_mcrfs =  { {0xFC, 0x00, 0x00, 0x80/*XX*/}, 0x00000 };
  const PPC64Instr instr_mtfsb0 =  { {0xFC, 0x00, 0x00, 0x8C/*A*/}, 0x00000 };
  const PPC64Instr instr_fmr =  { {0xFC, 0x00, 0x00, 0x90/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_frsp =  { {0xFC, 0x00, 0x00, 0x18/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fctiw =  { {0xFC, 0x00, 0x00, 0x1C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fctiwz =  { {0xFC, 0x00, 0x00, 0x1E/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_ftdiv =  { {0xFC, 0x00, 0x01, 0x00/*X-F.*/}, 0x00000 };
  const PPC64Instr instr_fctiwu =  { {0xFC, 0x00, 0x01, 0x1C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fctiwuz =  { {0xFC, 0x00, 0x01, 0x1E/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_mtfsfi =  { {0xFC, 0x00, 0x01, 0x0C/*AA*/}, 0x00000 };
  const PPC64Instr instr_fnabs =  { {0xFC, 0x00, 0x01, 0x10/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_ftsqrt =  { {0xFC, 0x00, 0x01, 0x40/*X-F.*/}, 0x00000 };
  const PPC64Instr instr_fabs =  { {0xFC, 0x00, 0x02, 0x10/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_frin =  { {0xFC, 0x00, 0x03, 0x10/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_friz =  { {0xFC, 0x00, 0x03, 0x50/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_frip =  { {0xFC, 0x00, 0x03, 0x90/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_frim =  { {0xFC, 0x00, 0x03, 0xD0/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_mffs =  { {0xFC, 0x00, 0x04, 0x8E/*F.*/}, 0x00000 };
  const PPC64Instr instr_fctid =  { {0xFC, 0x00, 0x06, 0x5C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fctidz =  { {0xFC, 0x00, 0x06, 0x5E/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fmrgow =  { {0xFC, 0x00, 0x06, 0x8C/*FFF*/}, 0x00000 };
  const PPC64Instr instr_fcfid =  { {0xFC, 0x00, 0x06, 0x9C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fctidu =  { {0xFC, 0x00, 0x07, 0x5C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fctiduz =  { {0xFC, 0x00, 0x07, 0x5E/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_fmrgew =  { {0xFC, 0x00, 0x07, 0x8C/*FFF*/}, 0x00000 };
  const PPC64Instr instr_fcfidu =  { {0xFC, 0x00, 0x07, 0x9C/*F-F.*/}, 0x00000 };
  const PPC64Instr instr_daddq =  { {0xFC, 0x00, 0x00, 0x04/*F:F:F:.*/}, 0x00000 };
  const PPC64Instr instr_dquaq =  { {0xFC, 0x00, 0x00, 0x06/*F:F:F:Z.*/}, 0x00000 };
  const PPC64Instr instr_dmulq =  { {0xFC, 0x00, 0x00, 0x44/*F:F:F:.*/}, 0x00000 };
  const PPC64Instr instr_drrndq =  { {0xFC, 0x00, 0x00, 0x46/*F:F:F:Z.*/}, 0x00000 };
  const PPC64Instr instr_dscliq =  { {0xFC, 0x00, 0x00, 0x84/*F:F:6.*/}, 0x00000 };
  const PPC64Instr instr_dquaiq =  { {0xFC, 0x00, 0x00, 0x86/*SF:~F:Z.*/}, 0x00000 };
  const PPC64Instr instr_dscriq =  { {0xFC, 0x00, 0x00, 0xC4/*F:F:6.*/}, 0x00000 };
  const PPC64Instr instr_drintxq =  { {0xFC, 0x00, 0x00, 0xC6/*1F:~F:Z.*/}, 0x00000 };
  const PPC64Instr instr_dcmpoq =  { {0xFC, 0x00, 0x01, 0x04/*XF:F:*/}, 0x00000 };
  const PPC64Instr instr_dtstexq =  { {0xFC, 0x00, 0x01, 0x44/*XF:F:*/}, 0x00000 };
  const PPC64Instr instr_dtstdcq =  { {0xFC, 0x00, 0x01, 0x84/*XF:6*/}, 0x00000 };
  const PPC64Instr instr_dtstdgq =  { {0xFC, 0x00, 0x01, 0xC4/*XF:6*/}, 0x00000 };
  const PPC64Instr instr_drintnq =  { {0xFC, 0x00, 0x01, 0xC6/*1F:~F:Z.*/}, 0x00000 };
  const PPC64Instr instr_dctqpq =  { {0xFC, 0x00, 0x02, 0x04/*F:-F:.*/}, 0x00000 };
  const PPC64Instr instr_dctfixq =  { {0xFC, 0x00, 0x02, 0x44/*F:-F:.*/}, 0x00000 };
  const PPC64Instr instr_ddedpdq =  { {0xFC, 0x00, 0x02, 0x84/*ZF:~F:.*/}, 0x00000 };
  const PPC64Instr instr_dxexq =  { {0xFC, 0x00, 0x02, 0xC4/*F:-F:.*/}, 0x00000 };
  const PPC64Instr instr_dsubq =  { {0xFC, 0x00, 0x04, 0x04/*F:F:F:.*/}, 0x00000 };
  const PPC64Instr instr_ddivq =  { {0xFC, 0x00, 0x04, 0x44/*F:F:F:.*/}, 0x00000 };
  const PPC64Instr instr_dcmpuq =  { {0xFC, 0x00, 0x05, 0x04/*XF:F:*/}, 0x00000 };
  const PPC64Instr instr_dtstsfq =  { {0xFC, 0x00, 0x05, 0x44/*XF:F:*/}, 0x00000 };
  const PPC64Instr instr_drdpq =  { {0xFC, 0x00, 0x06, 0x04/*F:-F:.*/}, 0x00000 };
  const PPC64Instr instr_dcffixq =  { {0xFC, 0x00, 0x06, 0x44/*F:-F:.*/}, 0x00000 };
  const PPC64Instr instr_denbcdq =  { {0xFC, 0x00, 0x06, 0x84/*YF:~F:.*/}, 0x00000 };
  const PPC64Instr instr_diexq =  { {0xFC, 0x00, 0x06, 0xC4/*F:FF:.*/}, 0x00000 };
  const PPC64Instr instr_evaddw =  { {0x10, 0x00, 0x02, 0x00/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evaddiw =  { {0x10, 0x00, 0x02, 0x02/*RAR~*/}, 0x00000 };
  const PPC64Instr instr_evsubw =  { {0x10, 0x00, 0x02, 0x04/*RRR~*/}, 0x00000 };
  const PPC64Instr instr_evsubiw =  { {0x10, 0x00, 0x02, 0x06/*RAR~*/}, 0x00000 };
  const PPC64Instr instr_evabs =  { {0x10, 0x00, 0x02, 0x08/*RR*/}, 0x00000 };
  const PPC64Instr instr_evneg =  { {0x10, 0x00, 0x02, 0x09/*RR*/}, 0x00000 };
  const PPC64Instr instr_evextsb =  { {0x10, 0x00, 0x02, 0x0A/*RR*/}, 0x00000 };
  const PPC64Instr instr_evextsh =  { {0x10, 0x00, 0x02, 0x0B/*RR*/}, 0x00000 };
  const PPC64Instr instr_evrndw =  { {0x10, 0x00, 0x02, 0x0C/*RR*/}, 0x00000 };
  const PPC64Instr instr_evcntlzw =  { {0x10, 0x00, 0x02, 0x0D/*RR*/}, 0x00000 };
  const PPC64Instr instr_evcntlsw =  { {0x10, 0x00, 0x02, 0x0E/*RR*/}, 0x00000 };
  const PPC64Instr instr_brinc =  { {0x10, 0x00, 0x02, 0x0F/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evand =  { {0x10, 0x00, 0x02, 0x11/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evandc =  { {0x10, 0x00, 0x02, 0x12/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evxor =  { {0x10, 0x00, 0x02, 0x16/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evor =  { {0x10, 0x00, 0x02, 0x17/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmr =  { {0x10, 0x00, 0x02, 0x17/*RR=*/}, 0x00000 };
  const PPC64Instr instr_evnor =  { {0x10, 0x00, 0x02, 0x18/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evnot =  { {0x10, 0x00, 0x02, 0x18/*RR=*/}, 0x00000 };
  const PPC64Instr instr_eveqv =  { {0x10, 0x00, 0x02, 0x19/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evorc =  { {0x10, 0x00, 0x02, 0x1B/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evnand =  { {0x10, 0x00, 0x02, 0x1E/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evsrwu =  { {0x10, 0x00, 0x02, 0x20/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evsrws =  { {0x10, 0x00, 0x02, 0x21/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evsrwiu =  { {0x10, 0x00, 0x02, 0x22/*RRA*/}, 0x00000 };
  const PPC64Instr instr_evsrwis =  { {0x10, 0x00, 0x02, 0x23/*RRA*/}, 0x00000 };
  const PPC64Instr instr_evslw =  { {0x10, 0x00, 0x02, 0x24/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evslwi =  { {0x10, 0x00, 0x02, 0x26/*RRA*/}, 0x00000 };
  const PPC64Instr instr_evrlw =  { {0x10, 0x00, 0x02, 0x28/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evsplati =  { {0x10, 0x00, 0x02, 0x29/*RS*/}, 0x00000 };
  const PPC64Instr instr_evrlwi =  { {0x10, 0x00, 0x02, 0x2A/*RRA*/}, 0x00000 };
  const PPC64Instr instr_evsplatfi =  { {0x10, 0x00, 0x02, 0x2B/*RS*/}, 0x00000 };
  const PPC64Instr instr_evmergehi =  { {0x10, 0x00, 0x02, 0x2C/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmergelo =  { {0x10, 0x00, 0x02, 0x2D/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evcmpgtu =  { {0x10, 0x00, 0x02, 0x30/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evcmpgtu_ =  { {0x10, 0x00, 0x02, 0x30/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evcmpgts =  { {0x10, 0x00, 0x02, 0x31/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evcmpgts_ =  { {0x10, 0x00, 0x02, 0x31/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evcmpltu =  { {0x10, 0x00, 0x02, 0x32/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evcmpltu_ =  { {0x10, 0x00, 0x02, 0x32/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evcmplts =  { {0x10, 0x00, 0x02, 0x33/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evcmplts_ =  { {0x10, 0x00, 0x02, 0x33/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evcmpeq =  { {0x10, 0x00, 0x02, 0x34/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evcmpeq_ =  { {0x10, 0x00, 0x02, 0x34/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evsel =  { {0x10, 0x00, 0x02, 0x78/*RRRW*/}, 0x00000 };
  const PPC64Instr instr_evsel_ =  { {0x10, 0x00, 0x02, 0x78/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evfsadd =  { {0x10, 0x00, 0x02, 0x80/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evfssub =  { {0x10, 0x00, 0x02, 0x81/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evfsabs =  { {0x10, 0x00, 0x02, 0x84/*RR*/}, 0x00000 };
  const PPC64Instr instr_evfsnabs =  { {0x10, 0x00, 0x02, 0x85/*RR*/}, 0x00000 };
  const PPC64Instr instr_evfsneg =  { {0x10, 0x00, 0x02, 0x86/*RR*/}, 0x00000 };
  const PPC64Instr instr_evfsmul =  { {0x10, 0x00, 0x02, 0x88/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evfsdiv =  { {0x10, 0x00, 0x02, 0x89/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evfscmpgt =  { {0x10, 0x00, 0x02, 0x8C/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evfscmpgt_ =  { {0x10, 0x00, 0x02, 0x8C/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evfscmplt =  { {0x10, 0x00, 0x02, 0x8D/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evfscmplt_ =  { {0x10, 0x00, 0x02, 0x8D/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evfscmpeq =  { {0x10, 0x00, 0x02, 0x8E/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evfscmpeq_ =  { {0x10, 0x00, 0x02, 0x8E/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evfscfui =  { {0x10, 0x00, 0x02, 0x90/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfscfsi =  { {0x10, 0x00, 0x02, 0x91/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfscfuf =  { {0x10, 0x00, 0x02, 0x92/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfscfsf =  { {0x10, 0x00, 0x02, 0x93/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfsctui =  { {0x10, 0x00, 0x02, 0x94/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfsctsi =  { {0x10, 0x00, 0x02, 0x95/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfsctuf =  { {0x10, 0x00, 0x02, 0x96/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfsctsf =  { {0x10, 0x00, 0x02, 0x97/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfsctuiz =  { {0x10, 0x00, 0x02, 0x98/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfsctsiz =  { {0x10, 0x00, 0x02, 0x9A/*R-R*/}, 0x00000 };
  const PPC64Instr instr_evfststgt =  { {0x10, 0x00, 0x02, 0x9C/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evfststgt_ =  { {0x10, 0x00, 0x02, 0x9C/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evfststlt =  { {0x10, 0x00, 0x02, 0x9D/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evfststlt_ =  { {0x10, 0x00, 0x02, 0x9D/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evfststeq =  { {0x10, 0x00, 0x02, 0x9E/*XRR*/}, 0x00000 };
  const PPC64Instr instr_evfststeq_ =  { {0x10, 0x00, 0x02, 0x9E/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efsadd =  { {0x10, 0x00, 0x02, 0xC0/*RRR*/}, 0x00000 };
  const PPC64Instr instr_efssub =  { {0x10, 0x00, 0x02, 0xC1/*RRR*/}, 0x00000 };
  const PPC64Instr instr_efsabs =  { {0x10, 0x00, 0x02, 0xC4/*RR*/}, 0x00000 };
  const PPC64Instr instr_efsnabs =  { {0x10, 0x00, 0x02, 0xC5/*RR*/}, 0x00000 };
  const PPC64Instr instr_efsneg =  { {0x10, 0x00, 0x02, 0xC6/*RR*/}, 0x00000 };
  const PPC64Instr instr_efsmul =  { {0x10, 0x00, 0x02, 0xC8/*RRR*/}, 0x00000 };
  const PPC64Instr instr_efsdiv =  { {0x10, 0x00, 0x02, 0xC9/*RRR*/}, 0x00000 };
  const PPC64Instr instr_efscmpgt =  { {0x10, 0x00, 0x02, 0xCC/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efscmpgt_ =  { {0x10, 0x00, 0x02, 0xCC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efscmplt =  { {0x10, 0x00, 0x02, 0xCD/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efscmplt_ =  { {0x10, 0x00, 0x02, 0xCD/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efscmpeq =  { {0x10, 0x00, 0x02, 0xCE/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efscmpeq_ =  { {0x10, 0x00, 0x02, 0xCE/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efscfd =  { {0x10, 0x00, 0x02, 0xCF/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efscfui =  { {0x10, 0x00, 0x02, 0xD0/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efscfsi =  { {0x10, 0x00, 0x02, 0xD1/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efscfuf =  { {0x10, 0x00, 0x02, 0xD2/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efscfsf =  { {0x10, 0x00, 0x02, 0xD3/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efsctui =  { {0x10, 0x00, 0x02, 0xD4/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efsctsi =  { {0x10, 0x00, 0x02, 0xD5/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efsctuf =  { {0x10, 0x00, 0x02, 0xD6/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efsctsf =  { {0x10, 0x00, 0x02, 0xD7/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efsctuiz =  { {0x10, 0x00, 0x02, 0xD8/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efsctsiz =  { {0x10, 0x00, 0x02, 0xDA/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efststgt =  { {0x10, 0x00, 0x02, 0xDC/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efststgt_ =  { {0x10, 0x00, 0x02, 0xDC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efststlt =  { {0x10, 0x00, 0x02, 0xDD/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efststlt_ =  { {0x10, 0x00, 0x02, 0xDD/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efststeq =  { {0x10, 0x00, 0x02, 0xDE/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efststeq_ =  { {0x10, 0x00, 0x02, 0xDE/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efdadd =  { {0x10, 0x00, 0x02, 0xE0/*RRR*/}, 0x00000 };
  const PPC64Instr instr_efdsub =  { {0x10, 0x00, 0x02, 0xE1/*RRR*/}, 0x00000 };
  const PPC64Instr instr_efdcfuid =  { {0x10, 0x00, 0x02, 0xE2/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdcfsid =  { {0x10, 0x00, 0x02, 0xE3/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdabs =  { {0x10, 0x00, 0x02, 0xE4/*RR*/}, 0x00000 };
  const PPC64Instr instr_efdnabs =  { {0x10, 0x00, 0x02, 0xE5/*RR*/}, 0x00000 };
  const PPC64Instr instr_efdneg =  { {0x10, 0x00, 0x02, 0xE6/*RR*/}, 0x00000 };
  const PPC64Instr instr_efdmul =  { {0x10, 0x00, 0x02, 0xE8/*RRR*/}, 0x00000 };
  const PPC64Instr instr_efddiv =  { {0x10, 0x00, 0x02, 0xE9/*RRR*/}, 0x00000 };
  const PPC64Instr instr_efdctuidz =  { {0x10, 0x00, 0x02, 0xEA/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdctsidz =  { {0x10, 0x00, 0x02, 0xEB/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdcmpgt =  { {0x10, 0x00, 0x02, 0xEC/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efdcmpgt_ =  { {0x10, 0x00, 0x02, 0xEC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efdcmplt =  { {0x10, 0x00, 0x02, 0xED/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efdcmplt_ =  { {0x10, 0x00, 0x02, 0xED/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efdcmpeq =  { {0x10, 0x00, 0x02, 0xEE/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efdcmpeq_ =  { {0x10, 0x00, 0x02, 0xEE/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efdcfs =  { {0x10, 0x00, 0x02, 0xEF/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdcfui =  { {0x10, 0x00, 0x02, 0xF0/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdcfsi =  { {0x10, 0x00, 0x02, 0xF1/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdcfuf =  { {0x10, 0x00, 0x02, 0xF2/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdcfsf =  { {0x10, 0x00, 0x02, 0xF3/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdctui =  { {0x10, 0x00, 0x02, 0xF4/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdctsi =  { {0x10, 0x00, 0x02, 0xF5/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdctuf =  { {0x10, 0x00, 0x02, 0xF6/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdctsf =  { {0x10, 0x00, 0x02, 0xF7/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdctuiz =  { {0x10, 0x00, 0x02, 0xF8/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdctsiz_ =  { {0x10, 0x00, 0x02, 0xFA/*R-R*/}, 0x00000 };
  const PPC64Instr instr_efdtstgt =  { {0x10, 0x00, 0x02, 0xFC/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efdtstgt_ =  { {0x10, 0x00, 0x02, 0xFC/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efdtstlt =  { {0x10, 0x00, 0x02, 0xFD/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efdtstlt_ =  { {0x10, 0x00, 0x02, 0xFD/*-RR*/}, 0x00000 };
  const PPC64Instr instr_efdtsteq =  { {0x10, 0x00, 0x02, 0xFE/*XRR*/}, 0x00000 };
  const PPC64Instr instr_efdtsteq_ =  { {0x10, 0x00, 0x02, 0xFE/*-RR*/}, 0x00000 };
  const PPC64Instr instr_evlddx =  { {0x10, 0x00, 0x03, 0x00/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evldd =  { {0x10, 0x00, 0x03, 0x01/*R8*/}, 0x00000 };
  const PPC64Instr instr_evldwx =  { {0x10, 0x00, 0x03, 0x02/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evldw =  { {0x10, 0x00, 0x03, 0x03/*R8*/}, 0x00000 };
  const PPC64Instr instr_evldhx =  { {0x10, 0x00, 0x03, 0x04/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evldh =  { {0x10, 0x00, 0x03, 0x05/*R8*/}, 0x00000 };
  const PPC64Instr instr_evlwhex =  { {0x10, 0x00, 0x03, 0x10/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evlwhe =  { {0x10, 0x00, 0x03, 0x11/*R4*/}, 0x00000 };
  const PPC64Instr instr_evlwhoux =  { {0x10, 0x00, 0x03, 0x14/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evlwhou =  { {0x10, 0x00, 0x03, 0x15/*R4*/}, 0x00000 };
  const PPC64Instr instr_evlwhosx =  { {0x10, 0x00, 0x03, 0x16/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evlwhos =  { {0x10, 0x00, 0x03, 0x17/*R4*/}, 0x00000 };
  const PPC64Instr instr_evstddx =  { {0x10, 0x00, 0x03, 0x20/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evstdd =  { {0x10, 0x00, 0x03, 0x21/*R8*/}, 0x00000 };
  const PPC64Instr instr_evstdwx =  { {0x10, 0x00, 0x03, 0x22/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evstdw =  { {0x10, 0x00, 0x03, 0x23/*R8*/}, 0x00000 };
  const PPC64Instr instr_evstdhx =  { {0x10, 0x00, 0x03, 0x24/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evstdh =  { {0x10, 0x00, 0x03, 0x25/*R8*/}, 0x00000 };
  const PPC64Instr instr_evstwhex =  { {0x10, 0x00, 0x03, 0x30/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evstwhe =  { {0x10, 0x00, 0x03, 0x31/*R4*/}, 0x00000 };
  const PPC64Instr instr_evstwhox =  { {0x10, 0x00, 0x03, 0x34/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evstwho =  { {0x10, 0x00, 0x03, 0x35/*R4*/}, 0x00000 };
  const PPC64Instr instr_evstwwex =  { {0x10, 0x00, 0x03, 0x38/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evstwwe =  { {0x10, 0x00, 0x03, 0x39/*R4*/}, 0x00000 };
  const PPC64Instr instr_evstwwox =  { {0x10, 0x00, 0x03, 0x3C/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evstwwo =  { {0x10, 0x00, 0x03, 0x3D/*R4*/}, 0x00000 };
  const PPC64Instr instr_evmhessf =  { {0x10, 0x00, 0x04, 0x03/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhossf =  { {0x10, 0x00, 0x04, 0x07/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmheumi =  { {0x10, 0x00, 0x04, 0x08/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhesmi =  { {0x10, 0x00, 0x04, 0x09/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhesmf =  { {0x10, 0x00, 0x04, 0x0B/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhoumi =  { {0x10, 0x00, 0x04, 0x0C/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhosmi =  { {0x10, 0x00, 0x04, 0x0D/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhosmf =  { {0x10, 0x00, 0x04, 0x0F/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhessfa =  { {0x10, 0x00, 0x04, 0x23/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhossfa =  { {0x10, 0x00, 0x04, 0x27/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmheumia =  { {0x10, 0x00, 0x04, 0x28/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhesmia =  { {0x10, 0x00, 0x04, 0x29/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhesmfa =  { {0x10, 0x00, 0x04, 0x2B/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhoumia =  { {0x10, 0x00, 0x04, 0x2C/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhosmia =  { {0x10, 0x00, 0x04, 0x2D/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhosmfa =  { {0x10, 0x00, 0x04, 0x2F/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwhssf =  { {0x10, 0x00, 0x04, 0x47/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlumi =  { {0x10, 0x00, 0x04, 0x48/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwhumi =  { {0x10, 0x00, 0x04, 0x4C/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwhsmi =  { {0x10, 0x00, 0x04, 0x4D/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwhsmf =  { {0x10, 0x00, 0x04, 0x4F/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwssf =  { {0x10, 0x00, 0x04, 0x53/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwumi =  { {0x10, 0x00, 0x04, 0x58/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwsmi =  { {0x10, 0x00, 0x04, 0x59/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwsmf =  { {0x10, 0x00, 0x04, 0x5B/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwhssfa =  { {0x10, 0x00, 0x04, 0x67/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlumia =  { {0x10, 0x00, 0x04, 0x68/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwhumia =  { {0x10, 0x00, 0x04, 0x6C/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwhsmia =  { {0x10, 0x00, 0x04, 0x6D/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwhsmfa =  { {0x10, 0x00, 0x04, 0x6F/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwssfa =  { {0x10, 0x00, 0x04, 0x73/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwumia =  { {0x10, 0x00, 0x04, 0x78/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwsmia =  { {0x10, 0x00, 0x04, 0x79/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwsmfa =  { {0x10, 0x00, 0x04, 0x7B/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmra =  { {0x10, 0x00, 0x04, 0xC4/*RR*/}, 0x00000 };
  const PPC64Instr instr_evdivws =  { {0x10, 0x00, 0x04, 0xC6/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evdivwu =  { {0x10, 0x00, 0x04, 0xC7/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwssfaa =  { {0x10, 0x00, 0x05, 0x53/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwumiaa =  { {0x10, 0x00, 0x05, 0x58/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwsmiaa =  { {0x10, 0x00, 0x05, 0x59/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwsmfaa =  { {0x10, 0x00, 0x05, 0x5B/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwssfan =  { {0x10, 0x00, 0x05, 0xD3/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwumian =  { {0x10, 0x00, 0x05, 0xD8/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwsmian =  { {0x10, 0x00, 0x05, 0xD9/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwsmfan =  { {0x10, 0x00, 0x05, 0xDB/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmergehilo =  { {0x10, 0x00, 0x02, 0x2E/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmergelohi =  { {0x10, 0x00, 0x02, 0x2F/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evlhhesplatx =  { {0x10, 0x00, 0x03, 0x08/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evlhhesplat =  { {0x10, 0x00, 0x03, 0x09/*R2*/}, 0x00000 };
  const PPC64Instr instr_evlhhousplatx =  { {0x10, 0x00, 0x03, 0x0C/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evlhhousplat =  { {0x10, 0x00, 0x03, 0x0D/*R2*/}, 0x00000 };
  const PPC64Instr instr_evlhhossplatx =  { {0x10, 0x00, 0x03, 0x0E/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evlhhossplat =  { {0x10, 0x00, 0x03, 0x0F/*R2*/}, 0x00000 };
  const PPC64Instr instr_evlwwsplatx =  { {0x10, 0x00, 0x03, 0x18/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evlwwsplat =  { {0x10, 0x00, 0x03, 0x19/*R4*/}, 0x00000 };
  const PPC64Instr instr_evlwhsplatx =  { {0x10, 0x00, 0x03, 0x1C/*RR0R*/}, 0x00000 };
  const PPC64Instr instr_evlwhsplat =  { {0x10, 0x00, 0x03, 0x1D/*R4*/}, 0x00000 };
  const PPC64Instr instr_evaddusiaaw =  { {0x10, 0x00, 0x04, 0xC0/*RR*/}, 0x00000 };
  const PPC64Instr instr_evaddssiaaw =  { {0x10, 0x00, 0x04, 0xC1/*RR*/}, 0x00000 };
  const PPC64Instr instr_evsubfusiaaw =  { {0x10, 0x00, 0x04, 0xC2/*RR*/}, 0x00000 };
  const PPC64Instr instr_evsubfssiaaw =  { {0x10, 0x00, 0x04, 0xC3/*RR*/}, 0x00000 };
  const PPC64Instr instr_evaddumiaaw =  { {0x10, 0x00, 0x04, 0xC8/*RR*/}, 0x00000 };
  const PPC64Instr instr_evaddsmiaaw =  { {0x10, 0x00, 0x04, 0xC9/*RR*/}, 0x00000 };
  const PPC64Instr instr_evsubfumiaaw =  { {0x10, 0x00, 0x04, 0xCA/*RR*/}, 0x00000 };
  const PPC64Instr instr_evsubfsmiaaw =  { {0x10, 0x00, 0x04, 0xCB/*RR*/}, 0x00000 };
  const PPC64Instr instr_evmheusiaaw =  { {0x10, 0x00, 0x05, 0x00/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhessiaaw =  { {0x10, 0x00, 0x05, 0x01/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhessfaaw =  { {0x10, 0x00, 0x05, 0x03/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhousiaaw =  { {0x10, 0x00, 0x05, 0x04/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhossiaaw =  { {0x10, 0x00, 0x05, 0x05/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhossfaaw =  { {0x10, 0x00, 0x05, 0x07/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmheumiaaw =  { {0x10, 0x00, 0x05, 0x08/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhesmiaaw =  { {0x10, 0x00, 0x05, 0x09/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhesmfaaw =  { {0x10, 0x00, 0x05, 0x0B/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhoumiaaw =  { {0x10, 0x00, 0x05, 0x0C/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhosmiaaw =  { {0x10, 0x00, 0x05, 0x0D/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhosmfaaw =  { {0x10, 0x00, 0x05, 0x0F/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhegumiaa =  { {0x10, 0x00, 0x05, 0x28/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhegsmiaa =  { {0x10, 0x00, 0x05, 0x29/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhegsmfaa =  { {0x10, 0x00, 0x05, 0x2B/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhogumiaa =  { {0x10, 0x00, 0x05, 0x2C/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhogsmiaa =  { {0x10, 0x00, 0x05, 0x2D/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhogsmfaa =  { {0x10, 0x00, 0x05, 0x2F/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlusiaaw =  { {0x10, 0x00, 0x05, 0x40/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlssiaaw =  { {0x10, 0x00, 0x05, 0x41/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlumiaaw =  { {0x10, 0x00, 0x05, 0x48/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlsmiaaw =  { {0x10, 0x00, 0x05, 0x49/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmheusianw =  { {0x10, 0x00, 0x05, 0x80/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhessianw =  { {0x10, 0x00, 0x05, 0x81/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhessfanw =  { {0x10, 0x00, 0x05, 0x83/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhousianw =  { {0x10, 0x00, 0x05, 0x84/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhossianw =  { {0x10, 0x00, 0x05, 0x85/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhossfanw =  { {0x10, 0x00, 0x05, 0x87/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmheumianw =  { {0x10, 0x00, 0x05, 0x88/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhesmianw =  { {0x10, 0x00, 0x05, 0x89/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhesmfanw =  { {0x10, 0x00, 0x05, 0x8B/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhoumianw =  { {0x10, 0x00, 0x05, 0x8C/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhosmianw =  { {0x10, 0x00, 0x05, 0x8D/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhosmfanw =  { {0x10, 0x00, 0x05, 0x8F/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhegumian =  { {0x10, 0x00, 0x05, 0xA8/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhegsmian =  { {0x10, 0x00, 0x05, 0xA9/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhegsmfan =  { {0x10, 0x00, 0x05, 0xAB/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhogumian =  { {0x10, 0x00, 0x05, 0xAC/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhogsmian =  { {0x10, 0x00, 0x05, 0xAD/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmhogsmfan =  { {0x10, 0x00, 0x05, 0xAF/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlusianw =  { {0x10, 0x00, 0x05, 0xC0/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlssianw =  { {0x10, 0x00, 0x05, 0xC1/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlumianw =  { {0x10, 0x00, 0x05, 0xC8/*RRR*/}, 0x00000 };
  const PPC64Instr instr_evmwlsmianw =  { {0x10, 0x00, 0x05, 0xC9/*RRR*/}, 0x00000 };


  RegNumber rn(Reg8 r)   { return RegNumber(int(r)); }
  RegNumber rn(Reg16 r)  { return RegNumber(int(r)); }
  RegNumber rn(Reg32 r)  { return RegNumber(int(r)); }
  RegNumber rn(Reg64 r)  { return RegNumber(int(r)); }
  RegNumber rn(RegXMM r) { return RegNumber(int(r)); }

public:
  //const PPC64Instr instr_or =  { {0x7C, 0x00, 0x03, 0x78/*RR~R.*/}, 0x00000 };
  ALWAYS_INLINE
  void emitRRR(PPC64Instr op, RegNumber nrt, RegNumber nra, RegNumber nrb) {
	int rt = int(nrt);
	int ra = int(nra);
	int rb = int(nrb);
	dword (op.table[0] << 27 | rt << 22 | ra << 17 | rb << 12 );
//	byte(0x40 | rex);
//	byte((x << 6) | ((y & 7) << 3) | (z & 7));
  }


private:

  bool byteRegNeedsRex(int rn) const {
    // Without a rex, 4 through 7 mean the high 8-bit byte registers.
    return rn >= 4 && rn <= 7;
  }

  int byteRegEncodeNumber(int rn, bool& seenHigh) const {
    // We flag a bit in ah, ch, dh, bh so byteRegNeedsRex doesn't
    // trigger.
    if (rn & 0x80) seenHigh = true;
    return rn & ~0x80;
  }

  // In 64-bit mode, you can't mix accesses to high byte registers
  // with low byte registers other than al,cl,bl,dl.  We assert this.
  void byteRegMisuse() const {
    assert(!"High byte registers can't be used with new x64 registers, or"
            " anything requiring a REX prefix");
  }

  void emitModrm(int x, int y, int z) {
    byte((x << 6) | ((y & 7) << 3) | (z & 7));
  }

  int computeImmediateSize(X64Instr op,
                           ssize_t imm,
                           int opsize = sz::dword) {
    // Most instructions take a 32-bit or 16-bit immediate,
    // depending on the presence of the opsize prefix (0x66).
    int immSize = opsize == sz::word ? sz::word : sz::dword;
    // ret always takes a 16-bit immediate.
    if (op.flags & IF_RET) {
      immSize = sz::word;
    }
    // Use an 8-bit immediate if the instruction supports it and if
    // the immediate value fits in a byte
    if (deltaFits(imm, sz::byte) && (op.flags & IF_HAS_IMM8) != 0) {
      immSize = sz::byte;
    }
    return immSize;
  }

  /*
   * The mov instruction supports an 8 byte immediate for the RI
   * address mode when opSz is qword.  It also supports a 4-byte
   * immediate with opSz qword (the immediate is sign-extended).
   *
   * On the other hand, if it fits in 32-bits as an unsigned, we can
   * change opSz to dword, which will zero the top 4 bytes instead of
   * sign-extending.
   */
  int computeImmediateSizeForMovRI64(X64Instr op, ssize_t imm, int& opSz) {
    assert(opSz == sz::qword);
    if (deltaFits(imm, sz::dword)) {
      return computeImmediateSize(op, imm);
    }
    if (magFits(imm, sz::dword)) {
      opSz = sz::dword;
      return sz::dword;
    }
    return sz::qword;
  }

  void emitImmediate(X64Instr op, ssize_t imm, int immSize) {
    if (immSize == sz::nosize) {
      return;
    }
    if ((op.flags & (IF_SHIFT | IF_SHIFTD)) == 0) {
      if (immSize == sz::dword) {
        dword(imm);
      } else if (immSize == sz::byte) {
        byte(imm);
      } else if (immSize == sz::word) {
        word(imm);
      } else {
        qword(imm);
      }
    } else {
      // we always use a byte-sized immediate for shift instructions
      byte(imm);
    }
  }

  void prefixBytes(unsigned long flags, int opSz) {
    if (opSz == sz::word && !(flags & IF_RET)) byte(kOpsizePrefix);
    if (flags & IF_66PREFIXED) byte(0x66);
    if (flags & IF_F2PREFIXED) byte(0xF2);
    if (flags & IF_F3PREFIXED) byte(0xF3);
  }

/*  RegNumber rn(Reg8 r)   { return RegNumber(int(r)); }
  RegNumber rn(Reg16 r)  { return RegNumber(int(r)); }
  RegNumber rn(Reg32 r)  { return RegNumber(int(r)); }
  RegNumber rn(Reg64 r)  { return RegNumber(int(r)); }
  RegNumber rn(RegXMM r) { return RegNumber(int(r)); }*/

  // Wraps a bunch of the emit* functions to make using them with the
  // typed wrappers more terse. We should have these replace
  // the emit functions eventually.

  CodeBlock& codeBlock;
};

//////////////////////////////////////////////////////////////////////

/*struct Label : private boost::noncopyable {
  explicit Label()
    : m_a(nullptr)
    , m_address(nullptr)
  {}

  ~Label() {
    if (!m_toPatch.empty()) {
      assert(m_a && m_address && "Label had jumps but was never set");
    }
    for (auto& ji : m_toPatch) {
      switch (ji.type) {
      case Branch::Jmp:   ji.a->patchJmp(ji.addr, m_address);  break;
      case Branch::Jmp8:  ji.a->patchJmp8(ji.addr, m_address); break;
      case Branch::Jcc:   ji.a->patchJcc(ji.addr, m_address);  break;
      case Branch::Jcc8:  ji.a->patchJcc8(ji.addr, m_address); break;
      case Branch::Call:  ji.a->patchCall(ji.addr, m_address); break;
      }
    }
  }

private:
  enum class Branch {
    Jcc,
    Jcc8,
    Jmp,
    Jmp8,
    Call
  };

  struct JumpInfo {
    Branch type;
    X64Assembler* a;
    CodeAddress addr;
  };

private:
  void addJump(X64Assembler* a, Branch type) {
    if (m_address) return;
    JumpInfo info;
    info.type = type;
    info.a = a;
    info.addr = a->codeBlock.frontier();
    m_toPatch.push_back(info);
  }

private:
  X64Assembler* m_a;
  CodeAddress m_address;
  std::vector<JumpInfo> m_toPatch;
};*/


//////////////////////////////////////////////////////////////////////

/*
 * Select the assembler which contains a given address.
 *
 * E.g.:
 *
 *   Asm& a = codeBlockChoose(toPatch, a, acold);
 *   a.patchJmp(...);
 */
inline CodeBlock& codeBlockChoose(CodeAddress addr) {
  always_assert_flog(false,
                     "address {} was not part of any known code block", addr);
}
template<class... Blocks>
CodeBlock& codeBlockChoose(CodeAddress addr, CodeBlock& a, Blocks&... as) {
  if (a.contains(addr)) return a;
  return codeBlockChoose(addr, as...);
}

//////////////////////////////////////////////////////////////////////

/*struct DecodedInstruction {
  explicit DecodedInstruction(uint8_t* ip) { decode(ip); }
  std::string toString();
  size_t size() { return m_size; }

  bool hasPicOffset() const { return m_flags.picOff; }
  uint8_t* picAddress() const;
  bool setPicAddress(uint8_t* target);

  bool hasOffset() const { return m_offSz != 0; }
  int32_t offset() const;

  bool hasImmediate() const { return m_immSz; }
  int64_t immediate() const;
  bool setImmediate(int64_t value);
  bool isNop() const;
  bool isBranch(bool allowCond = true) const;
  bool isCall() const;
  bool isJmp() const;
  bool isLea() const;
  ConditionCode jccCondCode() const;
  bool shrinkBranch();
  void widenBranch();
  uint8_t getModRm() const;
private:
  void decode(uint8_t* ip);
  bool decodePrefix(uint8_t* ip);
  int decodeRexVexXop(uint8_t* ip);
  int decodeOpcode(uint8_t* ip);
  void determineOperandsMap0(uint8_t* ip);
  void determineOperandsMap1(uint8_t* ip);
  void determineOperandsMap2(uint8_t* ip);
  void determineOperandsMap3(uint8_t* ip);
  int decodeModRm(uint8_t* ip);
  int decodeImm(uint8_t* ip);

  uint8_t*   m_ip;
  uint32_t   m_size;

  union {
    uint32_t m_flagsVal;
    struct {
      uint32_t lock      : 1;
      uint32_t repNE     : 1;
      uint32_t rep       : 1;

      uint32_t cs        : 1;
      uint32_t ss        : 1;
      uint32_t ds        : 1;
      uint32_t es        : 1;
      uint32_t fs        : 1;
      uint32_t gs        : 1;
      uint32_t bTaken    : 1;
      uint32_t bNotTaken : 1;

      uint32_t opndSzOvr : 1;
      uint32_t addrSzOvr : 1;

      uint32_t rex       : 1;
      uint32_t vex       : 1;
      uint32_t xop       : 1;

      uint32_t w         : 1;
      uint32_t r         : 1;
      uint32_t x         : 1;
      uint32_t b         : 1;
      uint32_t l         : 1;

      uint32_t def64     : 1;
      uint32_t immIsAddr : 1;
      uint32_t picOff    : 1;
      uint32_t hasModRm  : 1;
      uint32_t hasSib    : 1;
    } m_flags;
  };

  uint8_t       m_map_select;
  uint8_t       m_xtra_op;
  uint8_t       m_opcode;
  uint8_t       m_immSz;
  uint8_t       m_offSz;
};*/

#undef TRACEMOD
#undef logical_const
#undef CCS

}}}

#endif
