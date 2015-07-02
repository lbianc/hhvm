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
#ifndef incl_HPHP_UTIL_ASM_X64_H_
#define incl_HPHP_UTIL_ASM_X64_H_

#include <boost/noncopyable.hpp>
#include <type_traits>

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

namespace HPHP { namespace jit {

#define TRACEMOD ::HPHP::Trace::asmx64

//////////////////////////////////////////////////////////////////////

struct MemoryRef;
struct RIPRelativeRef;
struct ScaledIndex;
struct ScaledIndexDisp;
struct DispReg;

const uint8_t kOpsizePrefix = 0x66;

struct Reg64 {
  explicit constexpr Reg64(int rn) : rn(rn) {}

  // Integer conversion is allowed but only explicitly.  (It's not
  // unusual to want to printf registers, etc.  Just cast it first.)
  explicit constexpr operator int() const { return rn; }

  MemoryRef operator[](intptr_t disp) const;
  MemoryRef operator[](Reg64) const;
  MemoryRef operator[](ScaledIndex) const;
  MemoryRef operator[](ScaledIndexDisp) const;
  MemoryRef operator[](DispReg) const;

  constexpr bool operator==(Reg64 o) const { return rn == o.rn; }
  constexpr bool operator!=(Reg64 o) const { return rn != o.rn; }

private:
  int rn;
};

#define SIMPLE_REGTYPE(What)                                        \
  struct What {                                                     \
    explicit constexpr What(int rn) : rn(rn) {}                     \
    explicit constexpr operator int() const { return rn; }          \
    constexpr bool operator==(What o) const { return rn == o.rn; }  \
    constexpr bool operator!=(What o) const { return rn != o.rn; }  \
  private:                                                          \
    int rn;                                                         \
  }

SIMPLE_REGTYPE(Reg32);
SIMPLE_REGTYPE(Reg16);
SIMPLE_REGTYPE(Reg8);
SIMPLE_REGTYPE(RegXMM);
SIMPLE_REGTYPE(RegSF);

#undef SIMPLE_REGTYPE

struct RegRIP {
  RIPRelativeRef operator[](intptr_t disp) const;
};

// Convert between physical registers of different sizes
inline Reg8 rbyte(Reg32 r)     { return Reg8(int(r)); }
inline Reg16 r16(Reg8 r)       { return Reg16(int(r)); }
inline Reg32 r32(Reg8 r)       { return Reg32(int(r)); }
inline Reg32 r32(Reg16 r)      { return Reg32(int(r)); }
inline Reg32 r32(Reg64 r)      { return Reg32(int(r)); }
inline Reg32 r32(Reg32 r)      { return r; }

//////////////////////////////////////////////////////////////////////

/*
 * The following structures define intermediate types for various
 * addressing modes.  They overload some operators to allow using
 * registers to look somewhat like pointers.
 *
 * E.g. rax[rbx*2 + 3] or *(rax + rbx*2 + 3).
 *
 * These operators are not defined commutatively; the thought is it
 * mandates the order you normally write them in a .S, but it could be
 * changed if this proves undesirable.
 */

// reg*x
struct ScaledIndex {
  explicit ScaledIndex(Reg64 index, intptr_t scale)
    : index(index)
    , scale(scale)
  {
    assert((scale == 0x1 || scale == 0x2 || scale == 0x4 || scale == 0x8) &&
           "Invalid index register scaling (must be 1,2,4 or 8).");
    assert(int(index) != -1 && "invalid register");
  }

  Reg64 index;
  intptr_t scale;
};

// reg*x + disp
struct ScaledIndexDisp {
  explicit ScaledIndexDisp(ScaledIndex si, intptr_t disp)
    : si(si)
    , disp(disp)
  {}

  ScaledIndexDisp operator+(intptr_t x) const {
    return ScaledIndexDisp(si, disp + x);
  }

  ScaledIndexDisp operator-(intptr_t x) const {
    return ScaledIndexDisp(si, disp - x);
  }

  ScaledIndex si;
  intptr_t disp;
};

// reg+x
struct DispReg {
  explicit DispReg(Reg64 base, intptr_t disp = 0)
    : base(base)
    , disp(disp)
  {
    assert(int(base) != -1 && "invalid register");
  }

  // Constructor for baseless().
  explicit DispReg(intptr_t disp)
    : base(-1)
    , disp(disp)
  {}

  MemoryRef operator*() const;
  MemoryRef operator[](intptr_t) const;

  DispReg operator+(intptr_t x) const {
    return DispReg(base, disp + x);
  }

  DispReg operator-(intptr_t x) const {
    return DispReg(base, disp - x);
  }

  Reg64 base;
  intptr_t disp;
};

// reg + reg*x + y
struct IndexedDispReg {
  explicit IndexedDispReg(Reg64 base, ScaledIndex sr)
    : base(base)
    , index(sr.index)
    , scale(sr.scale)
    , disp(0)
  {}

  explicit IndexedDispReg(DispReg r)
    : base(r.base)
    , index(-1)
    , scale(1)
    , disp(r.disp)
  {}

  // Constructor for baseless()
  explicit IndexedDispReg(ScaledIndexDisp sid)
    : base(-1)
    , index(sid.si.index)
    , scale(sid.si.scale)
    , disp(sid.disp)
  {}

  MemoryRef operator*() const;
  MemoryRef operator[](intptr_t disp) const;

  IndexedDispReg operator+(intptr_t disp) const {
    auto ret = *this;
    ret.disp += disp;
    return ret;
  }

  IndexedDispReg operator-(intptr_t disp) const {
    auto ret = *this;
    ret.disp -= disp;
    return ret;
  }

  Reg64 base;
  Reg64 index;
  int scale;
  intptr_t disp; // TODO #4613274: should be int32_t
};

// rip+x
struct DispRIP {
  explicit DispRIP(intptr_t disp) : disp(disp) {}

  RIPRelativeRef operator*() const;
  RIPRelativeRef operator[](intptr_t x) const;

  DispRIP operator+(intptr_t x) const {
    return DispRIP(disp + x);
  }

  DispRIP operator-(intptr_t x) const {
    return DispRIP(disp - x);
  }

  intptr_t disp; // TODO #4613274: should be int32_t
};

// *(reg + x)
struct MemoryRef {
  explicit MemoryRef(DispReg dr) : r(dr) {}
  explicit MemoryRef(IndexedDispReg idr) : r(idr) {}
  IndexedDispReg r;
};

// *(rip + x)
struct RIPRelativeRef {
  explicit RIPRelativeRef(DispRIP r) : r(r) {}
  DispRIP r;
};

inline MemoryRef IndexedDispReg::operator*() const {
  return MemoryRef(*this);
}

inline MemoryRef IndexedDispReg::operator[](intptr_t x) const {
  return *(*this + x);
}

inline MemoryRef DispReg::operator*() const {
  return MemoryRef(*this);
}

inline MemoryRef DispReg::operator[](intptr_t x) const {
  return *(*this + x);
}

inline RIPRelativeRef DispRIP::operator*() const {
  return RIPRelativeRef(*this);
}

inline RIPRelativeRef DispRIP::operator[](intptr_t x) const {
  return *(*this + x);
}

inline DispReg operator+(Reg64 r, intptr_t d) { return DispReg(r, d); }
inline DispReg operator-(Reg64 r, intptr_t d) { return DispReg(r, -d); }
inline DispRIP operator+(RegRIP r, intptr_t d) { return DispRIP(d); }
inline DispRIP operator-(RegRIP r, intptr_t d) { return DispRIP(d); }

inline ScaledIndex operator*(Reg64 r, int scale) {
  return ScaledIndex(r, scale);
}
inline IndexedDispReg operator+(Reg64 base, ScaledIndex sr) {
  return IndexedDispReg(base, sr);
}
inline ScaledIndexDisp operator+(ScaledIndex si, intptr_t disp) {
  return ScaledIndexDisp(si, disp);
}
inline IndexedDispReg operator+(Reg64 b, Reg64 i) {
  return b + ScaledIndex(i, 0x1);
}

inline MemoryRef operator*(Reg64 r)  { return MemoryRef(DispReg(r)); }
inline DispRIP   operator*(RegRIP r) { return DispRIP(0); }

inline MemoryRef Reg64::operator[](intptr_t disp) const {
  return *(*this + disp);
}

inline MemoryRef Reg64::operator[](Reg64 idx) const {
  return *(*this + idx * 1);
}

inline MemoryRef Reg64::operator[](ScaledIndex si) const {
  return *(*this + si);
}

inline MemoryRef Reg64::operator[](DispReg dr) const {
  return *(*this + ScaledIndex(dr.base, 0x1) + dr.disp);
}

inline MemoryRef Reg64::operator[](ScaledIndexDisp sid) const {
  return *(*this + sid.si + sid.disp);
}

inline RIPRelativeRef RegRIP::operator[](intptr_t disp) const {
  return *(*this + disp);
}

/*
 * Used for the x64 addressing mode where there is a displacement,
 * possibly with a scaled index, but no base register.
 */
inline MemoryRef baseless(intptr_t disp) { return *(DispReg { disp }); }
inline MemoryRef baseless(ScaledIndexDisp sid) {
  return *(IndexedDispReg { sid });
}

//////////////////////////////////////////////////////////////////////

namespace reg {
  constexpr Reg64 rax(0);
  constexpr Reg64 rcx(1);
  constexpr Reg64 rdx(2);
  constexpr Reg64 rbx(3);
  constexpr Reg64 rsp(4);
  constexpr Reg64 rbp(5);
  constexpr Reg64 rsi(6);
  constexpr Reg64 rdi(7);

  constexpr Reg64 r8 (8);
  constexpr Reg64 r9 (9);
  constexpr Reg64 r10(10);
  constexpr Reg64 r11(11);
  constexpr Reg64 r12(12);
  constexpr Reg64 r13(13);
  constexpr Reg64 r14(14);
  constexpr Reg64 r15(15);

  constexpr RegRIP rip = RegRIP();

  constexpr Reg32 eax (0);
  constexpr Reg32 ecx (1);
  constexpr Reg32 edx (2);
  constexpr Reg32 ebx (3);
  constexpr Reg32 esp (4);
  constexpr Reg32 ebp (5);
  constexpr Reg32 esi (6);
  constexpr Reg32 edi (7);
  constexpr Reg32 r8d (8);
  constexpr Reg32 r9d (9);
  constexpr Reg32 r10d(10);
  constexpr Reg32 r11d(11);
  constexpr Reg32 r12d(12);
  constexpr Reg32 r13d(13);
  constexpr Reg32 r14d(14);
  constexpr Reg32 r15d(15);

  constexpr Reg16 ax  (0);
  constexpr Reg16 cx  (1);
  constexpr Reg16 dx  (2);
  constexpr Reg16 bx  (3);
  constexpr Reg16 sp  (4);
  constexpr Reg16 bp  (5);
  constexpr Reg16 si  (6);
  constexpr Reg16 di  (7);
  constexpr Reg16 r8w (8);
  constexpr Reg16 r9w (9);
  constexpr Reg16 r10w(10);
  constexpr Reg16 r11w(11);
  constexpr Reg16 r12w(12);
  constexpr Reg16 r13w(13);
  constexpr Reg16 r14w(14);
  constexpr Reg16 r15w(15);

  constexpr Reg8 al  (0);
  constexpr Reg8 cl  (1);
  constexpr Reg8 dl  (2);
  constexpr Reg8 bl  (3);
  constexpr Reg8 spl (4);
  constexpr Reg8 bpl (5);
  constexpr Reg8 sil (6);
  constexpr Reg8 dil (7);
  constexpr Reg8 r8b (8);
  constexpr Reg8 r9b (9);
  constexpr Reg8 r10b(10);
  constexpr Reg8 r11b(11);
  constexpr Reg8 r12b(12);
  constexpr Reg8 r13b(13);
  constexpr Reg8 r14b(14);
  constexpr Reg8 r15b(15);

  // Reminder: these registers may not be mixed in any instruction
  // using a REX prefix (i.e. anything using r8-r15, spl, bpl, sil,
  // dil, etc).
  constexpr Reg8 ah(0x80 | 4);
  constexpr Reg8 ch(0x80 | 5);
  constexpr Reg8 dh(0x80 | 6);
  constexpr Reg8 bh(0x80 | 7);

  constexpr RegXMM xmm0(0);
  constexpr RegXMM xmm1(1);
  constexpr RegXMM xmm2(2);
  constexpr RegXMM xmm3(3);
  constexpr RegXMM xmm4(4);
  constexpr RegXMM xmm5(5);
  constexpr RegXMM xmm6(6);
  constexpr RegXMM xmm7(7);
  constexpr RegXMM xmm8(8);
  constexpr RegXMM xmm9(9);
  constexpr RegXMM xmm10(10);
  constexpr RegXMM xmm11(11);
  constexpr RegXMM xmm12(12);
  constexpr RegXMM xmm13(13);
  constexpr RegXMM xmm14(14);
  constexpr RegXMM xmm15(15);

#define X(x) if (r == x) return "%"#x
  inline const char* regname(Reg64 r) {
    X(rax); X(rbx); X(rcx); X(rdx); X(rsp); X(rbp); X(rsi); X(rdi);
    X(r8); X(r9); X(r10); X(r11); X(r12); X(r13); X(r14); X(r15);
    return nullptr;
  }
  inline const char* regname(Reg32 r) {
    X(eax); X(ecx); X(edx); X(ebx); X(esp); X(ebp); X(esi); X(edi);
    X(r8d); X(r9d); X(r10d); X(r11d); X(r12d); X(r13d); X(r14d); X(r15d);
    return nullptr;
  }
  inline const char* regname(Reg16 r) {
    X(ax); X(cx); X(dx); X(bx); X(sp); X(bp); X(si); X(di);
    X(r8w); X(r9w); X(r10w); X(r11w); X(r12w); X(r13w); X(r14w); X(r15w);
    return nullptr;
  }
  inline const char* regname(Reg8 r) {
    X(al); X(cl); X(dl); X(bl); X(spl); X(bpl); X(sil); X(dil);
    X(r8b); X(r9b); X(r10b); X(r11b); X(r12b); X(r13b); X(r14b); X(r15b);
    X(ah); X(ch); X(dh); X(bh);
    return nullptr;
  }
  inline const char* regname(RegXMM r) {
    X(xmm0); X(xmm1); X(xmm2); X(xmm3); X(xmm4); X(xmm5); X(xmm6);
    X(xmm7); X(xmm8); X(xmm9); X(xmm10); X(xmm11); X(xmm12); X(xmm13);
    X(xmm14); X(xmm15);
    return nullptr;
  }
  inline const char* regname(RegSF r) {
    return "%flags";
  }
#undef X

}

//////////////////////////////////////////////////////////////////////

enum X64InstrFlags {
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

//                                    0    1    2    3    4    5     flags
const X64Instr instr_divsd =   { { 0x5E,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x10102 };
const X64Instr instr_movups =  { { 0x10,0x11,0xF1,0x00,0xF1,0xF1 }, 0x0103  };
const X64Instr instr_movdqa =  { { 0x6F,0x7F,0xF1,0x00,0xF1,0xF1 }, 0x4103  };
const X64Instr instr_movdqu =  { { 0x6F,0x7F,0xF1,0x00,0xF1,0xF1 }, 0x8103  };
const X64Instr instr_movsd =   { { 0x11,0x10,0xF1,0x00,0xF1,0xF1 }, 0x10102 };
const X64Instr instr_gpr2xmm = { { 0x6e,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x4002  };
const X64Instr instr_xmm2gpr = { { 0x7e,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x4002  };
const X64Instr instr_xmmsub =  { { 0x5c,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x10102 };
const X64Instr instr_xmmadd =  { { 0x58,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x10102 };
const X64Instr instr_xmmmul =  { { 0x59,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x10102 };
const X64Instr instr_xmmsqrt = { { 0x51,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x10102 };
const X64Instr instr_ucomisd = { { 0x2e,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x4102  };
const X64Instr instr_pxor=     { { 0xef,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x4102  };
const X64Instr instr_psrlq=    { { 0xF1,0xF1,0x73,0x02,0xF1,0xF1 }, 0x4112  };
const X64Instr instr_psllq=    { { 0xF1,0xF1,0x73,0x06,0xF1,0xF1 }, 0x4112  };
const X64Instr instr_cvtsi2sd= { { 0x2a,0x2a,0xF1,0x00,0xF1,0xF1 }, 0x10002 };
const X64Instr instr_cvttsd2si={ { 0x2c,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x10002 };
const X64Instr instr_lddqu =   { { 0xF0,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x10103 };
const X64Instr instr_unpcklpd ={ { 0x14,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x4102  };
const X64Instr instr_jmp =     { { 0xFF,0xF1,0xE9,0x04,0xE9,0xF1 }, 0x0910  };
const X64Instr instr_call =    { { 0xFF,0xF1,0xE8,0x02,0xE8,0xF1 }, 0x0900  };
const X64Instr instr_push =    { { 0xFF,0xF1,0x68,0x06,0xF1,0x50 }, 0x0510  };
const X64Instr instr_pop =     { { 0x8F,0xF1,0xF1,0x00,0xF1,0x58 }, 0x0500  };
const X64Instr instr_inc =     { { 0xFF,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_dec =     { { 0xFF,0xF1,0xF1,0x01,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_not =     { { 0xF7,0xF1,0xF1,0x02,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_notb =    { { 0xF6,0xF1,0xF1,0x02,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_neg =     { { 0xF7,0xF1,0xF1,0x03,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_negb =    { { 0xF6,0xF1,0xF1,0x03,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_add =     { { 0x01,0x03,0x81,0x00,0x05,0xF1 }, 0x0810  };
const X64Instr instr_addb =    { { 0x00,0x02,0x80,0x00,0x04,0xF1 }, 0x0810  };
const X64Instr instr_sub =     { { 0x29,0x2B,0x81,0x05,0x2D,0xF1 }, 0x0810  };
const X64Instr instr_subb =    { { 0x28,0x2A,0x80,0x05,0x2C,0xF1 }, 0x0810  };
const X64Instr instr_and =     { { 0x21,0x23,0x81,0x04,0x25,0xF1 }, 0x0810  };
const X64Instr instr_andb =    { { 0x20,0x22,0x80,0x04,0x24,0xF1 }, 0x0810  };
const X64Instr instr_or  =     { { 0x09,0x0B,0x81,0x01,0x0D,0xF1 }, 0x0810  };
const X64Instr instr_orb =     { { 0x08,0x0A,0x80,0x01,0x0C,0xF1 }, 0x0810  };
const X64Instr instr_xor =     { { 0x31,0x33,0x81,0x06,0x35,0xF1 }, 0x0810  };
const X64Instr instr_xorb =    { { 0x30,0x32,0x80,0x06,0x34,0xF1 }, 0x0810  };
const X64Instr instr_mov =     { { 0x89,0x8B,0xC7,0x00,0xF1,0xB8 }, 0x0600  };
const X64Instr instr_movb =    { { 0x88,0x8A,0xC6,0x00,0xF1,0xB0 }, 0x0610  };
const X64Instr instr_test =    { { 0x85,0x85,0xF7,0x00,0xA9,0xF1 }, 0x0800  };
const X64Instr instr_testb =   { { 0x84,0x84,0xF6,0x00,0xA8,0xF1 }, 0x0810  };
const X64Instr instr_cmp =     { { 0x39,0x3B,0x81,0x07,0x3D,0xF1 }, 0x0810  };
const X64Instr instr_cmpb =    { { 0x38,0x3A,0x80,0x07,0x3C,0xF1 }, 0x0810  };
const X64Instr instr_sbb =     { { 0x19,0x1B,0x81,0x03,0x1D,0xF1 }, 0x0810  };
const X64Instr instr_sbbb =    { { 0x18,0x1A,0x80,0x03,0x1C,0xF1 }, 0x0810  };
const X64Instr instr_adc =     { { 0x11,0x13,0x81,0x02,0x15,0xF1 }, 0x0810  };
const X64Instr instr_lea =     { { 0xF1,0x8D,0xF1,0x00,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_xchgb =   { { 0x86,0x86,0xF1,0x00,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_xchg =    { { 0x87,0x87,0xF1,0x00,0xF1,0x90 }, 0x1000  };
const X64Instr instr_imul =    { { 0xAF,0xF7,0x69,0x05,0xF1,0xF1 }, 0x0019  };
const X64Instr instr_mul =     { { 0xF7,0xF1,0xF1,0x04,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_div =     { { 0xF7,0xF1,0xF1,0x06,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_idiv =    { { 0xF7,0xF1,0xF1,0x07,0xF1,0xF1 }, 0x0000  };
const X64Instr instr_cdq =     { { 0xF1,0xF1,0xF1,0x00,0xF1,0x99 }, 0x0400  };
const X64Instr instr_ret =     { { 0xF1,0xF1,0xC2,0x00,0xF1,0xC3 }, 0x0540  };
const X64Instr instr_jcc =     { { 0xF1,0xF1,0x80,0x00,0xF1,0xF1 }, 0x0114  };
const X64Instr instr_cmovcc =  { { 0x40,0x40,0xF1,0x00,0xF1,0xF1 }, 0x0003  };
const X64Instr instr_setcc =   { { 0x90,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x0102  };
const X64Instr instr_movswx =  { { 0xBF,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x0003  };
const X64Instr instr_movsbx =  { { 0xBE,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x2003  };
const X64Instr instr_movzwx =  { { 0xB7,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x0003  };
const X64Instr instr_movzbx =  { { 0xB6,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x2003  };
const X64Instr instr_cwde =    { { 0xF1,0xF1,0xF1,0x00,0xF1,0x98 }, 0x0400  };
const X64Instr instr_cqo =     { { 0xF1,0xF1,0xF1,0x00,0xF1,0x99 }, 0x0000  };
const X64Instr instr_rol =     { { 0xD3,0xF1,0xC1,0x00,0xF1,0xF1 }, 0x0020  };
const X64Instr instr_ror =     { { 0xD3,0xF1,0xC1,0x01,0xF1,0xF1 }, 0x0020  };
const X64Instr instr_rcl =     { { 0xD3,0xF1,0xC1,0x02,0xF1,0xF1 }, 0x0020  };
const X64Instr instr_rcr =     { { 0xD3,0xF1,0xC1,0x03,0xF1,0xF1 }, 0x0020  };
const X64Instr instr_shl =     { { 0xD3,0xF1,0xC1,0x04,0xF1,0xF1 }, 0x0020  };
const X64Instr instr_shr =     { { 0xD3,0xF1,0xC1,0x05,0xF1,0xF1 }, 0x0020  };
const X64Instr instr_sar =     { { 0xD3,0xF1,0xC1,0x07,0xF1,0xF1 }, 0x0020  };
const X64Instr instr_xadd =    { { 0xC1,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x0002  };
const X64Instr instr_cmpxchg = { { 0xB1,0xF1,0xF1,0x00,0xF1,0xF1 }, 0x0002  };
const X64Instr instr_nop =     { { 0xF1,0xF1,0xF1,0x00,0xF1,0x90 }, 0x0500  };
const X64Instr instr_shld =    { { 0xA5,0xF1,0xA4,0x00,0xF1,0xF1 }, 0x0082  };
const X64Instr instr_shrd =    { { 0xAD,0xF1,0xAC,0x00,0xF1,0xF1 }, 0x0082  };
const X64Instr instr_int3 =    { { 0xF1,0xF1,0xF1,0x00,0xF1,0xCC }, 0x0500  };
const X64Instr instr_roundsd = { { 0xF1,0xF1,0x0b,0x00,0xF1,0xF1 }, 0x64112 };
const X64Instr instr_cmpsd =   { { 0xF1,0xF1,0xC2,0xF1,0xF1,0xF1 }, 0x10112 };



/**
 * PPC64 isntructions - TODO under development
 */
const PPC64Instr ppc64_instr_tdi =  { {0x08, 0x00, 0x00, 0x00/*ARI*/}, 0x00000 };
const PPC64Instr ppc64_instr_twi =  { {0x0C, 0x00, 0x00, 0x00/*ARI*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulli =  { {0x1C, 0x00, 0x00, 0x00/*RRI*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfic =  { {0x20, 0x00, 0x00, 0x00/*RRI*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmplwi =  { {0x28, 0x00, 0x00, 0x00/*XRU*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmplwi_ =  { {0x28, 0x00, 0x00, 0x00/*-RU*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpldi =  { {0x28, 0x20, 0x00, 0x00/*XRU*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpldi_ =  { {0x28, 0x20, 0x00, 0x00/*-RU*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpwi =  { {0x2C, 0x00, 0x00, 0x00/*XRI*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpwi_ =  { {0x2C, 0x00, 0x00, 0x00/*-RI*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpdi =  { {0x2C, 0x20, 0x00, 0x00/*XRI*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpdi_ =  { {0x2C, 0x20, 0x00, 0x00/*-RI*/}, 0x00000 };
const PPC64Instr ppc64_instr_addic =  { {0x30, 0x00, 0x00, 0x00/*RRI*/}, 0x00000 };
const PPC64Instr ppc64_instr_addic_ = { {0x34, 0x00, 0x00, 0x00/*RRI*/}, 0x00000 };
const PPC64Instr ppc64_instr_addi =  { {0x38, 0x00, 0x00, 0x00/*RR0I*/}, 0x00000 };
const PPC64Instr ppc64_instr_li =  { {0x38, 0x00, 0x00, 0x00/*RI*/}, 0x00000 };
const PPC64Instr ppc64_instr_la =  { {0x38, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_addis =  { {0x3C, 0x00, 0x00, 0x00/*RR0I*/}, 0x00000 };
const PPC64Instr ppc64_instr_lis =  { {0x3C, 0x00, 0x00, 0x00/*RI*/}, 0x00000 };
const PPC64Instr ppc64_instr_lus =  { {0x3C, 0x00, 0x00, 0x00/*RU*/}, 0x00000 };
const PPC64Instr ppc64_instr_bc =  { {0x40, 0x00, 0x00, 0x00/*AAK*/}, 0x00000 };
const PPC64Instr ppc64_instr_bcl =  { {0x40, 0x00, 0x00, 0x01/*AAK*/}, 0x00000 };
const PPC64Instr ppc64_instr_bdnz =  { {0x42, 0x00, 0x00, 0x00/*K*/}, 0x00000 };
const PPC64Instr ppc64_instr_bdz =  { {0x42, 0x40, 0x00, 0x00/*K*/}, 0x00000 };
const PPC64Instr ppc64_instr_sc_0 =  { {0x44, 0x00, 0x00, 0x00/**/}, 0x00000 };
const PPC64Instr ppc64_instr_b =  { {0x48, 0x00, 0x00, 0x00/*J*/}, 0x00000 };
const PPC64Instr ppc64_instr_bl =  { {0x48, 0x00, 0x00, 0x01/*J*/}, 0x00000 };
const PPC64Instr ppc64_instr_rlwimi =  { {0x50, 0x00, 0x00, 0x00/*RR~AAA.*/}, 0x00000 };
const PPC64Instr ppc64_instr_rlwinm =  { {0x54, 0x00, 0x00, 0x00/*RR~AAA.*/}, 0x00000 };
const PPC64Instr ppc64_instr_rlwnm =  { {0x5C, 0x00, 0x00, 0x00/*RR~RAA.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ori =  { {0x60, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
const PPC64Instr ppc64_instr_nop_0 =  { {0x60, 0x00, 0x00, 0x00/**/}, 0x00000 };
const PPC64Instr ppc64_instr_oris =  { {0x64, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
const PPC64Instr ppc64_instr_xori =  { {0x68, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
const PPC64Instr ppc64_instr_xoris =  { {0x6C, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
const PPC64Instr ppc64_instr_andi_ =  { {0x70, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
const PPC64Instr ppc64_instr_andis_ = { {0x74, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwz =  { {0x80, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwzu =  { {0x84, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lbz =  { {0x88, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lbzu =  { {0x8C, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stw =  { {0x90, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stwu =  { {0x94, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stb =  { {0x98, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stbu =  { {0x9C, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lhz =  { {0xA0, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lhzu =  { {0xA4, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lha =  { {0xA8, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lhau =  { {0xAC, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_sth =  { {0xB0, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_sthu =  { {0xB4, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lmw =  { {0xB8, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stmw =  { {0xBC, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfs =  { {0xC0, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfsu =  { {0xC4, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfd =  { {0xC8, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfdu =  { {0xCC, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfs =  { {0xD0, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfsu =  { {0xD4, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfd =  { {0xD8, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfdu =  { {0xDC, 0x00, 0x00, 0x00/*FD*/}, 0x00000 };
const PPC64Instr ppc64_instr_ld =  { {0xE8, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_ldu =  { {0xE8, 0x00, 0x00, 0x01/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwa =  { {0xE8, 0x00, 0x00, 0x02/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_std =  { {0xF8, 0x00, 0x00, 0x00/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_stdu =  { {0xF8, 0x00, 0x00, 0x01/*RD*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulhhwu =  { {0x10, 0x00, 0x00, 0x10/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_machhwu =  { {0x10, 0x00, 0x00, 0x18/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulhhw =  { {0x10, 0x00, 0x00, 0x50/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmachhw =  { {0x10, 0x00, 0x00, 0x5C/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_machhwsu =  { {0x10, 0x00, 0x00, 0x98/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_machhws =  { {0x10, 0x00, 0x00, 0xD8/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmachhws =  { {0x10, 0x00, 0x00, 0xDC/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulchwu =  { {0x10, 0x00, 0x01, 0x10/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_macchwu =  { {0x10, 0x00, 0x01, 0x18/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulchw =  { {0x10, 0x00, 0x01, 0x50/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_macchw =  { {0x10, 0x00, 0x01, 0x58/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmacchw =  { {0x10, 0x00, 0x01, 0x5C/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_macchwsu =  { {0x10, 0x00, 0x01, 0x98/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_macchws =  { {0x10, 0x00, 0x01, 0xD8/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmacchws =  { {0x10, 0x00, 0x01, 0xDC/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mullhw =  { {0x10, 0x00, 0x03, 0x50/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_maclhw =  { {0x10, 0x00, 0x03, 0x58/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmaclhw =  { {0x10, 0x00, 0x03, 0x5C/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_maclhwsu =  { {0x10, 0x00, 0x03, 0x98/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_maclhws =  { {0x10, 0x00, 0x03, 0xD8/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmaclhws =  { {0x10, 0x00, 0x03, 0xDC/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_machhwuo =  { {0x10, 0x00, 0x04, 0x18/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmachhwo =  { {0x10, 0x00, 0x04, 0x5C/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_machhwsuo =  { {0x10, 0x00, 0x04, 0x98/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_machhwso =  { {0x10, 0x00, 0x04, 0xD8/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmachhwso =  { {0x10, 0x00, 0x04, 0xDC/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_macchwuo =  { {0x10, 0x00, 0x05, 0x18/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_macchwo =  { {0x10, 0x00, 0x05, 0x58/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmacchwo =  { {0x10, 0x00, 0x05, 0x5C/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_macchwsuo =  { {0x10, 0x00, 0x05, 0x98/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_macchwso =  { {0x10, 0x00, 0x05, 0xD8/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmacchwso =  { {0x10, 0x00, 0x05, 0xDC/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_maclhwo =  { {0x10, 0x00, 0x07, 0x58/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmaclhwo =  { {0x10, 0x00, 0x07, 0x5C/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_maclhwsuo =  { {0x10, 0x00, 0x07, 0x98/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_maclhwso =  { {0x10, 0x00, 0x07, 0xD8/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nmaclhwso =  { {0x10, 0x00, 0x07, 0xDC/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_vaddubm =  { {0x10, 0x00, 0x00, 0x00/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaxub =  { {0x10, 0x00, 0x00, 0x02/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vrlb =  { {0x10, 0x00, 0x00, 0x04/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpequb =  { {0x10, 0x00, 0x00, 0x06/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmuloub =  { {0x10, 0x00, 0x00, 0x08/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vaddfp =  { {0x10, 0x00, 0x00, 0x0A/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmrghb =  { {0x10, 0x00, 0x00, 0x0C/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkuhum =  { {0x10, 0x00, 0x00, 0x0E/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmhaddshs =  { {0x10, 0x00, 0x00, 0x20/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmhraddshs =  { {0x10, 0x00, 0x00, 0x21/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmladduhm =  { {0x10, 0x00, 0x00, 0x22/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmsumubm =  { {0x10, 0x00, 0x00, 0x24/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmsummbm =  { {0x10, 0x00, 0x00, 0x25/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmsumuhm =  { {0x10, 0x00, 0x00, 0x26/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmsumuhs =  { {0x10, 0x00, 0x00, 0x27/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmsumshm =  { {0x10, 0x00, 0x00, 0x28/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmsumshs =  { {0x10, 0x00, 0x00, 0x29/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsel =  { {0x10, 0x00, 0x00, 0x2A/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vperm =  { {0x10, 0x00, 0x00, 0x2B/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsldoi =  { {0x10, 0x00, 0x00, 0x2C/*VVVP*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpermxor =  { {0x10, 0x00, 0x00, 0x2D/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaddfp =  { {0x10, 0x00, 0x00, 0x2E/*VVVV~*/}, 0x00000 };
const PPC64Instr ppc64_instr_vnmsubfp =  { {0x10, 0x00, 0x00, 0x2F/*VVVV~*/}, 0x00000 };
const PPC64Instr ppc64_instr_vaddeuqm =  { {0x10, 0x00, 0x00, 0x3C/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vaddecuq =  { {0x10, 0x00, 0x00, 0x3D/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubeuqm =  { {0x10, 0x00, 0x00, 0x3E/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubecuq =  { {0x10, 0x00, 0x00, 0x3F/*VVVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vadduhm =  { {0x10, 0x00, 0x00, 0x40/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaxuh =  { {0x10, 0x00, 0x00, 0x42/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vrlh =  { {0x10, 0x00, 0x00, 0x44/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpequh =  { {0x10, 0x00, 0x00, 0x46/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmulouh =  { {0x10, 0x00, 0x00, 0x48/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubfp =  { {0x10, 0x00, 0x00, 0x4A/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmrghh =  { {0x10, 0x00, 0x00, 0x4C/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkuwum =  { {0x10, 0x00, 0x00, 0x4E/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vadduwm =  { {0x10, 0x00, 0x00, 0x80/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaxuw =  { {0x10, 0x00, 0x00, 0x82/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vrlw =  { {0x10, 0x00, 0x00, 0x84/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpequw =  { {0x10, 0x00, 0x00, 0x86/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmulouw =  { {0x10, 0x00, 0x00, 0x88/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmuluwm =  { {0x10, 0x00, 0x00, 0x89/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmrghw =  { {0x10, 0x00, 0x00, 0x8C/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkuhus =  { {0x10, 0x00, 0x00, 0x8E/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vaddudm =  { {0x10, 0x00, 0x00, 0xC0/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaxud =  { {0x10, 0x00, 0x00, 0xC2/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vrld =  { {0x10, 0x00, 0x00, 0xC4/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpeqfp =  { {0x10, 0x00, 0x00, 0xC6/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpequd =  { {0x10, 0x00, 0x00, 0xC7/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkuwus =  { {0x10, 0x00, 0x00, 0xCE/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vadduqm =  { {0x10, 0x00, 0x01, 0x00/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaxsb =  { {0x10, 0x00, 0x01, 0x02/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vslb =  { {0x10, 0x00, 0x01, 0x04/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmulosb =  { {0x10, 0x00, 0x01, 0x08/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vrefp =  { {0x10, 0x00, 0x01, 0x0A/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmrglb =  { {0x10, 0x00, 0x01, 0x0C/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkshus =  { {0x10, 0x00, 0x01, 0x0E/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vaddcuq =  { {0x10, 0x00, 0x01, 0x40/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaxsh =  { {0x10, 0x00, 0x01, 0x42/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vslh =  { {0x10, 0x00, 0x01, 0x44/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmulosh =  { {0x10, 0x00, 0x01, 0x48/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vrsqrtefp =  { {0x10, 0x00, 0x01, 0x4A/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmrglh =  { {0x10, 0x00, 0x01, 0x4C/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkswus =  { {0x10, 0x00, 0x01, 0x4E/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vaddcuw =  { {0x10, 0x00, 0x01, 0x80/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaxsw =  { {0x10, 0x00, 0x01, 0x82/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vslw =  { {0x10, 0x00, 0x01, 0x84/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmulosw =  { {0x10, 0x00, 0x01, 0x88/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vexptefp =  { {0x10, 0x00, 0x01, 0x8A/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmrglw =  { {0x10, 0x00, 0x01, 0x8C/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkshss =  { {0x10, 0x00, 0x01, 0x8E/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaxsd =  { {0x10, 0x00, 0x01, 0xC2/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsl =  { {0x10, 0x00, 0x01, 0xC4/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgefp =  { {0x10, 0x00, 0x01, 0xC6/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vlogefp =  { {0x10, 0x00, 0x01, 0xCA/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkswss =  { {0x10, 0x00, 0x01, 0xCE/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vadduhs =  { {0x10, 0x00, 0x02, 0x40/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vminuh =  { {0x10, 0x00, 0x02, 0x42/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsrh =  { {0x10, 0x00, 0x02, 0x44/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtuh =  { {0x10, 0x00, 0x02, 0x46/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmuleuh =  { {0x10, 0x00, 0x02, 0x48/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vrfiz =  { {0x10, 0x00, 0x02, 0x4A/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsplth =  { {0x10, 0x00, 0x02, 0x4C/*VV3*/}, 0x00000 };
const PPC64Instr ppc64_instr_vupkhsh =  { {0x10, 0x00, 0x02, 0x4E/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vminuw =  { {0x10, 0x00, 0x02, 0x82/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vminud =  { {0x10, 0x00, 0x02, 0xC2/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtud =  { {0x10, 0x00, 0x02, 0xC7/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vrfim =  { {0x10, 0x00, 0x02, 0xCA/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtsb =  { {0x10, 0x00, 0x03, 0x06/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcfux =  { {0x10, 0x00, 0x03, 0x0A/*VVA~*/}, 0x00000 };
const PPC64Instr ppc64_instr_vaddshs =  { {0x10, 0x00, 0x03, 0x40/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vminsh =  { {0x10, 0x00, 0x03, 0x42/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsrah =  { {0x10, 0x00, 0x03, 0x44/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtsh =  { {0x10, 0x00, 0x03, 0x46/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmulesh =  { {0x10, 0x00, 0x03, 0x48/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcfsx =  { {0x10, 0x00, 0x03, 0x4A/*VVA~*/}, 0x00000 };
const PPC64Instr ppc64_instr_vspltish =  { {0x10, 0x00, 0x03, 0x4C/*VS*/}, 0x00000 };
const PPC64Instr ppc64_instr_vupkhpx =  { {0x10, 0x00, 0x03, 0x4E/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vaddsws =  { {0x10, 0x00, 0x03, 0x80/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vminsw =  { {0x10, 0x00, 0x03, 0x82/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsraw =  { {0x10, 0x00, 0x03, 0x84/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtsw =  { {0x10, 0x00, 0x03, 0x86/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmulesw =  { {0x10, 0x00, 0x03, 0x88/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vctuxs =  { {0x10, 0x00, 0x03, 0x8A/*VVA~*/}, 0x00000 };
const PPC64Instr ppc64_instr_vspltisw =  { {0x10, 0x00, 0x03, 0x8C/*VS*/}, 0x00000 };
const PPC64Instr ppc64_instr_vminsd =  { {0x10, 0x00, 0x03, 0xC2/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsrad =  { {0x10, 0x00, 0x03, 0xC4/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpbfp =  { {0x10, 0x00, 0x03, 0xC6/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtsd =  { {0x10, 0x00, 0x03, 0xC7/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vctsxs =  { {0x10, 0x00, 0x03, 0xCA/*VVA~*/}, 0x00000 };
const PPC64Instr ppc64_instr_vupklpx =  { {0x10, 0x00, 0x03, 0xCE/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsububm =  { {0x10, 0x00, 0x04, 0x00/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_bcdadd_ =  { {0x10, 0x00, 0x04, 0x01/*VVVY.*/}, 0x00000 };
const PPC64Instr ppc64_instr_vavgub =  { {0x10, 0x00, 0x04, 0x02/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vand =  { {0x10, 0x00, 0x04, 0x04/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpequb_ =  { {0x10, 0x00, 0x04, 0x06/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmaxfp =  { {0x10, 0x00, 0x04, 0x0A/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubuhm =  { {0x10, 0x00, 0x04, 0x40/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_bcdsub_ =  { {0x10, 0x00, 0x04, 0x41/*VVVY.*/}, 0x00000 };
const PPC64Instr ppc64_instr_vavguh =  { {0x10, 0x00, 0x04, 0x42/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vandc =  { {0x10, 0x00, 0x04, 0x44/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpequh_ =  { {0x10, 0x00, 0x04, 0x46/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vminfp =  { {0x10, 0x00, 0x04, 0x4A/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkudum =  { {0x10, 0x00, 0x04, 0x4E/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubuwm =  { {0x10, 0x00, 0x04, 0x80/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vavguw =  { {0x10, 0x00, 0x04, 0x82/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vor =  { {0x10, 0x00, 0x04, 0x84/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpequw_ =  { {0x10, 0x00, 0x04, 0x86/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpmsumw =  { {0x10, 0x00, 0x04, 0x88/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpeqfp_ =  { {0x10, 0x00, 0x04, 0xC6/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpequd_ =  { {0x10, 0x00, 0x04, 0xC7/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpkudus =  { {0x10, 0x00, 0x04, 0xCE/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vavgsb =  { {0x10, 0x00, 0x05, 0x02/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vavgsh =  { {0x10, 0x00, 0x05, 0x42/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vorc =  { {0x10, 0x00, 0x05, 0x44/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vbpermq =  { {0x10, 0x00, 0x05, 0x4C/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpksdus =  { {0x10, 0x00, 0x05, 0x4E/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vavgsw =  { {0x10, 0x00, 0x05, 0x82/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsld =  { {0x10, 0x00, 0x05, 0xC4/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgefp_ =  { {0x10, 0x00, 0x05, 0xC6/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpksdss =  { {0x10, 0x00, 0x05, 0xCE/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsububs =  { {0x10, 0x00, 0x06, 0x00/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_mfvscr =  { {0x10, 0x00, 0x06, 0x04/*V--*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsum4ubs =  { {0x10, 0x00, 0x06, 0x08/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubuhs =  { {0x10, 0x00, 0x06, 0x40/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtvscr =  { {0x10, 0x00, 0x06, 0x44/*--V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtuh_ =  { {0x10, 0x00, 0x06, 0x46/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsum4shs =  { {0x10, 0x00, 0x06, 0x48/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vupkhsw =  { {0x10, 0x00, 0x06, 0x4E/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubuws =  { {0x10, 0x00, 0x06, 0x80/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vshasigmaw =  { {0x10, 0x00, 0x06, 0x82/*VVYP*/}, 0x00000 };
const PPC64Instr ppc64_instr_veqv =  { {0x10, 0x00, 0x06, 0x84/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsum2sws =  { {0x10, 0x00, 0x06, 0x88/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmrgow =  { {0x10, 0x00, 0x06, 0x8C/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vshasigmad =  { {0x10, 0x00, 0x06, 0xC2/*VVYP*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsrd =  { {0x10, 0x00, 0x06, 0xC4/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtud_ =  { {0x10, 0x00, 0x06, 0xC7/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vupklsw =  { {0x10, 0x00, 0x06, 0xCE/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vupkslw =  { {0x10, 0x00, 0x06, 0xCE/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubsbs =  { {0x10, 0x00, 0x07, 0x00/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vclzb =  { {0x10, 0x00, 0x07, 0x02/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpopcntb =  { {0x10, 0x00, 0x07, 0x03/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtsb_ =  { {0x10, 0x00, 0x07, 0x06/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsum4sbs =  { {0x10, 0x00, 0x07, 0x08/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubshs =  { {0x10, 0x00, 0x07, 0x40/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vclzh =  { {0x10, 0x00, 0x07, 0x42/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpopcnth =  { {0x10, 0x00, 0x07, 0x43/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtsh_ =  { {0x10, 0x00, 0x07, 0x46/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsubsws =  { {0x10, 0x00, 0x07, 0x80/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vclzw =  { {0x10, 0x00, 0x07, 0x82/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpopcntw =  { {0x10, 0x00, 0x07, 0x83/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtsw_ =  { {0x10, 0x00, 0x07, 0x86/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vsumsws =  { {0x10, 0x00, 0x07, 0x88/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vmrgew =  { {0x10, 0x00, 0x07, 0x8C/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vclzd =  { {0x10, 0x00, 0x07, 0xC2/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vpopcntd =  { {0x10, 0x00, 0x07, 0xC3/*V-V*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpbfp_ =  { {0x10, 0x00, 0x07, 0xC6/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_vcmpgtsd_ =  { {0x10, 0x00, 0x07, 0xC7/*VVV*/}, 0x00000 };
const PPC64Instr ppc64_instr_mcrf =  { {0x4C, 0x00, 0x00, 0x00/*XX*/}, 0x00000 };
const PPC64Instr ppc64_instr_isync_0 =  { {0x4C, 0x00, 0x01, 0x2C/**/}, 0x00000 };
const PPC64Instr ppc64_instr_crnor =  { {0x4C, 0x00, 0x00, 0x42/*CCC*/}, 0x00000 };
const PPC64Instr ppc64_instr_crnot =  { {0x4C, 0x00, 0x00, 0x42/*CC=*/}, 0x00000 };
const PPC64Instr ppc64_instr_crandc =  { {0x4C, 0x00, 0x01, 0x02/*CCC*/}, 0x00000 };
const PPC64Instr ppc64_instr_crxor =  { {0x4C, 0x00, 0x01, 0x82/*CCC*/}, 0x00000 };
const PPC64Instr ppc64_instr_crclr =  { {0x4C, 0x00, 0x01, 0x82/*C==*/}, 0x00000 };
const PPC64Instr ppc64_instr_crnand =  { {0x4C, 0x00, 0x01, 0xC2/*CCC*/}, 0x00000 };
const PPC64Instr ppc64_instr_crand =  { {0x4C, 0x00, 0x02, 0x02/*CCC*/}, 0x00000 };
const PPC64Instr ppc64_instr_creqv =  { {0x4C, 0x00, 0x02, 0x42/*CCC*/}, 0x00000 };
const PPC64Instr ppc64_instr_crset =  { {0x4C, 0x00, 0x02, 0x42/*C==*/}, 0x00000 };
const PPC64Instr ppc64_instr_crorc =  { {0x4C, 0x00, 0x03, 0x42/*CCC*/}, 0x00000 };
const PPC64Instr ppc64_instr_cror =  { {0x4C, 0x00, 0x03, 0x82/*CCC*/}, 0x00000 };
const PPC64Instr ppc64_instr_crmove =  { {0x4C, 0x00, 0x03, 0x82/*CC=*/}, 0x00000 };
const PPC64Instr ppc64_instr_bclr =  { {0x4C, 0x00, 0x00, 0x20/*AA*/}, 0x00000 };
const PPC64Instr ppc64_instr_bclrl =  { {0x4C, 0x00, 0x00, 0x21/*AA*/}, 0x00000 };
const PPC64Instr ppc64_instr_bcctr =  { {0x4C, 0x00, 0x04, 0x20/*AA*/}, 0x00000 };
const PPC64Instr ppc64_instr_bcctrl =  { {0x4C, 0x00, 0x04, 0x21/*AA*/}, 0x00000 };
const PPC64Instr ppc64_instr_bctar =  { {0x4C, 0x00, 0x04, 0x60/*AA*/}, 0x00000 };
const PPC64Instr ppc64_instr_bctarl =  { {0x4C, 0x00, 0x04, 0x61/*AA*/}, 0x00000 };
const PPC64Instr ppc64_instr_blr_0 =  { {0x4E, 0x80, 0x00, 0x20/**/}, 0x00000 };
const PPC64Instr ppc64_instr_blrl_0 =  { {0x4E, 0x80, 0x00, 0x21/**/}, 0x00000 };
const PPC64Instr ppc64_instr_bctr_0 =  { {0x4E, 0x80, 0x04, 0x20/**/}, 0x00000 };
const PPC64Instr ppc64_instr_bctrl_0 =  { {0x4E, 0x80, 0x04, 0x21/**/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpw =  { {0x7C, 0x00, 0x00, 0x00/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpw_ =  { {0x7C, 0x00, 0x00, 0x00/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpd =  { {0x7C, 0x20, 0x00, 0x00/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpd_ =  { {0x7C, 0x20, 0x00, 0x00/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_tw =  { {0x7C, 0x00, 0x00, 0x08/*ARR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lvsl =  { {0x7C, 0x00, 0x00, 0x0C/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfc =  { {0x7C, 0x00, 0x00, 0x10/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_subc =  { {0x7C, 0x00, 0x00, 0x10/*RRR~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulhdu =  { {0x7C, 0x00, 0x00, 0x12/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_addc =  { {0x7C, 0x00, 0x00, 0x14/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulhwu =  { {0x7C, 0x00, 0x00, 0x16/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_isel =  { {0x7C, 0x00, 0x00, 0x1E/*RRRC*/}, 0x00000 };
const PPC64Instr ppc64_instr_isellt =  { {0x7C, 0x00, 0x00, 0x1E/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_iselgt =  { {0x7C, 0x00, 0x00, 0x5E/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_iseleq =  { {0x7C, 0x00, 0x00, 0x9E/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_mfcr =  { {0x7C, 0x00, 0x00, 0x26/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_mfocrf =  { {0x7C, 0x10, 0x00, 0x26/*RG*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtcrf =  { {0x7C, 0x00, 0x01, 0x20/*GR*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtocrf =  { {0x7C, 0x10, 0x01, 0x20/*GR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwarx =  { {0x7C, 0x00, 0x00, 0x28/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_ldx =  { {0x7C, 0x00, 0x00, 0x2A/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwzx =  { {0x7C, 0x00, 0x00, 0x2E/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_slw =  { {0x7C, 0x00, 0x00, 0x30/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_cntlzw =  { {0x7C, 0x00, 0x00, 0x34/*RR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_sld =  { {0x7C, 0x00, 0x00, 0x36/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_and =  { {0x7C, 0x00, 0x00, 0x38/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmplw =  { {0x7C, 0x00, 0x00, 0x40/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmplw_ =  { {0x7C, 0x00, 0x00, 0x40/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpld =  { {0x7C, 0x20, 0x00, 0x40/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpld_ =  { {0x7C, 0x20, 0x00, 0x40/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lvsr =  { {0x7C, 0x00, 0x00, 0x4C/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subf =  { {0x7C, 0x00, 0x00, 0x50/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_sub =  { {0x7C, 0x00, 0x00, 0x50/*RRR~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_lbarx =  { {0x7C, 0x00, 0x00, 0x68/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_ldux =  { {0x7C, 0x00, 0x00, 0x6A/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcbst =  { {0x7C, 0x00, 0x00, 0x6C/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwzux =  { {0x7C, 0x00, 0x00, 0x6E/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_cntlzd =  { {0x7C, 0x00, 0x00, 0x74/*RR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_andc =  { {0x7C, 0x00, 0x00, 0x78/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_td =  { {0x7C, 0x00, 0x00, 0x88/*ARR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lvewx =  { {0x7C, 0x00, 0x00, 0x8E/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulhd =  { {0x7C, 0x00, 0x00, 0x92/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_addg6s =  { {0x7C, 0x00, 0x00, 0x94/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulhw =  { {0x7C, 0x00, 0x00, 0x96/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dlmzb =  { {0x7C, 0x00, 0x00, 0x9C/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ldarx =  { {0x7C, 0x00, 0x00, 0xA8/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcbf =  { {0x7C, 0x00, 0x00, 0xAC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lbzx =  { {0x7C, 0x00, 0x00, 0xAE/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lvx =  { {0x7C, 0x00, 0x00, 0xCE/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_neg =  { {0x7C, 0x00, 0x00, 0xD0/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_lharx =  { {0x7C, 0x00, 0x00, 0xE8/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lbzux =  { {0x7C, 0x00, 0x00, 0xEE/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_popcntb =  { {0x7C, 0x00, 0x00, 0xF4/*RR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_not =  { {0x7C, 0x00, 0x00, 0xF8/*RR~%.*/}, 0x00000 };
const PPC64Instr ppc64_instr_nor =  { {0x7C, 0x00, 0x00, 0xF8/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_stvebx =  { {0x7C, 0x00, 0x01, 0x0E/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfe =  { {0x7C, 0x00, 0x01, 0x10/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_sube =  { {0x7C, 0x00, 0x01, 0x10/*RRR~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_adde =  { {0x7C, 0x00, 0x01, 0x14/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_stdx =  { {0x7C, 0x00, 0x01, 0x2A/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stwcx_ =  { {0x7C, 0x00, 0x01, 0x2D/*RR0R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_stwx =  { {0x7C, 0x00, 0x01, 0x2E/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_prtyw =  { {0x7C, 0x00, 0x01, 0x34/*RR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_stvehx =  { {0x7C, 0x00, 0x01, 0x4E/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stdux =  { {0x7C, 0x00, 0x01, 0x6A/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stqcx_ =  { {0x7C, 0x00, 0x01, 0x6D/*R:R0R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_stwux =  { {0x7C, 0x00, 0x01, 0x6E/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_prtyd =  { {0x7C, 0x00, 0x01, 0x74/*RR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_stvewx =  { {0x7C, 0x00, 0x01, 0x8E/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfze =  { {0x7C, 0x00, 0x01, 0x90/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_addze =  { {0x7C, 0x00, 0x01, 0x94/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_stdcx_ =  { {0x7C, 0x00, 0x01, 0xAD/*RR0R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_stbx =  { {0x7C, 0x00, 0x01, 0xAE/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stvx =  { {0x7C, 0x00, 0x01, 0xCE/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfme =  { {0x7C, 0x00, 0x01, 0xD0/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulld =  { {0x7C, 0x00, 0x01, 0xD2/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_addme =  { {0x7C, 0x00, 0x01, 0xD4/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mullw =  { {0x7C, 0x00, 0x01, 0xD6/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcbtst =  { {0x7C, 0x00, 0x01, 0xEC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stbux =  { {0x7C, 0x00, 0x01, 0xEE/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_bpermd =  { {0x7C, 0x00, 0x01, 0xF8/*RR~R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lvepxl =  { {0x7C, 0x00, 0x02, 0x0E/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_add =  { {0x7C, 0x00, 0x02, 0x14/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_lqarx =  { {0x7C, 0x00, 0x02, 0x28/*R:R0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcbt =  { {0x7C, 0x00, 0x02, 0x2C/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lhzx =  { {0x7C, 0x00, 0x02, 0x2E/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_cdtbcd =  { {0x7C, 0x00, 0x02, 0x34/*RR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_eqv =  { {0x7C, 0x00, 0x02, 0x38/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_lvepx =  { {0x7C, 0x00, 0x02, 0x4E/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_eciwx =  { {0x7C, 0x00, 0x02, 0x6C/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lhzux =  { {0x7C, 0x00, 0x02, 0x6E/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_cbcdtd =  { {0x7C, 0x00, 0x02, 0x74/*RR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_xor =  { {0x7C, 0x00, 0x02, 0x78/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mfspefscr =  { {0x7C, 0x00, 0x82, 0xA6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_mfxer =  { {0x7C, 0x01, 0x02, 0xA6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_mflr =  { {0x7C, 0x08, 0x02, 0xA6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_mfctr =  { {0x7C, 0x09, 0x02, 0xA6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwax =  { {0x7C, 0x00, 0x02, 0xAA/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lhax =  { {0x7C, 0x00, 0x02, 0xAE/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_mftb =  { {0x7C, 0x0C, 0x42, 0xE6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_mftbu =  { {0x7C, 0x0D, 0x42, 0xE6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lvxl =  { {0x7C, 0x00, 0x02, 0xCE/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwaux =  { {0x7C, 0x00, 0x02, 0xEA/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lhaux =  { {0x7C, 0x00, 0x02, 0xEE/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_popcntw =  { {0x7C, 0x00, 0x02, 0xF4/*RR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_divdeu =  { {0x7C, 0x00, 0x03, 0x12/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divweu =  { {0x7C, 0x00, 0x03, 0x16/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_sthx =  { {0x7C, 0x00, 0x03, 0x2E/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_orc =  { {0x7C, 0x00, 0x03, 0x38/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ecowx =  { {0x7C, 0x00, 0x03, 0x6C/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_sthux =  { {0x7C, 0x00, 0x03, 0x6E/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_or =  { {0x7C, 0x00, 0x03, 0x78/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mr =  { {0x7C, 0x00, 0x03, 0x78/*RR~%.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divdu =  { {0x7C, 0x00, 0x03, 0x92/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divwu =  { {0x7C, 0x00, 0x03, 0x96/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtspefscr =  { {0x7C, 0x00, 0x83, 0xA6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtxer =  { {0x7C, 0x01, 0x03, 0xA6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtlr =  { {0x7C, 0x08, 0x03, 0xA6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtctr =  { {0x7C, 0x09, 0x03, 0xA6/*R*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcbi =  { {0x7C, 0x00, 0x03, 0xAC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_nand =  { {0x7C, 0x00, 0x03, 0xB8/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dsn =  { {0x7C, 0x00, 0x03, 0xC6/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stvxl =  { {0x7C, 0x00, 0x03, 0xCE/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_divd =  { {0x7C, 0x00, 0x03, 0xD2/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divw =  { {0x7C, 0x00, 0x03, 0xD6/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_popcntd =  { {0x7C, 0x00, 0x03, 0xF4/*RR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_cmpb =  { {0x7C, 0x00, 0x03, 0xF8/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mcrxr =  { {0x7C, 0x00, 0x04, 0x00/*X*/}, 0x00000 };
const PPC64Instr ppc64_instr_lbdx =  { {0x7C, 0x00, 0x04, 0x06/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfco =  { {0x7C, 0x00, 0x04, 0x10/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_subco =  { {0x7C, 0x00, 0x04, 0x10/*RRR~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_addco =  { {0x7C, 0x00, 0x04, 0x14/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ldbrx =  { {0x7C, 0x00, 0x04, 0x28/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lswx =  { {0x7C, 0x00, 0x04, 0x2A/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwbrx =  { {0x7C, 0x00, 0x04, 0x2C/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfsx =  { {0x7C, 0x00, 0x04, 0x2E/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_srw =  { {0x7C, 0x00, 0x04, 0x30/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_srd =  { {0x7C, 0x00, 0x04, 0x36/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_lhdx =  { {0x7C, 0x00, 0x04, 0x46/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfo =  { {0x7C, 0x00, 0x04, 0x50/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_subo =  { {0x7C, 0x00, 0x04, 0x50/*RRR~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfsux =  { {0x7C, 0x00, 0x04, 0x6E/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lwdx =  { {0x7C, 0x00, 0x04, 0x86/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lswi =  { {0x7C, 0x00, 0x04, 0xAA/*RR0A*/}, 0x00000 };
const PPC64Instr ppc64_instr_sync_0 =  { {0x7C, 0x00, 0x04, 0xAC/**/}, 0x00000 };
const PPC64Instr ppc64_instr_lwsync_0 =  { {0x7C, 0x20, 0x04, 0xAC/**/}, 0x00000 };
const PPC64Instr ppc64_instr_ptesync_0 =  { {0x7C, 0x40, 0x04, 0xAC/**/}, 0x00000 };
const PPC64Instr ppc64_instr_lfdx =  { {0x7C, 0x00, 0x04, 0xAE/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lddx =  { {0x7C, 0x00, 0x04, 0xC6/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_nego =  { {0x7C, 0x00, 0x04, 0xD0/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfdux =  { {0x7C, 0x00, 0x04, 0xEE/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stbdx =  { {0x7C, 0x00, 0x05, 0x06/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfeo =  { {0x7C, 0x00, 0x05, 0x10/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_subeo =  { {0x7C, 0x00, 0x05, 0x10/*RRR~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_addeo =  { {0x7C, 0x00, 0x05, 0x14/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_stdbrx =  { {0x7C, 0x00, 0x05, 0x28/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stswx =  { {0x7C, 0x00, 0x05, 0x2A/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stwbrx =  { {0x7C, 0x00, 0x05, 0x2C/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfsx =  { {0x7C, 0x00, 0x05, 0x2E/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_sthdx =  { {0x7C, 0x00, 0x05, 0x46/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stbcx_ =  { {0x7C, 0x00, 0x05, 0x6D/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfsux =  { {0x7C, 0x00, 0x05, 0x6E/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stwdx =  { {0x7C, 0x00, 0x05, 0x86/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfzeo =  { {0x7C, 0x00, 0x05, 0x90/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_addzeo =  { {0x7C, 0x00, 0x05, 0x94/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_stswi =  { {0x7C, 0x00, 0x05, 0xAA/*RR0A*/}, 0x00000 };
const PPC64Instr ppc64_instr_sthcx_ =  { {0x7C, 0x00, 0x05, 0xAD/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfdx =  { {0x7C, 0x00, 0x05, 0xAE/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stddx =  { {0x7C, 0x00, 0x05, 0xC6/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_subfmeo =  { {0x7C, 0x00, 0x05, 0xD0/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mulldo =  { {0x7C, 0x00, 0x05, 0xD2/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_addmeo =  { {0x7C, 0x00, 0x05, 0xD4/*RR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mullwo =  { {0x7C, 0x00, 0x05, 0xD6/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcba =  { {0x7C, 0x00, 0x05, 0xEC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfdux =  { {0x7C, 0x00, 0x05, 0xEE/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stvepxl =  { {0x7C, 0x00, 0x06, 0x0E/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_addo =  { {0x7C, 0x00, 0x06, 0x14/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_lhbrx =  { {0x7C, 0x00, 0x06, 0x2C/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfdpx =  { {0x7C, 0x00, 0x06, 0x2E/*F:RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_sraw =  { {0x7C, 0x00, 0x06, 0x30/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_srad =  { {0x7C, 0x00, 0x06, 0x34/*RR~R.*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfddx =  { {0x7C, 0x00, 0x06, 0x46/*FRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stvepx =  { {0x7C, 0x00, 0x06, 0x4E/*VRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_srawi =  { {0x7C, 0x00, 0x06, 0x70/*RR~A.*/}, 0x00000 };
const PPC64Instr ppc64_instr_sradi =  { {0x7C, 0x00, 0x06, 0x74/*RR~H.*/}, 0x00000 };
const PPC64Instr ppc64_instr_eieio_0 =  { {0x7C, 0x00, 0x06, 0xAC/**/}, 0x00000 };
const PPC64Instr ppc64_instr_lfiwax =  { {0x7C, 0x00, 0x06, 0xAE/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_divdeuo =  { {0x7C, 0x00, 0x07, 0x12/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divweuo =  { {0x7C, 0x00, 0x07, 0x16/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_sthbrx =  { {0x7C, 0x00, 0x07, 0x2C/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfdpx =  { {0x7C, 0x00, 0x07, 0x2E/*F:RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_extsh =  { {0x7C, 0x00, 0x07, 0x34/*RR~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfddx =  { {0x7C, 0x00, 0x07, 0x46/*FRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_divdeo =  { {0x7C, 0x00, 0x07, 0x52/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divweo =  { {0x7C, 0x00, 0x07, 0x56/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_extsb =  { {0x7C, 0x00, 0x07, 0x74/*RR~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divduo =  { {0x7C, 0x00, 0x07, 0x92/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divwou =  { {0x7C, 0x00, 0x07, 0x96/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_icbi =  { {0x7C, 0x00, 0x07, 0xAC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfiwx =  { {0x7C, 0x00, 0x07, 0xAE/*FR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_extsw =  { {0x7C, 0x00, 0x07, 0xB4/*RR~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divdo =  { {0x7C, 0x00, 0x07, 0xD2/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_divwo =  { {0x7C, 0x00, 0x07, 0xD6/*RRR.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcbz =  { {0x7C, 0x00, 0x07, 0xEC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_tbegin_ =  { {0x7C, 0x00, 0x05, 0x1D/*1*/}, 0x00000 };
const PPC64Instr ppc64_instr_tbegin_0 =  { {0x7C, 0x00, 0x05, 0x1D/**/}, 0x00000 };
const PPC64Instr ppc64_instr_tend_ =  { {0x7C, 0x00, 0x05, 0x5D/*Y*/}, 0x00000 };
const PPC64Instr ppc64_instr_tend_0 =  { {0x7C, 0x00, 0x05, 0x5D/**/}, 0x00000 };
const PPC64Instr ppc64_instr_tendall_0 =  { {0x7E, 0x00, 0x05, 0x5D/**/}, 0x00000 };
const PPC64Instr ppc64_instr_tcheck =  { {0x7C, 0x00, 0x05, 0x9C/*X*/}, 0x00000 };
const PPC64Instr ppc64_instr_tsr_ =  { {0x7C, 0x00, 0x05, 0xDD/*1*/}, 0x00000 };
const PPC64Instr ppc64_instr_tsuspend_0 =  { {0x7C, 0x00, 0x05, 0xDD/**/}, 0x00000 };
const PPC64Instr ppc64_instr_tresume_0 =  { {0x7C, 0x20, 0x05, 0xDD/**/}, 0x00000 };
const PPC64Instr ppc64_instr_tabortwc_ =  { {0x7C, 0x00, 0x06, 0x1D/*ARR*/}, 0x00000 };
const PPC64Instr ppc64_instr_tabortdc_ =  { {0x7C, 0x00, 0x06, 0x5D/*ARR*/}, 0x00000 };
const PPC64Instr ppc64_instr_tabortwci_ =  { {0x7C, 0x00, 0x06, 0x9D/*ARS*/}, 0x00000 };
const PPC64Instr ppc64_instr_tabortdci_ =  { {0x7C, 0x00, 0x06, 0xDD/*ARS*/}, 0x00000 };
const PPC64Instr ppc64_instr_tabort_ =  { {0x7C, 0x00, 0x07, 0x1D/*-R-*/}, 0x00000 };
const PPC64Instr ppc64_instr_treclaim_ =  { {0x7C, 0x00, 0x07, 0x5D/*-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_trechkpt_0 =  { {0x7C, 0x00, 0x07, 0xDD/**/}, 0x00000 };
const PPC64Instr ppc64_instr_lxsiwzx =  { {0x7C, 0x00, 0x00, 0x18/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lxsiwax =  { {0x7C, 0x00, 0x00, 0x98/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_mfvsrd =  { {0x7C, 0x00, 0x00, 0x66/*-RQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_mfvsrwz =  { {0x7C, 0x00, 0x00, 0xE6/*-RQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_stxsiwx =  { {0x7C, 0x00, 0x01, 0x18/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtvsrd =  { {0x7C, 0x00, 0x01, 0x66/*QR*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtvsrwa =  { {0x7C, 0x00, 0x01, 0xA6/*QR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lxvdsx =  { {0x7C, 0x00, 0x02, 0x98/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lxsspx =  { {0x7C, 0x00, 0x04, 0x18/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lxsdx =  { {0x7C, 0x00, 0x04, 0x98/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stxsspx =  { {0x7C, 0x00, 0x05, 0x18/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stxsdx =  { {0x7C, 0x00, 0x05, 0x98/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lxvw4x =  { {0x7C, 0x00, 0x06, 0x18/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_lxvd2x =  { {0x7C, 0x00, 0x06, 0x98/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stxvw4x =  { {0x7C, 0x00, 0x07, 0x18/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_stxvd2x =  { {0x7C, 0x00, 0x07, 0x98/*QRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_rldicl =  { {0x78, 0x00, 0x00, 0x00/*RR~HM.*/}, 0x00000 };
const PPC64Instr ppc64_instr_rldicr =  { {0x78, 0x00, 0x00, 0x04/*RR~HM.*/}, 0x00000 };
const PPC64Instr ppc64_instr_rldic =  { {0x78, 0x00, 0x00, 0x08/*RR~HM.*/}, 0x00000 };
const PPC64Instr ppc64_instr_rldimi =  { {0x78, 0x00, 0x00, 0x0C/*RR~HM.*/}, 0x00000 };
const PPC64Instr ppc64_instr_rldcl =  { {0x78, 0x00, 0x00, 0x10/*RR~RM.*/}, 0x00000 };
const PPC64Instr ppc64_instr_rldcr =  { {0x78, 0x00, 0x00, 0x12/*RR~RM.*/}, 0x00000 };
const PPC64Instr ppc64_instr_sldi =  { {0x78, 0x00, 0x00, 0x04/*RR~L*/}, 0x00000 };
const PPC64Instr ppc64_instr_srdi =  { {0x78, 0x00, 0x00, 0x00/*RR~U*/}, 0x00000 };
const PPC64Instr ppc64_instr_lq =  { {0xE0, 0x00, 0x00, 0x00/*R:D*/}, 0x00000 };
const PPC64Instr ppc64_instr_lfdp =  { {0xE4, 0x00, 0x00, 0x00/*F:D*/}, 0x00000 };
const PPC64Instr ppc64_instr_fdivs =  { {0xEC, 0x00, 0x00, 0x24/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fsubs =  { {0xEC, 0x00, 0x00, 0x28/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fadds =  { {0xEC, 0x00, 0x00, 0x2A/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fsqrts =  { {0xEC, 0x00, 0x00, 0x2C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fres =  { {0xEC, 0x00, 0x00, 0x30/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fmuls =  { {0xEC, 0x00, 0x00, 0x32/*FF-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_frsqrtes =  { {0xEC, 0x00, 0x00, 0x34/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fmsubs =  { {0xEC, 0x00, 0x00, 0x38/*FFFF~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fmadds =  { {0xEC, 0x00, 0x00, 0x3A/*FFFF~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fnmsubs =  { {0xEC, 0x00, 0x00, 0x3C/*FFFF~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fnmadds =  { {0xEC, 0x00, 0x00, 0x3E/*FFFF~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fcfids =  { {0xEC, 0x00, 0x06, 0x9C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fcfidus =  { {0xEC, 0x00, 0x07, 0x9C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dadd =  { {0xEC, 0x00, 0x00, 0x04/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dqua =  { {0xEC, 0x00, 0x00, 0x06/*FFFZ.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dmul =  { {0xEC, 0x00, 0x00, 0x44/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_drrnd =  { {0xEC, 0x00, 0x00, 0x46/*FFFZ.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dscli =  { {0xEC, 0x00, 0x00, 0x84/*FF6.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dquai =  { {0xEC, 0x00, 0x00, 0x86/*SF~FZ.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dscri =  { {0xEC, 0x00, 0x00, 0xC4/*FF6.*/}, 0x00000 };
const PPC64Instr ppc64_instr_drintx =  { {0xEC, 0x00, 0x00, 0xC6/*1F~FZ.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcmpo =  { {0xEC, 0x00, 0x01, 0x04/*XFF*/}, 0x00000 };
const PPC64Instr ppc64_instr_dtstex =  { {0xEC, 0x00, 0x01, 0x44/*XFF*/}, 0x00000 };
const PPC64Instr ppc64_instr_dtstdc =  { {0xEC, 0x00, 0x01, 0x84/*XF6*/}, 0x00000 };
const PPC64Instr ppc64_instr_dtstdg =  { {0xEC, 0x00, 0x01, 0xC4/*XF6*/}, 0x00000 };
const PPC64Instr ppc64_instr_drintn =  { {0xEC, 0x00, 0x01, 0xC6/*1F~FZ.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dctdp =  { {0xEC, 0x00, 0x02, 0x04/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dctfix =  { {0xEC, 0x00, 0x02, 0x44/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ddedpd =  { {0xEC, 0x00, 0x02, 0x84/*ZF~F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dxex =  { {0xEC, 0x00, 0x02, 0xC4/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dsub =  { {0xEC, 0x00, 0x04, 0x04/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ddiv =  { {0xEC, 0x00, 0x04, 0x44/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcmpu =  { {0xEC, 0x00, 0x05, 0x04/*XFF*/}, 0x00000 };
const PPC64Instr ppc64_instr_dtstsf =  { {0xEC, 0x00, 0x05, 0x44/*XFF*/}, 0x00000 };
const PPC64Instr ppc64_instr_drsp =  { {0xEC, 0x00, 0x06, 0x04/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcffix =  { {0xEC, 0x00, 0x06, 0x44/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_denbcd =  { {0xEC, 0x00, 0x06, 0x84/*YF~F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_diex =  { {0xEC, 0x00, 0x06, 0xC4/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsaddsp =  { {0xF0, 0x00, 0x00, 0x00/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmaddasp =  { {0xF0, 0x00, 0x00, 0x08/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxsldwi =  { {0xF0, 0x00, 0x00, 0x10/*QQQZ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsrsqrtesp =  { {0xF0, 0x00, 0x00, 0x28/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xssqrtsp =  { {0xF0, 0x00, 0x00, 0x2C/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxsel =  { {0xF0, 0x00, 0x00, 0x30/*QQQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xssubsp =  { {0xF0, 0x00, 0x00, 0x40/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmaddmsp =  { {0xF0, 0x00, 0x00, 0x48/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxpermdi =  { {0xF0, 0x00, 0x00, 0x50/*QQQZ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsresp =  { {0xF0, 0x00, 0x00, 0x68/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmulsp =  { {0xF0, 0x00, 0x00, 0x80/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmsubasp =  { {0xF0, 0x00, 0x00, 0x88/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxmrghw =  { {0xF0, 0x00, 0x00, 0x90/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsdivsp =  { {0xF0, 0x00, 0x00, 0xC0/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmsubmsp =  { {0xF0, 0x00, 0x00, 0xC8/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsadddp =  { {0xF0, 0x00, 0x01, 0x00/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmaddadp =  { {0xF0, 0x00, 0x01, 0x08/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscmpudp =  { {0xF0, 0x00, 0x01, 0x18/*XQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvdpuxws =  { {0xF0, 0x00, 0x01, 0x20/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsrdpi =  { {0xF0, 0x00, 0x01, 0x24/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsrsqrtedp =  { {0xF0, 0x00, 0x01, 0x28/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xssqrtdp =  { {0xF0, 0x00, 0x01, 0x2C/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xssubdp =  { {0xF0, 0x00, 0x01, 0x40/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmaddmdp =  { {0xF0, 0x00, 0x01, 0x48/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscmpodp =  { {0xF0, 0x00, 0x01, 0x58/*XQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvdpsxws =  { {0xF0, 0x00, 0x01, 0x60/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsrdpiz =  { {0xF0, 0x00, 0x01, 0x64/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsredp =  { {0xF0, 0x00, 0x01, 0x68/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmuldp =  { {0xF0, 0x00, 0x01, 0x80/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmsubadp =  { {0xF0, 0x00, 0x01, 0x88/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxmrglw =  { {0xF0, 0x00, 0x01, 0x90/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsrdpip =  { {0xF0, 0x00, 0x01, 0xA4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xstsqrtdp =  { {0xF0, 0x00, 0x01, 0xA8/*X-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsrdpic =  { {0xF0, 0x00, 0x01, 0xAC/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsdivdp =  { {0xF0, 0x00, 0x01, 0xC0/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmsubmdp =  { {0xF0, 0x00, 0x01, 0xC8/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsrdpim =  { {0xF0, 0x00, 0x01, 0xE4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xstdivdp =  { {0xF0, 0x00, 0x01, 0xE8/*XQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvaddsp =  { {0xF0, 0x00, 0x02, 0x00/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmaddasp =  { {0xF0, 0x00, 0x02, 0x08/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpeqsp =  { {0xF0, 0x00, 0x02, 0x18/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvspuxws =  { {0xF0, 0x00, 0x02, 0x20/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrspi =  { {0xF0, 0x00, 0x02, 0x24/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrsqrtesp =  { {0xF0, 0x00, 0x02, 0x28/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvsqrtsp =  { {0xF0, 0x00, 0x02, 0x2C/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvsubsp =  { {0xF0, 0x00, 0x02, 0x40/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmaddmsp =  { {0xF0, 0x00, 0x02, 0x48/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpgtsp =  { {0xF0, 0x00, 0x02, 0x58/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvspsxws =  { {0xF0, 0x00, 0x02, 0x60/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrspiz =  { {0xF0, 0x00, 0x02, 0x64/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvresp =  { {0xF0, 0x00, 0x02, 0x68/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmulsp =  { {0xF0, 0x00, 0x02, 0x80/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmsubasp =  { {0xF0, 0x00, 0x02, 0x88/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxspltw =  { {0xF0, 0x00, 0x02, 0x90/*QQG~*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpgesp =  { {0xF0, 0x00, 0x02, 0x98/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvuxwsp =  { {0xF0, 0x00, 0x02, 0xA0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrspip =  { {0xF0, 0x00, 0x02, 0xA4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvtsqrtsp =  { {0xF0, 0x00, 0x02, 0xA8/*X-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrspic =  { {0xF0, 0x00, 0x02, 0xAC/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvdivsp =  { {0xF0, 0x00, 0x02, 0xC0/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmsubmsp =  { {0xF0, 0x00, 0x02, 0xC8/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvsxwsp =  { {0xF0, 0x00, 0x02, 0xE0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrspim =  { {0xF0, 0x00, 0x02, 0xE4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvtdivsp =  { {0xF0, 0x00, 0x02, 0xE8/*XQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvadddp =  { {0xF0, 0x00, 0x03, 0x00/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmaddadp =  { {0xF0, 0x00, 0x03, 0x08/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpeqdp =  { {0xF0, 0x00, 0x03, 0x18/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvdpuxws =  { {0xF0, 0x00, 0x03, 0x20/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrdpi =  { {0xF0, 0x00, 0x03, 0x24/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrsqrtedp =  { {0xF0, 0x00, 0x03, 0x28/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvsqrtdp =  { {0xF0, 0x00, 0x03, 0x2C/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvsubdp =  { {0xF0, 0x00, 0x03, 0x40/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmaddmdp =  { {0xF0, 0x00, 0x03, 0x48/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpgtdp =  { {0xF0, 0x00, 0x03, 0x58/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvdpsxws =  { {0xF0, 0x00, 0x03, 0x60/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrdpiz =  { {0xF0, 0x00, 0x03, 0x64/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvredp =  { {0xF0, 0x00, 0x03, 0x68/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmuldp =  { {0xF0, 0x00, 0x03, 0x80/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmsubadp =  { {0xF0, 0x00, 0x03, 0x88/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpgedp =  { {0xF0, 0x00, 0x03, 0x98/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvuxwdp =  { {0xF0, 0x00, 0x03, 0xA0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrdpip =  { {0xF0, 0x00, 0x03, 0xA4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvtsqrtdp =  { {0xF0, 0x00, 0x03, 0xA8/*X-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrdpic =  { {0xF0, 0x00, 0x03, 0xAC/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvdivdp =  { {0xF0, 0x00, 0x03, 0xC0/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmsubmdp =  { {0xF0, 0x00, 0x03, 0xC8/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvsxwdp =  { {0xF0, 0x00, 0x03, 0xE0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvrdpim =  { {0xF0, 0x00, 0x03, 0xE4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvtdivdp =  { {0xF0, 0x00, 0x03, 0xE8/*XQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnmaddasp =  { {0xF0, 0x00, 0x04, 0x08/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxland =  { {0xF0, 0x00, 0x04, 0x10/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvdpsp =  { {0xF0, 0x00, 0x04, 0x24/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvdpspn =  { {0xF0, 0x00, 0x04, 0x2C/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnmaddmsp =  { {0xF0, 0x00, 0x04, 0x48/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxlandc =  { {0xF0, 0x00, 0x04, 0x50/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsrsp =  { {0xF0, 0x00, 0x04, 0x64/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnmsubasp =  { {0xF0, 0x00, 0x04, 0x88/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxlor =  { {0xF0, 0x00, 0x04, 0x90/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvuxdsp =  { {0xF0, 0x00, 0x04, 0xA0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnmsubmsp =  { {0xF0, 0x00, 0x04, 0xC8/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxlxor =  { {0xF0, 0x00, 0x04, 0xD0/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvsxdsp =  { {0xF0, 0x00, 0x04, 0xE0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmaxdp =  { {0xF0, 0x00, 0x05, 0x00/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnmaddadp =  { {0xF0, 0x00, 0x05, 0x08/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxlnor =  { {0xF0, 0x00, 0x05, 0x10/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvdpuxds =  { {0xF0, 0x00, 0x05, 0x20/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvspdp =  { {0xF0, 0x00, 0x05, 0x24/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvspdpn =  { {0xF0, 0x00, 0x05, 0x2C/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsmindp =  { {0xF0, 0x00, 0x05, 0x40/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnmaddmdp =  { {0xF0, 0x00, 0x05, 0x48/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxlorc =  { {0xF0, 0x00, 0x05, 0x50/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvdpsxds =  { {0xF0, 0x00, 0x05, 0x60/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsabsdp =  { {0xF0, 0x00, 0x05, 0x64/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscpsgndp =  { {0xF0, 0x00, 0x05, 0x80/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnmsubadp =  { {0xF0, 0x00, 0x05, 0x88/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxlnand =  { {0xF0, 0x00, 0x05, 0x90/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvuxddp =  { {0xF0, 0x00, 0x05, 0xA0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnabsdp =  { {0xF0, 0x00, 0x05, 0xA4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnmsubmdp =  { {0xF0, 0x00, 0x05, 0xC8/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xxleqv =  { {0xF0, 0x00, 0x05, 0xD0/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xscvsxddp =  { {0xF0, 0x00, 0x05, 0xE0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xsnegdp =  { {0xF0, 0x00, 0x05, 0xE4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmaxsp =  { {0xF0, 0x00, 0x06, 0x00/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnmaddasp =  { {0xF0, 0x00, 0x06, 0x08/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpeqsp_ =  { {0xF0, 0x00, 0x06, 0x18/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvspuxds =  { {0xF0, 0x00, 0x06, 0x20/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvdpsp =  { {0xF0, 0x00, 0x06, 0x24/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvminsp =  { {0xF0, 0x00, 0x06, 0x40/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnmaddmsp =  { {0xF0, 0x00, 0x06, 0x48/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpgtsp_ =  { {0xF0, 0x00, 0x06, 0x58/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvspsxds =  { {0xF0, 0x00, 0x06, 0x60/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvabssp =  { {0xF0, 0x00, 0x06, 0x64/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcpsgnsp =  { {0xF0, 0x00, 0x06, 0x80/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnmsubasp =  { {0xF0, 0x00, 0x06, 0x88/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpgesp_ =  { {0xF0, 0x00, 0x06, 0x98/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvuxdsp =  { {0xF0, 0x00, 0x06, 0xA0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnabssp =  { {0xF0, 0x00, 0x06, 0xA4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnmsubmsp =  { {0xF0, 0x00, 0x06, 0xC8/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvsxdsp =  { {0xF0, 0x00, 0x06, 0xE0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnegsp =  { {0xF0, 0x00, 0x06, 0xE4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmaxdp =  { {0xF0, 0x00, 0x07, 0x00/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnmaddadp =  { {0xF0, 0x00, 0x07, 0x08/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpeqdp_ =  { {0xF0, 0x00, 0x07, 0x18/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvdpuxds =  { {0xF0, 0x00, 0x07, 0x20/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvspdp =  { {0xF0, 0x00, 0x07, 0x24/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvmindp =  { {0xF0, 0x00, 0x07, 0x40/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnmaddmdp =  { {0xF0, 0x00, 0x07, 0x48/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpgtdp_ =  { {0xF0, 0x00, 0x07, 0x58/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvdpsxds =  { {0xF0, 0x00, 0x07, 0x60/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvabsdp =  { {0xF0, 0x00, 0x07, 0x64/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcpsgndp =  { {0xF0, 0x00, 0x07, 0x80/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnmsubadp =  { {0xF0, 0x00, 0x07, 0x88/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcmpgedp_ =  { {0xF0, 0x00, 0x07, 0x98/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvuxddp =  { {0xF0, 0x00, 0x07, 0xA0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnabsdp =  { {0xF0, 0x00, 0x07, 0xA4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnmsubmdp =  { {0xF0, 0x00, 0x07, 0xC8/*QQQ*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvcvsxddp =  { {0xF0, 0x00, 0x07, 0xE0/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_xvnegdp =  { {0xF0, 0x00, 0x07, 0xE4/*Q-Q*/}, 0x00000 };
const PPC64Instr ppc64_instr_stfdp =  { {0xF4, 0x00, 0x00, 0x00/*F:D*/}, 0x00000 };
const PPC64Instr ppc64_instr_stq =  { {0xF8, 0x00, 0x00, 0x02/*R:D*/}, 0x00000 };
const PPC64Instr ppc64_instr_fdiv =  { {0xFC, 0x00, 0x00, 0x24/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fsub =  { {0xFC, 0x00, 0x00, 0x28/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fadd =  { {0xFC, 0x00, 0x00, 0x2A/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fsqrt =  { {0xFC, 0x00, 0x00, 0x2C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fsel =  { {0xFC, 0x00, 0x00, 0x2E/*FFFF~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fre =  { {0xFC, 0x00, 0x00, 0x30/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fmul =  { {0xFC, 0x00, 0x00, 0x32/*FF-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_frsqrte =  { {0xFC, 0x00, 0x00, 0x34/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fmsub =  { {0xFC, 0x00, 0x00, 0x38/*FFFF~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fmadd =  { {0xFC, 0x00, 0x00, 0x3A/*FFFF~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fnmsub =  { {0xFC, 0x00, 0x00, 0x3C/*FFFF~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fnmadd =  { {0xFC, 0x00, 0x00, 0x3E/*FFFF~.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fcmpu =  { {0xFC, 0x00, 0x00, 0x00/*XFF*/}, 0x00000 };
const PPC64Instr ppc64_instr_fcpsgn =  { {0xFC, 0x00, 0x00, 0x10/*FFF.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fcmpo =  { {0xFC, 0x00, 0x00, 0x40/*XFF*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtfsb1 =  { {0xFC, 0x00, 0x00, 0x4C/*A*/}, 0x00000 };
const PPC64Instr ppc64_instr_fneg =  { {0xFC, 0x00, 0x00, 0x50/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mcrfs =  { {0xFC, 0x00, 0x00, 0x80/*XX*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtfsb0 =  { {0xFC, 0x00, 0x00, 0x8C/*A*/}, 0x00000 };
const PPC64Instr ppc64_instr_fmr =  { {0xFC, 0x00, 0x00, 0x90/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_frsp =  { {0xFC, 0x00, 0x00, 0x18/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fctiw =  { {0xFC, 0x00, 0x00, 0x1C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fctiwz =  { {0xFC, 0x00, 0x00, 0x1E/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ftdiv =  { {0xFC, 0x00, 0x01, 0x00/*X-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fctiwu =  { {0xFC, 0x00, 0x01, 0x1C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fctiwuz =  { {0xFC, 0x00, 0x01, 0x1E/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mtfsfi =  { {0xFC, 0x00, 0x01, 0x0C/*AA*/}, 0x00000 };
const PPC64Instr ppc64_instr_fnabs =  { {0xFC, 0x00, 0x01, 0x10/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ftsqrt =  { {0xFC, 0x00, 0x01, 0x40/*X-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fabs =  { {0xFC, 0x00, 0x02, 0x10/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_frin =  { {0xFC, 0x00, 0x03, 0x10/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_friz =  { {0xFC, 0x00, 0x03, 0x50/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_frip =  { {0xFC, 0x00, 0x03, 0x90/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_frim =  { {0xFC, 0x00, 0x03, 0xD0/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_mffs =  { {0xFC, 0x00, 0x04, 0x8E/*F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fctid =  { {0xFC, 0x00, 0x06, 0x5C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fctidz =  { {0xFC, 0x00, 0x06, 0x5E/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fmrgow =  { {0xFC, 0x00, 0x06, 0x8C/*FFF*/}, 0x00000 };
const PPC64Instr ppc64_instr_fcfid =  { {0xFC, 0x00, 0x06, 0x9C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fctidu =  { {0xFC, 0x00, 0x07, 0x5C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fctiduz =  { {0xFC, 0x00, 0x07, 0x5E/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_fmrgew =  { {0xFC, 0x00, 0x07, 0x8C/*FFF*/}, 0x00000 };
const PPC64Instr ppc64_instr_fcfidu =  { {0xFC, 0x00, 0x07, 0x9C/*F-F.*/}, 0x00000 };
const PPC64Instr ppc64_instr_daddq =  { {0xFC, 0x00, 0x00, 0x04/*F:F:F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dquaq =  { {0xFC, 0x00, 0x00, 0x06/*F:F:F:Z.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dmulq =  { {0xFC, 0x00, 0x00, 0x44/*F:F:F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_drrndq =  { {0xFC, 0x00, 0x00, 0x46/*F:F:F:Z.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dscliq =  { {0xFC, 0x00, 0x00, 0x84/*F:F:6.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dquaiq =  { {0xFC, 0x00, 0x00, 0x86/*SF:~F:Z.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dscriq =  { {0xFC, 0x00, 0x00, 0xC4/*F:F:6.*/}, 0x00000 };
const PPC64Instr ppc64_instr_drintxq =  { {0xFC, 0x00, 0x00, 0xC6/*1F:~F:Z.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcmpoq =  { {0xFC, 0x00, 0x01, 0x04/*XF:F:*/}, 0x00000 };
const PPC64Instr ppc64_instr_dtstexq =  { {0xFC, 0x00, 0x01, 0x44/*XF:F:*/}, 0x00000 };
const PPC64Instr ppc64_instr_dtstdcq =  { {0xFC, 0x00, 0x01, 0x84/*XF:6*/}, 0x00000 };
const PPC64Instr ppc64_instr_dtstdgq =  { {0xFC, 0x00, 0x01, 0xC4/*XF:6*/}, 0x00000 };
const PPC64Instr ppc64_instr_drintnq =  { {0xFC, 0x00, 0x01, 0xC6/*1F:~F:Z.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dctqpq =  { {0xFC, 0x00, 0x02, 0x04/*F:-F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dctfixq =  { {0xFC, 0x00, 0x02, 0x44/*F:-F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ddedpdq =  { {0xFC, 0x00, 0x02, 0x84/*ZF:~F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dxexq =  { {0xFC, 0x00, 0x02, 0xC4/*F:-F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dsubq =  { {0xFC, 0x00, 0x04, 0x04/*F:F:F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_ddivq =  { {0xFC, 0x00, 0x04, 0x44/*F:F:F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcmpuq =  { {0xFC, 0x00, 0x05, 0x04/*XF:F:*/}, 0x00000 };
const PPC64Instr ppc64_instr_dtstsfq =  { {0xFC, 0x00, 0x05, 0x44/*XF:F:*/}, 0x00000 };
const PPC64Instr ppc64_instr_drdpq =  { {0xFC, 0x00, 0x06, 0x04/*F:-F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_dcffixq =  { {0xFC, 0x00, 0x06, 0x44/*F:-F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_denbcdq =  { {0xFC, 0x00, 0x06, 0x84/*YF:~F:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_diexq =  { {0xFC, 0x00, 0x06, 0xC4/*F:FF:.*/}, 0x00000 };
const PPC64Instr ppc64_instr_evaddw =  { {0x10, 0x00, 0x02, 0x00/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evaddiw =  { {0x10, 0x00, 0x02, 0x02/*RAR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsubw =  { {0x10, 0x00, 0x02, 0x04/*RRR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsubiw =  { {0x10, 0x00, 0x02, 0x06/*RAR~*/}, 0x00000 };
const PPC64Instr ppc64_instr_evabs =  { {0x10, 0x00, 0x02, 0x08/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evneg =  { {0x10, 0x00, 0x02, 0x09/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evextsb =  { {0x10, 0x00, 0x02, 0x0A/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evextsh =  { {0x10, 0x00, 0x02, 0x0B/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evrndw =  { {0x10, 0x00, 0x02, 0x0C/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcntlzw =  { {0x10, 0x00, 0x02, 0x0D/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcntlsw =  { {0x10, 0x00, 0x02, 0x0E/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_brinc =  { {0x10, 0x00, 0x02, 0x0F/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evand =  { {0x10, 0x00, 0x02, 0x11/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evandc =  { {0x10, 0x00, 0x02, 0x12/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evxor =  { {0x10, 0x00, 0x02, 0x16/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evor =  { {0x10, 0x00, 0x02, 0x17/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmr =  { {0x10, 0x00, 0x02, 0x17/*RR=*/}, 0x00000 };
const PPC64Instr ppc64_instr_evnor =  { {0x10, 0x00, 0x02, 0x18/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evnot =  { {0x10, 0x00, 0x02, 0x18/*RR=*/}, 0x00000 };
const PPC64Instr ppc64_instr_eveqv =  { {0x10, 0x00, 0x02, 0x19/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evorc =  { {0x10, 0x00, 0x02, 0x1B/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evnand =  { {0x10, 0x00, 0x02, 0x1E/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsrwu =  { {0x10, 0x00, 0x02, 0x20/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsrws =  { {0x10, 0x00, 0x02, 0x21/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsrwiu =  { {0x10, 0x00, 0x02, 0x22/*RRA*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsrwis =  { {0x10, 0x00, 0x02, 0x23/*RRA*/}, 0x00000 };
const PPC64Instr ppc64_instr_evslw =  { {0x10, 0x00, 0x02, 0x24/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evslwi =  { {0x10, 0x00, 0x02, 0x26/*RRA*/}, 0x00000 };
const PPC64Instr ppc64_instr_evrlw =  { {0x10, 0x00, 0x02, 0x28/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsplati =  { {0x10, 0x00, 0x02, 0x29/*RS*/}, 0x00000 };
const PPC64Instr ppc64_instr_evrlwi =  { {0x10, 0x00, 0x02, 0x2A/*RRA*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsplatfi =  { {0x10, 0x00, 0x02, 0x2B/*RS*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmergehi =  { {0x10, 0x00, 0x02, 0x2C/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmergelo =  { {0x10, 0x00, 0x02, 0x2D/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmpgtu =  { {0x10, 0x00, 0x02, 0x30/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmpgtu_ =  { {0x10, 0x00, 0x02, 0x30/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmpgts =  { {0x10, 0x00, 0x02, 0x31/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmpgts_ =  { {0x10, 0x00, 0x02, 0x31/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmpltu =  { {0x10, 0x00, 0x02, 0x32/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmpltu_ =  { {0x10, 0x00, 0x02, 0x32/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmplts =  { {0x10, 0x00, 0x02, 0x33/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmplts_ =  { {0x10, 0x00, 0x02, 0x33/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmpeq =  { {0x10, 0x00, 0x02, 0x34/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evcmpeq_ =  { {0x10, 0x00, 0x02, 0x34/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsel =  { {0x10, 0x00, 0x02, 0x78/*RRRW*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsel_ =  { {0x10, 0x00, 0x02, 0x78/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsadd =  { {0x10, 0x00, 0x02, 0x80/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfssub =  { {0x10, 0x00, 0x02, 0x81/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsabs =  { {0x10, 0x00, 0x02, 0x84/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsnabs =  { {0x10, 0x00, 0x02, 0x85/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsneg =  { {0x10, 0x00, 0x02, 0x86/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsmul =  { {0x10, 0x00, 0x02, 0x88/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsdiv =  { {0x10, 0x00, 0x02, 0x89/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscmpgt =  { {0x10, 0x00, 0x02, 0x8C/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscmpgt_ =  { {0x10, 0x00, 0x02, 0x8C/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscmplt =  { {0x10, 0x00, 0x02, 0x8D/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscmplt_ =  { {0x10, 0x00, 0x02, 0x8D/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscmpeq =  { {0x10, 0x00, 0x02, 0x8E/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscmpeq_ =  { {0x10, 0x00, 0x02, 0x8E/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscfui =  { {0x10, 0x00, 0x02, 0x90/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscfsi =  { {0x10, 0x00, 0x02, 0x91/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscfuf =  { {0x10, 0x00, 0x02, 0x92/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfscfsf =  { {0x10, 0x00, 0x02, 0x93/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsctui =  { {0x10, 0x00, 0x02, 0x94/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsctsi =  { {0x10, 0x00, 0x02, 0x95/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsctuf =  { {0x10, 0x00, 0x02, 0x96/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsctsf =  { {0x10, 0x00, 0x02, 0x97/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsctuiz =  { {0x10, 0x00, 0x02, 0x98/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfsctsiz_ =  { {0x10, 0x00, 0x02, 0x9A/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfststgt =  { {0x10, 0x00, 0x02, 0x9C/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfststgt_ =  { {0x10, 0x00, 0x02, 0x9C/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfststlt =  { {0x10, 0x00, 0x02, 0x9D/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfststlt_ =  { {0x10, 0x00, 0x02, 0x9D/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfststeq =  { {0x10, 0x00, 0x02, 0x9E/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evfststeq_ =  { {0x10, 0x00, 0x02, 0x9E/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsadd =  { {0x10, 0x00, 0x02, 0xC0/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efssub =  { {0x10, 0x00, 0x02, 0xC1/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsabs =  { {0x10, 0x00, 0x02, 0xC4/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsnabs =  { {0x10, 0x00, 0x02, 0xC5/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsneg =  { {0x10, 0x00, 0x02, 0xC6/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsmul =  { {0x10, 0x00, 0x02, 0xC8/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsdiv =  { {0x10, 0x00, 0x02, 0xC9/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscmpgt =  { {0x10, 0x00, 0x02, 0xCC/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscmpgt_ =  { {0x10, 0x00, 0x02, 0xCC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscmplt =  { {0x10, 0x00, 0x02, 0xCD/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscmplt_ =  { {0x10, 0x00, 0x02, 0xCD/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscmpeq =  { {0x10, 0x00, 0x02, 0xCE/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscmpeq_ =  { {0x10, 0x00, 0x02, 0xCE/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscfd =  { {0x10, 0x00, 0x02, 0xCF/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscfui =  { {0x10, 0x00, 0x02, 0xD0/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscfsi =  { {0x10, 0x00, 0x02, 0xD1/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscfuf =  { {0x10, 0x00, 0x02, 0xD2/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efscfsf =  { {0x10, 0x00, 0x02, 0xD3/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsctui =  { {0x10, 0x00, 0x02, 0xD4/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsctsi =  { {0x10, 0x00, 0x02, 0xD5/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsctuf =  { {0x10, 0x00, 0x02, 0xD6/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsctsf =  { {0x10, 0x00, 0x02, 0xD7/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsctuiz =  { {0x10, 0x00, 0x02, 0xD8/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efsctsiz =  { {0x10, 0x00, 0x02, 0xDA/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efststgt =  { {0x10, 0x00, 0x02, 0xDC/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efststgt_ =  { {0x10, 0x00, 0x02, 0xDC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efststlt =  { {0x10, 0x00, 0x02, 0xDD/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efststlt_ =  { {0x10, 0x00, 0x02, 0xDD/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efststeq =  { {0x10, 0x00, 0x02, 0xDE/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efststeq_ =  { {0x10, 0x00, 0x02, 0xDE/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdadd =  { {0x10, 0x00, 0x02, 0xE0/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdsub =  { {0x10, 0x00, 0x02, 0xE1/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcfuid =  { {0x10, 0x00, 0x02, 0xE2/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcfsid =  { {0x10, 0x00, 0x02, 0xE3/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdabs =  { {0x10, 0x00, 0x02, 0xE4/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdnabs =  { {0x10, 0x00, 0x02, 0xE5/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdneg =  { {0x10, 0x00, 0x02, 0xE6/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdmul =  { {0x10, 0x00, 0x02, 0xE8/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efddiv =  { {0x10, 0x00, 0x02, 0xE9/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdctuidz =  { {0x10, 0x00, 0x02, 0xEA/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdctsidz_ =  { {0x10, 0x00, 0x02, 0xEB/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcmpgt =  { {0x10, 0x00, 0x02, 0xEC/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcmpgt_ =  { {0x10, 0x00, 0x02, 0xEC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcmplt =  { {0x10, 0x00, 0x02, 0xED/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcmplt_ =  { {0x10, 0x00, 0x02, 0xED/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcmpeq =  { {0x10, 0x00, 0x02, 0xEE/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcmpeq_ =  { {0x10, 0x00, 0x02, 0xEE/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcfs =  { {0x10, 0x00, 0x02, 0xEF/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcfui =  { {0x10, 0x00, 0x02, 0xF0/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcfsi =  { {0x10, 0x00, 0x02, 0xF1/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcfuf =  { {0x10, 0x00, 0x02, 0xF2/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdcfsf =  { {0x10, 0x00, 0x02, 0xF3/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdctui =  { {0x10, 0x00, 0x02, 0xF4/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdctsi =  { {0x10, 0x00, 0x02, 0xF5/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdctuf =  { {0x10, 0x00, 0x02, 0xF6/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdctsf =  { {0x10, 0x00, 0x02, 0xF7/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdctuiz =  { {0x10, 0x00, 0x02, 0xF8/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdctsiz =  { {0x10, 0x00, 0x02, 0xFA/*R-R*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdtstgt =  { {0x10, 0x00, 0x02, 0xFC/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdtstgt_ =  { {0x10, 0x00, 0x02, 0xFC/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdtstlt =  { {0x10, 0x00, 0x02, 0xFD/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdtstlt_ =  { {0x10, 0x00, 0x02, 0xFD/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdtsteq =  { {0x10, 0x00, 0x02, 0xFE/*XRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_efdtsteq_ =  { {0x10, 0x00, 0x02, 0xFE/*-RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlddx =  { {0x10, 0x00, 0x03, 0x00/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evldd =  { {0x10, 0x00, 0x03, 0x01/*R8*/}, 0x00000 };
const PPC64Instr ppc64_instr_evldwx =  { {0x10, 0x00, 0x03, 0x02/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evldw =  { {0x10, 0x00, 0x03, 0x03/*R8*/}, 0x00000 };
const PPC64Instr ppc64_instr_evldhx =  { {0x10, 0x00, 0x03, 0x04/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evldh =  { {0x10, 0x00, 0x03, 0x05/*R8*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwhex =  { {0x10, 0x00, 0x03, 0x10/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwhe =  { {0x10, 0x00, 0x03, 0x11/*R4*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwhoux =  { {0x10, 0x00, 0x03, 0x14/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwhou =  { {0x10, 0x00, 0x03, 0x15/*R4*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwhosx =  { {0x10, 0x00, 0x03, 0x16/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwhos =  { {0x10, 0x00, 0x03, 0x17/*R4*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstddx =  { {0x10, 0x00, 0x03, 0x20/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstdd =  { {0x10, 0x00, 0x03, 0x21/*R8*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstdwx =  { {0x10, 0x00, 0x03, 0x22/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstdw =  { {0x10, 0x00, 0x03, 0x23/*R8*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstdhx =  { {0x10, 0x00, 0x03, 0x24/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstdh =  { {0x10, 0x00, 0x03, 0x25/*R8*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstwhex =  { {0x10, 0x00, 0x03, 0x30/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstwhe =  { {0x10, 0x00, 0x03, 0x31/*R4*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstwhox =  { {0x10, 0x00, 0x03, 0x34/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstwho =  { {0x10, 0x00, 0x03, 0x35/*R4*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstwwex =  { {0x10, 0x00, 0x03, 0x38/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstwwe =  { {0x10, 0x00, 0x03, 0x39/*R4*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstwwox =  { {0x10, 0x00, 0x03, 0x3C/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evstwwo =  { {0x10, 0x00, 0x03, 0x3D/*R4*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhessf =  { {0x10, 0x00, 0x04, 0x03/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhossf =  { {0x10, 0x00, 0x04, 0x07/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmheumi =  { {0x10, 0x00, 0x04, 0x08/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhesmi =  { {0x10, 0x00, 0x04, 0x09/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhesmf =  { {0x10, 0x00, 0x04, 0x0B/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhoumi =  { {0x10, 0x00, 0x04, 0x0C/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhosmi =  { {0x10, 0x00, 0x04, 0x0D/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhosmf =  { {0x10, 0x00, 0x04, 0x0F/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhessfa =  { {0x10, 0x00, 0x04, 0x23/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhossfa =  { {0x10, 0x00, 0x04, 0x27/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmheumia =  { {0x10, 0x00, 0x04, 0x28/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhesmia =  { {0x10, 0x00, 0x04, 0x29/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhesmfa =  { {0x10, 0x00, 0x04, 0x2B/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhoumia =  { {0x10, 0x00, 0x04, 0x2C/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhosmia =  { {0x10, 0x00, 0x04, 0x2D/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhosmfa =  { {0x10, 0x00, 0x04, 0x2F/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwhssf =  { {0x10, 0x00, 0x04, 0x47/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlumi =  { {0x10, 0x00, 0x04, 0x48/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwhumi =  { {0x10, 0x00, 0x04, 0x4C/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwhsmi =  { {0x10, 0x00, 0x04, 0x4D/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwhsmf =  { {0x10, 0x00, 0x04, 0x4F/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwssf =  { {0x10, 0x00, 0x04, 0x53/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwumi =  { {0x10, 0x00, 0x04, 0x58/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwsmi =  { {0x10, 0x00, 0x04, 0x59/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwsmf =  { {0x10, 0x00, 0x04, 0x5B/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwhssfa =  { {0x10, 0x00, 0x04, 0x67/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlumia =  { {0x10, 0x00, 0x04, 0x68/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwhumia =  { {0x10, 0x00, 0x04, 0x6C/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwhsmia =  { {0x10, 0x00, 0x04, 0x6D/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwhsmfa =  { {0x10, 0x00, 0x04, 0x6F/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwssfa =  { {0x10, 0x00, 0x04, 0x73/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwumia =  { {0x10, 0x00, 0x04, 0x78/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwsmia =  { {0x10, 0x00, 0x04, 0x79/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwsmfa =  { {0x10, 0x00, 0x04, 0x7B/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmra =  { {0x10, 0x00, 0x04, 0xC4/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evdivws =  { {0x10, 0x00, 0x04, 0xC6/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evdivwu =  { {0x10, 0x00, 0x04, 0xC7/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwssfaa =  { {0x10, 0x00, 0x05, 0x53/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwumiaa =  { {0x10, 0x00, 0x05, 0x58/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwsmiaa =  { {0x10, 0x00, 0x05, 0x59/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwsmfaa =  { {0x10, 0x00, 0x05, 0x5B/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwssfan =  { {0x10, 0x00, 0x05, 0xD3/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwumian =  { {0x10, 0x00, 0x05, 0xD8/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwsmian =  { {0x10, 0x00, 0x05, 0xD9/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwsmfan =  { {0x10, 0x00, 0x05, 0xDB/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmergehilo =  { {0x10, 0x00, 0x02, 0x2E/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmergelohi =  { {0x10, 0x00, 0x02, 0x2F/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlhhesplatx =  { {0x10, 0x00, 0x03, 0x08/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlhhesplat =  { {0x10, 0x00, 0x03, 0x09/*R2*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlhhousplatx =  { {0x10, 0x00, 0x03, 0x0C/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlhhousplat =  { {0x10, 0x00, 0x03, 0x0D/*R2*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlhhossplatx =  { {0x10, 0x00, 0x03, 0x0E/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlhhossplat =  { {0x10, 0x00, 0x03, 0x0F/*R2*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwwsplatx =  { {0x10, 0x00, 0x03, 0x18/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwwsplat =  { {0x10, 0x00, 0x03, 0x19/*R4*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwhsplatx =  { {0x10, 0x00, 0x03, 0x1C/*RR0R*/}, 0x00000 };
const PPC64Instr ppc64_instr_evlwhsplat =  { {0x10, 0x00, 0x03, 0x1D/*R4*/}, 0x00000 };
const PPC64Instr ppc64_instr_evaddusiaaw =  { {0x10, 0x00, 0x04, 0xC0/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evaddssiaaw =  { {0x10, 0x00, 0x04, 0xC1/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsubfusiaaw =  { {0x10, 0x00, 0x04, 0xC2/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsubfssiaaw =  { {0x10, 0x00, 0x04, 0xC3/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evaddumiaaw =  { {0x10, 0x00, 0x04, 0xC8/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evaddsmiaaw =  { {0x10, 0x00, 0x04, 0xC9/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsubfumiaaw =  { {0x10, 0x00, 0x04, 0xCA/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evsubfsmiaaw =  { {0x10, 0x00, 0x04, 0xCB/*RR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmheusiaaw =  { {0x10, 0x00, 0x05, 0x00/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhessiaaw =  { {0x10, 0x00, 0x05, 0x01/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhessfaaw =  { {0x10, 0x00, 0x05, 0x03/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhousiaaw =  { {0x10, 0x00, 0x05, 0x04/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhossiaaw =  { {0x10, 0x00, 0x05, 0x05/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhossfaaw =  { {0x10, 0x00, 0x05, 0x07/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmheumiaaw =  { {0x10, 0x00, 0x05, 0x08/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhesmiaaw =  { {0x10, 0x00, 0x05, 0x09/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhesmfaaw =  { {0x10, 0x00, 0x05, 0x0B/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhoumiaaw =  { {0x10, 0x00, 0x05, 0x0C/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhosmiaaw =  { {0x10, 0x00, 0x05, 0x0D/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhosmfaaw =  { {0x10, 0x00, 0x05, 0x0F/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhegumiaa =  { {0x10, 0x00, 0x05, 0x28/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhegsmiaa =  { {0x10, 0x00, 0x05, 0x29/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhegsmfaa =  { {0x10, 0x00, 0x05, 0x2B/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhogumiaa =  { {0x10, 0x00, 0x05, 0x2C/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhogsmiaa =  { {0x10, 0x00, 0x05, 0x2D/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhogsmfaa =  { {0x10, 0x00, 0x05, 0x2F/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlusiaaw =  { {0x10, 0x00, 0x05, 0x40/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlssiaaw =  { {0x10, 0x00, 0x05, 0x41/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlumiaaw =  { {0x10, 0x00, 0x05, 0x48/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlsmiaaw =  { {0x10, 0x00, 0x05, 0x49/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmheusianw =  { {0x10, 0x00, 0x05, 0x80/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhessianw =  { {0x10, 0x00, 0x05, 0x81/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhessfanw =  { {0x10, 0x00, 0x05, 0x83/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhousianw =  { {0x10, 0x00, 0x05, 0x84/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhossianw =  { {0x10, 0x00, 0x05, 0x85/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhossfanw =  { {0x10, 0x00, 0x05, 0x87/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmheumianw =  { {0x10, 0x00, 0x05, 0x88/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhesmianw =  { {0x10, 0x00, 0x05, 0x89/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhesmfanw =  { {0x10, 0x00, 0x05, 0x8B/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhoumianw =  { {0x10, 0x00, 0x05, 0x8C/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhosmianw =  { {0x10, 0x00, 0x05, 0x8D/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhosmfanw =  { {0x10, 0x00, 0x05, 0x8F/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhegumian =  { {0x10, 0x00, 0x05, 0xA8/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhegsmian =  { {0x10, 0x00, 0x05, 0xA9/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhegsmfan =  { {0x10, 0x00, 0x05, 0xAB/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhogumian =  { {0x10, 0x00, 0x05, 0xAC/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhogsmian =  { {0x10, 0x00, 0x05, 0xAD/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmhogsmfan =  { {0x10, 0x00, 0x05, 0xAF/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlusianw =  { {0x10, 0x00, 0x05, 0xC0/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlssianw =  { {0x10, 0x00, 0x05, 0xC1/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlumianw =  { {0x10, 0x00, 0x05, 0xC8/*RRR*/}, 0x00000 };
const PPC64Instr ppc64_instr_evmwlsmianw =  { {0x10, 0x00, 0x05, 0xC9/*RRR*/}, 0x00000 };




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

enum ConditionCode {
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
};

// names of condition codes, indexable by the ConditionCode enum value.
extern const char* cc_names[];

inline ConditionCode ccNegate(ConditionCode c) {
  return ConditionCode(int(c) ^ 1); // And you thought x86 was irregular!
}

///////////////////////////////////////////////////////////////////////////////

struct Label;

/**
 * Copyright (c) 2009, Andrew J. Paroski
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * The names of the contributors may not be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL ANDREW J. PAROSKI BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

class X64Assembler : private boost::noncopyable {
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
  explicit X64Assembler(CodeBlock& cb) : codeBlock(cb) {}

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

  /*
   * The following section defines the main interface for emitting
   * x64.
   *
   * Simple Examples:
   *
   *   a.  movq   (rax, rbx);       // order is AT&T: src, dest
   *   a.  loadq  (*rax, rbx);      // loads from *rax
   *   a.  loadq  (rax[0], rbx);    // also loads from *rax
   *   a.  storeq (rcx, rax[0xc]);  // store to rax + 0xc
   *   a.  addq   (0x1, rbx);       // increment rbx
   *
   * Addressing with index registers:
   *
   *   a.  movl   (index, ecx);
   *   a.  loadq  (*rax, rbx);
   *   a.  storeq (rbx, rbx[rcx*8]);
   *   a.  call   (rax);            // indirect call
   *
   */

#define BYTE_LOAD_OP(name, instr)                                     \
  void name##b(MemoryRef m, Reg8 r)        { instrMR(instr, m, r); }  \

#define LOAD_OP(name, instr)                                          \
  void name##q(MemoryRef m, Reg64 r) { instrMR(instr, m, r); }        \
  void name##l(MemoryRef m, Reg32 r) { instrMR(instr, m, r); }        \
  void name##w(MemoryRef m, Reg16 r) { instrMR(instr, m, r); }        \
  void name##q(RIPRelativeRef m, Reg64 r) { instrMR(instr, m, r); } \
  BYTE_LOAD_OP(name, instr##b)

#define BYTE_STORE_OP(name, instr)                                    \
  void name##b(Reg8 r, MemoryRef m)        { instrRM(instr, r, m); }  \
  void name##b(Immed i, MemoryRef m)       { instrIM8(instr, i, m); } \

#define STORE_OP(name, instr)                                           \
  void name##w(Immed i, MemoryRef m) { instrIM16(instr, i, m); }        \
  void name##l(Immed i, MemoryRef m) { instrIM32(instr, i, m); }        \
  void name##w(Reg16 r, MemoryRef m) { instrRM(instr, r, m); }          \
  void name##l(Reg32 r, MemoryRef m) { instrRM(instr, r, m); }          \
  void name##q(Reg64 r, MemoryRef m) { instrRM(instr, r, m); }          \
  BYTE_STORE_OP(name, instr ## b)

#define BYTE_REG_OP(name, instr)                              \
  void name##b(Reg8 r1, Reg8 r2) { instrRR(instr, r1, r2); }  \
  void name##b(Immed i, Reg8 r)  { instrIR(instr, i, r); }    \

#define REG_OP(name, instr)                                       \
  void name##q(Reg64 r1, Reg64 r2)   { instrRR(instr, r1, r2); }  \
  void name##l(Reg32 r1, Reg32 r2)   { instrRR(instr, r1, r2); }  \
  void name##w(Reg16 r1, Reg16 r2)   { instrRR(instr, r1, r2); }  \
  void name##l(Immed i, Reg32 r)     { instrIR(instr, i, r); }    \
  void name##w(Immed i, Reg16 r)     { instrIR(instr, i, r); }    \
  BYTE_REG_OP(name, instr##b)

  /*
   * For when we a have a memory operand and the operand size is
   * 64-bits, only a 32-bit (sign-extended) immediate is supported.
   */
#define IMM64_STORE_OP(name, instr)             \
  void name##q(Immed i, MemoryRef m) {          \
    return instrIM(instr, i, m);                \
  }

  /*
   * For instructions other than movq, even when the operand size is
   * 64 bits only a 32-bit (sign-extended) immediate is supported.
   */
#define IMM64R_OP(name, instr)                  \
  void name##q(Immed imm, Reg64 r) {            \
    always_assert(imm.fits(sz::dword));         \
    return instrIR(instr, imm, r);              \
  }

#define FULL_OP(name, instr)                    \
  LOAD_OP(name, instr)                          \
  STORE_OP(name, instr)                         \
  REG_OP(name, instr)                           \
  IMM64_STORE_OP(name, instr)                   \
  IMM64R_OP(name, instr)

  // We rename x64's mov to store and load for improved code
  // readability.
  LOAD_OP        (load,  instr_mov)
  STORE_OP       (store, instr_mov)
  IMM64_STORE_OP (store, instr_mov)
  REG_OP         (mov,   instr_mov)

  FULL_OP(add, instr_add)
  FULL_OP(xor, instr_xor)
  FULL_OP(sub, instr_sub)
  FULL_OP(and, instr_and)
  FULL_OP(or,  instr_or)
  FULL_OP(test,instr_test)
  FULL_OP(cmp, instr_cmp)
  FULL_OP(sbb, instr_sbb)

#undef IMM64_OP
#undef IMM64R_OP
#undef FULL_OP
#undef REG_OP
#undef STORE_OP
#undef LOAD_OP
#undef BYTE_LOAD_OP
#undef BYTE_STORE_OP
#undef BYTE_REG_OP

  // 64-bit immediates work with mov to a register.
  void movq(Immed64 imm, Reg64 r) { instrIR(instr_mov, imm, r); }

  // movzbx is a special snowflake. We don't have movzbq because it behaves
  // exactly the same as movzbl but takes an extra byte.
  void loadzbl(MemoryRef m, Reg32 r)        { instrMR(instr_movzbx,
                                                      m, rbyte(r)); }
  void movzbl(Reg8 src, Reg32 dest)         { emitRR32(instr_movzbx,
                                                       rn(src), rn(dest)); }
  void movsbl(Reg8 src, Reg32 dest)         { emitRR(instr_movsbx,
                                                       rn(src), rn(dest)); }

  void loadsbq(MemoryRef m, Reg64 r)        { instrMR(instr_movsbx,
                                                      m, r); }
  void movsbq(Reg8 src, Reg64 dest)         { emitRR(instr_movsbx,
                                                       rn(src), rn(dest)); }

  void lea(MemoryRef p, Reg64 reg)        { instrMR(instr_lea, p, reg); }
  void lea(RIPRelativeRef p, Reg64 reg)   { instrMR(instr_lea, p, reg); }

  void xchgq(Reg64 r1, Reg64 r2) { instrRR(instr_xchg, r1, r2); }
  void xchgl(Reg32 r1, Reg32 r2) { instrRR(instr_xchg, r1, r2); }
  void xchgb(Reg8 r1, Reg8 r2)   { instrRR(instr_xchgb, r1, r2); }

  void imul(Reg64 r1, Reg64 r2)  { instrRR(instr_imul, r1, r2); }

  void push(Reg64 r)  { instrR(instr_push, r); }
  void pushl(Reg32 r) { instrR(instr_push, r); }
  void pop (Reg64 r)  { instrR(instr_pop,  r); }
  void idiv(Reg64 r)  { instrR(instr_idiv, r); }
  void incq(Reg64 r)  { instrR(instr_inc,  r); }
  void incl(Reg32 r)  { instrR(instr_inc,  r); }
  void incw(Reg16 r)  { instrR(instr_inc,  r); }
  void decq(Reg64 r)  { instrR(instr_dec,  r); }
  void decl(Reg32 r)  { instrR(instr_dec,  r); }
  void decw(Reg16 r)  { instrR(instr_dec,  r); }
  void notb(Reg8 r)   { instrR(instr_notb, r); }
  void not(Reg64 r)   { instrR(instr_not,  r); }
  void neg(Reg64 r)   { instrR(instr_neg,  r); }
  void negb(Reg8 r)   { instrR(instr_negb, r); }
  void ret()          { emit(instr_ret); }
  void ret(Immed i)   { emitI(instr_ret, i.w(), sz::word); }
  void cqo()          { emit(instr_cqo); }
  void nop()          { emit(instr_nop); }
  void int3()         { emit(instr_int3); }
  void ud2()          { byte(0x0f); byte(0x0b); }
  void pushf()        { byte(0x9c); }
  void popf()         { byte(0x9d); }
  void lock()         { byte(0xF0); }

  void push(MemoryRef m) { instrM(instr_push, m); }
  void pop (MemoryRef m) { instrM(instr_pop,  m); }
  void incq(MemoryRef m) { instrM(instr_inc,  m); }
  void incl(MemoryRef m) { instrM32(instr_inc, m); }
  void incw(MemoryRef m) { instrM16(instr_inc, m); }
  void decq(MemoryRef m) { instrM(instr_dec,  m); }
  void decl(MemoryRef m) { instrM32(instr_dec, m); }
  void decw(MemoryRef m) { instrM16(instr_dec, m); }

  void movups(RegXMM x, MemoryRef m)        { instrRM(instr_movups, x, m); }
  void movups(MemoryRef m, RegXMM x)        { instrMR(instr_movups, m, x); }
  void movdqu(RegXMM x, MemoryRef m)        { instrRM(instr_movdqu, x, m); }
  void movdqu(MemoryRef m, RegXMM x)        { instrMR(instr_movdqu, m, x); }
  void movdqa(RegXMM x, RegXMM y)           { instrRR(instr_movdqa, x, y); }
  void movdqa(RegXMM x, MemoryRef m)        { instrRM(instr_movdqa, x, m); }
  void movdqa(MemoryRef m, RegXMM x)        { instrMR(instr_movdqa, m, x); }
  void movsd (RegXMM x, RegXMM y)           { instrRR(instr_movsd,  x, y); }
  void movsd (RegXMM x, MemoryRef m)        { instrRM(instr_movsd,  x, m); }
  void movsd (MemoryRef m, RegXMM x)        { instrMR(instr_movsd,  m, x); }
  void movsd (RIPRelativeRef m, RegXMM x)   { instrMR(instr_movsd,  m, x); }
  void lddqu (MemoryRef m, RegXMM x)        { instrMR(instr_lddqu, m, x); }
  void unpcklpd(RegXMM s, RegXMM d)         { instrRR(instr_unpcklpd, d, s); }

  void rorq  (Immed i, Reg64 r) { instrIR(instr_ror, i, r); }
  void shlq  (Immed i, Reg64 r) { instrIR(instr_shl, i, r); }
  void shrq  (Immed i, Reg64 r) { instrIR(instr_shr, i, r); }
  void sarq  (Immed i, Reg64 r) { instrIR(instr_sar, i, r); }
  void shll  (Immed i, Reg32 r) { instrIR(instr_shl, i, r); }
  void shrl  (Immed i, Reg32 r) { instrIR(instr_shr, i, r); }
  void shlw  (Immed i, Reg16 r) { instrIR(instr_shl, i, r); }
  void shrw  (Immed i, Reg16 r) { instrIR(instr_shr, i, r); }

  void shlq (Reg64 r) { instrR(instr_shl, r); }
  void sarq (Reg64 r) { instrR(instr_sar, r); }

  void roundsd (RoundDirection d, RegXMM src, RegXMM dst) {
    emitIRR(instr_roundsd, rn(dst), rn(src), ssize_t(d));
  }

  void cmpsd(RegXMM src, RegXMM dst, ComparisonPred pred) {
    emitIRR(instr_cmpsd, rn(dst), rn(src), ssize_t(pred));
  }

  /*
   * Control-flow directives.  Primitive labeling/patching facilities
   * are available, as well as slightly higher-level ones via the
   * Label class.
   */

  bool jmpDeltaFits(CodeAddress dest) {
    int64_t delta = dest - (codeBlock.frontier() + 5);
    return deltaFits(delta, sz::dword);
  }

  void jmp(Reg64 r)            { instrR(instr_jmp, r); }
  void jmp(MemoryRef m)        { instrM(instr_jmp, m); }
  void call(Reg64 r)           { instrR(instr_call, r); }
  void call(MemoryRef m)       { instrM(instr_call, m); }
  void call(RIPRelativeRef m)  { instrM(instr_call, m); }

  void jmp8(CodeAddress dest)  { emitJ8(instr_jmp, ssize_t(dest)); }

  void jmp(CodeAddress dest) {
    always_assert(dest && jmpDeltaFits(dest));
    emitJ32(instr_jmp, ssize_t(dest));
  }

  void call(CodeAddress dest) {
    always_assert(dest && jmpDeltaFits(dest));
    emitJ32(instr_call, ssize_t(dest));
  }

  void jcc(ConditionCode cond, CodeAddress dest) {
    emitCJ32(instr_jcc, cond, (ssize_t)dest);
  }

  void jcc8(ConditionCode cond, CodeAddress dest) {
    emitCJ8(instr_jcc, cond, (ssize_t)dest);
  }

  void jmpAuto(CodeAddress dest) {
    auto delta = dest - (codeBlock.frontier() + 2);
    if (deltaFits(delta, sz::byte)) {
      jmp8(dest);
    } else {
      jmp(dest);
    }
  }

  void jccAuto(ConditionCode cc, CodeAddress dest) {
    auto delta = dest - (codeBlock.frontier() + 2);
    if (deltaFits(delta, sz::byte)) {
      jcc8(cc, dest);
    } else {
      jcc(cc, dest);
    }
  }

  void call(Label&);
  void jmp(Label&);
  void jmp8(Label&);
  void jcc(ConditionCode, Label&);
  void jcc8(ConditionCode, Label&);

#define CCS \
  CC(o,   CC_O)         \
  CC(no,  CC_NO)        \
  CC(nae, CC_NAE)       \
  CC(ae,  CC_AE)        \
  CC(nb,  CC_NB)        \
  CC(e,   CC_E)         \
  CC(z,   CC_Z)         \
  CC(ne,  CC_NE)        \
  CC(nz,  CC_NZ)        \
  CC(b,   CC_B)         \
  CC(be,  CC_BE)        \
  CC(nbe, CC_NBE)       \
  CC(s,   CC_S)         \
  CC(ns,  CC_NS)        \
  CC(p,   CC_P)         \
  CC(np,  CC_NP)        \
  CC(nge, CC_NGE)       \
  CC(g,   CC_G)         \
  CC(l,   CC_L)         \
  CC(ge,  CC_GE)        \
  CC(nl,  CC_NL)        \
  CC(ng,  CC_NG)        \
  CC(le,  CC_LE)        \
  CC(nle, CC_NLE)

#define CC(_nm, _code)                                        \
  void j ## _nm(CodeAddress dest)      { jcc(_code, dest); }  \
  void j ## _nm ## 8(CodeAddress dest) { jcc8(_code, dest); } \
  void j ## _nm(Label&);                                      \
  void j ## _nm ## 8(Label&);
  CCS
#undef CC

  void setcc(int cc, Reg8 byteReg) {
    emitCR(instr_setcc, cc, rn(byteReg), sz::byte);
  }

#define CC(_nm, _cond)                          \
  void set ## _nm(Reg8 byteReg) {               \
    setcc(_cond, byteReg);                      \
  }
  CCS
#undef CC

  void psllq(Immed i, RegXMM r) { emitIR(instr_psllq, rn(r), i.b()); }
  void psrlq(Immed i, RegXMM r) { emitIR(instr_psrlq, rn(r), i.b()); }

  void movq_rx(Reg64 rSrc, RegXMM rdest) {
    emitRR(instr_gpr2xmm, rn(rdest), rn(rSrc));
  }
  void movq_xr(RegXMM rSrc, Reg64 rdest) {
    emitRR(instr_xmm2gpr, rn(rSrc), rn(rdest));
  }

  void addsd(RegXMM src, RegXMM srcdest) {
    emitRR(instr_xmmadd, rn(srcdest), rn(src));
  }
  void mulsd(RegXMM src, RegXMM srcdest) {
    emitRR(instr_xmmmul, rn(srcdest), rn(src));
  }
  void subsd(RegXMM src, RegXMM srcdest) {
    emitRR(instr_xmmsub, rn(srcdest), rn(src));
  }
  void pxor(RegXMM src, RegXMM srcdest) {
    emitRR(instr_pxor, rn(srcdest), rn(src));
  }
  void cvtsi2sd(Reg64 src, RegXMM dest) {
    emitRR(instr_cvtsi2sd, rn(dest), rn(src));
  }
  void cvtsi2sd(MemoryRef m, RegXMM dest) {
    instrMR(instr_cvtsi2sd, m, dest);
  }
  void ucomisd(RegXMM l, RegXMM r) {
    emitRR(instr_ucomisd, rn(l), rn(r));
  }
  void sqrtsd(RegXMM src, RegXMM dest) {
    emitRR(instr_xmmsqrt, rn(dest), rn(src));
  }

  void divsd(RegXMM src, RegXMM srcdest) {
    emitRR(instr_divsd, rn(srcdest), rn(src));
  }
  void cvttsd2siq(RegXMM src, Reg64 dest) {
    emitRR(instr_cvttsd2si, rn(dest), rn(src));
  }

  /*
   * The following utility functions do more than emit specific code.
   * (E.g. combine common idioms or patterns, smash code, etc.)
   */

  void emitImmReg(Immed64 imm, Reg64 dest) {
    if (imm.q() == 0) {
      // Zeros the top bits also.
      xorl  (r32(dest), r32(dest));
      return;
    }
    if (LIKELY(imm.q() > 0 && imm.fits(sz::dword))) {
      // This will zero out the high-order bits.
      movl (imm.l(), r32(dest));
      return;
    }
    movq (imm.q(), dest);
  }

  static void patchJcc(CodeAddress jmp, CodeAddress dest) {
    assert(jmp[0] == 0x0F && (jmp[1] & 0xF0) == 0x80);
    ssize_t diff = dest - (jmp + 6);
    *(int32_t*)(jmp + 2) = safe_cast<int32_t>(diff);
  }

  static void patchJcc8(CodeAddress jmp, CodeAddress dest) {
    assert((jmp[0] & 0xF0) == 0x70);
    ssize_t diff = dest - (jmp + 2);  // one for opcode, one for offset
    *(int8_t*)(jmp + 1) = safe_cast<int8_t>(diff);
  }

  static void patchJmp(CodeAddress jmp, CodeAddress dest) {
    assert(jmp[0] == 0xE9);
    ssize_t diff = dest - (jmp + 5);
    *(int32_t*)(jmp + 1) = safe_cast<int32_t>(diff);
  }

  static void patchJmp8(CodeAddress jmp, CodeAddress dest) {
    assert(jmp[0] == 0xEB);
    ssize_t diff = dest - (jmp + 2);  // one for opcode, one for offset
    *(int8_t*)(jmp + 1) = safe_cast<int8_t>(diff);
  }

  static void patchCall(CodeAddress call, CodeAddress dest) {
    assert(call[0] == 0xE8);
    ssize_t diff = dest - (call + 5);
    *(int32_t*)(call + 1) = safe_cast<int32_t>(diff);
  }

  void emitInt3s(int n) {
    for (auto i = 0; i < n; ++i) {
      byte(0xcc);
    }
  }

  void emitNop(int n) {
    if (n == 0) return;
    static const uint8_t nops[][9] = {
      { },
      { 0x90 },
      { 0x66, 0x90 },
      { 0x0f, 0x1f, 0x00 },
      { 0x0f, 0x1f, 0x40, 0x00 },
      { 0x0f, 0x1f, 0x44, 0x00, 0x00 },
      { 0x66, 0x0f, 0x1f, 0x44, 0x00, 0x00 },
      { 0x0f, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00 },
      { 0x0f, 0x1f, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00 },
      { 0x66, 0x0f, 0x1f, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00 },
    };
    // While n >= 9, emit 9 byte NOPs
    while (n >= 9) {
      bytes(9, nops[9]);
      n -= 9;
    }
    bytes(n, nops[n]);
  }

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

  // op %r
  // ------
  // Restrictions:
  //     r cannot be set to 'none'
  ALWAYS_INLINE
  void emitCR(X64Instr op, int jcond, RegNumber regN, int opSz = sz::qword) {
    assert(regN != noreg);
    int r = int(regN);

    // Opsize prefix
    if (opSz == sz::word) {
      byte(kOpsizePrefix);
    }

    // REX
    unsigned char rex = 0;
    bool highByteReg = false;
    if (opSz == sz::byte) {
      if (byteRegNeedsRex(r)) {
        rex |= 0x40;
      }
      r = byteRegEncodeNumber(r, highByteReg);
    }
    if ((op.flags & IF_NO_REXW) == 0 && opSz == sz::qword) rex |= 8;
    if (r & 8) rex |= 1;
    if (rex) {
      byte(0x40 | rex);
      if (highByteReg) byteRegMisuse();
    }
    // If the instruction supports compact-R mode, use that
    if (op.flags & IF_COMPACTR) {
      byte(op.table[5] | (r & 7));
      return;
    }
    char opcode = (op.flags & IF_REVERSE) ? op.table[1] : op.table[0];
    char rval = op.table[3];
    // Handle two byte opcodes
    if (op.flags & IF_TWOBYTEOP) byte(0x0F);
    byte(opcode | jcond);
    emitModrm(3, rval, r);
  }

  ALWAYS_INLINE
  void emitR(X64Instr op, RegNumber r, int opSz = sz::qword) {
    emitCR(op, 0, r, opSz);
  }

  ALWAYS_INLINE
  void emitR32(X64Instr op, RegNumber r) {
    emitCR(op, 0, r, sz::dword);
  }

  ALWAYS_INLINE
  void emitR16(X64Instr op, RegNumber r) {
    emitCR(op, 0, r, sz::word);
  }

  // op %r2, %r1
  // -----------
  // Restrictions:
  //     r1 cannot be set to noreg
  //     r2 cannot be set to noreg
  ALWAYS_INLINE
  void emitCRR(X64Instr op, int jcond, RegNumber rn1, RegNumber rn2,
               int opSz = sz::qword) {
    assert(rn1 != noreg && rn2 != noreg);
    int r1 = int(rn1);
    int r2 = int(rn2);
    bool reverse = ((op.flags & IF_REVERSE) != 0);
    prefixBytes(op.flags, opSz);
    // The xchg instruction is special; we have compact encodings for
    // exchanging with rax or eax.
    if (op.flags & IF_XCHG) {
      if (r1 == int(reg::rax)) {
        // REX
        unsigned char rex = 0;
        if ((op.flags & IF_NO_REXW) == 0 && opSz == sz::qword) rex |= 8;
        assert(!(op.flags & IF_BYTEREG));
        if (r2 & 8) rex |= (reverse ? 4 : 1);
        if (rex) byte(0x40 | rex);
        // If the second register is rax, emit opcode with the first
        // register id embedded
        byte(op.table[5] | (r2 & 7));
        return;
      } else if (r2 == int(reg::rax)) {
        reverse = !reverse;
        // REX
        unsigned char rex = 0;
        if ((op.flags & IF_NO_REXW) == 0 && opSz == sz::qword) {
          rex |= 8;
        }
        if (r1 & 8) rex |= (reverse ? 1 : 4);
        if (rex) byte(0x40 | rex);
        // If the first register is rax, emit opcode with the second
        // register id embedded
        byte(op.table[5] | (r1 & 7));
        return;
      }
    }
    // REX
    unsigned char rex = 0;
    if ((op.flags & IF_NO_REXW) == 0 && opSz == sz::qword) rex |= 8;
    bool highByteReg = false;
    // movzbx's first operand is a bytereg regardless of operand size
    if (opSz == sz::byte || (op.flags & IF_BYTEREG)) {
      if (byteRegNeedsRex(r1) ||
          (!(op.flags & IF_BYTEREG) && byteRegNeedsRex(r2))) {
        rex |= 0x40;
      }
      r1 = byteRegEncodeNumber(r1, highByteReg);
      r2 = byteRegEncodeNumber(r2, highByteReg);
    }
    if (r1 & 8) rex |= (reverse ? 1 : 4);
    if (r2 & 8) rex |= (reverse ? 4 : 1);
    if (rex) {
      byte(0x40 | rex);
      if (highByteReg) byteRegMisuse();
    }
    // For two byte opcodes
    if ((op.flags & (IF_TWOBYTEOP | IF_IMUL)) != 0) byte(0x0F);
    byte(op.table[0] | jcond);
    if (reverse) {
      emitModrm(3, r2, r1);
    } else {
      emitModrm(3, r1, r2);
    }
  }

  ALWAYS_INLINE
  void emitCRR32(X64Instr op, int jcond, RegNumber r1, RegNumber r2) {
    emitCRR(op, jcond, r1, r2, sz::dword);
  }

  ALWAYS_INLINE
  void emitRR(X64Instr op, RegNumber r1, RegNumber r2, int opSz = sz::qword) {
    emitCRR(op, 0, r1, r2, opSz);
  }

  ALWAYS_INLINE
  void emitRR32(X64Instr op, RegNumber r1, RegNumber r2) {
    emitCRR(op, 0, r1, r2, sz::dword);
  }

  ALWAYS_INLINE
  void emitRR16(X64Instr op, RegNumber r1, RegNumber r2) {
    emitCRR(op, 0, r1, r2, sz::word);
  }

  ALWAYS_INLINE
  void emitRR8(X64Instr op, RegNumber r1, RegNumber r2) {
    emitCRR(op, 0, r1, r2, sz::byte);
  }

  // op $imm, %r
  // -----------
  // Restrictions:
  //     r cannot be set to noreg
  ALWAYS_INLINE
  void emitIR(X64Instr op, RegNumber rname, ssize_t imm,
              int opSz = sz::qword) {
    assert(rname != noreg);
    int r = int(rname);
    // Opsize prefix
    prefixBytes(op.flags, opSz);
    // Determine the size of the immediate.  This might change opSz so
    // do it first.
    int immSize;
    if ((op.flags & IF_MOV) && opSz == sz::qword) {
      immSize = computeImmediateSizeForMovRI64(op, imm, opSz);
    } else {
      immSize = computeImmediateSize(op, imm, opSz);
    }
    // REX
    unsigned char rex = 0;
    bool highByteReg = false;
    if (opSz == sz::byte) {
      if (byteRegNeedsRex(r)) {
        rex |= 0x40;
      }
      r = byteRegEncodeNumber(r, highByteReg);
    }
    if ((op.flags & IF_NO_REXW) == 0 && opSz == sz::qword) rex |= 8;
    if (r & 8) rex |= 1;
    if (rex) {
      byte(0x40 | rex);
      if (highByteReg) byteRegMisuse();
    }
    // Use the special rax encoding if the instruction supports it
    if (r == int(reg::rax) && immSize == sz::dword &&
        (op.flags & IF_RAX)) {
      byte(op.table[4]);
      emitImmediate(op, imm, immSize);
      return;
    }
    // Use the compact-R encoding if the operand size and the immediate
    // size are the same
    if ((op.flags & IF_COMPACTR) && immSize == opSz) {
      byte(op.table[5] | (r & 7));
      emitImmediate(op, imm, immSize);
      return;
    }
    // For two byte opcodes
    if ((op.flags & (IF_TWOBYTEOP | IF_IMUL)) != 0) byte(0x0F);
    int rval = op.table[3];
    // shift/rotate instructions have special opcode when
    // immediate is 1
    if ((op.flags & IF_SHIFT) != 0 && imm == 1) {
      byte(0xd1);
      emitModrm(3, rval, r);
      // don't emit immediate
      return;
    }
    int opcode = (immSize == sz::byte && opSz != sz::byte) ?
      (op.table[2] | 2) : op.table[2];
    byte(opcode);
    emitModrm(3, rval, r);
    emitImmediate(op, imm, immSize);
  }

  ALWAYS_INLINE
  void emitIR32(X64Instr op, RegNumber r, ssize_t imm) {
    emitIR(op, r, imm, sz::dword);
  }

  ALWAYS_INLINE
  void emitIR16(X64Instr op, RegNumber r, ssize_t imm) {
    emitIR(op, r, safe_cast<int16_t>(imm), sz::word);
  }

  ALWAYS_INLINE
  void emitIR8(X64Instr op, RegNumber r, ssize_t imm) {
    emitIR(op, r, safe_cast<int8_t>(imm), sz::byte);
  }

  // op $imm, %r2, %r1
  // -----------------
  // Restrictions:
  //     r1 cannot be set to noreg
  //     r2 cannot be set to noreg
  ALWAYS_INLINE
  void emitIRR(X64Instr op, RegNumber rn1, RegNumber rn2, ssize_t imm,
               int opSz = sz::qword) {
    assert(rn1 != noreg && rn2 != noreg);
    int r1 = int(rn1);
    int r2 = int(rn2);
    bool reverse = ((op.flags & IF_REVERSE) != 0);
    // Opsize prefix
    prefixBytes(op.flags, opSz);
    // REX
    unsigned char rex = 0;
    if ((op.flags & IF_NO_REXW) == 0 && opSz == sz::qword) rex |= 8;
    bool highByteReg = false;
    if (opSz == sz::byte || (op.flags & IF_BYTEREG)) {
      if (byteRegNeedsRex(r1) ||
          (!(op.flags & IF_BYTEREG) && byteRegNeedsRex(r2))) {
        rex |= 0x40;
      }
      r1 = byteRegEncodeNumber(r1, highByteReg);
      r2 = byteRegEncodeNumber(r2, highByteReg);
    }
    if (r1 & 8) rex |= (reverse ? 1 : 4);
    if (r2 & 8) rex |= (reverse ? 4 : 1);
    if (rex) {
      byte(0x40 | rex);
      if (highByteReg) byteRegMisuse();
    }
    // Determine the size of the immediate
    int immSize = computeImmediateSize(op, imm, opSz);
    if (op.flags & IF_TWOBYTEOP || op.flags & IF_THREEBYTEOP) byte(0x0F);
    if (op.flags & IF_THREEBYTEOP) byte(0x3a);
    int opcode = (immSize == sz::byte && opSz != sz::byte &&
                  (op.flags & IF_ROUND) == 0) ?
      (op.table[2] | 2) : op.table[2];
    byte(opcode);
    if (reverse) {
      emitModrm(3, r2, r1);
    } else {
      emitModrm(3, r1, r2);
    }
    emitImmediate(op, imm, immSize);
  }

  ALWAYS_INLINE
  void emitCI(X64Instr op, int jcond, ssize_t imm, int opSz = sz::qword) {
    // Opsize prefix
    prefixBytes(op.flags, opSz);
    // REX
    if ((op.flags & IF_NO_REXW) == 0) {
      byte(0x48);
    }
    // Determine the size of the immediate
    int immSize = computeImmediateSize(op, imm, opSz);
    // Emit opcode
    if ((op.flags & IF_JCC) != 0) {
      // jcc is weird so we handle it separately
      if (immSize != sz::byte) {
        byte(0x0F);
        byte(jcond | 0x80);
      } else {
        byte(jcond | 0x70);
      }
    } else {
      int opcode = (immSize == sz::byte && opSz != sz::byte) ?
        (op.table[2] | 2) : op.table[2];
      byte(jcond | opcode);
    }
    emitImmediate(op, imm, immSize);
  }

  ALWAYS_INLINE
  void emitI(X64Instr op, ssize_t imm, int opSz = sz::qword) {
    emitCI(op, 0, imm, opSz);
  }

  ALWAYS_INLINE
  void emitJ8(X64Instr op, ssize_t imm) {
    assert((op.flags & IF_JCC) == 0);
    ssize_t delta = imm - ((ssize_t)codeBlock.frontier() + 2);
    // Emit opcode and 8-bit immediate
    byte(0xEB);
    byte(safe_cast<int8_t>(delta));
  }

  ALWAYS_INLINE
  void emitCJ8(X64Instr op, int jcond, ssize_t imm) {
    // this is for jcc only
    assert(op.flags & IF_JCC);
    ssize_t delta = imm - ((ssize_t)codeBlock.frontier() + 2);
    // Emit opcode
    byte(jcond | 0x70);
    // Emit 8-bit offset
    byte(safe_cast<int8_t>(delta));
  }

  ALWAYS_INLINE
  void emitJ32(X64Instr op, ssize_t imm) {
    // call and jmp are supported, jcc is not supported
    assert((op.flags & IF_JCC) == 0);
    int32_t delta =
      safe_cast<int32_t>(imm - ((ssize_t)codeBlock.frontier() + 5));
    uint8_t *bdelta = (uint8_t*)&delta;
    uint8_t instr[] = { op.table[2],
      bdelta[0], bdelta[1], bdelta[2], bdelta[3] };
    bytes(5, instr);
  }

  ALWAYS_INLINE
  void emitCJ32(X64Instr op, int jcond, ssize_t imm) {
    // jcc is supported, call and jmp are not supported
    assert(op.flags & IF_JCC);
    int32_t delta =
      safe_cast<int32_t>(imm - ((ssize_t)codeBlock.frontier() + 6));
    uint8_t* bdelta = (uint8_t*)&delta;
    uint8_t instr[6] = { 0x0f, uint8_t(0x80 | jcond),
      bdelta[0], bdelta[1], bdelta[2], bdelta[3] };
    bytes(6, instr);
  }

  // op disp(%br,%ir,s)
  //   (for reverse == false, hasImmediate == false, r == noreg)
  // op $imm, disp(%br,%ir,s)
  //   (for reverse == false, hasImmediate == true,  r == noreg)
  // op %r, disp(%br,%ir,s)
  //   (for reverse == false, hasImmediate == false, r != noreg)
  // op $imm, %r, disp(%br,%ir,s)
  //   (for reverse == false, hasImmediate == true,  r != noreg)
  // op disp(%br,%ir,s), %r
  //   (for reverse == true,  hasImmediate == false, r != noreg)
  // op $imm, disp(%br,%ir,s), %r
  //   (for reverse == true,  hasImmediate == true,  r != noreg)
  // -----------------------------------------------------------------
  // Restrictions:
  //     ir cannot be set to 'sp'
  ALWAYS_INLINE
  void emitCMX(X64Instr op, int jcond, RegNumber brName, RegNumber irName,
               int s, int64_t disp,
               RegNumber rName,
               bool reverse = false,
               ssize_t imm = 0,
               bool hasImmediate = false,
               int opSz = sz::qword,
               bool ripRelative = false) {
    assert(irName != rn(reg::rsp));

    int ir = int(irName);
    int r = int(rName);
    int br = int(brName);

    // The opsize prefix can be placed here, if the instruction
    // deals with words.
    // When an instruction has a manditory prefix, it goes before the
    // REX byte if we end up needing one.
    prefixBytes(op.flags, opSz);

    // Determine immSize from the 'hasImmediate' flag
    int immSize = sz::nosize;
    if (hasImmediate) {
      immSize = computeImmediateSize(op, imm, opSz);
    }
    if ((op.flags & IF_REVERSE) != 0) reverse = !reverse;
    // Determine if we need to use a two byte opcode;
    // imul is weird so we have a special case for it
    bool twoByteOpcode = ((op.flags & IF_TWOBYTEOP) != 0) ||
      ((op.flags & IF_IMUL) != 0 && rName != noreg &&
      immSize == sz::nosize);
    // Again, imul is weird
    if ((op.flags & IF_IMUL) != 0 && rName != noreg) {
      reverse = !reverse;
    }
    // The wily rex byte, a multipurpose extension to the opcode space for x64
    unsigned char rex = 0;
    if ((op.flags & IF_NO_REXW) == 0 && opSz == sz::qword) rex |= 8;

    bool highByteReg = false;
    // XXX: This IF_BYTEREG check is a special case for movzbl: we currently
    // encode it using an opSz of sz::byte but it doesn't actually have a
    // byte-sized operand like other instructions can.
    if (!(op.flags & IF_BYTEREG) && opSz == sz::byte && rName != noreg) {
      if (byteRegNeedsRex(r)) {
        rex |= 0x40;
      }
      r = byteRegEncodeNumber(r, highByteReg);
    }

    if (rName != noreg && (r & 8)) rex |= 4;
    if (irName != noreg && (ir & 8)) rex |= 2;
    if (brName != noreg && (br & 8)) rex |= 1;
    if (rex) {
      byte(0x40 | rex);
      if (highByteReg) byteRegMisuse();
    }
    // Emit the opcode
    if (immSize != sz::nosize) {
      if (twoByteOpcode) byte(0x0F);
      if (immSize == sz::byte && opSz != sz::byte) {
        byte(op.table[2] | 2 | jcond);
      } else {
        byte(op.table[2] | jcond);
      }
    } else {
      if (twoByteOpcode) byte(0x0F);
      int opcode;
      if ((op.flags & IF_IMUL) != 0) {
        opcode = (rName == noreg) ? op.table[1] : op.table[0];
      } else {
        opcode = reverse ? op.table[1] : op.table[0];
      }
      byte(opcode | jcond);
    }
    // SIB byte if:
    //   1. We're using an index register.
    //   2. The base register is rsp-like.
    //   3. We're doing a baseless disp access and it is not rip-relative.
    bool sibIsNeeded =
      ir != int(noreg) ||                      /* 1 */
      br == int(reg::rsp) || br == int(reg::r12) || /* 2 */
      (br == int(noreg) && !ripRelative);
    // If there is no register and no immediate, use the /r value
    if (r == int(noreg)) r = op.table[3];
    // If noreg was specified for 'ir', we use
    // the encoding for the sp register
    if (ir == int(noreg)) ir = 4;
    int dispSize = sz::nosize;
    if (disp != 0) {
      if (!ripRelative && disp <= 127 && disp >= -128) {
        dispSize = sz::byte;
      } else {
        dispSize = sz::dword;
      }
    }
    // Set 'mod' based on the size of the displacement
    int mod;
    switch (dispSize) {
      case sz::nosize: mod = 0; break;
      case sz::byte: mod = 1; break;
      default: mod = 2; break;
    }
    // Handle special cases for 'br'
    if (br == int(noreg)) {
      // If noreg was specified for 'br', we use the encoding
      // for the rbp register (or rip, if we're emitting a
      // rip-relative instruction), and we must set mod=0 and
      // "upgrade" to a DWORD-sized displacement
      br = 5;
      mod = 0;
      dispSize = sz::dword;
    } else if ((br & 7) == 5 && dispSize == sz::nosize) {
      // If br == rbp and no displacement was specified, we
      // must "upgrade" to using a 1-byte displacement value
      dispSize = sz::byte;
      mod = 1;
    }
    // Emit modr/m and the sib
    if (sibIsNeeded) {
      // s:                               0  1  2   3  4   5   6   7  8
      static const int scaleLookup[] = { -1, 0, 1, -1, 2, -1, -1, -1, 3 };
      assert(s > 0 && s <= 8);
      int scale = scaleLookup[s];
      assert(scale != -1);
      emitModrm(mod, r, 4);
      byte((scale << 6) | ((ir & 7) << 3) | (br & 7));
    } else {
      emitModrm(mod, r, br);
    }
    // Emit displacement if needed
    if (dispSize == sz::dword) {
      if (ripRelative) {
        disp -= (int64_t)codeBlock.frontier() + immSize + dispSize;
      }
      dword(disp);
    } else if (dispSize == sz::byte) {
      byte(disp & 0xff);
    }
    // Emit immediate if needed
    if (immSize != sz::nosize) {
      emitImmediate(op, imm, immSize);
    }
  }

  ALWAYS_INLINE
  void emitIM(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
              ssize_t imm, int opSz = sz::qword) {
    emitCMX(op, 0, br, ir, s, disp, noreg, false, imm, true, opSz);
  }

  ALWAYS_INLINE
  void emitIM8(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
               ssize_t imm) {
    emitCMX(op, 0, br, ir, s, disp, noreg, false, imm, true,
            sz::byte);
  }

  ALWAYS_INLINE
  void emitIM16(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
                ssize_t imm) {
    emitCMX(op, 0, br, ir, s, disp, noreg, false, imm, true,
            sz::word);
  }

  ALWAYS_INLINE
  void emitIM32(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
                ssize_t imm) {
    emitCMX(op, 0, br, ir, s, disp, noreg, false, imm, true, sz::dword);
  }

  ALWAYS_INLINE
  void emitRM(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
              RegNumber r, int opSz = sz::qword) {
    emitCMX(op, 0, br, ir, s, disp, r, false, 0, false, opSz);
  }

  ALWAYS_INLINE
  void emitRM32(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
                RegNumber r) {
    emitCMX(op, 0, br, ir, s, disp, r, false, 0, false, sz::dword);
  }

  ALWAYS_INLINE
  void emitRM16(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
                RegNumber r) {
    emitCMX(op, 0, br, ir, s, disp, r, false, 0, false, sz::word);
  }

  ALWAYS_INLINE
  void emitRM8(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
               RegNumber r) {
    emitCMX(op, 0, br, ir, s, disp, r, false, 0, false, sz::byte);
  }

  ALWAYS_INLINE
  void emitCMR(X64Instr op, int jcond, RegNumber br, RegNumber ir,
               int s, int disp, RegNumber r, int opSz = sz::qword) {
    emitCMX(op, jcond, br, ir, s, disp, r, true, 0, false, opSz);
  }

  ALWAYS_INLINE
  void emitMR(X64Instr op, RegNumber br, RegNumber ir, int s, int64_t disp,
              RegNumber r, int opSz = sz::qword, bool ripRelative = false) {
    emitCMX(op, 0, br, ir, s, disp, r, true, 0, false, opSz, ripRelative);
  }

  ALWAYS_INLINE
  void emitMR32(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
                RegNumber r) {
    emitCMX(op, 0, br, ir, s, disp, r, true, 0, false, sz::dword);
  }

  ALWAYS_INLINE
  void emitMR16(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
                RegNumber r) {
    emitCMX(op, 0, br, ir, s, disp, r, true, 0, false, sz::word);
  }

  ALWAYS_INLINE
  void emitMR8(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
               RegNumber r) {
    emitCMX(op, 0, br, ir, s, disp, r, true, 0, false, sz::byte);
  }

  ALWAYS_INLINE
  void emitIRM(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
               RegNumber r, ssize_t imm, int opSz = sz::qword) {
    emitCMX(op, 0, br, ir, s, disp, r, false, imm, true, opSz);
  }

  ALWAYS_INLINE
  void emitIMR(X64Instr op, RegNumber br, RegNumber ir, int s, int disp,
               RegNumber r, ssize_t imm, int opSz = sz::qword) {
    emitCMX(op, 0, br, ir, s, disp, r, true, imm, true, opSz);
  }

  ALWAYS_INLINE
  void emitM(X64Instr op, RegNumber br, RegNumber ir, int s, int64_t disp,
             int opSz = sz::qword, bool ripRelative = false) {
    emitCMX(op, 0, br, ir, s, disp, noreg, false, 0, false, opSz,
            ripRelative);
  }

  ALWAYS_INLINE
  void emitM32(X64Instr op, RegNumber br, RegNumber ir, int s, int disp) {
    emitCMX(op, 0, br, ir, s, disp, noreg, false, 0, false, sz::dword);
  }

  ALWAYS_INLINE
  void emitM16(X64Instr op, RegNumber br, RegNumber ir, int s, int disp) {
    emitCMX(op, 0, br, ir, s, disp, noreg, false, 0, false, sz::word);
  }

  ALWAYS_INLINE
  void emitCM(X64Instr op, int jcond, RegNumber br,
              RegNumber ir, int s, int disp, int opSz = sz::qword) {
    emitCMX(op, jcond, br, ir, s, disp, noreg, false, 0, false, opSz);
  }

  // emit (with no arguments)
  ALWAYS_INLINE
  void emit(X64Instr op) {
    if ((op.flags & IF_NO_REXW) == 0) {
      byte(0x48);
    }
    byte(op.table[5]);
  }

  // Segment register prefixes.
  X64Assembler& fs()  { byte(0x64); return *this; }
  X64Assembler& gs()  { byte(0x65); return *this; }

public:
  /*
   * The following functions use a naming convention for an older API
   * to the assembler; conditional loads and moves haven't yet been
   * ported.
   */

  // CMOVcc [rbase + off], rdest
  inline void cload_reg64_disp_reg64(ConditionCode cc, Reg64 rbase,
                                     int off, Reg64 rdest) {
    emitCMX(instr_cmovcc, cc, rn(rbase), noreg, sz::byte, off, rn(rdest),
            false /*reverse*/);

  }
  inline void cload_reg64_disp_reg32(ConditionCode cc, Reg64 rbase,
                                     int off, Reg32 rdest) {
    emitCMX(instr_cmovcc, cc, rn(rbase), noreg, sz::byte, off, rn(rdest),
            false /*reverse*/,
            0 /*imm*/,
            false /*hasImmediate*/,
            sz::dword /*opSz*/);
  }
  inline void cmov_reg64_reg64(ConditionCode cc, Reg64 rsrc, Reg64 rdest) {
    emitCRR(instr_cmovcc, cc, rn(rsrc), rn(rdest));
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

  void emitModrm(int x, int y, int z) {
    byte((x << 6) | ((y & 7) << 3) | (z & 7));
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

private:
  RegNumber rn(Reg8 r)   { return RegNumber(int(r)); }
  RegNumber rn(Reg16 r)  { return RegNumber(int(r)); }
  RegNumber rn(Reg32 r)  { return RegNumber(int(r)); }
  RegNumber rn(Reg64 r)  { return RegNumber(int(r)); }
  RegNumber rn(RegXMM r) { return RegNumber(int(r)); }

  // Wraps a bunch of the emit* functions to make using them with the
  // typed wrappers more terse. We should have these replace
  // the emit functions eventually.

#define UMR(m) rn(m.r.base), rn(m.r.index), m.r.scale, m.r.disp
#define URIP(m) noreg, noreg, sz::byte, m.r.disp

  void instrR(X64Instr   op, Reg64  r)           { emitR(op,    rn(r));        }
  void instrR(X64Instr   op, Reg32  r)           { emitR32(op,  rn(r));        }
  void instrR(X64Instr   op, Reg16  r)           { emitR16(op,  rn(r));        }
  void instrR(X64Instr   op, Reg8   r)           { emitR(op, rn(r), sz::byte); }
  void instrRR(X64Instr  op, Reg64  x, Reg64  y) { emitRR(op,   rn(x), rn(y)); }
  void instrRR(X64Instr  op, Reg32  x, Reg32  y) { emitRR32(op, rn(x), rn(y)); }
  void instrRR(X64Instr  op, Reg16  x, Reg16  y) { emitRR16(op, rn(x), rn(y)); }
  void instrRR(X64Instr  op, Reg8   x, Reg8   y) { emitRR8(op,  rn(x), rn(y)); }
  void instrRR(X64Instr  op, RegXMM x, RegXMM y) { emitRR(op,   rn(x), rn(y)); }
  void instrM(X64Instr   op, MemoryRef m)        { emitM(op,    UMR(m));       }
  void instrM(X64Instr   op, RIPRelativeRef m)   { emitM(op,    URIP(m),
                                                         sz::qword, true);     }
  void instrM32(X64Instr op, MemoryRef m)        { emitM32(op,  UMR(m));       }
  void instrM16(X64Instr op, MemoryRef m)        { emitM16(op,  UMR(m));       }

  void instrRM(X64Instr op,
               Reg64 r,
               MemoryRef m)        { emitRM(op, UMR(m), rn(r)); }
  void instrRM(X64Instr op,
               Reg32 r,
               MemoryRef m)        { emitRM32(op, UMR(m), rn(r)); }
  void instrRM(X64Instr op,
               Reg16 r,
               MemoryRef m)        { emitRM16(op, UMR(m), rn(r)); }
  void instrRM(X64Instr op,
               Reg8 r,
               MemoryRef m)        { emitRM8(op, UMR(m), rn(r)); }
  void instrRM(X64Instr op,
               RegXMM x,
               MemoryRef m)        { emitRM(op, UMR(m), rn(x)); }

  void instrMR(X64Instr op,
               MemoryRef m,
               Reg64 r)            { emitMR(op, UMR(m), rn(r)); }
  void instrMR(X64Instr op,
               MemoryRef m,
               Reg32 r)            { emitMR32(op, UMR(m), rn(r)); }
  void instrMR(X64Instr op,
               MemoryRef m,
               Reg16 r)            { emitMR16(op, UMR(m), rn(r)); }
  void instrMR(X64Instr op,
               MemoryRef m,
               Reg8 r)             { emitMR8(op, UMR(m), rn(r)); }
  void instrMR(X64Instr op,
               MemoryRef m,
               RegXMM x)           { emitMR(op, UMR(m), rn(x)); }
  void instrMR(X64Instr op,
               RIPRelativeRef m,
               Reg64 r)            { emitMR(op, URIP(m), rn(r),
                                            sz::qword, true); }
  void instrMR(X64Instr op,
               RIPRelativeRef m,
               RegXMM r)           { emitMR(op, URIP(m), rn(r),
                                            sz::qword, true); }

  void instrIR(X64Instr op, Immed64 i, Reg64 r) {
    emitIR(op, rn(r), i.q());
  }
  void instrIR(X64Instr op, Immed i, Reg64 r) {
    emitIR(op, rn(r), i.q());
  }
  void instrIR(X64Instr op, Immed i, Reg32 r) {
    emitIR32(op, rn(r), i.l());
  }
  void instrIR(X64Instr op, Immed i, Reg16 r) {
    emitIR16(op, rn(r), i.w());
  }
  void instrIR(X64Instr op, Immed i, Reg8 r) {
    emitIR8(op, rn(r), i.b());
  }

  void instrIM(X64Instr op, Immed i, MemoryRef m) {
    emitIM(op, UMR(m), i.q());
  }
  void instrIM32(X64Instr op, Immed i, MemoryRef m) {
    emitIM32(op, UMR(m), i.l());
  }
  void instrIM16(X64Instr op, Immed i, MemoryRef m) {
    emitIM16(op, UMR(m), i.w());
  }
  void instrIM8(X64Instr op, Immed i, MemoryRef m) {
    emitIM8(op, UMR(m), i.b());
  }

#undef UMR
#undef URIP

  CodeBlock& codeBlock;
};

//////////////////////////////////////////////////////////////////////

struct Label : private boost::noncopyable {
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

  void jmp(X64Assembler& a) {
    addJump(&a, Branch::Jmp);
    a.jmp(m_address ? m_address : a.frontier());
  }

  void jmp8(X64Assembler& a) {
    addJump(&a, Branch::Jmp8);
    a.jmp8(m_address ? m_address : a.frontier());
  }

  void jcc(X64Assembler& a, ConditionCode cc) {
    addJump(&a, Branch::Jcc);
    a.jcc(cc, m_address ? m_address : a.frontier());
  }

  void jcc8(X64Assembler& a, ConditionCode cc) {
    addJump(&a, Branch::Jcc8);
    a.jcc8(cc, m_address ? m_address : a.frontier());
  }

  void call(X64Assembler& a) {
    addJump(&a, Branch::Call);
    a.call(m_address ? m_address : a.frontier());
  }

  void jmpAuto(X64Assembler& a) {
    assert(m_address);
    auto delta = m_address - (a.frontier() + 2);
    if (deltaFits(delta, sz::byte)) {
      jmp8(a);
    } else {
      jmp(a);
    }
  }

  void jccAuto(X64Assembler& a, ConditionCode cc) {
    assert(m_address);
    auto delta = m_address - (a.frontier() + 2);
    if (deltaFits(delta, sz::byte)) {
      jcc8(a, cc);
    } else {
      jcc(a, cc);
    }
  }

  friend void asm_label(X64Assembler& a, Label& l) {
    assert(!l.m_address && !l.m_a && "Label was already set");
    l.m_a = &a;
    l.m_address = a.frontier();
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
};

inline void X64Assembler::jmp(Label& l) { l.jmp(*this); }
inline void X64Assembler::jmp8(Label& l) { l.jmp8(*this); }
inline void X64Assembler::jcc(ConditionCode c, Label& l) { l.jcc(*this, c); }
inline void X64Assembler::jcc8(ConditionCode c, Label& l) { l.jcc8(*this, c); }
inline void X64Assembler::call(Label& l) { l.call(*this); }

#define CC(nm, code)                                                    \
  inline void X64Assembler::j##nm(Label& l) { l.jcc(*this, code); }     \
  inline void X64Assembler::j##nm##8(Label& l) { l.jcc8(*this, code); }
  CCS
#undef CC

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

struct DecodedInstruction {
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
};

#undef TRACEMOD
#undef logical_const
#undef CCS

}}

#endif
