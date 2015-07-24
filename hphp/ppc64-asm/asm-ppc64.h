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

#ifndef INCLUDE_ASM_PPC64_H_
#define INCLUDE_ASM_PPC64_H_

#include <cstdint>
#include <cassert>
#include <vector>
#include <iostream>

#include "hphp/util/data-block.h"
#include "hphp/runtime/vm/jit/types.h"

#include "hphp/ppc64-asm/isa-ppc64.h"

namespace ppc64_asm {

enum class RegisterType {
    kInvalid = 0,
    kConditionRegister,
    kLinkRegister,
    kCountRegister,
    kGeneralPurpouseRegister,
    kFixedPointExceptionRegister,
    kFloatPointStatusControlRegister,
    kFloatPointRegister,
    kVectorStatusControlRegister,
    kVectorRegister,
    kVectorScalarRegister,
    kVRSaveRegister,
    kEmbeddedRegister,
    kAccumulatorRegister,
    kSignalProcessingEmbeddedFPStatusRegister,
};

class MemoryRef; // Forward class declaration

/**
 * This is a Simple Register class. This is based on
 * X64 assembler of HHVM
 */
class SimpleRegister {
public:
   explicit constexpr SimpleRegister(int rn) : rn_(rn){}
   explicit constexpr operator uint32_t() const { return rn_; }

   MemoryRef operator[](intptr_t disp) const;
   MemoryRef operator[](SimpleRegister) const;

   constexpr bool operator==(SimpleRegister reg) const { return rn_ == reg.rn_; }
   constexpr bool operator!=(SimpleRegister reg) const { return rn_ != reg.rn_; }
private:
   uint32_t rn_;
};

typedef SimpleRegister Reg64;
enum class RegNumber : uint32_t {};

RegNumber rn(Reg64 r)  { return RegNumber(uint32_t(r)); }
RegNumber rn(int n) {  return RegNumber(uint32_t(n)); }


/**
* This class is used to represent address modes to load/store instructions
*/

/*
  The PowerPC architecture supports the following simple addressing modes for memory
access instructions: 
•    EA = (RA|0) (register indirect) (D-Form)
•    EA = (RA|0) + immediate(register indirect with immediate index) (D-Form/DS-Form/DQ-Form)
•    EA = (RA|0) + RB (register indirect with index) (X-Form)
     For Branch instructions:
•    EA = immediate (absolute) (I-Form)
•    EA = PC + immediate (relative) (I-Form ?)
•    EA = EA - LR/CR (B-Form)
*/

/*
 This is Displacement Register Address mode or DForm address mode.
 This class will overload the operators () and [] to allow use the register 
 like pointers
*/
class DFormAddressMode {

};

struct XFormAddressMode {

};

class MemoryRef {

};

// inline MemoryRef SimpleRegister::operator[](const ptrdiff_t offset) const {
//   return MemoryRef { *this, offset };
// }

// inline MemoryRef SimpleRegister::operator[](const SimpleRegister& offset) const {
//   return MemoryRef { *this, offset };
// }


/**
 * Real register class can hold data. Can be used for debug purpose or for decoding
 * an interpret a instruction
 */
template<typename T>
class Register {
public:
   
   constexpr Register() : type_(RegisterType::kInvalid), register_number_(0), data_(0){}
   constexpr Register(unsigned rn): type_(RegisterType::kInvalid), register_number_(rn), data_(0){}
   constexpr Register(unsigned rn, RegisterType type) : type_(type), register_number_(rn), data_(0){}

   RegisterType type() const { return type_; }
   unsigned reg_number() const { return register_number_; }
   bool is_valid() const { return (type_ != RegisterType::kInvalid); }
   unsigned size() { return sizeof(T); }

   T read() { return data_; }
   void write(T data) { data_ = data; }; 


   ~Register(){}
protected:
   RegisterType type_;
   unsigned register_number_;
   T data_;
};

template<typename T2>
class RegisterBank {
public:
    RegisterBank(unsigned sz) : size_(sz){
    registers_ = new std::vector<Register<T2>* >(sz);
   }

   RegisterBank(unsigned sz, RegisterType type) : size_(sz){
    registers_ = new std::vector<Register<T2>* >(sz);
    unsigned rn = 0;
    for (auto& reg : *registers_) {
        reg = new Register<T2>(rn, type);
        rn++;
    }
   }

   unsigned size() { return size_; }

   ~RegisterBank(){
      for (auto& reg : *registers_) {
          delete reg;
      }
      delete registers_;
     registers_ = nullptr;
   }

private:
    RegisterBank(); //Disallow default constructor
    unsigned size_;
    std::vector<Register<T2>* >* registers_; //TODO(IBM): use smart_ptr
};


/*
 TODO(IBM) We need to check if we can have a 1:1 mapping between vasm opcodes: jccs, jmps, call
 if don't we need to implement this like ARM calling a backend to squash the N instructions to perform
 vasm operations into one.
*/
class Label {
public:
  Label() {}
  ~Label() {}

friend class Assembler;

private:
  void addJump(){}
};


class Assembler {
public:
   explicit Assembler(HPHP::CodeBlock& cb) : codeBlock(cb) {}
   ~Assembler(){}

   HPHP::CodeBlock& code() const { return codeBlock; }

   HPHP::CodeAddress base() const {
     return codeBlock.base();
   }

   HPHP::CodeAddress frontier() const {
     return codeBlock.frontier();
   }

   void setFrontier(HPHP::CodeAddress newFrontier) {
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

   bool contains(HPHP::CodeAddress addr) const {
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

  //PPC64 ISA - Only Fixed Point instructions has been implemented
  void add(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addc(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addco(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void adde(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addeo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addi(const Reg64& rt, const Reg64& ra, uint16_t imm);
  void addic(const Reg64& rt, const Reg64& ra, uint16_t imm, bool rc);
  void addis(const Reg64& rt, const Reg64& ra, uint16_t imm);
  void addme(const Reg64& rt, const Reg64& ra, bool rc);
  void addmeo(const Reg64& rt, const Reg64& ra, bool rc);
  void addo(const Reg64& rt, const Reg64& ra, const Reg64& rb, bool rc);
  void addze(const Reg64& rt, const Reg64& ra, bool rc);
  void addzeo(const Reg64& rt, const Reg64& ra, bool rc);
  void and_(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void andc(const Reg64& rs, const Reg64& ra, const Reg64& rb, bool rc);
  void andi(const Reg64& rs, const Reg64& ra, uint16_t imm);
  void andis(const Reg64& rs, const Reg64& ra, uint16_t imm);
  void b(uint32_t target_addr);
  void ba(uint32_t target_addr);
  void bl(uint32_t target_addr);
  void bla(uint32_t target_addr);
  void bc(uint8_t bo, uint8_t bi, uint16_t target_addr);
  void bca(uint8_t bo, uint8_t bi, uint16_t target_addr);
  void bcctr(uint8_t bo, uint8_t bi, uint16_t bh);
  void bcctrl(uint8_t bo, uint8_t bi, uint16_t bh);
  void bcl(uint8_t bo, uint8_t bi, uint16_t target_addr);
  void bcla(uint8_t bo, uint8_t bi, uint16_t target_addr);
  void bclr(uint8_t bo, uint8_t bi, uint16_t bh);
  void bclrl(uint8_t bo, uint8_t bi, uint16_t bh);
  void bctar(uint8_t bo, uint8_t bi, uint16_t bh);
  void bctarl(uint8_t bo, uint8_t bi, uint16_t bh);
  void bpermd(const Reg64& rs, const Reg64& ra, const Reg64& rv);
  void cmp(uint16_t bf, bool l, Reg64& ra, Reg64& rb);
  void cmpi(uint16_t bf, bool l, Reg64& ra, uint16_t imm);
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
  void lbz(const Reg64& rt, const Reg64& rb, uint16_t imm);
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
  void lwz(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lwzu(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lwzux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lwzx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lwa(const Reg64& rt, const Reg64& rb, uint16_t imm);
  void lwaux(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lwax(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void lwbrx(const Reg64& rt, const Reg64& ra, const Reg64& rb);
  void mcrf(uint16_t bf, uint16_t bfa);
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
  void ori(const Reg64& rs, const Reg64& ra, uint16_t imm);
  void oris(const Reg64& rs, const Reg64& ra, uint16_t imm);
  void popcntb(const Reg64& ra, const Reg64& rs);
  void popcntd(const Reg64& ra, const Reg64& rs);
  void popcntw(const Reg64& ra, const Reg64& rs);
  void prtyd(const Reg64& ra, const Reg64& rs);
  void prtyw(const Reg64& ra, const Reg64& rs);
  void rldcl(const Reg64& ra, const Reg64& rs, const Reg64& rb, uint8_t mb, bool rc);
  void rldcr(const Reg64& ra, const Reg64& rs,  const Reg64& rb, uint8_t mb, bool rc);
  void rldic(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, bool rc);
  void rldicl(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, bool rc);
  void rldicr(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, bool rc);
  void rldimi(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, bool rc);
  void rlwimi(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, uint16_t me, bool rc);
  void rlwinm(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, uint16_t me, bool rc);
  void rlwnm(const Reg64& ra, const Reg64& rs, uint8_t sh, uint8_t mb, uint16_t me, bool rc);
  void sc(uint16_t lev);
  void sld(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void slw(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void srad(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void sraw(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void srawi(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void srd(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void srw(const Reg64& ra, const Reg64& rs, const Reg64& rb, bool rc);
  void stb(const Reg64& rt, const Reg64& rb, uint16_t imm);
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
  void xori(const Reg64& rs, const Reg64& ra, uint16_t imm);
  void xoris(const Reg64& rs, const Reg64& ra, uint16_t imm);

  //Unimplemented Instructions
  void addg6s() {}
  void bcdadd() {}
  void bcdsub() {}
  void brinc() {}
  void cbcdtd() {}
  void cdtbcd() {}
  void clrbhrb() {}
  void dadd() {}
  void daddq() {}
  void dcba() {}
  void dcbf() {}
  void dcbfep() {}
  void dcbi() {}
  void dcblc() {}
  void dcblq() {}
  void dcbst() {}
  void dcbstep() {}
  void dcbt() {}
  void dcbtep() {}
  void dcbtls() {}
  void dcbtst() {}
  void dcbtstep() {}
  void dcbtstls() {}
  void dcbz() {}
  void dcbzep() {}
  void dcffix() {}
  void dcffixq() {}
  void dci() {}
  void dcmpo() {}
  void dcmpoq() {}
  void dcmpu() {}
  void dcmpuq() {}
  void dcread() {}
  void dctdp() {}
  void dctfix() {}
  void dctfixq() {}
  void dctqpq() {}
  void ddedpd() {}
  void ddedpdq() {}
  void ddiv() {}
  void ddivq() {}
  void denbcd() {}
  void denbcdq() {}
  void diex() {}
  void diexq() {}
  void dlmzb() {}
  void dmul() {}
  void dmulq() {}
  void dnh() {}
  void doze() {}
  void dqua() {}
  void dquai() {}
  void dquaiq() {}
  void dquaq() {}
  void drdpq() {}
  void drintn() {}
  void drintnq() {}
  void drintx() {}
  void drintxq() {}
  void drrnd() {}
  void drrndq() {}
  void drsp() {}
  void dscli() {}
  void dscliq() {}
  void dscri() {}
  void dscriq() {}
  void dsn() {}
  void dsub() {}
  void dsubq() {}
  void dtstdc() {}
  void dtstdcq() {}
  void dtstdg() {}
  void dtstdgq() {}
  void dtstex() {}
  void dtstexq() {}
  void dtstsf() {}
  void dtstsfq() {}
  void dxex() {}
  void dxexq() {}
  void eciwx() {}
  void ecowx() {}
  void efdabs() {}
  void efdadd() {}
  void efdcfs() {}
  void efdcfsf() {}
  void efdcfsi() {}
  void efdcfsid() {}
  void efdcfuf() {}
  void efdcfui() {}
  void efdcfuid() {}
  void efdcmpeq() {}
  void efdcmpgt() {}
  void efdcmplt() {}
  void efdctsf() {}
  void efdctsi() {}
  void efdctsidz() {}
  void efdctsiz() {}
  void efdctuf() {}
  void efdctui() {}
  void efdctuidz() {}
  void efdctuiz() {}
  void efddiv() {}
  void efdmul() {}
  void efdnabs() {}
  void efdneg() {}
  void efdsub() {}
  void efdtsteq() {}
  void efdtstgt() {}
  void efdtstlt() {}
  void efsabs() {}
  void efsadd() {}
  void efscfd() {}
  void efscfsf() {}
  void efscfsi() {}
  void efscfuf() {}
  void efscfui() {}
  void efscmpeq() {}
  void efscmpgt() {}
  void efscmplt() {}
  void efsctsf() {}
  void efsctsi() {}
  void efsctsiz() {}
  void efsctuf() {}
  void efsctui() {}
  void efsctuiz() {}
  void efsdiv() {}
  void efsmul() {}
  void efsnabs() {}
  void efsneg() {}
  void efssub() {}
  void efststeq() {}
  void efststgt() {}
  void efststlt() {}
  void ehpriv() {}
  void eieio() {}
  void evabs() {}
  void evaddiw() {}
  void evaddsmiaaw() {}
  void evaddssiaaw() {}
  void evaddumiaaw() {}
  void evaddusiaaw() {}
  void evaddw() {}
  void evand() {}
  void evandc() {}
  void evcmpeq() {}
  void evcmpgts() {}
  void evcmpgtu() {}
  void evcmplts() {}
  void evcmpltu() {}
  void evcntlsw() {}
  void evcntlzw() {}
  void evdivws() {}
  void evdivwu() {}
  void eveqv() {}
  void evextsb() {}
  void evextsh() {}
  void evfsabs() {}
  void evfsadd() {}
  void evfscfsf() {}
  void evfscfsi() {}
  void evfscfuf() {}
  void evfscfui() {}
  void evfscmpeq() {}
  void evfscmpgt() {}
  void evfscmplt() {}
  void evfsctsf() {}
  void evfsctsi() {}
  void evfsctsiz() {}
  void evfsctuf() {}
  void evfsctui() {}
  void evfsctuiz() {}
  void evfsdiv() {}
  void evfsmul() {}
  void evfsnabs() {}
  void evfsneg() {}
  void evfssub() {}
  void evfststeq() {}
  void evfststgt() {}
  void evfststlt() {}
  void evldd() {}
  void evlddepx() {}
  void evlddx() {}
  void evldh() {}
  void evldhx() {}
  void evldw() {}
  void evldwx() {}
  void evlhhesplat() {}
  void evlhhesplatx() {}
  void evlhhossplat() {}
  void evlhhossplatx() {}
  void evlhhousplat() {}
  void evlhhousplatx() {}
  void evlwhe() {}
  void evlwhex() {}
  void evlwhos() {}
  void evlwhosx() {}
  void evlwhou() {}
  void evlwhoux() {}
  void evlwhsplat() {}
  void evlwhsplatx() {}
  void evlwwsplat() {}
  void evlwwsplatx() {}
  void evmergehi() {}
  void evmergehilo() {}
  void evmergelo() {}
  void evmergelohi() {}
  void evmhegsmfaa() {}
  void evmhegsmfan() {}
  void evmhegsmiaa() {}
  void evmhegsmian() {}
  void evmhegumiaa() {}
  void evmhegumian() {}
  void evmhesmf() {}
  void evmhesmfa() {}
  void evmhesmfaaw() {}
  void evmhesmfanw() {}
  void evmhesmi() {}
  void evmhesmia() {}
  void evmhesmiaaw() {}
  void evmhesmianw() {}
  void evmhessf() {}
  void evmhessfa() {}
  void evmhessfaaw() {}
  void evmhessfanw() {}
  void evmhessiaaw() {}
  void evmhessianw() {}
  void evmheumi() {}
  void evmheumia() {}
  void evmheumiaaw() {}
  void evmheumianw() {}
  void evmheusiaaw() {}
  void evmheusianw() {}
  void evmhogsmfaa() {}
  void evmhogsmfan() {}
  void evmhogsmiaa() {}
  void evmhogsmian() {}
  void evmhogumiaa() {}
  void evmhogumian() {}
  void evmhosmf() {}
  void evmhosmfa() {}
  void evmhosmfaaw() {}
  void evmhosmfanw() {}
  void evmhosmi() {}
  void evmhosmia() {}
  void evmhosmiaaw() {}
  void evmhosmianw() {}
  void evmhossf() {}
  void evmhossfa() {}
  void evmhossfaaw() {}
  void evmhossfanw() {}
  void evmhossiaaw() {}
  void evmhossianw() {}
  void evmhoumi() {}
  void evmhoumia() {}
  void evmhoumiaaw() {}
  void evmhoumianw() {}
  void evmhousiaaw() {}
  void evmhousianw() {}
  void evmra() {}
  void evmwhsmf() {}
  void evmwhsmfa() {}
  void evmwhsmi() {}
  void evmwhsmia() {}
  void evmwhssf() {}
  void evmwhssfa() {}
  void evmwhumi() {}
  void evmwhumia() {}
  void evmwlsmiaaw() {}
  void evmwlsmianw() {}
  void evmwlssiaaw() {}
  void evmwlssianw() {}
  void evmwlumi() {}
  void evmwlumia() {}
  void evmwlumiaaw() {}
  void evmwlumianw() {}
  void evmwlusiaaw() {}
  void evmwlusianw() {}
  void evmwsmf() {}
  void evmwsmfa() {}
  void evmwsmfaa() {}
  void evmwsmfan() {}
  void evmwsmi() {}
  void evmwsmia() {}
  void evmwsmiaa() {}
  void evmwsmian() {}
  void evmwssf() {}
  void evmwssfa() {}
  void evmwssfaa() {}
  void evmwssfan() {}
  void evmwumi() {}
  void evmwumia() {}
  void evmwumiaa() {}
  void evmwumian() {}
  void evnand() {}
  void evneg() {}
  void evnor() {}
  void evor() {}
  void evorc() {}
  void evrlw() {}
  void evrlwi() {}
  void evrndw() {}
  void evsel() {}
  void evslw() {}
  void evslwi() {}
  void evsplatfi() {}
  void evsplati() {}
  void evsrwis() {}
  void evsrwiu() {}
  void evsrws() {}
  void evsrwu() {}
  void evstdd() {}
  void evstddepx() {}
  void evstddx() {}
  void evstdh() {}
  void evstdhx() {}
  void evstdw() {}
  void evstdwx() {}
  void evstwhe() {}
  void evstwhex() {}
  void evstwho() {}
  void evstwhox() {}
  void evstwwe() {}
  void evstwwex() {}
  void evstwwo() {}
  void evstwwox() {}
  void evsubfsmiaaw() {}
  void evsubfssiaaw() {}
  void evsubfumiaaw() {}
  void evsubfusiaaw() {}
  void evsubfw() {}
  void evsubifw() {}
  void evxor() {}
  void fabs() {}
  void fadd() {}
  void fadds() {}
  void fcfid() {}
  void fcfids() {}
  void fcfidu() {}
  void fcfidus() {}
  void fcmpo() {}
  void fcmpu() {}
  void fcpsgn() {}
  void fctid() {}
  void fctidu() {}
  void fctiduz() {}
  void fctidz() {}
  void fctiw() {}
  void fctiwu() {}
  void fctiwuz() {}
  void fctiwz() {}
  void fdiv() {}
  void fdivs() {}
  void fmadd() {}
  void fmadds() {}
  void fmr() {}
  void fmrgew() {}
  void fmrgow() {}
  void fmsub() {}
  void fmsubs() {}
  void fmul() {}
  void fmuls() {}
  void fnabs() {}
  void fneg() {}
  void fnmadd() {}
  void fnmadds() {}
  void fnmsub() {}
  void fnmsubs() {}
  void fre() {}
  void fres() {}
  void frim() {}
  void frin() {}
  void frip() {}
  void friz() {}
  void frsp() {}
  void frsqrte() {}
  void fsel() {}
  void fsqrt() {}
  void fsqrts() {}
  void fsub() {}
  void fsubs() {}
  void ftdiv() {}
  void ftsqrt() {}
  void hrfid() {}
  void icbi() {}
  void icbiep() {}
  void icblc() {}
  void icblq() {}
  void icbt() {}
  void icbtls() {}
  void ici() {}
  void icread() {}
  void isync() {}
  void lbarx() {}
  void lbdx() {}
  void lbepx() {}
  void lbzcix() {}
  void ldarx() {}
  void ldcix() {}
  void lddx() {}
  void ldepx() {}
  void lfd() {}
  void lfddx() {}
  void lfdepx() {}
  void lfdp() {}
  void lfdpx() {}
  void lfdu() {}
  void lfdux() {}
  void lfdx() {}
  void lfiwax() {}
  void lfiwzx() {}
  void lfs() {}
  void lfsu() {}
  void lfsux() {}
  void lfsx() {}
  void lharx() {}   
  void lvebx() {}
  void lvehx() {}
  void lvepx() {}
  void lvepxl() {}
  void lvewx() {}
  void lvsl() {}
  void lvsr() {}
  void lvx() {}
  void lvxl() {}
  void lwarx() {}
  void lwdx() {}
  void lwepx() {}
  void lwzcix() {}
  void lhdx() {}
  void lhepx() {}
  void lhzcix() {}
  void lqarx() {}
  void lxsdx() {}
  void lxsiwax() {}
  void lxsiwzx() {}
  void lxsspx() {}
  void lxvdsx() {}
  void lxvdx() {}
  void lxvwx() {}
  void macchw() {}
  void macchwo() {}
  void macchws() {}
  void macchwso() {}
  void macchwsu() {}
  void macchwsuo() {}
  void macchwu() {}
  void macchwuo() {}
  void machhw() {}
  void machhwo() {}
  void machhws() {}
  void machhwso() {}
  void machhwsu() {}
  void machhwsuo() {}
  void machhwu() {}
  void machhwuo() {}
  void maclhw() {}
  void maclhwo() {}
  void maclhws() {}
  void maclhwso() {}
  void maclhwsu() {}
  void maclhwsuo() {}
  void maclhwu() {}
  void maclhwuo() {}
  void mbar() {}
  void mcrfs() {}
  void mcrxr() {}
  void mfbhrbe() {}
  void mfcr() {}
  void mfdcr() {}
  void mfdcrux () {}
  void mfdcrx() {}
  void mffs() {}
  void mfocrf() {}
  void mfpmr () {}
  void mfsr() {}
  void mfsrin() {}
  void mftbmfvscr   () {}
  void mfvsrd() {}
  void mfvsrwz() {}
  void msgclr() {}
  void msgclrp() {}
  void msgsnd() {}
  void msgsndp() {}
  void mtcrf() {}
  void mtdcr() {}
  void mtdcrux() {}
  void mtdcrx () {}
  void mtfsb0() {}
  void mtfsb1() {}
  void mtfsf() {}
  void mtfsfi() {}
  void mtsr() {}
  void mtsrin() {}
  void mtvscr() {}
  void mtvsrd() {}
  void mtvsrwa() {}
  void mtvsrwz() {}
  void mulchw() {}
  void mulchwu() {}
  void mulhhw() {}
  void mulhhwu() {}
  void mullhw() {}
  void mullhwu() {}
  void nap() {}
  void nmacchw() {}
  void nmacchwo() {}
  void nmacchws() {}
  void nmacchwso() {}
  void nmachhw() {}
  void nmachhwo() {}
  void nmachhws() {}
  void nmachhwso() {}
  void nmaclhw() {}
  void nmaclhwo() {}
  void nmaclhws() {}
  void nmaclhwso() {}
  void rfci() {}
  void rfdi() {}
  void rfebb() {}
  void rfgi() {}
  void rfi() {}
  void rfid() {}
  void rfmci() {}
  void rvwinkle() {}
  void rvwinklesc() {}
  void slbfee() {}
  void slbia() {}
  void slbie() {}
  void slbmfee() {}
  void slbmfev() {}
  void slbmte() {}
  void sleep() {}
  void sradi() {}
  void stbcix() {}
  void stbcx() {}
  void stbdx() {}
  void stbepx() {}
  void stdcix() {}
  void stdcx() {}
  void stddx() {}
  void stdepx() {}
  void stfd() {}
  void stfddx() {}
  void stfdepx() {}
  void stfdp() {}
  void stfdpx() {}
  void stfdu() {}
  void stfdux() {}
  void stfdx() {}
  void stfiwx() {}
  void stfs() {}
  void stfsu() {}
  void stfsux() {}
  void stfsx() {}
  void sthcix() {}
  void sthcx() {}
  void sthdx() {}
  void sthepx() {}
  void stqcx() {}
  void stvebx() {}
  void stvehx() {}
  void stvepx() {}
  void stvepxl() {}
  void stvewx() {}
  void stvx() {}
  void stvxl() {}
  void stwcix() {}
  void stwcx() {}
  void stwdx() {}
  void stwepx() {}
  void stxsdx() {}
  void stxsiwx() {}
  void stxsspx() {}
  void stxvd2x() {}
  void stxvw4x() {}
  void sync() {}
  void tabort() {}
  void tabortdc() {}
  void tabortdci() {}
  void tabortwc() {}
  void tabortwci() {}
  void tbegin() {}
  void tcheck() {}
  void tend() {}
  void tlbia() {}
  void tlbie() {}
  void tlbiel() {}
  void tlbilx() {}
  void tlbivax() {}
  void tlbre() {}
  void tlbsrx() {}
  void tlbsx() {}
  void tlbsync() {}
  void tlbwe() {}
  void trechkpt() {}
  void treclaim() {}
  void tsr() {}
  void vaddcuq() {}
  void vaddcuw() {}
  void vaddecuq() {}
  void vaddeuqm() {}
  void vaddfp() {}
  void vaddsbs() {}
  void vaddshs() {}
  void vaddsws() {}
  void vaddubm() {}
  void vaddubs() {}
  void vaddudm() {}
  void vadduhm() {}
  void vadduhs() {}
  void vadduqm() {}
  void vadduwm() {}
  void vadduws() {}
  void vand() {}
  void vandc() {}
  void vavgsb() {}
  void vavgsh() {}
  void vavgsw() {}
  void vavgub() {}
  void vavguh() {}
  void vavguw() {}
  void vbpermq() {}
  void vcfsx() {}
  void vcfux() {}
  void vcipher() {}
  void vcipherlast() {}
  void vclzb() {}
  void vclzd() {}
  void vclzh() {}
  void vclzw() {}
  void vcmpbfp() {}
  void vcmpeqfp() {}
  void vcmpequb() {}
  void vcmpequd() {}
  void vcmpequh() {}
  void vcmpequw() {}
  void vcmpgefp() {}
  void vcmpgtfp() {}
  void vcmpgtsb() {}
  void vcmpgtsd() {}
  void vcmpgtsh() {}
  void vcmpgtsw() {}
  void vcmpgtub() {}
  void vcmpgtud() {}
  void vcmpgtuh() {}
  void vcmpgtuw() {}
  void vctsxs() {}
  void vctuxs() {}
  void veqv() {}
  void vexptefp() {}
  void vgbbd() {}
  void vlogefp() {}
  void vmaddfp() {}
  void vmaxfp() {}
  void vmaxsb() {}
  void vmaxsd() {}
  void vmaxsh() {}
  void vmaxsw() {}
  void vmaxub() {}
  void vmaxud() {}
  void vmaxuh() {}
  void vmaxuw() {}
  void vmhaddshs() {}
  void vmhraddshs() {}
  void vminfp() {}
  void vminsb() {}
  void vminsd() {}
  void vminsh() {}
  void vminsw() {}
  void vminub() {}
  void vminud() {}
  void vminuh() {}
  void vminuw() {}
  void vmladduhm() {}
  void vmrgew() {}
  void vmrghb() {}
  void vmrghh() {}
  void vmrghw() {}
  void vmrglb() {}
  void vmrglh() {}
  void vmrglw() {}
  void vmrgow() {}
  void vmsummbm() {}
  void vmsumshm() {}
  void vmsumshs() {}
  void vmsumubm() {}
  void vmsumuhm() {}
  void vmsumuhs() {}
  void vmulesb() {}
  void vmulesh() {}
  void vmulesw() {}
  void vmuleub() {}
  void vmuleuh() {}
  void vmuleuw() {}
  void vmulosb() {}
  void vmulosh() {}
  void vmulosw() {}
  void vmuloub() {}
  void vmulouh() {}
  void vmulouw() {}
  void vmuluwm() {}
  void vnand() {}
  void vncipher() {}
  void vncipherlast() {}
  void vnmsubfp() {}
  void vnor() {}
  void vor() {}
  void vorc() {}
  void vperm() {}
  void vpermxor() {}
  void vpkpx() {}
  void vpksdss() {}
  void vpksdus() {}
  void vpkshss() {}
  void vpkshus() {}
  void vpkswss() {}
  void vpkswus() {}
  void vpkudum() {}
  void vpkudus() {}
  void vpkuhum() {}
  void vpkuhus() {}
  void vpkuwum() {}
  void vpkuwus() {}
  void vpmsumb() {}
  void vpmsumd() {}
  void vpmsumh() {}
  void vpmsumw() {}
  void vpopcntb() {}
  void vpopcntd() {}
  void vpopcnth() {}
  void vpopcntw() {}
  void vrefp() {}
  void vrfim() {}
  void vrfin() {}
  void vrfip() {}
  void vrfiz() {}
  void vrlb() {}
  void vrld() {}
  void vrlh() {}
  void vrlw() {}
  void vrsqrtefp() {}
  void vsbox() {}
  void vsel() {}
  void vshasigmad() {}
  void vshasigmaw() {}
  void vsl() {}
  void vslb() {}
  void vsld() {}
  void vsldoi() {}
  void vslh() {}
  void vslo() {}
  void vslw() {}
  void vspltb() {}
  void vsplth() {}
  void vspltisb() {}
  void vspltish() {}
  void vspltisw() {}
  void vspltw() {}
  void vsr() {}
  void vsrab() {}
  void vsrad() {}
  void vsrah() {}
  void vsraw() {}
  void vsrb() {}
  void vsrd() {}
  void vsrh() {}
  void vsro() {}
  void vsrw() {}
  void vsubcuq() {}
  void vsubcuw() {}
  void vsubecuq() {}
  void vsubeuqm() {}
  void vsubfp() {}
  void vsubsbs() {}
  void vsubshs() {}
  void vsubsws() {}
  void vsububm() {}
  void vsububs() {}
  void vsubudm() {}
  void vsubuhm() {}
  void vsubuhs() {}
  void vsubuqm() {}
  void vsubuwm() {}
  void vsubuws() {}
  void vsum2sws() {}
  void vsum4sbs() {}
  void vsum4shs() {}
  void vsum4ubs() {}
  void vsumsws() {}
  void vupkhpx() {}
  void vupkhsb() {}
  void vupkhsh() {}
  void vupkhsw() {}
  void vupklpx() {}
  void vupklsb() {}
  void vupklsh() {}
  void vupklsw() {}
  void vxor() {}
  void wait() {}
  void wrtee() {}
  void wrteei() {}
  void xsabsdp() {}
  void xsadddp() {}
  void xsaddsp() {}
  void xscmpodp() {}
  void xscmpudp() {}
  void xscpsgndp() {}
  void xscvdpsp() {}
  void xscvdpspn() {}
  void xscvdpsxds() {}
  void xscvdpsxws() {}
  void xscvdpuxds() {}
  void xscvdpuxws() {}
  void xscvspdp() {}
  void xscvspdpn() {}
  void xscvsxddp() {}
  void xscvsxdsp() {}
  void xscvuxddp() {}
  void xscvuxdsp() {}
  void xsdivdp() {}
  void xsdivsp() {}
  void xsmaddadp() {}
  void xsmaddasp() {}
  void xsmaddmdp() {}
  void xsmaddmsp() {}
  void xsmaxdp() {}
  void xsmindp() {}
  void xsmsubadp() {}
  void xsmsubasp() {}
  void xsmsubmdp() {}
  void xsmsubmsp() {}
  void xsmuldp() {}
  void xsmulsp() {}
  void xsnabsdp() {}
  void xsnegdp() {}
  void xsnmaddadp() {}
  void xsnmaddasp() {}
  void xsnmaddmdp() {}
  void xsnmaddmsp() {}
  void xsnmsubadp() {}
  void xsnmsubasp() {}
  void xsnmsubmdp() {}
  void xsnmsubmsp() {}
  void xsrdpi() {}
  void xsrdpic() {}
  void xsrdpim() {}
  void xsrdpip() {}
  void xsrdpiz() {}
  void xsredp() {}
  void xsresp() {}
  void xsrsp() {}
  void xsrsqrtedp() {}
  void xsrsqrtesp() {}
  void xssqrtdp() {}
  void xssqrtsp() {}
  void xssubdp() {}
  void xssubsp() {}
  void xstdivdp() {}
  void xstsqrtdp() {}
  void xvabsdp() {}
  void xvabssp() {}
  void xvadddp() {}
  void xvaddsp() {}
  void xvcmpeqdp() {}
  void xvcmpeqsp() {}
  void xvcmpgedp() {}
  void xvcmpgesp() {}
  void xvcmpgtdp() {}
  void xvcmpgtsp() {}
  void xvcpsgndp() {}
  void xvcpsgnsp() {}
  void xvcvdpsp() {}
  void xvcvdpsxds() {}
  void xvcvdpsxws() {}
  void xvcvdpuxds() {}
  void xvcvdpuxws() {}
  void xvcvspdp() {}
  void xvcvspsxds() {}
  void xvcvspsxws() {}
  void xvcvspuxds() {}
  void xvcvspuxws() {}
  void xvcvsxddp() {}
  void xvcvsxdsp() {}
  void xvcvsxwdp() {}
  void xvcvsxwsp() {}
  void xvcvuxddp() {}
  void xvcvuxdsp() {}
  void xvcvuxwdp() {}
  void xvcvuxwsp() {}
  void xvdivdp() {}
  void xvdivsp() {}
  void xvmaddadp() {}
  void xvmaddasp() {}
  void xvmaddmdp() {}
  void xvmaddmsp() {}
  void xvmaxdp() {}
  void xvmaxsp() {}
  void xvmindp() {}
  void xvminsp() {}
  void xvmsubadp() {}
  void xvmsubasp() {}
  void xvmsubmdp() {}
  void xvmsubmsp() {}
  void xvmuldp() {}
  void xvmulsp() {}
  void xvnabsdp() {}
  void xvnabssp() {}
  void xvnegdp() {}
  void xvnegsp() {}
  void xvnmaddadp() {}
  void xvnmaddasp() {}
  void xvnmaddmdp() {}
  void xvnmaddmsp() {}
  void xvnmsubadp() {}
  void xvnmsubasp() {}
  void xvnmsubmdp() {}
  void xvnmsubmsp() {}
  void xvrdpi() {}
  void xvrdpic() {}
  void xvrdpim() {}
  void xvrdpip() {}
  void xvrdpiz() {}
  void xvredp() {}
  void xvresp() {}
  void xvrspi() {}
  void xvrspic() {}
  void xvrspim() {}
  void xvrspip() {}
  void xvrspiz() {}
  void xvrsqrtedp() {}
  void xvrsqrtesp() {}
  void xvsqrtdp() {}
  void xvsqrtsp() {}
  void xvsubdp() {}
  void xvsubsp() {}
  void xvtdivdp() {}
  void xvtdivsp() {}
  void xvtsqrtdp() {}
  void xvtsqrtsp() {}
  void xxland() {}
  void xxlandc() {}
  void xxleqv() {}
  void xxlnand() {}
  void xxlnor() {}
  void xxlor() {}
  void xxlorc() {}
  void xxlxor() {}
  void xxmrghw() {}
  void xxmrglw() {}
  void xxpermdi() {}
  void xxsel() {}
  void xxsldwi() {}
  void xxspltw() {}

  //Extended/Synthetic PPC64 Instructions
  void blt() {}           //Extended bc 12,0,target
  void bne() {}           //Extended bc 4,10,target
  void bdnz() {}          //Extended bc 16,0,target
  void bltlr() {}         //Extended bclr 12,0,0
  void bnelr() {}         //Extended bclr 4,10,0
  void bdnzlr() {}        //Extended bclr 16,0,0
  void bltctr() {}        //Extended bcctr 12,0,0
  void bnectr() {}        //Extended bcctr 4,10,0
  void crmov() {}         //Extended cror Bx,By,By
  void crclr() {}         //Extended crxor Bx,Bx,BX
  void crnot() {}         //Extended crnor Bx,By,By
  void crset() {}         //Extended creqv Bx,Bx,Bx
  void li() {}            //Extended addi Rx,0,value
  void la() {}            //Extended addi Rx,Ry,disp
  void subi() {}          //Extended addi Rx,Ry,-value
  void lis() {}           //Extended addis Rx,0,value
  void subis() {}         //Extended addis Rx,Ry,-value
  void sub() {}           //Extended subf Rx,Rz,Ry
  void subic() {}         //Extended addic Rx,Ry,-value
  void subc() {}          //Extended subfc Rx,Rz,Ry
  void cmpdi() {}         //Extended cmpi 0,1,Rx,value
  void cmpwi() {}         //Extended cmpi 3,0,Rx,value
  void cmpd() {}          //Extended cmp 0,1,Rx,Ry
  void cmpw () {}         //Extended cmp 3,0,Rx,Ry
  void cmpldi() {}        //Extended cmpli 0,1,Rx,value
  void cmplwi() {}        //Extended cmpli 3,0,Rx,value
  void cmpld() {}         //Extended cmpl 0,1,Rx,Ry
  void cmplw() {}         //Extended cmpl 3,0,Rx,Ry
  void twgti() {}         //Extended twi 8,Rx,value
  void twllei() {}        //Extended twi 6,Rx,value
  void tweq() {}          //Extended tw 4,Rx,Ry
  void twlge(){}          //Extended tw 5,Rx,Ry
  void trap() {}          //Extended tw 31,0,0
  void tdlti() {}         //Extended tdi 16,Rx,value
  void tdnei() {}         //Extended tdi 24,Rx,value
  void tdeg() {}          //Extended td 12,Rx,Ry
  void isellt() {}        //Extended isel Rx,Ry,Rz,0
  void iselgt() {}        //Extended isel Rx,Ry,Rz,1
  void iseleq() {}        //Extended isel Rx,Ry,Rz,1
  void no_op(){}          //Extended
  void andis() {}         //Extended
  void xnop(){}           //Extended
  void mr(){}             //Extended
  void not_(){}           //Extended
  void srwi(){}           //Extended
  void clrwi(){}          //Extended
  void extwi() {}         //Extended
  void rotlw() {}         //Extended
  void inslwi() {}        //Extended
  void extrdi() {}        //Extended
  void srdi() {}          //Extended
  void clrldi() {}        //Extended
  void extldi() {}        //Extended
  void sldi() {}          //Extended
  void clrrdi() {}        //Extended
  void clrlsldi() {}      //Extended
  void rotld() {}         //Extended
  void insrdi() {}        //Extended
  void mtcr() {}          //Extended


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
                 const uint16_t imm) {

    D_form_t d_formater { 
                          imm,
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
                   const uint8_t Sh,
                   const uint8_t mb,
                   const uint8_t xop,
                   const bool sh = 0,
                   const bool rc = 0) {

      MD_form_t md_formater {
                             rc,
                             sh,
                             xop,
                             mb,
                             Sh,
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

  //TODO(IBM): Unimplemented instruction formaters
  void EmitXFXForm(){}
  void EmitXFLForm(){}
  void EmitXX1Form(){}
  void EmitXX2Form(){}
  void EmitXX3Form(){}
  void EmitXX4Form(){}
  void EmitXSForm(){}
  void EmitVAForm(){}
  void EmitVCForm(){}
  void EmitVXForm(){}
  void EmitZ23Form(){}
  void EmitEVXForm(){}
  void EmitEVSForm(){}
  void EmitZ22Form(){}

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

};

} // namespace ppc64_asm

#endif