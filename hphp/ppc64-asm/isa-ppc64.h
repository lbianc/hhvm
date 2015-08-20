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

#ifndef INCLUDE_PPC64_ISA_H_
#define INCLUDE_PPC64_ISA_H_

#include <cstdint>

namespace ppc64_asm {

typedef uint32_t PPC64Instr;

const int kDecoderSize = 1374;

enum class Form {
  kInvalid = 0,
  kXForm,
  kXOForm,
  kXSForm,
  kDForm,
  kIForm,
  kBForm,
  kSCForm,
  kDSForm,
  kDQForm,
  kXLForm,
  kXFXForm,
  kXFLForm,
  kXX1Form,
  kXX2Form,
  kXX3Form,
  kXX4Form,
  kAForm,
  kMForm,
  kMDForm,
  kMDSForm,
  kVAForm,
  kVCForm,
  kVXForm,
  kEVXForm,
  kEVSForm,
  kZ22Form,
  kZ23Form,
};

//Declare class as struct to initialize like an array
struct DecoderInfo {
private:
    uint32_t opcode_;
    Form form_;
    std::string mnemonic_;
    DecoderInfo* next_; 
public:
  DecoderInfo(uint32_t op, Form form, std::string mn) 
  : opcode_(op)
  , form_(form)
  , mnemonic_(mn)
  , next_(nullptr) 
  {}

  ~DecoderInfo(){
    delete next_;
    next_ = nullptr;
  }

  DecoderInfo() = delete;

  inline Form form() const { return form_; }
  inline uint32_t opcode() const { return opcode_; }
  inline std::string mnemonic() const { return mnemonic_; }
  void next(DecoderInfo* i) { next_ = i;}
  DecoderInfo* next() { return next_; }

  inline bool operator==(const DecoderInfo& i) {
    return (i.form() == form_ && 
            i.opcode() == opcode_ && 
            i.mnemonic() == mnemonic_); 
  }

  inline bool operator!=(const DecoderInfo& i) {
    return (i.form() != form_ || 
            i.opcode() != opcode_ || 
            i.mnemonic() != mnemonic_); 
  }
};

class DecoderTable {
public:
  DecoderTable() {
    decoder_table = new DecoderInfo*[kDecoderSize];
    for(int i = 0; i < kDecoderSize; i++)
      decoder_table[i] = nullptr;

#define D(name, op, type, mnemonic)\
  DecoderInfo instr_##name {op, type, #mnemonic};\
  SetInstruction(instr_##name)

  D(add             ,0x7C000214,    Form::kXOForm,    add);
  D(adddot          ,0x7C000215,    Form::kXOForm,    add.);
  D(addc            ,0x7C000014,    Form::kXOForm,    addc);
  D(addcdot         ,0x7C000015,    Form::kXOForm,    addc.);
  D(addco           ,0x7C000414,    Form::kXOForm,    addco);
  D(addcodot        ,0x7C000415,    Form::kXOForm,    addco.);
  D(adde            ,0x7C000114,    Form::kXOForm,    adde);
  D(addedot         ,0x7C000115,    Form::kXOForm,    adde.);
  D(addeo           ,0x7C000514,    Form::kXOForm,    addeo);
  D(addeodot        ,0x7C000515,    Form::kXOForm,    addeo.);
  D(addg6s          ,0x7C000094,    Form::kXOForm,    addg6s);
  D(addi            ,0x38000000,    Form::kDForm,     addi);
  D(addic           ,0x30000000,    Form::kDForm,     addic);
  D(addicdot        ,0x34000000,    Form::kDForm,     addic.);
  D(addis           ,0x3C000000,    Form::kDForm,     addis);
  D(addme           ,0x7C0001D4,    Form::kXOForm,    addme);
  D(addmedot        ,0x7C0001D5,    Form::kXOForm,    addme.);
  D(addmeo          ,0x7C0005D4,    Form::kXOForm,    addmeo);
  D(addmeodot       ,0x7C0005D5,    Form::kXOForm,    addmeo.);
  D(addo            ,0x7C000614,    Form::kXOForm,    addo);
  D(addodot         ,0x7C000615,    Form::kXOForm,    addo.);
  D(addze           ,0x7C000194,    Form::kXOForm,    addze);
  D(addzedot        ,0x7C000195,    Form::kXOForm,    addze.);
  D(addzeo          ,0x7C000594,    Form::kXOForm,    addzeo);
  D(addzeodot       ,0x7C000595,    Form::kXOForm,    addzeo.);
  D(and             ,0x7C000038,    Form::kXForm,     and);
  D(anddot          ,0x7C000039,    Form::kXForm,     and.);
  D(andc            ,0x7C000078,    Form::kXForm,     andc);
  D(andcdot         ,0x7C000079,    Form::kXForm,     andc.);
  D(andidot         ,0x70000000,    Form::kDForm,     andi.);
  D(andisdot        ,0x74000000,    Form::kDForm,     andis.);
  D(b               ,0x48000000,    Form::kIForm,     b);
  D(ba              ,0x48000002,    Form::kIForm,     ba);
  D(bc              ,0x40000000,    Form::kBForm,     bc);
  D(bca             ,0x40000002,    Form::kBForm,     bca);
  D(bcctr           ,0x4C000420,    Form::kXLForm,    bcctr);
  D(bcctrl          ,0x4C000421,    Form::kXLForm,    bcctrl);
  D(bcdadddot       ,0x10000401,    Form::kVXForm,    bcdadd.);
  D(bcdsubdot       ,0x10000441,    Form::kVXForm,    bcdsub.);
  D(bcl             ,0x40000001,    Form::kBForm,     bcl);
  D(bcla            ,0x40000003,    Form::kBForm,     bcla);
  D(bclr            ,0x4C000020,    Form::kXLForm,    bclr);
  D(bclrl           ,0x4C000021,    Form::kXLForm,    bclrl);
  D(bctar           ,0x4C000460,    Form::kXForm,     bctar);
  D(bctarl          ,0x4C000461,    Form::kXForm,     bctarl);
  D(bl              ,0x48000001,    Form::kIForm,     bl);
  D(bla             ,0x48000003,    Form::kIForm,     bla);
  D(bpermd          ,0x7C0001F8,    Form::kXForm,     bpermd);
  D(brinc           ,0x1000020F,    Form::kEVXForm,   brinc);
  D(cbcdtd          ,0x7C000274,    Form::kXForm,     cbcdtd);
  D(cdtbcd          ,0x7C000234,    Form::kXForm,     cdtbcd);
  D(clrbhrb         ,0x7C00035C,    Form::kXForm,     clrbhrb);
  D(cmp             ,0x7C000000,    Form::kXForm,     cmp);
  D(cmpb            ,0x7C0003F8,    Form::kXForm,     cmpb);
  D(cmpi            ,0x2C000000,    Form::kDForm,     cmpi);
  D(cmpl            ,0x7C000040,    Form::kXForm,     cmpl);
  D(cmpli           ,0x28000000,    Form::kDForm,     cmpli);
  D(cntlzd          ,0x7C000074,    Form::kXForm,     cntlzd);
  D(cntlzddot       ,0x7C000075,    Form::kXForm,     cntlzd.);
  D(cntlzw          ,0x7C000034,    Form::kXForm,     cntlzw);
  D(cntlzwdot       ,0x7C000035,    Form::kXForm,     cntlzw.);
  D(crand           ,0x4C000202,    Form::kXLForm,    crand);
  D(crandc          ,0x4C000102,    Form::kXLForm,    crandc);
  D(creqv           ,0x4C000242,    Form::kXLForm,    creqv);
  D(crnand          ,0x4C0001C2,    Form::kXLForm,    crnand);
  D(crnor           ,0x4C000042,    Form::kXLForm,    crnor);
  D(cror            ,0x4C000382,    Form::kXLForm,    cror);
  D(crorc           ,0x4C000342,    Form::kXLForm,    crorc);
  D(crxor           ,0x4C000182,    Form::kXLForm,    crxor);
  D(dadd            ,0xEC000004,    Form::kXForm,     dadd);
  D(dadddot         ,0xEC000005,    Form::kXForm,     dadd.);
  D(daddq           ,0xFC000004,    Form::kXForm,     daddq);
  D(daddqdot        ,0xFC000005,    Form::kXForm,     daddq.);
  D(dcba            ,0x7C0005EC,    Form::kXForm,     dcba);
  D(dcbf            ,0x7C0000AC,    Form::kXForm,     dcbf);
  D(dcbfep          ,0x7C0000FE,    Form::kXForm,     dcbfep);
  D(dcbi            ,0x7C0003AC,    Form::kXForm,     dcbi);
  D(dcblc           ,0x7C00030C,    Form::kXForm,     dcblc);
  D(dcblqdot        ,0x7C00034D,    Form::kXForm,     dcblq.);
  D(dcbst           ,0x7C00006C,    Form::kXForm,     dcbst);
  D(dcbstep         ,0x7C00007E,    Form::kXForm,     dcbstep);
  D(dcbt            ,0x7C00022C,    Form::kXForm,     dcbt);
  D(dcbtep          ,0x7C00027E,    Form::kXForm,     dcbtep);
  D(dcbtls          ,0x7C00014C,    Form::kXForm,     dcbtls);
  D(dcbtst          ,0x7C0001EC,    Form::kXForm,     dcbtst);
  D(dcbtstep        ,0x7C0001FE,    Form::kXForm,     dcbtstep);
  D(dcbtstls        ,0x7C00010C,    Form::kXForm,     dcbtstls);
  D(dcbz            ,0x7C0007EC,    Form::kXForm,     dcbz);
  D(dcbzep          ,0x7C0007FE,    Form::kXForm,     dcbzep);
  D(dcffix          ,0xEC000644,    Form::kXForm,     dcffix);
  D(dcffixdot       ,0xEC000645,    Form::kXForm,     dcffix.);
  D(dcffixq         ,0xFC000644,    Form::kXForm,     dcffixq);
  D(dcffixqdot      ,0xFC000645,    Form::kXForm,     dcffixq.);
  D(dci             ,0x7C00038C,    Form::kXForm,     dci);
  D(dcmpo           ,0xEC000104,    Form::kXForm,     dcmpo);
  D(dcmpoq          ,0xFC000104,    Form::kXForm,     dcmpoq);
  D(dcmpu           ,0xEC000504,    Form::kXForm,     dcmpu);
  D(dcmpuq          ,0xFC000504,    Form::kXForm,     dcmpuq);
  D(dcread          ,0x7C00028C,    Form::kXForm,     dcread);
  D(dcread2         ,0x7C0003CC,    Form::kXForm,     dcread);
  D(dctdp           ,0xEC000204,    Form::kXForm,     dctdp);
  D(dctdpdot        ,0xEC000205,    Form::kXForm,     dctdp.);
  D(dctfix          ,0xEC000244,    Form::kXForm,     dctfix);
  D(dctfixdot       ,0xEC000245,    Form::kXForm,     dctfix.);
  D(dctfixq         ,0xFC000244,    Form::kXForm,     dctfixq);
  D(dctfixqdot      ,0xFC000245,    Form::kXForm,     dctfixq.);
  D(dctqpq          ,0xFC000204,    Form::kXForm,     dctqpq);
  D(dctqpqdot       ,0xFC000205,    Form::kXForm,     dctqpq.);
  D(ddedpd          ,0xEC000284,    Form::kXForm,     ddedpd);
  D(ddedpddot       ,0xEC000285,    Form::kXForm,     ddedpd.);
  D(ddedpdq         ,0xFC000284,    Form::kXForm,     ddedpdq);
  D(ddedpdqdot      ,0xFC000285,    Form::kXForm,     ddedpdq.);
  D(ddiv            ,0xEC000444,    Form::kXForm,     ddiv);
  D(ddivdot         ,0xEC000445,    Form::kXForm,     ddiv.);
  D(ddivq           ,0xFC000444,    Form::kXForm,     ddivq);
  D(ddivqdot        ,0xFC000445,    Form::kXForm,     ddivq.);
  D(denbcd          ,0xEC000684,    Form::kXForm,     denbcd);
  D(denbcddot       ,0xEC000685,    Form::kXForm,     denbcd.);
  D(denbcdq         ,0xFC000684,    Form::kXForm,     denbcdq);
  D(denbcdqdot      ,0xFC000685,    Form::kXForm,     denbcdq.);
  D(diex            ,0xEC0006C4,    Form::kXForm,     diex);
  D(diexdot         ,0xEC0006C5,    Form::kXForm,     diex.);
  D(diexq           ,0xFC0006C4,    Form::kXForm,     diexq);
  D(diexqdot        ,0xFC0006C5,    Form::kXForm,     diexq.);
  D(divd            ,0x7C0003D2,    Form::kXOForm,    divd);
  D(divddot         ,0x7C0003D3,    Form::kXOForm,    divd.);
  D(divde           ,0x7C000352,    Form::kXOForm,    divde);
  D(divdedot        ,0x7C000353,    Form::kXOForm,    divde.);
  D(divdeo          ,0x7C000752,    Form::kXOForm,    divdeo);
  D(divdeodot       ,0x7C000753,    Form::kXOForm,    divdeo.);
  D(divdeu          ,0x7C000312,    Form::kXOForm,    divdeu);
  D(divdeudot       ,0x7C000313,    Form::kXOForm,    divdeu.);
  D(divdeuo         ,0x7C000712,    Form::kXOForm,    divdeuo);
  D(divdeuodot      ,0x7C000713,    Form::kXOForm,    divdeuo.);
  D(divdo           ,0x7C0007D2,    Form::kXOForm,    divdo);
  D(divdodot        ,0x7C0007D3,    Form::kXOForm,    divdo.);
  D(divdu           ,0x7C000392,    Form::kXOForm,    divdu);
  D(divdudot        ,0x7C000393,    Form::kXOForm,    divdu.);
  D(divduo          ,0x7C000792,    Form::kXOForm,    divduo);
  D(divduodot       ,0x7C000793,    Form::kXOForm,    divduo.);
  D(divw            ,0x7C0003D6,    Form::kXOForm,    divw);
  D(divwdot         ,0x7C0003D7,    Form::kXOForm,    divw.);
  D(divwe           ,0x7C000356,    Form::kXOForm,    divwe);
  D(divwedot        ,0x7C000357,    Form::kXOForm,    divwe.);
  D(divweo          ,0x7C000756,    Form::kXOForm,    divweo);
  D(divweodot       ,0x7C000757,    Form::kXOForm,    divweo.);
  D(divweu          ,0x7C000316,    Form::kXOForm,    divweu);
  D(divweudot       ,0x7C000317,    Form::kXOForm,    divweu.);
  D(divweuo         ,0x7C000716,    Form::kXOForm,    divweuo);
  D(divweuodot      ,0x7C000717,    Form::kXOForm,    divweuo.);
  D(divwo           ,0x7C0007D6,    Form::kXOForm,    divwo);
  D(divwodot        ,0x7C0007D7,    Form::kXOForm,    divwo.);
  D(divwu           ,0x7C000396,    Form::kXOForm,    divwu);
  D(divwudot        ,0x7C000397,    Form::kXOForm,    divwu.);
  D(divwuo          ,0x7C000796,    Form::kXOForm,    divwuo);
  D(divwuodot       ,0x7C000797,    Form::kXOForm,    divwuo.);
  D(dlmzb           ,0x7C00009C,    Form::kXForm,     dlmzb);
  D(dlmzbdot        ,0x7C00009D,    Form::kXForm,     dlmzb.);
  D(dmul            ,0xEC000044,    Form::kXForm,     dmul);
  D(dmuldot         ,0xEC000045,    Form::kXForm,     dmul.);
  D(dmulq           ,0xFC000044,    Form::kXForm,     dmulq);
  D(dmulqdot        ,0xFC000045,    Form::kXForm,     dmulq.);
  D(dnh             ,0x4C00018C,    Form::kXFXForm,   dnh);
  D(doze            ,0x4C000324,    Form::kXLForm,    doze);
  D(dqua            ,0xEC000006,    Form::kZ23Form,   dqua);
  D(dquadot         ,0xEC000007,    Form::kZ23Form,   dqua.);
  D(dquai           ,0xEC000086,    Form::kZ23Form,   dquai);
  D(dquaidot        ,0xEC000087,    Form::kZ23Form,   dquai.);
  D(dquaiq          ,0xFC000086,    Form::kZ23Form,   dquaiq);
  D(dquaiqdot       ,0xFC000087,    Form::kZ23Form,   dquaiq.);
  D(dquaq           ,0xFC000006,    Form::kZ23Form,   dquaq);
  D(dquaqdot        ,0xFC000007,    Form::kZ23Form,   dquaq.);
  D(drdpq           ,0xFC000604,    Form::kXForm,     drdpq);
  D(drdpqdot        ,0xFC000605,    Form::kXForm,     drdpq.);
  D(drintn          ,0xEC0001C6,    Form::kZ23Form,   drintn);
  D(drintndot       ,0xEC0001C7,    Form::kZ23Form,   drintn.);
  D(drintnq         ,0xFC0001C6,    Form::kZ23Form,   drintnq);
  D(drintnqdot      ,0xFC0001C7,    Form::kZ23Form,   drintnq.);
  D(drintx          ,0xEC0000C6,    Form::kZ23Form,   drintx);
  D(drintxdot       ,0xEC0000C7,    Form::kZ23Form,   drintx.);
  D(drintxq         ,0xFC0000C6,    Form::kZ23Form,   drintxq);
  D(drintxqdot      ,0xFC0000C7,    Form::kZ23Form,   drintxq.);
  D(drrnd           ,0xEC000046,    Form::kZ23Form,   drrnd);
  D(drrnddot        ,0xEC000047,    Form::kZ23Form,   drrnd.);
  D(drrndq          ,0xFC000046,    Form::kZ23Form,   drrndq);
  D(drrndqdot       ,0xFC000047,    Form::kZ23Form,   drrndq.);
  D(drsp            ,0xEC000604,    Form::kXForm,     drsp);
  D(drspdot         ,0xEC000605,    Form::kXForm,     drsp.);
  D(dscli           ,0xEC000084,    Form::kZ22Form,   dscli);
  D(dsclidot        ,0xEC000085,    Form::kZ22Form,   dscli.);
  D(dscliq          ,0xFC000084,    Form::kZ22Form,   dscliq);
  D(dscliqdot       ,0xFC000085,    Form::kZ22Form,   dscliq.);
  D(dscri           ,0xEC0000C4,    Form::kZ22Form,   dscri);
  D(dscridot        ,0xEC0000C5,    Form::kZ22Form,   dscri.);
  D(dscriq          ,0xFC0000C4,    Form::kZ22Form,   dscriq);
  D(dscriqdot       ,0xFC0000C5,    Form::kZ22Form,   dscriq.);
  D(dsn             ,0x7C0003C6,    Form::kXForm,     dsn);
  D(dsub            ,0xEC000404,    Form::kXForm,     dsub);
  D(dsubdot         ,0xEC000405,    Form::kXForm,     dsub.);
  D(dsubq           ,0xFC000404,    Form::kXForm,     dsubq);
  D(dsubqdot        ,0xFC000405,    Form::kXForm,     dsubq.);
  D(dtstdc          ,0xEC000184,    Form::kZ22Form,   dtstdc);
  D(dtstdcq         ,0xFC000184,    Form::kZ22Form,   dtstdcq);
  D(dtstdg          ,0xEC0001C4,    Form::kZ22Form,   dtstdg);
  D(dtstdgq         ,0xFC0001C4,    Form::kZ22Form,   dtstdgq);
  D(dtstex          ,0xEC000144,    Form::kXForm,     dtstex);
  D(dtstexq         ,0xFC000144,    Form::kXForm,     dtstexq);
  D(dtstsf          ,0xEC000544,    Form::kXForm,     dtstsf);
  D(dtstsfq         ,0xFC000544,    Form::kXForm,     dtstsfq);
  D(dxex            ,0xEC0002C4,    Form::kXForm,     dxex);
  D(dxexdot         ,0xEC0002C5,    Form::kXForm,     dxex.);
  D(dxexq           ,0xFC0002C4,    Form::kXForm,     dxexq);
  D(dxexqdot        ,0xFC0002C5,    Form::kXForm,     dxexq.);
  D(eciwx           ,0x7C00026C,    Form::kXForm,     eciwx);
  D(ecowx           ,0x7C00036C,    Form::kXForm,     ecowx);
  D(efdabs          ,0x100002E4,    Form::kEVXForm,   efdabs);
  D(efdadd          ,0x100002E0,    Form::kEVXForm,   efdadd);
  D(efdcfs          ,0x100002EF,    Form::kEVXForm,   efdcfs);
  D(efdcfsf         ,0x100002F3,    Form::kEVXForm,   efdcfsf);
  D(efdcfsi         ,0x100002F1,    Form::kEVXForm,   efdcfsi);
  D(efdcfsid        ,0x100002E3,    Form::kEVXForm,   efdcfsid);
  D(efdcfuf         ,0x100002F2,    Form::kEVXForm,   efdcfuf);
  D(efdcfui         ,0x100002F0,    Form::kEVXForm,   efdcfui);
  D(efdcfuid        ,0x100002E2,    Form::kEVXForm,   efdcfuid);
  D(efdcmpeq        ,0x100002EE,    Form::kEVXForm,   efdcmpeq);
  D(efdcmpgt        ,0x100002EC,    Form::kEVXForm,   efdcmpgt);
  D(efdcmplt        ,0x100002ED,    Form::kEVXForm,   efdcmplt);
  D(efdctsf         ,0x100002F7,    Form::kEVXForm,   efdctsf);
  D(efdctsi         ,0x100002F5,    Form::kEVXForm,   efdctsi);
  D(efdctsidz       ,0x100002EB,    Form::kEVXForm,   efdctsidz);
  D(efdctsiz        ,0x100002FA,    Form::kEVXForm,   efdctsiz);
  D(efdctuf         ,0x100002F6,    Form::kEVXForm,   efdctuf);
  D(efdctui         ,0x100002F4,    Form::kEVXForm,   efdctui);
  D(efdctuidz       ,0x100002EA,    Form::kEVXForm,   efdctuidz);
  D(efdctuiz        ,0x100002F8,    Form::kEVXForm,   efdctuiz);
  D(efddiv          ,0x100002E9,    Form::kEVXForm,   efddiv);
  D(efdmul          ,0x100002E8,    Form::kEVXForm,   efdmul);
  D(efdnabs         ,0x100002E5,    Form::kEVXForm,   efdnabs);
  D(efdneg          ,0x100002E6,    Form::kEVXForm,   efdneg);
  D(efdsub          ,0x100002E1,    Form::kEVXForm,   efdsub);
  D(efdtsteq        ,0x100002FE,    Form::kEVXForm,   efdtsteq);
  D(efdtstgt        ,0x100002FC,    Form::kEVXForm,   efdtstgt);
  D(efdtstlt        ,0x100002FD,    Form::kEVXForm,   efdtstlt);
  D(efsabs          ,0x100002C4,    Form::kEVXForm,   efsabs);
  D(efsadd          ,0x100002C0,    Form::kEVXForm,   efsadd);
  D(efscfd          ,0x100002CF,    Form::kEVXForm,   efscfd);
  D(efscfsf         ,0x100002D3,    Form::kEVXForm,   efscfsf);
  D(efscfsi         ,0x100002D1,    Form::kEVXForm,   efscfsi);
  D(efscfuf         ,0x100002D2,    Form::kEVXForm,   efscfuf);
  D(efscfui         ,0x100002D0,    Form::kEVXForm,   efscfui);
  D(efscmpeq        ,0x100002CE,    Form::kEVXForm,   efscmpeq);
  D(efscmpgt        ,0x100002CC,    Form::kEVXForm,   efscmpgt);
  D(efscmplt        ,0x100002CD,    Form::kEVXForm,   efscmplt);
  D(efsctsf         ,0x100002D7,    Form::kEVXForm,   efsctsf);
  D(efsctsi         ,0x100002D5,    Form::kEVXForm,   efsctsi);
  D(efsctsiz        ,0x100002DA,    Form::kEVXForm,   efsctsiz);
  D(efsctuf         ,0x100002D6,    Form::kEVXForm,   efsctuf);
  D(efsctui         ,0x100002D4,    Form::kEVXForm,   efsctui);
  D(efsctuiz        ,0x100002D8,    Form::kEVXForm,   efsctuiz);
  D(efsdiv          ,0x100002C9,    Form::kEVXForm,   efsdiv);
  D(efsmul          ,0x100002C8,    Form::kEVXForm,   efsmul);
  D(efsnabs         ,0x100002C5,    Form::kEVXForm,   efsnabs);
  D(efsneg          ,0x100002C6,    Form::kEVXForm,   efsneg);
  D(efssub          ,0x100002C1,    Form::kEVXForm,   efssub);
  D(efststeq        ,0x100002DE,    Form::kEVXForm,   efststeq);
  D(efststgt        ,0x100002DC,    Form::kEVXForm,   efststgt);
  D(efststlt        ,0x100002DD,    Form::kEVXForm,   efststlt);
  D(ehpriv          ,0x7C00021C,    Form::kXLForm,    ehpriv);
  D(eieio           ,0x7C0006AC,    Form::kXForm,     eieio);
  D(eqv             ,0x7C000238,    Form::kXForm,     eqv);
  D(eqvdot          ,0x7C000239,    Form::kXForm,     eqv.);
  D(evabs           ,0x10000208,    Form::kEVXForm,   evabs);
  D(evaddiw         ,0x10000202,    Form::kEVXForm,   evaddiw);
  D(evaddsmiaaw     ,0x100004C9,    Form::kEVXForm,   evaddsmiaaw);
  D(evaddssiaaw     ,0x100004C1,    Form::kEVXForm,   evaddssiaaw);
  D(evaddumiaaw     ,0x100004C8,    Form::kEVXForm,   evaddumiaaw);
  D(evaddusiaaw     ,0x100004C0,    Form::kEVXForm,   evaddusiaaw);
  D(evaddw          ,0x10000200,    Form::kEVXForm,   evaddw);
  D(evand           ,0x10000211,    Form::kEVXForm,   evand);
  D(evandc          ,0x10000212,    Form::kEVXForm,   evandc);
  D(evcmpeq         ,0x10000234,    Form::kEVXForm,   evcmpeq);
  D(evcmpgts        ,0x10000231,    Form::kEVXForm,   evcmpgts);
  D(evcmpgtu        ,0x10000230,    Form::kEVXForm,   evcmpgtu);
  D(evcmplts        ,0x10000233,    Form::kEVXForm,   evcmplts);
  D(evcmpltu        ,0x10000232,    Form::kEVXForm,   evcmpltu);
  D(evcntlsw        ,0x1000020E,    Form::kEVXForm,   evcntlsw);
  D(evcntlzw        ,0x1000020D,    Form::kEVXForm,   evcntlzw);
  D(evdivws         ,0x100004C6,    Form::kEVXForm,   evdivws);
  D(evdivwu         ,0x100004C7,    Form::kEVXForm,   evdivwu);
  D(eveqv           ,0x10000219,    Form::kEVXForm,   eveqv);
  D(evextsb         ,0x1000020A,    Form::kEVXForm,   evextsb);
  D(evextsh         ,0x1000020B,    Form::kEVXForm,   evextsh);
  D(evfsabs         ,0x10000284,    Form::kEVXForm,   evfsabs);
  D(evfsadd         ,0x10000280,    Form::kEVXForm,   evfsadd);
  D(evfscfsf        ,0x10000293,    Form::kEVXForm,   evfscfsf);
  D(evfscfsi        ,0x10000291,    Form::kEVXForm,   evfscfsi);
  D(evfscfuf        ,0x10000292,    Form::kEVXForm,   evfscfuf);
  D(evfscfui        ,0x10000290,    Form::kEVXForm,   evfscfui);
  D(evfscmpeq       ,0x1000028E,    Form::kEVXForm,   evfscmpeq);
  D(evfscmpgt       ,0x1000028C,    Form::kEVXForm,   evfscmpgt);
  D(evfscmplt       ,0x1000028D,    Form::kEVXForm,   evfscmplt);
  D(evfsctsf        ,0x10000297,    Form::kEVXForm,   evfsctsf);
  D(evfsctsi        ,0x10000295,    Form::kEVXForm,   evfsctsi);
  D(evfsctsiz       ,0x1000029A,    Form::kEVXForm,   evfsctsiz);
  D(evfsctuf        ,0x10000296,    Form::kEVXForm,   evfsctuf);
  D(evfsctui        ,0x10000294,    Form::kEVXForm,   evfsctui);
  D(evfsctuiz       ,0x10000298,    Form::kEVXForm,   evfsctuiz);
  D(evfsdiv         ,0x10000289,    Form::kEVXForm,   evfsdiv);
  D(evfsmul         ,0x10000288,    Form::kEVXForm,   evfsmul);
  D(evfsnabs        ,0x10000285,    Form::kEVXForm,   evfsnabs);
  D(evfsneg         ,0x10000286,    Form::kEVXForm,   evfsneg);
  D(evfssub         ,0x10000281,    Form::kEVXForm,   evfssub);
  D(evfststeq       ,0x1000029E,    Form::kEVXForm,   evfststeq);
  D(evfststgt       ,0x1000029C,    Form::kEVXForm,   evfststgt);
  D(evfststlt       ,0x1000029D,    Form::kEVXForm,   evfststlt);
  D(evldd           ,0x10000301,    Form::kEVXForm,   evldd);
  D(evlddepx        ,0x7C00063E,    Form::kEVXForm,   evlddepx);
  D(evlddx          ,0x10000300,    Form::kEVXForm,   evlddx);
  D(evldh           ,0x10000305,    Form::kEVXForm,   evldh);
  D(evldhx          ,0x10000304,    Form::kEVXForm,   evldhx);
  D(evldw           ,0x10000303,    Form::kEVXForm,   evldw);
  D(evldwx          ,0x10000302,    Form::kEVXForm,   evldwx);
  D(evlhhesplat     ,0x10000309,    Form::kEVXForm,   evlhhesplat);
  D(evlhhesplatx    ,0x10000308,    Form::kEVXForm,   evlhhesplatx);
  D(evlhhossplat    ,0x1000030F,    Form::kEVXForm,   evlhhossplat);
  D(evlhhossplatx   ,0x1000030E,    Form::kEVXForm,   evlhhossplatx);
  D(evlhhousplat    ,0x1000030D,    Form::kEVXForm,   evlhhousplat);
  D(evlhhousplatx   ,0x1000030C,    Form::kEVXForm,   evlhhousplatx);
  D(evlwhe          ,0x10000311,    Form::kEVXForm,   evlwhe);
  D(evlwhex         ,0x10000310,    Form::kEVXForm,   evlwhex);
  D(evlwhos         ,0x10000317,    Form::kEVXForm,   evlwhos);
  D(evlwhosx        ,0x10000316,    Form::kEVXForm,   evlwhosx);
  D(evlwhou         ,0x10000315,    Form::kEVXForm,   evlwhou);
  D(evlwhoux        ,0x10000314,    Form::kEVXForm,   evlwhoux);
  D(evlwhsplat      ,0x1000031D,    Form::kEVXForm,   evlwhsplat);
  D(evlwhsplatx     ,0x1000031C,    Form::kEVXForm,   evlwhsplatx);
  D(evlwwsplat      ,0x10000319,    Form::kEVXForm,   evlwwsplat);
  D(evlwwsplatx     ,0x10000318,    Form::kEVXForm,   evlwwsplatx);
  D(evmergehi       ,0x1000022C,    Form::kEVXForm,   evmergehi);
  D(evmergehilo     ,0x1000022E,    Form::kEVXForm,   evmergehilo);
  D(evmergelo       ,0x1000022D,    Form::kEVXForm,   evmergelo);
  D(evmergelohi     ,0x1000022F,    Form::kEVXForm,   evmergelohi);
  D(evmhegsmfaa     ,0x1000052B,    Form::kEVXForm,   evmhegsmfaa);
  D(evmhegsmfan     ,0x100005AB,    Form::kEVXForm,   evmhegsmfan);
  D(evmhegsmiaa     ,0x10000529,    Form::kEVXForm,   evmhegsmiaa);
  D(evmhegsmian     ,0x100005A9,    Form::kEVXForm,   evmhegsmian);
  D(evmhegumiaa     ,0x10000528,    Form::kEVXForm,   evmhegumiaa);
  D(evmhegumian     ,0x100005A8,    Form::kEVXForm,   evmhegumian);
  D(evmhesmf        ,0x1000040B,    Form::kEVXForm,   evmhesmf);
  D(evmhesmfa       ,0x1000042B,    Form::kEVXForm,   evmhesmfa);
  D(evmhesmfaaw     ,0x1000050B,    Form::kEVXForm,   evmhesmfaaw);
  D(evmhesmfanw     ,0x1000058B,    Form::kEVXForm,   evmhesmfanw);
  D(evmhesmi        ,0x10000409,    Form::kEVXForm,   evmhesmi);
  D(evmhesmia       ,0x10000429,    Form::kEVXForm,   evmhesmia);
  D(evmhesmiaaw     ,0x10000509,    Form::kEVXForm,   evmhesmiaaw);
  D(evmhesmianw     ,0x10000589,    Form::kEVXForm,   evmhesmianw);
  D(evmhessf        ,0x10000403,    Form::kEVXForm,   evmhessf);
  D(evmhessfa       ,0x10000423,    Form::kEVXForm,   evmhessfa);
  D(evmhessfaaw     ,0x10000503,    Form::kEVXForm,   evmhessfaaw);
  D(evmhessfanw     ,0x10000583,    Form::kEVXForm,   evmhessfanw);
  D(evmhessiaaw     ,0x10000501,    Form::kEVXForm,   evmhessiaaw);
  D(evmhessianw     ,0x10000581,    Form::kEVXForm,   evmhessianw);
  D(evmheumi        ,0x10000408,    Form::kEVXForm,   evmheumi);
  D(evmheumia       ,0x10000428,    Form::kEVXForm,   evmheumia);
  D(evmheumiaaw     ,0x10000508,    Form::kEVXForm,   evmheumiaaw);
  D(evmheumianw     ,0x10000588,    Form::kEVXForm,   evmheumianw);
  D(evmheusiaaw     ,0x10000500,    Form::kEVXForm,   evmheusiaaw);
  D(evmheusianw     ,0x10000580,    Form::kEVXForm,   evmheusianw);
  D(evmhogsmfaa     ,0x1000052F,    Form::kEVXForm,   evmhogsmfaa);
  D(evmhogsmfan     ,0x100005AF,    Form::kEVXForm,   evmhogsmfan);
  D(evmhogsmiaa     ,0x1000052D,    Form::kEVXForm,   evmhogsmiaa);
  D(evmhogsmian     ,0x100005AD,    Form::kEVXForm,   evmhogsmian);
  D(evmhogumiaa     ,0x1000052C,    Form::kEVXForm,   evmhogumiaa);
  D(evmhogumian     ,0x100005AC,    Form::kEVXForm,   evmhogumian);
  D(evmhosmf        ,0x1000040F,    Form::kEVXForm,   evmhosmf);
  D(evmhosmfa       ,0x1000042F,    Form::kEVXForm,   evmhosmfa);
  D(evmhosmfaaw     ,0x1000050F,    Form::kEVXForm,   evmhosmfaaw);
  D(evmhosmfanw     ,0x1000058F,    Form::kEVXForm,   evmhosmfanw);
  D(evmhosmi        ,0x1000040D,    Form::kEVXForm,   evmhosmi);
  D(evmhosmia       ,0x1000042D,    Form::kEVXForm,   evmhosmia);
  D(evmhosmiaaw     ,0x1000050D,    Form::kEVXForm,   evmhosmiaaw);
  D(evmhosmianw     ,0x1000058D,    Form::kEVXForm,   evmhosmianw);
  D(evmhossf        ,0x10000407,    Form::kEVXForm,   evmhossf);
  D(evmhossfa       ,0x10000427,    Form::kEVXForm,   evmhossfa);
  D(evmhossfaaw     ,0x10000507,    Form::kEVXForm,   evmhossfaaw);
  D(evmhossfanw     ,0x10000587,    Form::kEVXForm,   evmhossfanw);
  D(evmhossiaaw     ,0x10000505,    Form::kEVXForm,   evmhossiaaw);
  D(evmhossianw     ,0x10000585,    Form::kEVXForm,   evmhossianw);
  D(evmhoumi        ,0x1000040C,    Form::kEVXForm,   evmhoumi);
  D(evmhoumia       ,0x1000042C,    Form::kEVXForm,   evmhoumia);
  D(evmhoumiaaw     ,0x1000050C,    Form::kEVXForm,   evmhoumiaaw);
  D(evmhoumianw     ,0x1000058C,    Form::kEVXForm,   evmhoumianw);
  D(evmhousiaaw     ,0x10000504,    Form::kEVXForm,   evmhousiaaw);
  D(evmhousianw     ,0x10000584,    Form::kEVXForm,   evmhousianw);
  D(evmra           ,0x100004C4,    Form::kEVXForm,   evmra);
  D(evmwhsmf        ,0x1000044F,    Form::kEVXForm,   evmwhsmf);
  D(evmwhsmfa       ,0x1000046F,    Form::kEVXForm,   evmwhsmfa);
  D(evmwhsmi        ,0x1000044D,    Form::kEVXForm,   evmwhsmi);
  D(evmwhsmia       ,0x1000046D,    Form::kEVXForm,   evmwhsmia);
  D(evmwhssf        ,0x10000447,    Form::kEVXForm,   evmwhssf);
  D(evmwhssfa       ,0x10000467,    Form::kEVXForm,   evmwhssfa);
  D(evmwhumi        ,0x1000044C,    Form::kEVXForm,   evmwhumi);
  D(evmwhumia       ,0x1000046C,    Form::kEVXForm,   evmwhumia);
  D(evmwlsmiaaw     ,0x10000549,    Form::kEVXForm,   evmwlsmiaaw);
  D(evmwlsmianw     ,0x100005C9,    Form::kEVXForm,   evmwlsmianw);
  D(evmwlssiaaw     ,0x10000541,    Form::kEVXForm,   evmwlssiaaw);
  D(evmwlssianw     ,0x100005C1,    Form::kEVXForm,   evmwlssianw);
  D(evmwlumi        ,0x10000448,    Form::kEVXForm,   evmwlumi);
  D(evmwlumia       ,0x10000468,    Form::kEVXForm,   evmwlumia);
  D(evmwlumiaaw     ,0x10000548,    Form::kEVXForm,   evmwlumiaaw);
  D(evmwlumianw     ,0x100005C8,    Form::kEVXForm,   evmwlumianw);
  D(evmwlusiaaw     ,0x10000540,    Form::kEVXForm,   evmwlusiaaw);
  D(evmwlusianw     ,0x100005C0,    Form::kEVXForm,   evmwlusianw);
  D(evmwsmf         ,0x1000045B,    Form::kEVXForm,   evmwsmf);
  D(evmwsmfa        ,0x1000047B,    Form::kEVXForm,   evmwsmfa);
  D(evmwsmfaa       ,0x1000055B,    Form::kEVXForm,   evmwsmfaa);
  D(evmwsmfan       ,0x100005DB,    Form::kEVXForm,   evmwsmfan);
  D(evmwsmi         ,0x10000459,    Form::kEVXForm,   evmwsmi);
  D(evmwsmia        ,0x10000479,    Form::kEVXForm,   evmwsmia);
  D(evmwsmiaa       ,0x10000559,    Form::kEVXForm,   evmwsmiaa);
  D(evmwsmian       ,0x100005D9,    Form::kEVXForm,   evmwsmian);
  D(evmwssf         ,0x10000453,    Form::kEVXForm,   evmwssf);
  D(evmwssfa        ,0x10000473,    Form::kEVXForm,   evmwssfa);
  D(evmwssfaa       ,0x10000553,    Form::kEVXForm,   evmwssfaa);
  D(evmwssfan       ,0x100005D3,    Form::kEVXForm,   evmwssfan);
  D(evmwumi         ,0x10000458,    Form::kEVXForm,   evmwumi);
  D(evmwumia        ,0x10000478,    Form::kEVXForm,   evmwumia);
  D(evmwumiaa       ,0x10000558,    Form::kEVXForm,   evmwumiaa);
  D(evmwumian       ,0x100005D8,    Form::kEVXForm,   evmwumian);
  D(evnand          ,0x1000021E,    Form::kEVXForm,   evnand);
  D(evneg           ,0x10000209,    Form::kEVXForm,   evneg);
  D(evnor           ,0x10000218,    Form::kEVXForm,   evnor);
  D(evor            ,0x10000217,    Form::kEVXForm,   evor);
  D(evorc           ,0x1000021B,    Form::kEVXForm,   evorc);
  D(evrlw           ,0x10000228,    Form::kEVXForm,   evrlw);
  D(evrlwi          ,0x1000022A,    Form::kEVXForm,   evrlwi);
  D(evrndw          ,0x1000020C,    Form::kEVXForm,   evrndw);
  D(evsel           ,0x10000278,    Form::kEVSForm,   evsel);
  D(evslw           ,0x10000224,    Form::kEVXForm,   evslw);
  D(evslwi          ,0x10000226,    Form::kEVXForm,   evslwi);
  D(evsplatfi       ,0x1000022B,    Form::kEVXForm,   evsplatfi);
  D(evsplati        ,0x10000229,    Form::kEVXForm,   evsplati);
  D(evsrwis         ,0x10000223,    Form::kEVXForm,   evsrwis);
  D(evsrwiu         ,0x10000222,    Form::kEVXForm,   evsrwiu);
  D(evsrws          ,0x10000221,    Form::kEVXForm,   evsrws);
  D(evsrwu          ,0x10000220,    Form::kEVXForm,   evsrwu);
  D(evstdd          ,0x10000321,    Form::kEVXForm,   evstdd);
  D(evstddepx       ,0x7C00073E,    Form::kEVXForm,   evstddepx);
  D(evstddx         ,0x10000320,    Form::kEVXForm,   evstddx);
  D(evstdh          ,0x10000325,    Form::kEVXForm,   evstdh);
  D(evstdhx         ,0x10000324,    Form::kEVXForm,   evstdhx);
  D(evstdw          ,0x10000323,    Form::kEVXForm,   evstdw);
  D(evstdwx         ,0x10000322,    Form::kEVXForm,   evstdwx);
  D(evstwhe         ,0x10000331,    Form::kEVXForm,   evstwhe);
  D(evstwhex        ,0x10000330,    Form::kEVXForm,   evstwhex);
  D(evstwho         ,0x10000335,    Form::kEVXForm,   evstwho);
  D(evstwhox        ,0x10000334,    Form::kEVXForm,   evstwhox);
  D(evstwwe         ,0x10000339,    Form::kEVXForm,   evstwwe);
  D(evstwwex        ,0x10000338,    Form::kEVXForm,   evstwwex);
  D(evstwwo         ,0x1000033D,    Form::kEVXForm,   evstwwo);
  D(evstwwox        ,0x1000033C,    Form::kEVXForm,   evstwwox);
  D(evsubfsmiaaw    ,0x100004CB,    Form::kEVXForm,   evsubfsmiaaw);
  D(evsubfssiaaw    ,0x100004C3,    Form::kEVXForm,   evsubfssiaaw);
  D(evsubfumiaaw    ,0x100004CA,    Form::kEVXForm,   evsubfumiaaw);
  D(evsubfusiaaw    ,0x100004C2,    Form::kEVXForm,   evsubfusiaaw);
  D(evsubfw         ,0x10000204,    Form::kEVXForm,   evsubfw);
  D(evsubifw        ,0x10000206,    Form::kEVXForm,   evsubifw);
  D(evxor           ,0x10000216,    Form::kEVXForm,   evxor);
  D(extsb           ,0x7C000774,    Form::kXForm,     extsb);
  D(extsbdot        ,0x7C000775,    Form::kXForm,     extsb.);
  D(extsh           ,0x7C000734,    Form::kXForm,     extsh);
  D(extshdot        ,0x7C000735,    Form::kXForm,     extsh.);
  D(extsw           ,0x7C0007B4,    Form::kXForm,     extsw);
  D(extswdot        ,0x7C0007B5,    Form::kXForm,     extsw.);
  D(fabs            ,0xFC000210,    Form::kXForm,     fabs);
  D(fabsdot         ,0xFC000211,    Form::kXForm,     fabs.);
  D(fadd            ,0xFC00002A,    Form::kAForm,     fadd);
  D(fadddot         ,0xFC00002B,    Form::kAForm,     fadd.);
  D(fadds           ,0xEC00002A,    Form::kAForm,     fadds);
  D(faddsdot        ,0xEC00002B,    Form::kAForm,     fadds.);
  D(fcfid           ,0xFC00069C,    Form::kXForm,     fcfid);
  D(fcfiddot        ,0xFC00069D,    Form::kXForm,     fcfid.);
  D(fcfids          ,0xEC00069C,    Form::kXForm,     fcfids);
  D(fcfidsdot       ,0xEC00069D,    Form::kXForm,     fcfids.);
  D(fcfidu          ,0xFC00079C,    Form::kXForm,     fcfidu);
  D(fcfidudot       ,0xFC00079D,    Form::kXForm,     fcfidu.);
  D(fcfidus         ,0xEC00079C,    Form::kXForm,     fcfidus);
  D(fcfidusdot      ,0xEC00079D,    Form::kXForm,     fcfidus.);
  D(fcmpo           ,0xFC000040,    Form::kXForm,     fcmpo);
  D(fcmpu           ,0xFC000000,    Form::kXForm,     fcmpu);
  D(fcpsgn          ,0xFC000010,    Form::kXForm,     fcpsgn);
  D(fcpsgndot       ,0xFC000011,    Form::kXForm,     fcpsgn.);
  D(fctid           ,0xFC00065C,    Form::kXForm,     fctid);
  D(fctiddot        ,0xFC00065D,    Form::kXForm,     fctid.);
  D(fctidu          ,0xFC00075C,    Form::kXForm,     fctidu);
  D(fctidudot       ,0xFC00075D,    Form::kXForm,     fctidu.);
  D(fctiduz         ,0xFC00075E,    Form::kXForm,     fctiduz);
  D(fctiduzdot      ,0xFC00075F,    Form::kXForm,     fctiduz.);
  D(fctidz          ,0xFC00065E,    Form::kXForm,     fctidz);
  D(fctidzdot       ,0xFC00065F,    Form::kXForm,     fctidz.);
  D(fctiw           ,0xFC00001C,    Form::kXForm,     fctiw);
  D(fctiwdot        ,0xFC00001D,    Form::kXForm,     fctiw.);
  D(fctiwu          ,0xFC00011C,    Form::kXForm,     fctiwu);
  D(fctiwudot       ,0xFC00011D,    Form::kXForm,     fctiwu.);
  D(fctiwuz         ,0xFC00011E,    Form::kXForm,     fctiwuz);
  D(fctiwuzdot      ,0xFC00011F,    Form::kXForm,     fctiwuz.);
  D(fctiwz          ,0xFC00001E,    Form::kXForm,     fctiwz);
  D(fctiwzdot       ,0xFC00001F,    Form::kXForm,     fctiwz.);
  D(fdiv            ,0xFC000024,    Form::kAForm,     fdiv);
  D(fdivdot         ,0xFC000025,    Form::kAForm,     fdiv.);
  D(fdivs           ,0xEC000024,    Form::kAForm,     fdivs);
  D(fdivsdot        ,0xEC000025,    Form::kAForm,     fdivs.);
  D(fmadd           ,0xFC00003A,    Form::kAForm,     fmadd);
  D(fmadddot        ,0xFC00003B,    Form::kAForm,     fmadd.);
  D(fmadds          ,0xEC00003A,    Form::kAForm,     fmadds);
  D(fmaddsdot       ,0xEC00003B,    Form::kAForm,     fmadds.);
  D(fmr             ,0xFC000090,    Form::kXForm,     fmr);
  D(fmrdot          ,0xFC000091,    Form::kXForm,     fmr.);
  D(fmrgew          ,0xFC00078C,    Form::kXForm,     fmrgew);
  D(fmrgow          ,0xFC00068C,    Form::kXForm,     fmrgow);
  D(fmsub           ,0xFC000038,    Form::kAForm,     fmsub);
  D(fmsubdot        ,0xFC000039,    Form::kAForm,     fmsub.);
  D(fmsubs          ,0xEC000038,    Form::kAForm,     fmsubs);
  D(fmsubsdot       ,0xEC000039,    Form::kAForm,     fmsubs.);
  D(fmul            ,0xFC000032,    Form::kAForm,     fmul);
  D(fmuldot         ,0xFC000033,    Form::kAForm,     fmul.);
  D(fmuls           ,0xEC000032,    Form::kAForm,     fmuls);
  D(fmulsdot        ,0xEC000033,    Form::kAForm,     fmuls.);
  D(fnabs           ,0xFC000110,    Form::kXForm,     fnabs);
  D(fnabsdot        ,0xFC000111,    Form::kXForm,     fnabs.);
  D(fneg            ,0xFC000050,    Form::kXForm,     fneg);
  D(fnegdot         ,0xFC000051,    Form::kXForm,     fneg.);
  D(fnmadd          ,0xFC00003E,    Form::kAForm,     fnmadd);
  D(fnmadddot       ,0xFC00003F,    Form::kAForm,     fnmadd.);
  D(fnmadds         ,0xEC00003E,    Form::kAForm,     fnmadds);
  D(fnmaddsdot      ,0xEC00003F,    Form::kAForm,     fnmadds.);
  D(fnmsub          ,0xFC00003C,    Form::kAForm,     fnmsub);
  D(fnmsubdot       ,0xFC00003D,    Form::kAForm,     fnmsub.);
  D(fnmsubs         ,0xEC00003C,    Form::kAForm,     fnmsubs);
  D(fnmsubsdot      ,0xEC00003D,    Form::kAForm,     fnmsubs.);
  D(fre             ,0xFC000030,    Form::kAForm,     fre);
  D(fredot          ,0xFC000031,    Form::kAForm,     fre.);
  D(fres            ,0xEC000030,    Form::kAForm,     fres);
  D(fresdot         ,0xEC000031,    Form::kAForm,     fres.);
  D(frim            ,0xFC0003D0,    Form::kXForm,     frim);
  D(frimdot         ,0xFC0003D1,    Form::kXForm,     frim.);
  D(frin            ,0xFC000310,    Form::kXForm,     frin);
  D(frindot         ,0xFC000311,    Form::kXForm,     frin.);
  D(frip            ,0xFC000390,    Form::kXForm,     frip);
  D(fripdot         ,0xFC000391,    Form::kXForm,     frip.);
  D(friz            ,0xFC000350,    Form::kXForm,     friz);
  D(frizdot         ,0xFC000351,    Form::kXForm,     friz.);
  D(frsp            ,0xFC000018,    Form::kXForm,     frsp);
  D(frspdot         ,0xFC000019,    Form::kXForm,     frsp.);
  D(frsqrte         ,0xFC000034,    Form::kAForm,     frsqrte);
  D(frsqrtedot      ,0xFC000035,    Form::kAForm,     frsqrte.);
  D(frsqrtes        ,0xEC000034,    Form::kAForm,     frsqrtes);
  D(frsqrtesdot     ,0xEC000035,    Form::kAForm,     frsqrtes.);
  D(fsel            ,0xFC00002E,    Form::kAForm,     fsel);
  D(fseldot         ,0xFC00002F,    Form::kAForm,     fsel.);
  D(fsqrt           ,0xFC00002C,    Form::kAForm,     fsqrt);
  D(fsqrtdot        ,0xFC00002D,    Form::kAForm,     fsqrt.);
  D(fsqrts          ,0xEC00002C,    Form::kAForm,     fsqrts);
  D(fsqrtsdot       ,0xEC00002D,    Form::kAForm,     fsqrts.);
  D(fsub            ,0xFC000028,    Form::kAForm,     fsub);
  D(fsubdot         ,0xFC000029,    Form::kAForm,     fsub.);
  D(fsubs           ,0xEC000028,    Form::kAForm,     fsubs);
  D(fsubsdot        ,0xEC000029,    Form::kAForm,     fsubs.);
  D(ftdiv           ,0xFC000100,    Form::kXForm,     ftdiv);
  D(ftsqrt          ,0xFC000140,    Form::kXForm,     ftsqrt);
  D(hrfid           ,0x4C000224,    Form::kXLForm,    hrfid);
  D(icbi            ,0x7C0007AC,    Form::kXForm,     icbi);
  D(icbiep          ,0x7C0007BE,    Form::kXForm,     icbiep);
  D(icblc           ,0x7C0001CC,    Form::kXForm,     icblc);
  D(icblqdot        ,0x7C00018D,    Form::kXForm,     icblq.);
  D(icbt            ,0x7C00002C,    Form::kXForm,     icbt);
  D(icbtls          ,0x7C0003CC,    Form::kXForm,     icbtls);
  D(ici             ,0x7C00078C,    Form::kXForm,     ici);
  D(icread          ,0x7C0007CC,    Form::kXForm,     icread);
  D(isel            ,0x7C00001E,    Form::kAForm,     isel);
  D(isync           ,0x4C00012C,    Form::kXLForm,    isync);
  D(lbarx           ,0x7C000068,    Form::kXForm,     lbarx);
  D(lbdx            ,0x7C000406,    Form::kXForm,     lbdx);
  D(lbepx           ,0x7C0000BE,    Form::kXForm,     lbepx);
  D(lbz             ,0x88000000,    Form::kDForm,     lbz);
  D(lbzcix          ,0x7C0006AA,    Form::kXForm,     lbzcix);
  D(lbzu            ,0x8C000000,    Form::kDForm,     lbzu);
  D(lbzux           ,0x7C0000EE,    Form::kXForm,     lbzux);
  D(lbzx            ,0x7C0000AE,    Form::kXForm,     lbzx);
  D(ld              ,0xE8000000,    Form::kDSForm,    ld);
  D(ldarx           ,0x7C0000A8,    Form::kXForm,     ldarx);
  D(ldbrx           ,0x7C000428,    Form::kXForm,     ldbrx);
  D(ldcix           ,0x7C0006EA,    Form::kXForm,     ldcix);
  D(lddx            ,0x7C0004C6,    Form::kXForm,     lddx);
  D(ldepx           ,0x7C00003A,    Form::kXForm,     ldepx);
  D(ldu             ,0xE8000001,    Form::kDSForm,    ldu);
  D(ldux            ,0x7C00006A,    Form::kXForm,     ldux);
  D(ldx             ,0x7C00002A,    Form::kXForm,     ldx);
  D(lfd             ,0xC8000000,    Form::kDForm,     lfd);
  D(lfddx           ,0x7C000646,    Form::kXForm,     lfddx);
  D(lfdepx          ,0x7C0004BE,    Form::kXForm,     lfdepx);
  D(lfdp            ,0xE4000000,    Form::kDSForm,    lfdp);
  D(lfdpx           ,0x7C00062E,    Form::kXForm,     lfdpx);
  D(lfdu            ,0xCC000000,    Form::kDForm,     lfdu);
  D(lfdux           ,0x7C0004EE,    Form::kXForm,     lfdux);
  D(lfdx            ,0x7C0004AE,    Form::kXForm,     lfdx);
  D(lfiwax          ,0x7C0006AE,    Form::kXForm,     lfiwax);
  D(lfiwzx          ,0x7C0006EE,    Form::kXForm,     lfiwzx);
  D(lfs             ,0xC0000000,    Form::kDForm,     lfs);
  D(lfsu            ,0xC4000000,    Form::kDForm,     lfsu);
  D(lfsux           ,0x7C00046E,    Form::kXForm,     lfsux);
  D(lfsx            ,0x7C00042E,    Form::kXForm,     lfsx);
  D(lha             ,0xA8000000,    Form::kDForm,     lha);
  D(lharx           ,0x7C0000E8,    Form::kXForm,     lharx);
  D(lhau            ,0xAC000000,    Form::kDForm,     lhau);
  D(lhaux           ,0x7C0002EE,    Form::kXForm,     lhaux);
  D(lhax            ,0x7C0002AE,    Form::kXForm,     lhax);
  D(lhbrx           ,0x7C00062C,    Form::kXForm,     lhbrx);
  D(lhdx            ,0x7C000446,    Form::kXForm,     lhdx);
  D(lhepx           ,0x7C00023E,    Form::kXForm,     lhepx);
  D(lhz             ,0xA0000000,    Form::kDForm,     lhz);
  D(lhzcix          ,0x7C00066A,    Form::kXForm,     lhzcix);
  D(lhzu            ,0xA4000000,    Form::kDForm,     lhzu);
  D(lhzux           ,0x7C00026E,    Form::kXForm,     lhzux);
  D(lhzx            ,0x7C00022E,    Form::kXForm,     lhzx);
  D(lmw             ,0xB8000000,    Form::kDForm,     lmw);
  D(lq              ,0xE0000000,    Form::kDQForm,    lq);
  D(lqarx           ,0x7C000228,    Form::kXForm,     lqarx);
  D(lswi            ,0x7C0004AA,    Form::kXForm,     lswi);
  D(lswx            ,0x7C00042A,    Form::kXForm,     lswx);
  D(lvebx           ,0x7C00000E,    Form::kXForm,     lvebx);
  D(lvehx           ,0x7C00004E,    Form::kXForm,     lvehx);
  D(lvepx           ,0x7C00024E,    Form::kXForm,     lvepx);
  D(lvepxl          ,0x7C00020E,    Form::kXForm,     lvepxl);
  D(lvewx           ,0x7C00008E,    Form::kXForm,     lvewx);
  D(lvsl            ,0x7C00000C,    Form::kXForm,     lvsl);
  D(lvsr            ,0x7C00004C,    Form::kXForm,     lvsr);
  D(lvx             ,0x7C0000CE,    Form::kXForm,     lvx);
  D(lvxl            ,0x7C0002CE,    Form::kXForm,     lvxl);
  D(lwa             ,0xE8000002,    Form::kDSForm,    lwa);
  D(lwarx           ,0x7C000028,    Form::kXForm,     lwarx);
  D(lwaux           ,0x7C0002EA,    Form::kXForm,     lwaux);
  D(lwax            ,0x7C0002AA,    Form::kXForm,     lwax);
  D(lwbrx           ,0x7C00042C,    Form::kXForm,     lwbrx);
  D(lwdx            ,0x7C000486,    Form::kXForm,     lwdx);
  D(lwepx           ,0x7C00003E,    Form::kXForm,     lwepx);
  D(lwz             ,0x80000000,    Form::kDForm,     lwz);
  D(lwzcix          ,0x7C00062A,    Form::kXForm,     lwzcix);
  D(lwzu            ,0x84000000,    Form::kDForm,     lwzu);
  D(lwzux           ,0x7C00006E,    Form::kXForm,     lwzux);
  D(lwzx            ,0x7C00002E,    Form::kXForm,     lwzx);
  D(lxsdx           ,0x7C000498,    Form::kXX1Form,   lxsdx);
  D(lxsiwax         ,0x7C000098,    Form::kXX1Form,   lxsiwax);
  D(lxsiwzx         ,0x7C000018,    Form::kXX1Form,   lxsiwzx);
  D(lxsspx          ,0x7C000418,    Form::kXX1Form,   lxsspx);
  D(lxvd2x          ,0x7C000698,    Form::kXX1Form,   lxvd2x);
  D(lxvdsx          ,0x7C000298,    Form::kXX1Form,   lxvdsx);
  D(lxvw4x          ,0x7C000618,    Form::kXX1Form,   lxvw4x);
  D(macchw          ,0x10000158,    Form::kXOForm,    macchw);
  D(macchwdot       ,0x10000159,    Form::kXOForm,    macchw.);
  D(macchwo         ,0x10000558,    Form::kXOForm,    macchwo);
  D(macchwodot      ,0x10000559,    Form::kXOForm,    macchwo.);
  D(macchws         ,0x100001D8,    Form::kXOForm,    macchws);
  D(macchwsdot      ,0x100001D9,    Form::kXOForm,    macchws.);
  D(macchwso        ,0x100005D8,    Form::kXOForm,    macchwso);
  D(macchwsodot     ,0x100005D9,    Form::kXOForm,    macchwso.);
  D(macchwsu        ,0x10000198,    Form::kXOForm,    macchwsu);
  D(macchwsudot     ,0x10000199,    Form::kXOForm,    macchwsu.);
  D(macchwsuo       ,0x10000598,    Form::kXOForm,    macchwsuo);
  D(macchwsuodot    ,0x10000599,    Form::kXOForm,    macchwsuo.);
  D(macchwu         ,0x10000118,    Form::kXOForm,    macchwu);
  D(macchwudot      ,0x10000119,    Form::kXOForm,    macchwu.);
  D(macchwuo        ,0x10000518,    Form::kXOForm,    macchwuo);
  D(macchwuodot     ,0x10000519,    Form::kXOForm,    macchwuo.);
  D(machhw          ,0x10000058,    Form::kXOForm,    machhw);
  D(machhwdot       ,0x10000059,    Form::kXOForm,    machhw.);
  D(machhwo         ,0x10000458,    Form::kXOForm,    machhwo);
  D(machhwodot      ,0x10000459,    Form::kXOForm,    machhwo.);
  D(machhws         ,0x100000D8,    Form::kXOForm,    machhws);
  D(machhwsdot      ,0x100000D9,    Form::kXOForm,    machhws.);
  D(machhwso        ,0x100004D8,    Form::kXOForm,    machhwso);
  D(machhwsodot     ,0x100004D9,    Form::kXOForm,    machhwso.);
  D(machhwsu        ,0x10000098,    Form::kXOForm,    machhwsu);
  D(machhwsudot     ,0x10000099,    Form::kXOForm,    machhwsu.);
  D(machhwsuo       ,0x10000498,    Form::kXOForm,    machhwsuo);
  D(machhwsuodot    ,0x10000499,    Form::kXOForm,    machhwsuo.);
  D(machhwu         ,0x10000018,    Form::kXOForm,    machhwu);
  D(machhwudot      ,0x10000019,    Form::kXOForm,    machhwu.);
  D(machhwuo        ,0x10000418,    Form::kXOForm,    machhwuo);
  D(machhwuodot     ,0x10000419,    Form::kXOForm,    machhwuo.);
  D(maclhw          ,0x10000358,    Form::kXOForm,    maclhw);
  D(maclhwdot       ,0x10000359,    Form::kXOForm,    maclhw.);
  D(maclhwo         ,0x10000758,    Form::kXOForm,    maclhwo);
  D(maclhwodot      ,0x10000759,    Form::kXOForm,    maclhwo.);
  D(maclhws         ,0x100003D8,    Form::kXOForm,    maclhws);
  D(maclhwsdot      ,0x100003D9,    Form::kXOForm,    maclhws.);
  D(maclhwso        ,0x100007D8,    Form::kXOForm,    maclhwso);
  D(maclhwsodot     ,0x100007D9,    Form::kXOForm,    maclhwso.);
  D(maclhwsu        ,0x10000398,    Form::kXOForm,    maclhwsu);
  D(maclhwsudot     ,0x10000399,    Form::kXOForm,    maclhwsu.);
  D(maclhwsuo       ,0x10000798,    Form::kXOForm,    maclhwsuo);
  D(maclhwsuodot    ,0x10000799,    Form::kXOForm,    maclhwsuo.);
  D(maclhwu         ,0x10000318,    Form::kXOForm,    maclhwu);
  D(maclhwudot      ,0x10000319,    Form::kXOForm,    maclhwu.);
  D(maclhwuo        ,0x10000718,    Form::kXOForm,    maclhwuo);
  D(maclhwuodot     ,0x10000719,    Form::kXOForm,    maclhwuo.);
  D(mbar            ,0x7C0006AC,    Form::kXForm,     mbar);
  D(mcrf            ,0x4C000000,    Form::kXLForm,    mcrf);
  D(mcrfs           ,0xFC000080,    Form::kXForm,     mcrfs);
  D(mcrxr           ,0x7C000400,    Form::kXForm,     mcrxr);
  D(mfbhrbe         ,0x7C00025C,    Form::kXFXForm,   mfbhrbe);
  D(mfcr            ,0x7C000026,    Form::kXFXForm,   mfcr);
  D(mfdcr           ,0x7C000286,    Form::kXFXForm,   mfdcr);
  D(mfdcrux         ,0x7C000246,    Form::kXForm,     mfdcrux);
  D(mfdcrx          ,0x7C000206,    Form::kXForm,     mfdcrx);
  D(mffs            ,0xFC00048E,    Form::kXForm,     mffs);
  D(mffsdot         ,0xFC00048D,    Form::kXForm,     mffs.);
  D(mfmsr           ,0x7C0000A6,    Form::kXForm,     mfmsr);
  D(mfocrf          ,0x7C100026,    Form::kXFXForm,   mfocrf);
  D(mfpmr           ,0x7C00029C,    Form::kXFXForm,   mfpmr);
  D(mfspr           ,0x7C0002A6,    Form::kXFXForm,   mfspr);
  D(mfsr            ,0x7C0004A6,    Form::kXForm,     mfsr);
  D(mfsrin          ,0x7C000526,    Form::kXForm,     mfsrin);
  D(mftb            ,0x7C0002E6,    Form::kXFXForm,   mftb);
  D(mfvscr          ,0x10000604,    Form::kVXForm,    mfvscr);
  D(mfvsrd          ,0x7C000066,    Form::kXX1Form,   mfvsrd);
  D(mfvsrwz         ,0x7C0000E6,    Form::kXX1Form,   mfvsrwz);
  D(msgclr          ,0x7C0001DC,    Form::kXForm,     msgclr);
  D(msgclrp         ,0x7C00015C,    Form::kXForm,     msgclrp);
  D(msgsnd          ,0x7C00019C,    Form::kXForm,     msgsnd);
  D(msgsndp         ,0x7C00011C,    Form::kXForm,     msgsndp);
  D(mtcrf           ,0x7C000120,    Form::kXFXForm,   mtcrf);
  D(mtdcr           ,0x7C000386,    Form::kXFXForm,   mtdcr);
  D(mtdcrux         ,0x7C000346,    Form::kXForm,     mtdcrux);
  D(mtdcrx          ,0x7C000306,    Form::kXForm,     mtdcrx);
  D(mtfsb0          ,0xFC00008C,    Form::kXForm,     mtfsb0);
  D(mtfsb0dot       ,0xFC00008D,    Form::kXForm,     mtfsb0.);
  D(mtfsb1          ,0xFC00004C,    Form::kXForm,     mtfsb1);
  D(mtfsb1dot       ,0xFC00004D,    Form::kXForm,     mtfsb1.);
  D(mtfsf           ,0xFC00058E,    Form::kXFLForm,    mtfsf);
  D(mtfsfdot        ,0xFC00058F,    Form::kXFLForm,    mtfsf.);
  D(mtfsfi          ,0xFC00010C,    Form::kXForm,     mtfsfi);
  D(mtfsfidot       ,0xFC00010D,    Form::kXForm,     mtfsfi.);
  D(mtmsr           ,0x7C000124,    Form::kXForm,     mtmsr);
  D(mtmsrd          ,0x7C000164,    Form::kXForm,     mtmsrd);
  D(mtocrf          ,0x7C100120,    Form::kXFXForm,   mtocrf);
  D(mtpmr           ,0x7C00039C,    Form::kXFXForm,   mtpmr);
  D(mtspr           ,0x7C0003A6,    Form::kXFXForm,   mtspr);
  D(mtsr            ,0x7C0001A4,    Form::kXForm,     mtsr);
  D(mtsrin          ,0x7C0001E4,    Form::kXForm,     mtsrin);
  D(mtvscr          ,0x10000644,    Form::kVXForm,    mtvscr);
  D(mtvsrd          ,0x7C000166,    Form::kXX1Form,   mtvsrd);
  D(mtvsrwa         ,0x7C0001A6,    Form::kXX1Form,   mtvsrwa);
  D(mtvsrwz         ,0x7C0001E6,    Form::kXX1Form,   mtvsrwz);
  D(mulchw          ,0x10000150,    Form::kXForm,     mulchw);
  D(mulchwdot       ,0x10000151,    Form::kXForm,     mulchw.);
  D(mulchwu         ,0x10000110,    Form::kXForm,     mulchwu);
  D(mulchwudot      ,0x10000111,    Form::kXForm,     mulchwu.);
  D(mulhd           ,0x7C000092,    Form::kXOForm,    mulhd);
  D(mulhddot        ,0x7C000093,    Form::kXOForm,    mulhd.);
  D(mulhdu          ,0x7C000012,    Form::kXOForm,    mulhdu);
  D(mulhdudot       ,0x7C000013,    Form::kXOForm,    mulhdu.);
  D(mulhhw          ,0x10000050,    Form::kXForm,     mulhhw);
  D(mulhhwdot       ,0x10000051,    Form::kXForm,     mulhhw.);
  D(mulhhwu         ,0x10000010,    Form::kXForm,     mulhhwu);
  D(mulhhwudot      ,0x10000011,    Form::kXForm,     mulhhwu.);
  D(mulhw           ,0x7C000096,    Form::kXOForm,    mulhw);
  D(mulhwdot        ,0x7C000097,    Form::kXOForm,    mulhw.);
  D(mulhwu          ,0x7C000016,    Form::kXOForm,    mulhwu);
  D(mulhwudot       ,0x7C000017,    Form::kXOForm,    mulhwu.);
  D(mulld           ,0x7C0001D2,    Form::kXOForm,    mulld);
  D(mullddot        ,0x7C0001D3,    Form::kXOForm,    mulld.);
  D(mulldo          ,0x7C0005D2,    Form::kXOForm,    mulldo);
  D(mulldodot       ,0x7C0005D3,    Form::kXOForm,    mulldo.);
  D(mullhw          ,0x10000350,    Form::kXForm,     mullhw);
  D(mullhwdot       ,0x10000351,    Form::kXForm,     mullhw.);
  D(mullhwu         ,0x10000310,    Form::kXForm,     mullhwu);
  D(mullhwudot      ,0x10000311,    Form::kXForm,     mullhwu.);
  D(mulli           ,0x1C000000,    Form::kDForm,     mulli);
  D(mullw           ,0x7C0001D6,    Form::kXOForm,    mullw);
  D(mullwdot        ,0x7C0001D7,    Form::kXOForm,    mullw.);
  D(mullwo          ,0x7C0005D6,    Form::kXOForm,    mullwo);
  D(mullwodot       ,0x7C0005D7,    Form::kXOForm,    mullwo.);
  D(nand            ,0x7C0003B8,    Form::kXForm,     nand);
  D(nanddot         ,0x7C0003B9,    Form::kXForm,     nand.);
  D(nap             ,0x4C000364,    Form::kXLForm,    nap);
  D(neg             ,0x7C0000D0,    Form::kXOForm,    neg);
  D(negdot          ,0x7C0000D1,    Form::kXOForm,    neg.);
  D(nego            ,0x7C0004D0,    Form::kXOForm,    nego);
  D(negodot         ,0x7C0004D1,    Form::kXOForm,    nego.);
  D(nmacchw         ,0x1000015C,    Form::kXOForm,    nmacchw);
  D(nmacchwdot      ,0x1000015D,    Form::kXOForm,    nmacchw.);
  D(nmacchwo        ,0x1000055C,    Form::kXOForm,    nmacchwo);
  D(nmacchwodot     ,0x1000055D,    Form::kXOForm,    nmacchwo.);
  D(nmacchws        ,0x100001DC,    Form::kXOForm,    nmacchws);
  D(nmacchwsdot     ,0x100001DD,    Form::kXOForm,    nmacchws.);
  D(nmacchwso       ,0x100005DC,    Form::kXOForm,    nmacchwso);
  D(nmacchwsodot    ,0x100005DD,    Form::kXOForm,    nmacchwso.);
  D(nmachhw         ,0x1000005C,    Form::kXOForm,    nmachhw);
  D(nmachhwdot      ,0x1000005D,    Form::kXOForm,    nmachhw.);
  D(nmachhwo        ,0x1000045C,    Form::kXOForm,    nmachhwo);
  D(nmachhwodot     ,0x1000045D,    Form::kXOForm,    nmachhwo.);
  D(nmachhws        ,0x100000DC,    Form::kXOForm,    nmachhws);
  D(nmachhwsdot     ,0x100000DD,    Form::kXOForm,    nmachhws.);
  D(nmachhwso       ,0x100004DC,    Form::kXOForm,    nmachhwso);
  D(nmachhwsodot    ,0x100004DD,    Form::kXOForm,    nmachhwso.);
  D(nmaclhw         ,0x1000035C,    Form::kXOForm,    nmaclhw);
  D(nmaclhwdot      ,0x1000035D,    Form::kXOForm,    nmaclhw.);
  D(nmaclhwo        ,0x1000075C,    Form::kXOForm,    nmaclhwo);
  D(nmaclhwodot     ,0x1000075D,    Form::kXOForm,    nmaclhwo.);
  D(nmaclhws        ,0x100003DC,    Form::kXOForm,    nmaclhws);
  D(nmaclhwsdot     ,0x100003DD,    Form::kXOForm,    nmaclhws.);
  D(nmaclhwso       ,0x100007DC,    Form::kXOForm,    nmaclhwso);
  D(nmaclhwsodot    ,0x100007DD,    Form::kXOForm,    nmaclhwso.);
  D(nor             ,0x7C0000F8,    Form::kXForm,     nor);
  D(nordot          ,0x7C0000F9,    Form::kXForm,     nor.);
  D(or              ,0x7C000378,    Form::kXForm,     or);
  D(ordot           ,0x7C000379,    Form::kXForm,     or.);
  D(orc             ,0x7C000338,    Form::kXForm,     orc);
  D(orcdot          ,0x7C000339,    Form::kXForm,     orc.);
  D(ori             ,0x60000000,    Form::kDForm,     ori);
  D(oris            ,0x64000000,    Form::kDForm,     oris);
  D(popcntb         ,0x7C0000F4,    Form::kXForm,     popcntb);
  D(popcntd         ,0x7C0003F4,    Form::kXForm,     popcntd);
  D(popcntw         ,0x7C0002F4,    Form::kXForm,     popcntw);
  D(prtyd           ,0x7C000174,    Form::kXForm,     prtyd);
  D(prtyw           ,0x7C000134,    Form::kXForm,     prtyw);
  D(rfci            ,0x4C000066,    Form::kXLForm,    rfci);
  D(rfdi            ,0x4C00004E,    Form::kXForm,     rfdi);
  D(rfebb           ,0x4C000124,    Form::kXLForm,    rfebb);
  D(rfgi            ,0x4C0000CC,    Form::kXLForm,    rfgi);
  D(rfi             ,0x4C000064,    Form::kXLForm,    rfi);
  D(rfid            ,0x4C000024,    Form::kXLForm,    rfid);
  D(rfmci           ,0x4C00004C,    Form::kXLForm,    rfmci);
  D(rldcl           ,0x78000010,    Form::kMDSForm,   rldcl);
  D(rldcldot        ,0x78000011,    Form::kMDSForm,   rldcl.);
  D(rldcr           ,0x78000012,    Form::kMDSForm,   rldcr);
  D(rldcrdot        ,0x78000013,    Form::kMDSForm,   rldcr.);
  D(rldic           ,0x78000008,    Form::kMDForm,    rldic);
  D(rldicdot        ,0x78000009,    Form::kMDForm,    rldic.);
  D(rldicl          ,0x78000000,    Form::kMDForm,    rldicl);
  D(rldicldot       ,0x78000001,    Form::kMDForm,    rldicl.);
  D(rldicr          ,0x78000004,    Form::kMDForm,    rldicr);
  D(rldicrdot       ,0x78000005,    Form::kMDForm,    rldicr.);
  D(rldimi          ,0x7800000C,    Form::kMDForm,    rldimi);
  D(rldimidot       ,0x7800000D,    Form::kMDForm,    rldimi.);
  D(rlwimi          ,0x50000000,    Form::kMForm,     rlwimi);
  D(rlwimidot       ,0x50000001,    Form::kMForm,     rlwimi.);
  D(rlwinm          ,0x54000000,    Form::kMForm,     rlwinm);
  D(rlwinmdot       ,0x54000001,    Form::kMForm,     rlwinm.);
  D(rlwnm           ,0x5C000000,    Form::kMForm,     rlwnm);
  D(rlwnmdot        ,0x5C000001,    Form::kMForm,     rlwnm.);
  D(rvwinkle        ,0x4C0003E4,    Form::kXLForm,    rvwinkle);
  D(sc              ,0x44000002,    Form::kSCForm,    sc);
  D(slbfeedot       ,0x7C0007A7,    Form::kXForm,     slbfee.);
  D(slbia           ,0x7C0003E4,    Form::kXForm,     slbia);
  D(slbie           ,0x7C000364,    Form::kXForm,     slbie);
  D(slbmfee         ,0x7C000726,    Form::kXForm,     slbmfee);
  D(slbmfev         ,0x7C0006A6,    Form::kXForm,     slbmfev);
  D(slbmte          ,0x7C000324,    Form::kXForm,     slbmte);
  D(sld             ,0x7C000036,    Form::kXForm,     sld);
  D(slddot          ,0x7C000037,    Form::kXForm,     sld.);
  D(sleep           ,0x4C0003A4,    Form::kXLForm,    sleep);
  D(slw             ,0x7C000030,    Form::kXForm,     slw);
  D(slwdot          ,0x7C000031,    Form::kXForm,     slw.);
  D(srad            ,0x7C000634,    Form::kXForm,     srad);
  D(sraddot         ,0x7C000635,    Form::kXForm,     srad.);
  D(sradi           ,0x7C000674,    Form::kXSForm,    sradi);
  D(sradidot        ,0x7C000675,    Form::kXSForm,    sradi.);
  D(sraw            ,0x7C000630,    Form::kXForm,     sraw);
  D(srawdot         ,0x7C000631,    Form::kXForm,     sraw.);
  D(srawi           ,0x7C000670,    Form::kXForm,     srawi);
  D(srawidot        ,0x7C000671,    Form::kXForm,     srawi.);
  D(srd             ,0x7C000436,    Form::kXForm,     srd);
  D(srddot          ,0x7C000437,    Form::kXForm,     srd.);
  D(srw             ,0x7C000430,    Form::kXForm,     srw);
  D(srwdot          ,0x7C000431,    Form::kXForm,     srw.);
  D(stb             ,0x98000000,    Form::kDForm,     stb);
  D(stbcix          ,0x7C0007AA,    Form::kXForm,     stbcix);
  D(stbcxdot        ,0x7C00056D,    Form::kXForm,     stbcx.);
  D(stbdx           ,0x7C000506,    Form::kXForm,     stbdx);
  D(stbepx          ,0x7C0001BE,    Form::kXForm,     stbepx);
  D(stbu            ,0x9C000000,    Form::kDForm,     stbu);
  D(stbux           ,0x7C0001EE,    Form::kXForm,     stbux);
  D(stbx            ,0x7C0001AE,    Form::kXForm,     stbx);
  D(std             ,0xF8000000,    Form::kDSForm,    std);
  D(stdbrx          ,0x7C000528,    Form::kXForm,     stdbrx);
  D(stdcix          ,0x7C0007EA,    Form::kXForm,     stdcix);
  D(stdcxdot        ,0x7C0001AD,    Form::kXForm,     stdcx.);
  D(stddx           ,0x7C0005C6,    Form::kXForm,     stddx);
  D(stdepx          ,0x7C00013A,    Form::kXForm,     stdepx);
  D(stdu            ,0xF8000001,    Form::kDSForm,    stdu);
  D(stdux           ,0x7C00016A,    Form::kXForm,     stdux);
  D(stdx            ,0x7C00012A,    Form::kXForm,     stdx);
  D(stfd            ,0xD8000000,    Form::kDForm,     stfd);
  D(stfddx          ,0x7C000746,    Form::kXForm,     stfddx);
  D(stfdepx         ,0x7C0005BE,    Form::kXForm,     stfdepx);
  D(stfdp           ,0xF4000000,    Form::kDSForm,    stfdp);
  D(stfdpx          ,0x7C00072E,    Form::kXForm,     stfdpx);
  D(stfdu           ,0xDC000000,    Form::kDForm,     stfdu);
  D(stfdux          ,0x7C0005EE,    Form::kXForm,     stfdux);
  D(stfdx           ,0x7C0005AE,    Form::kXForm,     stfdx);
  D(stfiwx          ,0x7C0007AE,    Form::kXForm,     stfiwx);
  D(stfs            ,0xD0000000,    Form::kDForm,     stfs);
  D(stfsu           ,0xD4000000,    Form::kDForm,     stfsu);
  D(stfsux          ,0x7C00056E,    Form::kXForm,     stfsux);
  D(stfsx           ,0x7C00052E,    Form::kXForm,     stfsx);
  D(sth             ,0xB0000000,    Form::kDForm,     sth);
  D(sthbrx          ,0x7C00072C,    Form::kXForm,     sthbrx);
  D(sthcix          ,0x7C00076A,    Form::kXForm,     sthcix);
  D(sthcxdot        ,0x7C0005AD,    Form::kXForm,     sthcx.);
  D(sthdx           ,0x7C000546,    Form::kXForm,     sthdx);
  D(sthepx          ,0x7C00033E,    Form::kXForm,     sthepx);
  D(sthu            ,0xB4000000,    Form::kDForm,     sthu);
  D(sthux           ,0x7C00036E,    Form::kXForm,     sthux);
  D(sthx            ,0x7C00032E,    Form::kXForm,     sthx);
  D(stmw            ,0xBC000000,    Form::kDForm,     stmw);
  D(stq             ,0xF8000002,    Form::kDSForm,    stq);
  D(stqcxdot        ,0x7C00016D,    Form::kXForm,     stqcx.);
  D(stswi           ,0x7C0005AA,    Form::kXForm,     stswi);
  D(stswx           ,0x7C00052A,    Form::kXForm,     stswx);
  D(stvebx          ,0x7C00010E,    Form::kXForm,     stvebx);
  D(stvehx          ,0x7C00014E,    Form::kXForm,     stvehx);
  D(stvepx          ,0x7C00064E,    Form::kXForm,     stvepx);
  D(stvepxl         ,0x7C00060E,    Form::kXForm,     stvepxl);
  D(stvewx          ,0x7C00018E,    Form::kXForm,     stvewx);
  D(stvx            ,0x7C0001CE,    Form::kXForm,     stvx);
  D(stvxl           ,0x7C0003CE,    Form::kXForm,     stvxl);
  D(stw             ,0x90000000,    Form::kDForm,     stw);
  D(stwbrx          ,0x7C00052C,    Form::kXForm,     stwbrx);
  D(stwcix          ,0x7C00072A,    Form::kXForm,     stwcix);
  D(stwcxdot        ,0x7C00012D,    Form::kXForm,     stwcx.);
  D(stwdx           ,0x7C000586,    Form::kXForm,     stwdx);
  D(stwepx          ,0x7C00013E,    Form::kXForm,     stwepx);
  D(stwu            ,0x94000000,    Form::kDForm,     stwu);
  D(stwux           ,0x7C00016E,    Form::kXForm,     stwux);
  D(stwx            ,0x7C00012E,    Form::kXForm,     stwx);
  D(stxsdx          ,0x7C000598,    Form::kXX1Form,   stxsdx);
  D(stxsiwx         ,0x7C000118,    Form::kXX1Form,   stxsiwx);
  D(stxsspx         ,0x7C000518,    Form::kXX1Form,   stxsspx);
  D(stxvd2x         ,0x7C000798,    Form::kXX1Form,   stxvd2x);
  D(stxvw4x         ,0x7C000718,    Form::kXX1Form,   stxvw4x);
  D(subf            ,0x7C000050,    Form::kXOForm,    subf);
  D(subfdot         ,0x7C000051,    Form::kXOForm,    subf.);
  D(subfc           ,0x7C000010,    Form::kXOForm,    subfc);
  D(subfcdot        ,0x7C000011,    Form::kXOForm,    subfc.);
  D(subfco          ,0x7C000410,    Form::kXOForm,    subfco);
  D(subfcodot       ,0x7C000411,    Form::kXOForm,    subfco.);
  D(subfe           ,0x7C000110,    Form::kXOForm,    subfe);
  D(subfedot        ,0x7C000111,    Form::kXOForm,    subfe.);
  D(subfeo          ,0x7C000510,    Form::kXOForm,    subfeo);
  D(subfeodot       ,0x7C000511,    Form::kXOForm,    subfeo.);
  D(subfic          ,0x20000000,    Form::kDForm,     subfic);
  D(subfme          ,0x7C0001D0,    Form::kXOForm,    subfme);
  D(subfmedot       ,0x7C0001D1,    Form::kXOForm,    subfme.);
  D(subfmeo         ,0x7C0005D0,    Form::kXOForm,    subfmeo);
  D(subfmeodot      ,0x7C0005D1,    Form::kXOForm,    subfmeo.);
  D(subfo           ,0x7C000450,    Form::kXOForm,    subfo);
  D(subfodot        ,0x7C000451,    Form::kXOForm,    subfo.);
  D(subfze          ,0x7C000190,    Form::kXOForm,    subfze);
  D(subfzedot       ,0x7C000191,    Form::kXOForm,    subfze.);
  D(subfzeo         ,0x7C000590,    Form::kXOForm,    subfzeo);
  D(subfzeodot      ,0x7C000591,    Form::kXOForm,    subfzeo.);
  D(sync            ,0x7C0004AC,    Form::kXForm,     sync);
  D(tabortdot       ,0x7C00071D,    Form::kXForm,     tabort.);
  D(tabortdcdot     ,0x7C00065D,    Form::kXForm,     tabortdc.);
  D(tabortdcidot    ,0x7C0006DD,    Form::kXForm,     tabortdci.);
  D(tabortwcdot     ,0x7C00061D,    Form::kXForm,     tabortwc.);
  D(tabortwcidot    ,0x7C00069D,    Form::kXForm,     tabortwci.);
  D(tbegindot       ,0x7C00051D,    Form::kXForm,     tbegin.);
  D(tcheck          ,0x7C00059C,    Form::kXForm,     tcheck);
  D(td              ,0x7C000088,    Form::kXForm,     td);
  D(tdi             ,0x08000000,    Form::kDForm,     tdi);
  D(tenddot         ,0x7C00055C,    Form::kXForm,     tend.);
  D(tlbia           ,0x7C0002E4,    Form::kXForm,     tlbia);
  D(tlbie           ,0x7C000264,    Form::kXForm,     tlbie);
  D(tlbiel          ,0x7C000224,    Form::kXForm,     tlbiel);
  D(tlbilx          ,0x7C000024,    Form::kXForm,     tlbilx);
  D(tlbivax         ,0x7C000624,    Form::kXForm,     tlbivax);
  D(tlbre           ,0x7C000764,    Form::kXForm,     tlbre);
  D(tlbsrxdot       ,0x7C0006A5,    Form::kXForm,     tlbsrx.);
  D(tlbsx           ,0x7C000724,    Form::kXForm,     tlbsx);
  D(tlbsync         ,0x7C00046C,    Form::kXForm,     tlbsync);
  D(tlbwe           ,0x7C0007A4,    Form::kXForm,     tlbwe);
  D(trechkptdot     ,0x7C0007DD,    Form::kXForm,     trechkpt.);
  D(treclaimdot     ,0x7C00075D,    Form::kXForm,     treclaim.);
  D(tsrdot          ,0x7C0005DC,    Form::kXForm,     tsr.);
  D(tw              ,0x7C000008,    Form::kXForm,     tw);
  D(twi             ,0x0C000000,    Form::kDForm,     twi);
  D(vaddcuq         ,0x10000140,    Form::kVXForm,    vaddcuq);
  D(vaddcuw         ,0x10000180,    Form::kVXForm,    vaddcuw);
  D(vaddecuq        ,0x1000003D,    Form::kVAForm,    vaddecuq);
  D(vaddeuqm        ,0x1000003C,    Form::kVAForm,    vaddeuqm);
  D(vaddfp          ,0x1000000A,    Form::kVXForm,    vaddfp);
  D(vaddsbs         ,0x10000300,    Form::kVXForm,    vaddsbs);
  D(vaddshs         ,0x10000340,    Form::kVXForm,    vaddshs);
  D(vaddsws         ,0x10000380,    Form::kVXForm,    vaddsws);
  D(vaddubm         ,0x10000000,    Form::kVXForm,    vaddubm);
  D(vaddubs         ,0x10000200,    Form::kVXForm,    vaddubs);
  D(vaddudm         ,0x100000C0,    Form::kVXForm,    vaddudm);
  D(vadduhm         ,0x10000040,    Form::kVXForm,    vadduhm);
  D(vadduhs         ,0x10000240,    Form::kVXForm,    vadduhs);
  D(vadduqm         ,0x10000100,    Form::kVXForm,    vadduqm);
  D(vadduwm         ,0x10000080,    Form::kVXForm,    vadduwm);
  D(vadduws         ,0x10000280,    Form::kVXForm,    vadduws);
  D(vand            ,0x10000404,    Form::kVXForm,    vand);
  D(vandc           ,0x10000444,    Form::kVXForm,    vandc);
  D(vavgsb          ,0x10000502,    Form::kVXForm,    vavgsb);
  D(vavgsh          ,0x10000542,    Form::kVXForm,    vavgsh);
  D(vavgsw          ,0x10000582,    Form::kVXForm,    vavgsw);
  D(vavgub          ,0x10000402,    Form::kVXForm,    vavgub);
  D(vavguh          ,0x10000442,    Form::kVXForm,    vavguh);
  D(vavguw          ,0x10000482,    Form::kVXForm,    vavguw);
  D(vbpermq         ,0x1000054C,    Form::kVXForm,    vbpermq);
  D(vcfsx           ,0x1000034A,    Form::kVXForm,    vcfsx);
  D(vcfux           ,0x1000030A,    Form::kVXForm,    vcfux);
  D(vcipher         ,0x10000508,    Form::kVXForm,    vcipher);
  D(vcipherlast     ,0x10000509,    Form::kVXForm,    vcipherlast);
  D(vclzb           ,0x10000702,    Form::kVXForm,    vclzb);
  D(vclzd           ,0x100007C2,    Form::kVXForm,    vclzd);
  D(vclzh           ,0x10000742,    Form::kVXForm,    vclzh);
  D(vclzw           ,0x10000782,    Form::kVXForm,    vclzw);
  D(vcmpbfp         ,0x100003C6,    Form::kVCForm,    vcmpbfp);
  D(vcmpbfpdot      ,0x100007C6,    Form::kVCForm,    vcmpbfp.);
  D(vcmpeqfp        ,0x100000C6,    Form::kVCForm,    vcmpeqfp);
  D(vcmpeqfpdot     ,0x100004C6,    Form::kVCForm,    vcmpeqfp.);
  D(vcmpequb        ,0x10000006,    Form::kVCForm,    vcmpequb);
  D(vcmpequbdot     ,0x10000406,    Form::kVCForm,    vcmpequb.);
  D(vcmpequd        ,0x100000C7,    Form::kVCForm,    vcmpequd);
  D(vcmpequddot     ,0x100004C7,    Form::kVCForm,    vcmpequd.);
  D(vcmpequh        ,0x10000046,    Form::kVCForm,    vcmpequh);
  D(vcmpequhdot     ,0x10000446,    Form::kVCForm,    vcmpequh.);
  D(vcmpequw        ,0x10000086,    Form::kVCForm,    vcmpequw);
  D(vcmpequwdot     ,0x10000486,    Form::kVCForm,    vcmpequw.);
  D(vcmpgefp        ,0x100001C6,    Form::kVCForm,    vcmpgefp);
  D(vcmpgefpdot     ,0x100005C6,    Form::kVCForm,    vcmpgefp.);
  D(vcmpgtfp        ,0x100002C6,    Form::kVCForm,    vcmpgtfp);
  D(vcmpgtfpdot     ,0x100006C6,    Form::kVCForm,    vcmpgtfp.);
  D(vcmpgtsb        ,0x10000306,    Form::kVCForm,    vcmpgtsb);
  D(vcmpgtsbdot     ,0x10000706,    Form::kVCForm,    vcmpgtsb.);
  D(vcmpgtsd        ,0x100003C7,    Form::kVCForm,    vcmpgtsd);
  D(vcmpgtsddot     ,0x100007C7,    Form::kVCForm,    vcmpgtsd.);
  D(vcmpgtsh        ,0x10000346,    Form::kVCForm,    vcmpgtsh);
  D(vcmpgtshdot     ,0x10000746,    Form::kVCForm,    vcmpgtsh.);
  D(vcmpgtsw        ,0x10000386,    Form::kVCForm,    vcmpgtsw);
  D(vcmpgtswdot     ,0x10000786,    Form::kVCForm,    vcmpgtsw.);
  D(vcmpgtub        ,0x10000206,    Form::kVCForm,    vcmpgtub);
  D(vcmpgtubdot     ,0x10000606,    Form::kVCForm,    vcmpgtub.);
  D(vcmpgtud        ,0x100002C7,    Form::kVCForm,    vcmpgtud);
  D(vcmpgtuddot     ,0x100006C7,    Form::kVCForm,    vcmpgtud.);
  D(vcmpgtuh        ,0x10000246,    Form::kVCForm,    vcmpgtuh);
  D(vcmpgtuhdot     ,0x10000646,    Form::kVCForm,    vcmpgtuh.);
  D(vcmpgtuw        ,0x10000286,    Form::kVCForm,    vcmpgtuw);
  D(vcmpgtuwdot     ,0x10000686,    Form::kVCForm,    vcmpgtuw.);
  D(vctsxs          ,0x100003CA,    Form::kVXForm,    vctsxs);
  D(vctuxs          ,0x1000038A,    Form::kVXForm,    vctuxs);
  D(veqv            ,0x10000684,    Form::kVXForm,    veqv);
  D(vexptefp        ,0x1000018A,    Form::kVXForm,    vexptefp);
  D(vgbbd           ,0x1000050C,    Form::kVXForm,    vgbbd);
  D(vlogefp         ,0x100001CA,    Form::kVXForm,    vlogefp);
  D(vmaddfp         ,0x1000002E,    Form::kVAForm,    vmaddfp);
  D(vmaxfp          ,0x1000040A,    Form::kVXForm,    vmaxfp);
  D(vmaxsb          ,0x10000102,    Form::kVXForm,    vmaxsb);
  D(vmaxsd          ,0x100001C2,    Form::kVXForm,    vmaxsd);
  D(vmaxsh          ,0x10000142,    Form::kVXForm,    vmaxsh);
  D(vmaxsw          ,0x10000182,    Form::kVXForm,    vmaxsw);
  D(vmaxub          ,0x10000002,    Form::kVXForm,    vmaxub);
  D(vmaxud          ,0x100000C2,    Form::kVXForm,    vmaxud);
  D(vmaxuh          ,0x10000042,    Form::kVXForm,    vmaxuh);
  D(vmaxuw          ,0x10000082,    Form::kVXForm,    vmaxuw);
  D(vmhaddshs       ,0x10000020,    Form::kVAForm,    vmhaddshs);
  D(vmhraddshs      ,0x10000021,    Form::kVAForm,    vmhraddshs);
  D(vminfp          ,0x1000044A,    Form::kVXForm,    vminfp);
  D(vminsb          ,0x10000302,    Form::kVXForm,    vminsb);
  D(vminsd          ,0x100003C2,    Form::kXForm,     vminsd);
  D(vminsh          ,0x10000342,    Form::kVXForm,    vminsh);
  D(vminsw          ,0x10000382,    Form::kVXForm,    vminsw);
  D(vminub          ,0x10000202,    Form::kVXForm,    vminub);
  D(vminud          ,0x100002C2,    Form::kVXForm,    vminud);
  D(vminuh          ,0x10000242,    Form::kVXForm,    vminuh);
  D(vminuw          ,0x10000282,    Form::kVXForm,    vminuw);
  D(vmladduhm       ,0x10000022,    Form::kVAForm,    vmladduhm);
  D(vmrgew          ,0x1000078C,    Form::kVXForm,    vmrgew);
  D(vmrghb          ,0x1000000C,    Form::kVXForm,    vmrghb);
  D(vmrghh          ,0x1000004C,    Form::kVXForm,    vmrghh);
  D(vmrghw          ,0x1000008C,    Form::kVXForm,    vmrghw);
  D(vmrglb          ,0x1000010C,    Form::kVXForm,    vmrglb);
  D(vmrglh          ,0x1000014C,    Form::kVXForm,    vmrglh);
  D(vmrglw          ,0x1000018C,    Form::kVXForm,    vmrglw);
  D(vmrgow          ,0x1000068C,    Form::kVXForm,    vmrgow);
  D(vmsummbm        ,0x10000025,    Form::kVAForm,    vmsummbm);
  D(vmsumshm        ,0x10000028,    Form::kVAForm,    vmsumshm);
  D(vmsumshs        ,0x10000029,    Form::kVAForm,    vmsumshs);
  D(vmsumubm        ,0x10000024,    Form::kVAForm,    vmsumubm);
  D(vmsumuhm        ,0x10000026,    Form::kVAForm,    vmsumuhm);
  D(vmsumuhs        ,0x10000027,    Form::kVAForm,    vmsumuhs);
  D(vmulesb         ,0x10000308,    Form::kVXForm,    vmulesb);
  D(vmulesh         ,0x10000348,    Form::kVXForm,    vmulesh);
  D(vmulesw         ,0x10000388,    Form::kVXForm,    vmulesw);
  D(vmuleub         ,0x10000208,    Form::kVXForm,    vmuleub);
  D(vmuleuh         ,0x10000248,    Form::kVXForm,    vmuleuh);
  D(vmuleuw         ,0x10000288,    Form::kVXForm,    vmuleuw);
  D(vmulosb         ,0x10000108,    Form::kVXForm,    vmulosb);
  D(vmulosh         ,0x10000148,    Form::kVXForm,    vmulosh);
  D(vmulosw         ,0x10000188,    Form::kVXForm,    vmulosw);
  D(vmuloub         ,0x10000008,    Form::kVXForm,    vmuloub);
  D(vmulouh         ,0x10000048,    Form::kVXForm,    vmulouh);
  D(vmulouw         ,0x10000088,    Form::kVXForm,    vmulouw);
  D(vmuluwm         ,0x10000089,    Form::kVXForm,    vmuluwm);
  D(vnand           ,0x10000584,    Form::kVXForm,    vnand);
  D(vncipher        ,0x10000548,    Form::kVXForm,    vncipher);
  D(vncipherlast    ,0x10000549,    Form::kVXForm,    vncipherlast);
  D(vnmsubfp        ,0x1000002F,    Form::kVAForm,    vnmsubfp);
  D(vnor            ,0x10000504,    Form::kVXForm,    vnor);
  D(vor             ,0x10000484,    Form::kVXForm,    vor);
  D(vorc            ,0x10000544,    Form::kVXForm,    vorc);
  D(vperm           ,0x1000002B,    Form::kVAForm,    vperm);
  D(vpermxor        ,0x1000002D,    Form::kVAForm,    vpermxor);
  D(vpkpx           ,0x1000030E,    Form::kVXForm,    vpkpx);
  D(vpksdss         ,0x100005CE,    Form::kVXForm,    vpksdss);
  D(vpksdus         ,0x1000054E,    Form::kVXForm,    vpksdus);
  D(vpkshss         ,0x1000018E,    Form::kVXForm,    vpkshss);
  D(vpkshus         ,0x1000010E,    Form::kVXForm,    vpkshus);
  D(vpkswss         ,0x100001CE,    Form::kVXForm,    vpkswss);
  D(vpkswus         ,0x1000014E,    Form::kVXForm,    vpkswus);
  D(vpkudum         ,0x1000044E,    Form::kVXForm,    vpkudum);
  D(vpkudus         ,0x100004CE,    Form::kVXForm,    vpkudus);
  D(vpkuhum         ,0x1000000E,    Form::kVXForm,    vpkuhum);
  D(vpkuhus         ,0x1000008E,    Form::kVXForm,    vpkuhus);
  D(vpkuwum         ,0x1000004E,    Form::kVXForm,    vpkuwum);
  D(vpkuwus         ,0x100000CE,    Form::kVXForm,    vpkuwus);
  D(vpmsumb         ,0x10000408,    Form::kVXForm,    vpmsumb);
  D(vpmsumd         ,0x100004C8,    Form::kVXForm,    vpmsumd);
  D(vpmsumh         ,0x10000448,    Form::kVXForm,    vpmsumh);
  D(vpmsumw         ,0x10000488,    Form::kVXForm,    vpmsumw);
  D(vpopcntb        ,0x10000703,    Form::kVXForm,    vpopcntb);
  D(vpopcntd        ,0x100007C3,    Form::kVXForm,    vpopcntd);
  D(vpopcnth        ,0x10000743,    Form::kVXForm,    vpopcnth);
  D(vpopcntw        ,0x10000783,    Form::kVXForm,    vpopcntw);
  D(vrefp           ,0x1000010A,    Form::kVXForm,    vrefp);
  D(vrfim           ,0x100002CA,    Form::kVXForm,    vrfim);
  D(vrfin           ,0x1000020A,    Form::kVXForm,    vrfin);
  D(vrfip           ,0x1000028A,    Form::kVXForm,    vrfip);
  D(vrfiz           ,0x1000024A,    Form::kVXForm,    vrfiz);
  D(vrlb            ,0x10000004,    Form::kVXForm,    vrlb);
  D(vrld            ,0x100000C4,    Form::kVXForm,    vrld);
  D(vrlh            ,0x10000044,    Form::kVXForm,    vrlh);
  D(vrlw            ,0x10000084,    Form::kVXForm,    vrlw);
  D(vrsqrtefp       ,0x1000014A,    Form::kVXForm,    vrsqrtefp);
  D(vsbox           ,0x100005C8,    Form::kVXForm,    vsbox);
  D(vsel            ,0x1000002A,    Form::kVAForm,    vsel);
  D(vshasigmad      ,0x100006C2,    Form::kVXForm,    vshasigmad);
  D(vshasigmaw      ,0x10000682,    Form::kVXForm,    vshasigmaw);
  D(vsl             ,0x100001C4,    Form::kVXForm,    vsl);
  D(vslb            ,0x10000104,    Form::kVXForm,    vslb);
  D(vsld            ,0x100005C4,    Form::kVXForm,    vsld);
  D(vsldoi          ,0x1000002C,    Form::kVAForm,    vsldoi);
  D(vslh            ,0x10000144,    Form::kVXForm,    vslh);
  D(vslo            ,0x1000040C,    Form::kVXForm,    vslo);
  D(vslw            ,0x10000184,    Form::kVXForm,    vslw);
  D(vspltb          ,0x1000020C,    Form::kVXForm,    vspltb);
  D(vsplth          ,0x1000024C,    Form::kVXForm,    vsplth);
  D(vspltisb        ,0x1000030C,    Form::kVXForm,    vspltisb);
  D(vspltish        ,0x1000034C,    Form::kVXForm,    vspltish);
  D(vspltisw        ,0x1000038C,    Form::kVXForm,    vspltisw);
  D(vspltw          ,0x1000028C,    Form::kVXForm,    vspltw);
  D(vsr             ,0x100002C4,    Form::kVXForm,    vsr);
  D(vsrab           ,0x10000304,    Form::kVXForm,    vsrab);
  D(vsrad           ,0x100003C4,    Form::kVXForm,    vsrad);
  D(vsrah           ,0x10000344,    Form::kVXForm,    vsrah);
  D(vsraw           ,0x10000384,    Form::kVXForm,    vsraw);
  D(vsrb            ,0x10000204,    Form::kVXForm,    vsrb);
  D(vsrd            ,0x100006C4,    Form::kVXForm,    vsrd);
  D(vsrh            ,0x10000244,    Form::kVXForm,    vsrh);
  D(vsro            ,0x1000044C,    Form::kVXForm,    vsro);
  D(vsrw            ,0x10000284,    Form::kVXForm,    vsrw);
  D(vsubcuq         ,0x10000540,    Form::kVXForm,    vsubcuq);
  D(vsubcuw         ,0x10000580,    Form::kVXForm,    vsubcuw);
  D(vsubecuq        ,0x1000003F,    Form::kVAForm,    vsubecuq);
  D(vsubeuqm        ,0x1000003E,    Form::kVAForm,    vsubeuqm);
  D(vsubfp          ,0x1000004A,    Form::kVXForm,    vsubfp);
  D(vsubsbs         ,0x10000700,    Form::kVXForm,    vsubsbs);
  D(vsubshs         ,0x10000740,    Form::kVXForm,    vsubshs);
  D(vsubsws         ,0x10000780,    Form::kVXForm,    vsubsws);
  D(vsububm         ,0x10000400,    Form::kVXForm,    vsububm);
  D(vsububs         ,0x10000600,    Form::kVXForm,    vsububs);
  D(vsubudm         ,0x100004C0,    Form::kVXForm,    vsubudm);
  D(vsubuhm         ,0x10000440,    Form::kVXForm,    vsubuhm);
  D(vsubuhs         ,0x10000640,    Form::kVXForm,    vsubuhs);
  D(vsubuqm         ,0x10000500,    Form::kVXForm,    vsubuqm);
  D(vsubuwm         ,0x10000480,    Form::kVXForm,    vsubuwm);
  D(vsubuws         ,0x10000680,    Form::kVXForm,    vsubuws);
  D(vsum2sws        ,0x10000688,    Form::kVXForm,    vsum2sws);
  D(vsum4sbs        ,0x10000708,    Form::kVXForm,    vsum4sbs);
  D(vsum4shs        ,0x10000648,    Form::kVXForm,    vsum4shs);
  D(vsum4ubs        ,0x10000608,    Form::kVXForm,    vsum4ubs);
  D(vsumsws         ,0x10000788,    Form::kVXForm,    vsumsws);
  D(vupkhpx         ,0x1000034E,    Form::kVXForm,    vupkhpx);
  D(vupkhsb         ,0x1000020E,    Form::kVXForm,    vupkhsb);
  D(vupkhsh         ,0x1000024E,    Form::kVXForm,    vupkhsh);
  D(vupkhsw         ,0x1000064E,    Form::kVXForm,    vupkhsw);
  D(vupklpx         ,0x100003CE,    Form::kVXForm,    vupklpx);
  D(vupklsb         ,0x1000028E,    Form::kVXForm,    vupklsb);
  D(vupklsh         ,0x100002CE,    Form::kVXForm,    vupklsh);
  D(vupklsw         ,0x100006CE,    Form::kVXForm,    vupklsw);
  D(vxor            ,0x100004C4,    Form::kVXForm,    vxor);
  D(wait            ,0x7C00007C,    Form::kXForm,     wait);
  D(wrtee           ,0x7C000106,    Form::kXForm,     wrtee);
  D(wrteei          ,0x7C000146,    Form::kXForm,     wrteei);
  D(xnop            ,0x68000000,    Form::kXForm,     xnop);
  D(xor             ,0x7C000278,    Form::kXForm,     xor);
  D(xordot          ,0x7C000279,    Form::kXForm,     xor.);
  D(xori            ,0x68000000,    Form::kDForm,     xori);
  D(xoris           ,0x6C000000,    Form::kDForm,     xoris);
  D(xsabsdp         ,0xF0000564,    Form::kXX2Form,   xsabsdp);
  D(xsadddp         ,0xF0000100,    Form::kXX3Form,   xsadddp);
  D(xsaddsp         ,0xF0000000,    Form::kXX3Form,   xsaddsp);
  D(xscmpodp        ,0xF0000158,    Form::kXX3Form,   xscmpodp);
  D(xscmpudp        ,0xF0000118,    Form::kXX3Form,   xscmpudp);
  D(xscpsgndp       ,0xF0000580,    Form::kXX3Form,   xscpsgndp);
  D(xscvdpsp        ,0xF0000424,    Form::kXX2Form,   xscvdpsp);
  D(xscvdpspn       ,0xF000042C,    Form::kXX2Form,   xscvdpspn);
  D(xscvdpsxds      ,0xF0000560,    Form::kXX2Form,   xscvdpsxds);
  D(xscvdpsxws      ,0xF0000160,    Form::kXX2Form,   xscvdpsxws);
  D(xscvdpuxds      ,0xF0000520,    Form::kXX2Form,   xscvdpuxds);
  D(xscvdpuxws      ,0xF0000120,    Form::kXX2Form,   xscvdpuxws);
  D(xscvspdp        ,0xF0000524,    Form::kXX2Form,   xscvspdp);
  D(xscvspdpn       ,0xF000052C,    Form::kXX2Form,   xscvspdpn);
  D(xscvsxddp       ,0xF00005E0,    Form::kXX2Form,   xscvsxddp);
  D(xscvsxdsp       ,0xF00004E0,    Form::kXX2Form,   xscvsxdsp);
  D(xscvuxddp       ,0xF00005A0,    Form::kXX2Form,   xscvuxddp);
  D(xscvuxdsp       ,0xF00004A0,    Form::kXX2Form,   xscvuxdsp);
  D(xsdivdp         ,0xF00001C0,    Form::kXX3Form,   xsdivdp);
  D(xsdivsp         ,0xF00000C0,    Form::kXX3Form,   xsdivsp);
  D(xsmaddadp       ,0xF0000108,    Form::kXX3Form,   xsmaddadp);
  D(xsmaddasp       ,0xF0000008,    Form::kXX3Form,   xsmaddasp);
  D(xsmaddmdp       ,0xF0000148,    Form::kXX3Form,   xsmaddmdp);
  D(xsmaddmsp       ,0xF0000048,    Form::kXX3Form,   xsmaddmsp);
  D(xsmaxdp         ,0xF0000500,    Form::kXX3Form,   xsmaxdp);
  D(xsmindp         ,0xF0000540,    Form::kXX3Form,   xsmindp);
  D(xsmsubadp       ,0xF0000188,    Form::kXX3Form,   xsmsubadp);
  D(xsmsubasp       ,0xF0000088,    Form::kXX3Form,   xsmsubasp);
  D(xsmsubmdp       ,0xF00001C8,    Form::kXX3Form,   xsmsubmdp);
  D(xsmsubmsp       ,0xF00000C8,    Form::kXX3Form,   xsmsubmsp);
  D(xsmuldp         ,0xF0000180,    Form::kXX3Form,   xsmuldp);
  D(xsmulsp         ,0xF0000080,    Form::kXX3Form,   xsmulsp);
  D(xsnabsdp        ,0xF00005A4,    Form::kXX2Form,   xsnabsdp);
  D(xsnegdp         ,0xF00005E4,    Form::kXX2Form,   xsnegdp);
  D(xsnmaddadp      ,0xF0000508,    Form::kXX3Form,   xsnmaddadp);
  D(xsnmaddasp      ,0xF0000408,    Form::kXX3Form,   xsnmaddasp);
  D(xsnmaddmdp      ,0xF0000548,    Form::kXX3Form,   xsnmaddmdp);
  D(xsnmaddmsp      ,0xF0000448,    Form::kXX3Form,   xsnmaddmsp);
  D(xsnmsubadp      ,0xF0000588,    Form::kXX3Form,   xsnmsubadp);
  D(xsnmsubasp      ,0xF0000488,    Form::kXX3Form,   xsnmsubasp);
  D(xsnmsubmdp      ,0xF00005C8,    Form::kXX3Form,   xsnmsubmdp);
  D(xsnmsubmsp      ,0xF00004C8,    Form::kXX3Form,   xsnmsubmsp);
  D(xsrdpi          ,0xF0000124,    Form::kXX2Form,   xsrdpi);
  D(xsrdpic         ,0xF00001AC,    Form::kXX2Form,   xsrdpic);
  D(xsrdpim         ,0xF00001E4,    Form::kXX2Form,   xsrdpim);
  D(xsrdpip         ,0xF00001A4,    Form::kXX2Form,   xsrdpip);
  D(xsrdpiz         ,0xF0000164,    Form::kXX2Form,   xsrdpiz);
  D(xsredp          ,0xF0000168,    Form::kXX1Form,   xsredp);
  D(xsresp          ,0xF0000068,    Form::kXX2Form,   xsresp);
  D(xsrsp           ,0xF0000464,    Form::kXX2Form,   xsrsp);
  D(xsrsqrtedp      ,0xF0000128,    Form::kXX2Form,   xsrsqrtedp);
  D(xsrsqrtesp      ,0xF0000028,    Form::kXX2Form,   xsrsqrtesp);
  D(xssqrtdp        ,0xF000012C,    Form::kXX2Form,   xssqrtdp);
  D(xssqrtsp        ,0xF000002C,    Form::kXX2Form,   xssqrtsp);
  D(xssubdp         ,0xF0000140,    Form::kXX3Form,   xssubdp);
  D(xssubsp         ,0xF0000040,    Form::kXX3Form,   xssubsp);
  D(xstdivdp        ,0xF00001E8,    Form::kXX3Form,   xstdivdp);
  D(xstsqrtdp       ,0xF00001A8,    Form::kXX2Form,   xstsqrtdp);
  D(xvabsdp         ,0xF0000764,    Form::kXX2Form,   xvabsdp);
  D(xvabssp         ,0xF0000664,    Form::kXX2Form,   xvabssp);
  D(xvadddp         ,0xF0000300,    Form::kXX3Form,   xvadddp);
  D(xvaddsp         ,0xF0000200,    Form::kXX3Form,   xvaddsp);
  D(xvcmpeqdp       ,0xF0000318,    Form::kXX3Form,   xvcmpeqdp);
  D(xvcmpeqdpdot    ,0xF0000718,    Form::kXX3Form,   xvcmpeqdp.);
  D(xvcmpeqsp       ,0xF0000218,    Form::kXX3Form,   xvcmpeqsp);
  D(xvcmpeqspdot    ,0xF0000618,    Form::kXX3Form,   xvcmpeqsp.);
  D(xvcmpgedp       ,0xF0000398,    Form::kXX3Form,   xvcmpgedp);
  D(xvcmpgedpdot    ,0xF0000798,    Form::kXX3Form,   xvcmpgedp.);
  D(xvcmpgesp       ,0xF0000298,    Form::kXX3Form,   xvcmpgesp);
  D(xvcmpgespdot    ,0xF0000698,    Form::kXX3Form,   xvcmpgesp.);
  D(xvcmpgtdp       ,0xF0000358,    Form::kXX3Form,   xvcmpgtdp);
  D(xvcmpgtdpdot    ,0xF0000758,    Form::kXX3Form,   xvcmpgtdp.);
  D(xvcmpgtsp       ,0xF0000258,    Form::kXX3Form,   xvcmpgtsp);
  D(xvcmpgtspdot    ,0xF0000658,    Form::kXX3Form,   xvcmpgtsp.);
  D(xvcpsgndp       ,0xF0000780,    Form::kXX3Form,   xvcpsgndp);
  D(xvcpsgnsp       ,0xF0000680,    Form::kXX3Form,   xvcpsgnsp);
  D(xvcvdpsp        ,0xF0000624,    Form::kXX2Form,   xvcvdpsp);
  D(xvcvdpsxds      ,0xF0000760,    Form::kXX2Form,   xvcvdpsxds);
  D(xvcvdpsxws      ,0xF0000360,    Form::kXX2Form,   xvcvdpsxws);
  D(xvcvdpuxds      ,0xF0000720,    Form::kXX2Form,   xvcvdpuxds);
  D(xvcvdpuxws      ,0xF0000320,    Form::kXX2Form,   xvcvdpuxws);
  D(xvcvspdp        ,0xF0000724,    Form::kXX2Form,   xvcvspdp);
  D(xvcvspsxds      ,0xF0000660,    Form::kXX2Form,   xvcvspsxds);
  D(xvcvspsxws      ,0xF0000260,    Form::kXX2Form,   xvcvspsxws);
  D(xvcvspuxds      ,0xF0000620,    Form::kXX2Form,   xvcvspuxds);
  D(xvcvspuxws      ,0xF0000220,    Form::kXX2Form,   xvcvspuxws);
  D(xvcvsxddp       ,0xF00007E0,    Form::kXX2Form,   xvcvsxddp);
  D(xvcvsxdsp       ,0xF00006E0,    Form::kXX2Form,   xvcvsxdsp);
  D(xvcvsxwdp       ,0xF00003E0,    Form::kXX2Form,   xvcvsxwdp);
  D(xvcvsxwsp       ,0xF00002E0,    Form::kXX2Form,   xvcvsxwsp);
  D(xvcvuxddp       ,0xF00007A0,    Form::kXX2Form,   xvcvuxddp);
  D(xvcvuxdsp       ,0xF00006A0,    Form::kXX2Form,   xvcvuxdsp);
  D(xvcvuxwdp       ,0xF00003A0,    Form::kXX2Form,   xvcvuxwdp);
  D(xvcvuxwsp       ,0xF00002A0,    Form::kXX2Form,   xvcvuxwsp);
  D(xvdivdp         ,0xF00003C0,    Form::kXX3Form,   xvdivdp);
  D(xvdivsp         ,0xF00002C0,    Form::kXX3Form,   xvdivsp);
  D(xvmaddadp       ,0xF0000308,    Form::kXX3Form,   xvmaddadp);
  D(xvmaddasp       ,0xF0000208,    Form::kXX3Form,   xvmaddasp);
  D(xvmaddmdp       ,0xF0000348,    Form::kXX3Form,   xvmaddmdp);
  D(xvmaddmsp       ,0xF0000248,    Form::kXX3Form,   xvmaddmsp);
  D(xvmaxdp         ,0xF0000700,    Form::kXX3Form,   xvmaxdp);
  D(xvmaxsp         ,0xF0000600,    Form::kXX3Form,   xvmaxsp);
  D(xvmindp         ,0xF0000740,    Form::kXX3Form,   xvmindp);
  D(xvminsp         ,0xF0000640,    Form::kXX3Form,   xvminsp);
  D(xvmsubadp       ,0xF0000388,    Form::kXX3Form,   xvmsubadp);
  D(xvmsubasp       ,0xF0000288,    Form::kXX3Form,   xvmsubasp);
  D(xvmsubmdp       ,0xF00003C8,    Form::kXX3Form,   xvmsubmdp);
  D(xvmsubmsp       ,0xF00002C8,    Form::kXX3Form,   xvmsubmsp);
  D(xvmuldp         ,0xF0000380,    Form::kXX3Form,   xvmuldp);
  D(xvmulsp         ,0xF0000280,    Form::kXX3Form,   xvmulsp);
  D(xvnabsdp        ,0xF00007A4,    Form::kXX2Form,   xvnabsdp);
  D(xvnabssp        ,0xF00006A4,    Form::kXX2Form,   xvnabssp);
  D(xvnegdp         ,0xF00007E4,    Form::kXX2Form,   xvnegdp);
  D(xvnegsp         ,0xF00006E4,    Form::kXX2Form,   xvnegsp);
  D(xvnmaddadp      ,0xF0000708,    Form::kXX3Form,   xvnmaddadp);
  D(xvnmaddasp      ,0xF0000608,    Form::kXX3Form,   xvnmaddasp);
  D(xvnmaddmdp      ,0xF0000748,    Form::kXX3Form,   xvnmaddmdp);
  D(xvnmaddmsp      ,0xF0000648,    Form::kXX3Form,   xvnmaddmsp);
  D(xvnmsubadp      ,0xF0000788,    Form::kXX3Form,   xvnmsubadp);
  D(xvnmsubasp      ,0xF0000688,    Form::kXX3Form,   xvnmsubasp);
  D(xvnmsubmdp      ,0xF00007C8,    Form::kXX3Form,   xvnmsubmdp);
  D(xvnmsubmsp      ,0xF00006C8,    Form::kXX3Form,   xvnmsubmsp);
  D(xvrdpi          ,0xF0000324,    Form::kXX2Form,   xvrdpi);
  D(xvrdpic         ,0xF00003AC,    Form::kXX2Form,   xvrdpic);
  D(xvrdpim         ,0xF00003E4,    Form::kXX2Form,   xvrdpim);
  D(xvrdpip         ,0xF00003A4,    Form::kXX2Form,   xvrdpip);
  D(xvrdpiz         ,0xF0000364,    Form::kXX2Form,   xvrdpiz);
  D(xvredp          ,0xF0000368,    Form::kXX2Form,   xvredp);
  D(xvresp          ,0xF0000268,    Form::kXX2Form,   xvresp);
  D(xvrspi          ,0xF0000224,    Form::kXX2Form,   xvrspi);
  D(xvrspic         ,0xF00002AC,    Form::kXX2Form,   xvrspic);
  D(xvrspim         ,0xF00002E4,    Form::kXX2Form,   xvrspim);
  D(xvrspip         ,0xF00002A4,    Form::kXX2Form,   xvrspip);
  D(xvrspiz         ,0xF0000264,    Form::kXX2Form,   xvrspiz);
  D(xvrsqrtedp      ,0xF0000328,    Form::kXX2Form,   xvrsqrtedp);
  D(xvrsqrtesp      ,0xF0000228,    Form::kXX2Form,   xvrsqrtesp);
  D(xvsqrtdp        ,0xF000032C,    Form::kXX2Form,   xvsqrtdp);
  D(xvsqrtsp        ,0xF000022C,    Form::kXX2Form,   xvsqrtsp);
  D(xvsubdp         ,0xF0000340,    Form::kXX3Form,   xvsubdp);
  D(xvsubsp         ,0xF0000240,    Form::kXX3Form,   xvsubsp);
  D(xvtdivdp        ,0xF00003E8,    Form::kXX3Form,   xvtdivdp);
  D(xvtdivsp        ,0xF00002E8,    Form::kXX3Form,   xvtdivsp);
  D(xvtsqrtdp       ,0xF00003A8,    Form::kXX2Form,   xvtsqrtdp);
  D(xvtsqrtsp       ,0xF00002A8,    Form::kXX2Form,   xvtsqrtsp);
  D(xxland          ,0xF0000410,    Form::kXX3Form,   xxland);
  D(xxlandc         ,0xF0000450,    Form::kXX3Form,   xxlandc);
  D(xxleqv          ,0xF00005D0,    Form::kXX3Form,   xxleqv);
  D(xxlnand         ,0xF0000590,    Form::kXX3Form,   xxlnand);
  D(xxlnor          ,0xF0000510,    Form::kXX3Form,   xxlnor);
  D(xxlor           ,0xF0000490,    Form::kXX3Form,   xxlor);
  D(xxlorc          ,0xF0000550,    Form::kXX3Form,   xxlorc);
  D(xxlxor          ,0xF00004D0,    Form::kXX3Form,   xxlxor);
  D(xxmrghw         ,0xF0000090,    Form::kXX3Form,   xxmrghw);
  D(xxmrglw         ,0xF0000190,    Form::kXX3Form,   xxmrglw);
  D(xxpermdi        ,0xF0000050,    Form::kXX3Form,   xxpermdi);
  D(xxsel           ,0xF0000030,    Form::kXX4Form,   xxsel);
  D(xxsldwi         ,0xF0000010,    Form::kXX3Form,   xxsldwi);
  D(xxspltw         ,0xF0000290,    Form::kXX2Form,   xxspltw);
  #undef D
  }

  DecoderInfo* GetInstruction(uint32_t opcode) {

    uint32_t index = (opcode % kDecoderSize);

    while (decoder_table[index] != nullptr && 
           decoder_table[index]->opcode() != opcode) {
      index = (index + 1) % kDecoderSize;
    }

    if (decoder_table[index] != nullptr) {
      return decoder_table[index];
    } else {
      DecoderInfo* invalid = new DecoderInfo(0, Form::kInvalid, "");
      return invalid;
    }
  }

  ~DecoderTable() {
    for(int i = 0; i < kDecoderSize; i++)
       if(decoder_table[i] != nullptr) {
          delete decoder_table[i];
       }
    delete[] decoder_table;
  }

private:

  void SetInstruction(DecoderInfo dinfo) {

    uint32_t index = (dinfo.opcode() % kDecoderSize);

    while (decoder_table[index] != nullptr && 
           decoder_table[index]->opcode() != dinfo.opcode()) {
        index = (index + 1) % kDecoderSize;
    }

    if (decoder_table[index] != nullptr && *decoder_table[index] != dinfo) {
        // in some architectures instructions can share the same opcode or
        // decoder info. Rhe meaning of the instruction will depends on 
        // processor mode or execution mode
        decoder_table[index]->next(new DecoderInfo(dinfo));
    } else {
      delete decoder_table[index];
      decoder_table[index] = new DecoderInfo(dinfo);
    }
  }

  DecoderInfo **decoder_table;
};

/**
 * Instruction Format encoders
 */
typedef union XO_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:9;
    uint32_t OE:1;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XO_form_t;
static_assert(sizeof(XO_format) == sizeof(uint32_t), "XO_format_t size != 4");

typedef union X_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:10;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} X_form_t;
static_assert(sizeof(X_format) == sizeof(uint32_t), "X_format_t size != 4");

typedef union D_format {
  struct {
    uint32_t D:16;
    uint32_t RA:5;
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} D_form_t;
static_assert(sizeof(D_format) == sizeof(uint32_t), "D_format_t size != 4");

typedef union I_format {
  struct {
    uint32_t LK:1;
    uint32_t AA:1;
    uint32_t LI:24;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} I_form_t;
static_assert(sizeof(I_format) == sizeof(uint32_t), "I_format_t size != 4");

typedef union B_format {
  struct {
    uint32_t LK:1;
    uint32_t AA:1;
    uint32_t BD:14;
    uint32_t BI:5;
    uint32_t BO:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} B_form_t;
static_assert(sizeof(B_format) == sizeof(uint32_t), "B_format_t size != 4");

typedef union SC_format {
  struct {
    uint32_t B31:1;
    uint32_t B30:1;
    uint32_t LEV:7;
    uint32_t RSV3:4;
    uint32_t RSV2:5;
    uint32_t RSV1:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} SC_form_t;
static_assert(sizeof(SC_format) == sizeof(uint32_t), "SC_format_t size != 4");

typedef union DS_format {
  struct {
    uint32_t XO:2;
    uint32_t DS:14;
    uint32_t RA:5;
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} DS_form_t;
static_assert(sizeof(DS_format) == sizeof(uint32_t), "DS_format_t size != 4");

typedef union DQ_format {
  struct {
    uint32_t RSV:4;
    uint32_t DQ:17;
    uint32_t RA:5;
    uint32_t RTp:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} DQ_form_t;
static_assert(sizeof(DQ_format) == sizeof(uint32_t), "DQ_format_t size != 4");

typedef union XL_form {
  struct {
    uint32_t LK:1;
    uint32_t XO:10;
    uint32_t BB:5;
    uint32_t BA:5;
    uint32_t BT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XL_form_t;
static_assert(sizeof(XL_form) == sizeof(uint32_t), "XL_form size != 4");

typedef union XFX_format {
  struct {
    uint32_t RSV:1;
    uint32_t XO:10;
    uint32_t SPRlo:5;  // see note 1: the order of the two 5-bit halves
    uint32_t SPRhi:5;  // of the SPR number is reversed
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XFX_form_t;
static_assert(sizeof(XFX_format) == sizeof(uint32_t), "XFX_format_t size != 4");

typedef union XFL_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:10;
    uint32_t FRB:5;
    uint32_t W:1;
    uint32_t FLM:8;
    uint32_t L:1;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XFL_form_t;
static_assert(sizeof(XFL_format) == sizeof(uint32_t), "XFL_format_t size != 4");

typedef union XX1_format {
  struct {
    uint32_t TX:1;
    uint32_t XO:10;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t T:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XX1_form_t;
static_assert(sizeof(XX1_format) == sizeof(uint32_t), "XX1_format_t size != 4");

typedef union XX2_format {
  struct {
    //TODO
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XX2_form_t;
static_assert(sizeof(XX2_format) == sizeof(uint32_t), "XX2_format_t size != 4");

typedef union XX3_format {
  struct {
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XX3_form_t;
static_assert(sizeof(XX3_format) == sizeof(uint32_t), "XX3_format_t size != 4");

typedef union XX4_format {
  struct {
    uint32_t TX:1;
    uint32_t BX:1;
    uint32_t AX:1;
    uint32_t CX:1;
    uint32_t XO:2;
    uint32_t C:5;
    uint32_t B:5;
    uint32_t A:5;
    uint32_t T:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XX4_form_t;
static_assert(sizeof(XX4_format) == sizeof(uint32_t), "XX4_format_t size != 4");

typedef union XS_format {
  struct {
    uint32_t Rc:1;
    uint32_t sh:1;
    uint32_t XO:9;
    uint32_t SH:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XS_form_t;
static_assert(sizeof(XS_format) == sizeof(uint32_t), "XS_format_t size != 4");

typedef union A_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:5;
    uint32_t FRC:5;
    uint32_t FRB:5;
    uint32_t FRA:5;
    uint32_t FRT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} A_form_t;
static_assert(sizeof(A_format) == sizeof(uint32_t), "A_format_t size != 4");

typedef union M_format {
  struct {
    uint32_t Rc:1;
    uint32_t ME:5;
    uint32_t MB:5;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} M_form_t;
static_assert(sizeof(M_format) == sizeof(uint32_t), "M_format_t size != 4");

typedef union MD_format {
  struct {
    uint32_t Rc:1;
    uint32_t sh:1;
    uint32_t XO:3;
    uint32_t MB:6;
    uint32_t SH:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} MD_form_t;
static_assert(sizeof(MD_format) == sizeof(uint32_t), "MD_format_t size != 4");

typedef union MDS_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:4;
    uint32_t MB:6;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} MDS_form_t;
static_assert(sizeof(MDS_format) == sizeof(uint32_t), "MDS_format_t size != 4");

typedef union VA_format {
  struct {
    uint32_t XO:6;
    uint32_t VRC:5;
    uint32_t VRB:5;
    uint32_t VRA:5;
    uint32_t VRT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} VA_form_t;
static_assert(sizeof(VA_format) == sizeof(uint32_t), "VA_format_t size != 4");

typedef union VC_format {
  struct {
    uint32_t XO:10;
    uint32_t Rc:1;
    uint32_t VRB:5;
    uint32_t VRA:5;
    uint32_t VRT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} VC_form_t;
static_assert(sizeof(VC_format) == sizeof(uint32_t), "VC_format_t size != 4");

typedef union VX_format {
  struct {
    uint32_t XO:11;
    uint32_t VRB:5;
    uint32_t VRA:5;
    uint32_t VRT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} VX_form_t;
static_assert(sizeof(VX_format) == sizeof(uint32_t), "VX_format_t size != 4");

typedef union EVX_format {
  struct {
    uint32_t XO:11;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} EVX_form_t;
static_assert(sizeof(EVX_format) == sizeof(uint32_t), "EVX_format_t size != 4");

typedef union EVS_format {
  struct {
    uint32_t BFA:3;
    uint32_t XO:8;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} EVS_form_t;
static_assert(sizeof(EVS_format) == sizeof(uint32_t), "EVS_format_t size != 4");

typedef union Z22_format {
  struct {
    //TODO:
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} Z22_form_t;
static_assert(sizeof(Z22_format) == sizeof(uint32_t), "Z22_format_t size != 4");

typedef union Z23_format {
  struct {
    //TODO:
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} Z23_form_t;
static_assert(sizeof(Z23_format) == sizeof(uint32_t), "Z23_format_t size != 4");

} // namespace ppc64_asm

#endif
