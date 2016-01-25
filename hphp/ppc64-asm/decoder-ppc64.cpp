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

#include <cassert>

#include "hphp/ppc64-asm/decoder-ppc64.h"
#include "hphp/ppc64-asm/isa-ppc64.h"

namespace ppc64_asm {

Decoder* Decoder::decoder = nullptr;

std::string DecoderInfo::toString(){

  std::string instr;

  if (form_ == Form::kInvalid) {
    return ".long " + std::to_string(instr_image_);
  }
  if (isNop()) {
    return "nop";
  }

  instr = mnemonic();
  instr += " ";

  bool hasParen = false;
  for (auto i = 0; i < operand_size_; i++) {
     int s = operand_list_[i].operandShift();
     auto op = (instr_image_ & operand_list_[i].mask_);
     op = op >> s;

     if (operand_list_[i].flags_ & PPC_OPERAND_GPR) {
        instr += "r";
     }
     if (operand_list_[i].flags_ & PPC_OPERAND_GPR_0) {
       if (op != 0) {
         instr += "r";
       }
     }
     if (operand_list_[i].flags_ & PPC_OPERAND_FPR) {
       instr += "f";
     }
     if (operand_list_[i].flags_ & PPC_OPERAND_VR) {
      instr += "v";
     }
     if (operand_list_[i].flags_ & PPC_OPERAND_SIGNED) {
       int32_t n = static_cast<int32_t>(op);
       if (n < 0) {
         instr += "-";
       }
       instr += std::to_string(n);
     } else {
       instr += std::to_string(op);
     }
     if (i+1 < operand_size_) {
       hasParen = (operand_list_[i].flags_ & PPC_OPERAND_PAREN);
       (hasParen && (i+1 == 2)) ? instr += "(" : instr += ",";
     } else {
       if (hasParen) {
         instr += ")";
         hasParen = false;
       }
     }
  }
  return instr;
}

bool DecoderInfo::isNop() const {
  // no-op is a mnemonic of ori 0,0,0
  if ((form_ == Form::kD) && (opn_ == OpcodeNames::op_ori)) {
    D_form_t dform;
    dform.instruction = instr_image_;
    if ((!dform.D) && (!dform.RA) && (!dform.RT)) {
      // no-op
      return true;
    }
  }
  return false;
}


bool DecoderInfo::isBranch(bool allowCond /* = true */) const {
  // allowCond: true
  //   b, ba, bl - unconditional branches
  //   bc, bca, bcctr, bcctrl, bcl, bcla, bclr, bclrl, bctar, bctarl
  //
  // allowCond: false
  //   b, ba, bl - unconditional branches
  //  And also, if condition is "branch always" (BO field is 1x1xx):
  //   bc, bca, bcctr, bcctrl, bcl, bcla, bclr, bclrl, bctar, bctarl
  //
  // (based on the branch instructions defined on this Decoder)
  constexpr uint32_t uncondition_bo = 0x14;

  switch (opn_) {
    case OpcodeNames::op_b:
    case OpcodeNames::op_ba:
    case OpcodeNames::op_bl:
      return true;
      break;
    case OpcodeNames::op_bc:
    case OpcodeNames::op_bca:
    case OpcodeNames::op_bcl:
    case OpcodeNames::op_bcla:
    case OpcodeNames::op_bclr:
      if (!allowCond) {
        // checking if the condition is "always branch", then it counts as an
        // unconditional branch
        assert(form_ == Form::kB);
        B_form_t bform;
        bform.instruction = instr_image_;
        return ((bform.BO & uncondition_bo) == uncondition_bo);
      }
      return true;
      break;
    case OpcodeNames::op_bcctr:
    case OpcodeNames::op_bcctrl:
    case OpcodeNames::op_bclrl:
    case OpcodeNames::op_bctar:
    case OpcodeNames::op_bctarl:
      if (!allowCond) {
        // checking if the condition is "always branch", then it counts as an
        // unconditional branch
        assert(form_ == Form::kXL);
        XL_form_t xlform;
        xlform.instruction = instr_image_;
        return ((xlform.BT & uncondition_bo) == uncondition_bo);
      }
      return true;
      break;
    default:
      break;
  }
  return false;
}

bool DecoderInfo::isClearSignBit() const {
  // clrldi is a mnemonic to rldicl when
  if (opn_ == OpcodeNames::op_rldicl) {
    MD_form_t instr_md;
    instr_md.instruction = instr_image_;
    if ((instr_md.SH == 0) && (instr_md.sh == 0)) {
      // it's the clrldi mnemonic!
      switch (instr_md.MB) {
        case 16:
        case 32:
        case 48:
          return true;
        break;
        default:
        break;
      }
    }
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////

DecoderInfo* Decoder::decode(PPC64Instr instr) {
  // To decode a instruction we extract the decoder fields
  // masking the instruction and test if it 'hits' the decoder table.
  for (size_t i = 0; i < sizeof(DecoderList)/sizeof(PPC64Instr); i++) {
    auto decoded_instr = instr & DecoderList[i];
    auto index = (decoded_instr % kDecoderSize);

    while (decoder_table[index] != nullptr &&
        decoder_table[index]->opcode() != decoded_instr) {
      index = (index + 1) % kDecoderSize;
    }

    if (decoder_table[index] != nullptr) {
      decoder_table[index]->instruction_image(instr);
      return decoder_table[index];
    }
  }

  // invalid instruction! Use fallback.
  return getInvalid();
}

} // namespace ppc64_ams
