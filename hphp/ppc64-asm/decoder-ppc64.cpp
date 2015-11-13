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

#include "hphp/ppc64-asm/decoder-ppc64.h"

namespace ppc64_asm {

Decoder* Decoder::decoder = nullptr;

std::string DecoderInfo::toString(){

  std::string instr;

  if (form() == Form::kInvalid) {
    return ".long " + std::to_string(instruction_image());
  }
  if (isNop(instruction_image())) {
    return "nop";
  }

  instr = mnemonic();
  instr += " ";

  bool hasParen = false;
  for (auto i = 0; i < operand_size_; i++) {
     int s = operand_list_[i].operandShift();
     uint32_t op = (instruction_image() & operand_list_[i].mask_);
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

bool DecoderInfo::isNop(uint32_t instr) {
  return (instr == opcode()) && (opcode() == 0x60000000);
}

DecoderInfo* Decoder::decode(uint32_t instr) {
  // To decode a instruction we extract the decoder fields
  // masking the instruction and test if it 'hits' the decoder table.
  DecoderInfo* decoded_instr;

  for (int i = 0; i < kDecoderListSize; i++) {
     uint32_t instr_image = instr;
     instr_image &= DecoderList[i];

     uint32_t index = (instr_image % kDecoderSize);

     while (decoder_table[index] != nullptr &&
          decoder_table[index]->opcode() != instr_image) {
          index = (index + 1) % kDecoderSize;
     }

     if (decoder_table[index] != nullptr) {
       decoder_table[index]->instruction_image(instr);
       return decoder_table[index];
     }
  }
  decoded_instr = new DecoderInfo(0x0, Form::kInvalid, "", { UN });
  decoded_instr->instruction_image(instr);
  #undef UN
  return decoded_instr;
}

} // namespace ppc64_ams
