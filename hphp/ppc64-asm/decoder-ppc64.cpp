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

DecoderTable* DecoderTable::decoder = nullptr;

std::string DecoderTable::ToString(){
    /*
     * TODO(rcardoso):
     * DRAFT:
     *     Print Instruction
     *     1- Read instruction name (done)
     *     2- Get Operand masks
     *     3- Convert operands to apropriated string and concat with name
     *     5- Return formated string
     * CAVEATS:
     *   If instruction is kInvalid maybe it's data. What to do in this case?
     */
    if(decoded_instr_->form() == Form::kInvalid) {
      return ".long ";
    }
    return decoded_instr_->mnemonic();
}

void DecoderTable::DecodeInstruction(uint32_t ip) {
  /*
   * TODO(rcardoso):
   * The decoder table is in fact a hash table. We can have k decoder masks
   * so we can need to test and retrieve the first who match. In worst case
   * we have k*x where x is a number of access to decoder table
   * and depends on number of  collisions in best case 1 (no collisions)
   * or, in worst case, n (if all keys collide).
   * And we have k*x + y where y is the number of alternate instructions
   * (next pointer on DecoderInfo). It's '+' y because we retrieve the whole
   * structure do a linear search if we need. So, in worst case y=n,
   * then we have k*n + n => O(k*2n). But in pratice, this will not
   * gonna happens because we have a limited number of execution modes and
   * we cannot have a 'n' size collision so, we have O(k*c + c) = O(1) at most.
   * This is not the best algorithm but this will work. Sorry for any math
   * mistakes.
   */

  // To decode a instruction we extract the decoder fields
  // masking the instruction and test if it 'hits' the decoder table.
  decoded_instr_ = nullptr;
  for(int i = 0; i < kDecoderListSize; i++) {
    uint32_t instr_image = ip;
    instr_image &= DecoderList[i];

    decoded_instr_ = GetInstruction(instr_image);
    if(decoded_instr_->form() != Form::kInvalid)
       break;
  }
}

} //namespace ppc64_asm
