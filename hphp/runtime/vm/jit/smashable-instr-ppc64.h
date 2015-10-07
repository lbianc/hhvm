/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | Copyright (c) 2010-2015 Facebook, Inc. (http://www.facebook.com)     |
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

#ifndef incl_HPHP_JIT_SMASHABLE_INSTR_PPC64_H_
#define incl_HPHP_JIT_SMASHABLE_INSTR_PPC64_H_

#include "hphp/runtime/vm/jit/types.h"
#include "hphp/runtime/vm/jit/phys-reg.h"

#include "hphp/runtime/vm/jit/abi-ppc64.h" // For PPC64_HAS_PUSH_POP definition
#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/util/data-block.h"

namespace HPHP { namespace jit { namespace ppc64 {

///////////////////////////////////////////////////////////////////////////////

/*
 * Mirrors the API of smashable-instr.h.
 */

/// Standard PPC64 instructions are 4 bytes long
static constexpr int kStdIns = 4;

constexpr size_t smashableMovqLen() { return kStdIns * 5; }  // li64's worst case
constexpr size_t smashableCmpqLen() { return kStdIns * 6; }  // li64 + cmpd
#if PPC64_HAS_PUSH_POP
constexpr size_t smashableCallLen() { return kStdIns * 15; } // worst case
#else
constexpr size_t smashableCallLen() { return kStdIns * 13; } // worst case
#endif
constexpr size_t smashableJmpLen()  { return kStdIns * 1; }  // b
constexpr size_t smashableJccLen()  { return kStdIns * 1; }  // bc

TCA emitSmashableMovq(CodeBlock& cb, uint64_t imm, PhysReg d);
TCA emitSmashableCmpq(CodeBlock& cb, int32_t imm, PhysReg r, int8_t disp);
TCA emitSmashableCall(CodeBlock& cb, TCA target);
TCA emitSmashableJmp(CodeBlock& cb, TCA target);
TCA emitSmashableJcc(CodeBlock& cb, TCA target, ConditionCode cc);
std::pair<TCA,TCA>
emitSmashableJccAndJmp(CodeBlock& cb, TCA target, ConditionCode cc);

void smashMovq(TCA inst, uint64_t imm);
void smashCmpq(TCA inst, uint32_t imm);
void smashCall(TCA inst, TCA target);
void smashJmp(TCA inst, TCA target);
void smashJcc(TCA inst, TCA target, ConditionCode cc = CC_None);

uint64_t smashableMovqImm(TCA inst);
uint32_t smashableCmpqImm(TCA inst);
TCA smashableCallTarget(TCA inst);
TCA smashableJmpTarget(TCA inst);
TCA smashableJccTarget(TCA inst);
ConditionCode smashableJccCond(TCA inst);

///////////////////////////////////////////////////////////////////////////////

}}}

#endif
