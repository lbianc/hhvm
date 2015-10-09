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

#include "hphp/runtime/vm/jit/smashable-instr-ppc64.h"

#include "hphp/runtime/vm/jit/align-ppc64.h"
#include "hphp/runtime/vm/jit/mc-generator.h"

#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/util/data-block.h"

namespace HPHP { namespace jit { namespace ppc64 {

///////////////////////////////////////////////////////////////////////////////

/*
 * Concurrent modification and execution of instructions is safe if all
 * of the following hold:
 *
 *  1/  The modification is done with a single processor store.
 *  2/  Only one instruction in the original stream is modified.
 *  3/  The modified instruction does not cross a cacheline boundary.
 *
 * Cache alignment is required for mutable instructions to make sure mutations
 * don't "tear" on remote CPUs.
 */

#define EMIT_BODY(cb, inst, Inst, ...)  \
  ([&] {                                \
    align(cb, Alignment::Smash##Inst,   \
          AlignContext::Live);          \
    auto const start = cb.frontier();   \
    ppc64_asm::Assembler a { cb };      \
    a.inst(__VA_ARGS__);                \
    return start;                       \
  }())

TCA emitSmashableMovq(CodeBlock& cb, uint64_t imm, PhysReg d) {
  auto const start = EMIT_BODY(cb, li64, Movq, d, 0xdeadbeeffeedface);
  auto immp = reinterpret_cast<uint64_t*>(
    cb.frontier() - smashableMovqLen()
  );
  *immp = imm;

  return start;
}

TCA emitSmashableCmpq(CodeBlock& cb, int32_t imm, PhysReg r, int8_t disp) {
  align(cb, Alignment::SmashCmpq, AlignContext::Live);

  auto const start = cb.frontier();

  ppc64_asm::Assembler a { cb };
  a.ld(rvasmtmp() ,r[disp]);
  a.cmpdi(rvasmtmp(), imm);

  return start; 
}

TCA emitSmashableCall(CodeBlock& cb, TCA target) {
  not_implemented();
  return nullptr;
}

TCA emitSmashableJmp(CodeBlock& cb, TCA target) {
  return EMIT_BODY(cb, branchAuto, Jmp, target);
}

TCA emitSmashableJcc(CodeBlock& cb, TCA target, ConditionCode cc) {
  assertx(cc != CC_None);
  return EMIT_BODY(cb, branchAuto, Jcc, target, cc);
}

std::pair<TCA,TCA>
emitSmashableJccAndJmp(CodeBlock& cb, TCA target, ConditionCode cc) {
  assertx(cc != CC_None);

  align(cb, Alignment::SmashJccAndJmp, AlignContext::Live);

  ppc64_asm::Assembler a { cb };
  auto const jcc = cb.frontier();
  a.branchAuto(target, cc);
  auto const jmp = cb.frontier();
  a.branchAuto(target);

  return std::make_pair(jcc, jmp);
}

#undef EMIT_BODY

///////////////////////////////////////////////////////////////////////////////

void smashMovq(TCA inst, uint64_t imm) {
  always_assert(is_aligned(inst, Alignment::SmashMovq));
  not_implemented();
  // should use a->patchLi64 ?
  *reinterpret_cast<uint64_t*>(inst) = imm;
}

void smashCmpq(TCA inst, uint32_t imm) {
  always_assert(is_aligned(inst, Alignment::SmashCmpq));
  not_implemented();
  // should use a->patchLi64 ?
  *reinterpret_cast<uint32_t*>(inst) = imm;
}

void smashCall(TCA inst, TCA target) {
  not_implemented();
}

void smashJmp(TCA inst, TCA target) {
  not_implemented();
}

void smashJcc(TCA inst, TCA target, ConditionCode cc) {
  not_implemented();
}

///////////////////////////////////////////////////////////////////////////////

uint64_t smashableMovqImm(TCA inst) {
  return *reinterpret_cast<uint64_t*>(inst);
}

uint32_t smashableCmpqImm(TCA inst) {
  return *reinterpret_cast<uint32_t*>(inst);
}

TCA smashableCallTarget(TCA inst) {
  if (((inst[3] >> 2) & 0x3F) != 31) return nullptr; // from mflr
#if PPC64_HAS_PUSH_POP
  return inst + kStdIns * 5;
#else
  return inst + kStdIns * 4;
#endif
}

TCA smashableJmpTarget(TCA inst) {
  return smashableJmpTarget(inst); // for now, it's also a "bc" instruction
}

TCA smashableJccTarget(TCA inst) {
  if (((inst[3] >> 2) & 0x3F) != 16) return nullptr; // from patchBc
  // target found at the beginning of the instruction
  return inst;
}

ConditionCode smashableJccCond(TCA inst) {
  return DecodedInstruction(inst).jccCondCode();
}

///////////////////////////////////////////////////////////////////////////////

}}}
