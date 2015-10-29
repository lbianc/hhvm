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
  a.ld(rAsm ,r[disp]);
  a.cmpdi(rAsm, imm);

  return start;
}

TCA emitSmashableCall(CodeBlock& cb, TCA target) {
  align(cb, Alignment::SmashCmpq, AlignContext::Live);

  auto const start = cb.frontier();

  ppc64_asm::Assembler a { cb };

  a.mflr(ppc64::rfuncln());
  Vptr p(ppc64::rsp(), lr_position_on_callstack);
  a.std(ppc64::rfuncln(), p);
  p.disp = -min_callstack_size;

#if PPC64_HAS_PUSH_POP
  p.base = ppc64::rstktop();
  a.stdu(ppc64::rsp(), p);
  a.mr(ppc64::rsp(), ppc64::rstktop());
#else
  a.stdu(ppc64::rsp(), p);
#endif
  a.branchAuto(target, ppc64_asm::BranchConditions::Always,
  ppc64_asm::LinkReg::Save);
#if PPC64_HAS_PUSH_POP
  a.addi(ppc64::rstktop(), ppc64::rsp(), min_callstack_size);
  Vptr bc(ppc64::rsp(), 0); // backchain
  a.ld(ppc64::rsp(), bc);
#else
  a.addi(ppc64::rsp(), ppc64::rsp(), min_callstack_size);
#endif
  p.disp = lr_position_on_callstack;
  a.ld(ppc64::rfuncln(), p);
  a.mtlr(ppc64::rfuncln());

  return start;
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
  always_assert(is_aligned(inst, Alignment::SmashJmp));

  auto& cb = mcg->code.blockFor(inst);
  CodeCursor cursor { cb, inst };
  ppc64_asm::Assembler a { cb };

  if (target > inst && target - inst <= smashableJmpLen()) {
    a.emitNop(target - inst);
  } else {
    a.branchAuto(target);
  }
}

void smashJcc(TCA inst, TCA target, ConditionCode cc) {
  always_assert(is_aligned(inst, Alignment::SmashJcc));

  if (cc == CC_None) {
    ppc64_asm::Assembler::patchBctr(inst, target);
  } else {
    auto& cb = mcg->code.blockFor(inst);
    CodeCursor cursor { cb, inst };
    ppc64_asm::Assembler a { cb };
    a.branchAuto(target, cc);
  }
}

///////////////////////////////////////////////////////////////////////////////

uint64_t smashableMovqImm(TCA inst) {
  return *reinterpret_cast<uint64_t*>(inst);
}

uint32_t smashableCmpqImm(TCA inst) {
  return *reinterpret_cast<uint32_t*>(inst);
}

TCA smashableCallTarget(TCA inst) {
  // a call always begin with a call to pushMinCallStack
  if (((inst[3] >> 2) & 0x3F) != 31) return nullptr; // from mflr

  // points out how many instructions ahead the li64 can be found
#if PPC64_HAS_PUSH_POP
  uint8_t skipPushMinCallStack = 5;
#else
  uint8_t skipPushMinCallStack = 4;
#endif

  return reinterpret_cast<TCA>(
      ppc64_asm::Assembler::getLi64(inst + kStdIns * skipPushMinCallStack));
}

TCA smashableJmpTarget(TCA inst) {
  return smashableJccTarget(inst); // for now, it's the same as Jcc
}

TCA smashableJccTarget(TCA inst) {
  // from patchBctr:
  // It has to skip 6 instructions: li64 (5 instructions) and mtctr
  // uint8_t fits 4 times in a instr, so multiply instructions number by 4
  CodeAddress bctr_addr = inst + 4 * 6;
  // Opcode located at the 6 most significant bits
  if (((bctr_addr[3] >> 2) & 0x3F) != 19) return nullptr; // from bctr

  return reinterpret_cast<TCA>(ppc64_asm::Assembler::getLi64(inst));
}

ConditionCode smashableJccCond(TCA inst) {
  not_implemented();
  return DecodedInstruction(inst).jccCondCode();
}

///////////////////////////////////////////////////////////////////////////////

}}}
