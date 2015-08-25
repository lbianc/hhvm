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

/*
 * This pragma was set to do not show warnings of no return value for the
 * "implemented" functions for this class.
 * This file was created just to handle PPC64 architecture and initially
 * to support PPC64 with EvalJit=false.
 * This is a work in progress to port HHVM Jit to PPC64 architecture.
 * */

#pragma GCC diagnostic ignored "-Wreturn-type"

#include "hphp/runtime/vm/jit/back-end-ppc64.h"

#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/util/disasm.h"
#include "hphp/util/text-color.h"

#include "hphp/runtime/vm/func.h"
#include "hphp/runtime/vm/jit/abi-ppc64.h"
#include "hphp/runtime/vm/jit/block.h"
#include "hphp/runtime/vm/jit/check.h"
#include "hphp/runtime/vm/jit/code-gen-helpers-ppc64.h"
#include "hphp/runtime/vm/jit/code-gen-x64.h"
#include "hphp/runtime/vm/jit/func-prologues-ppc64.h"
#include "hphp/runtime/vm/jit/cfg.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/print.h"
#include "hphp/runtime/vm/jit/service-requests-inline.h"
#include "hphp/runtime/vm/jit/service-requests-ppc64.h"
#include "hphp/runtime/vm/jit/timer.h"
#include "hphp/runtime/vm/jit/unique-stubs-ppc64.h"
#include "hphp/runtime/vm/jit/vasm-print.h"
#include "hphp/runtime/vm/jit/vasm-llvm.h"
#include "hphp/runtime/vm/jit/relocation.h"

namespace HPHP { namespace jit {

namespace ppc64 {

extern "C" void enterTCHelper(Cell* vm_sp,
                              ActRec* vm_fp,
                              TCA start,
                              ActRec* firstAR,
                              void* targetCacheBase,
                              ActRec* stashedAR);

struct BackEnd final : public jit::BackEnd {
  BackEnd() {}
  ~BackEnd() {}

  Abi abi() override {
    return ppc64::abi;
  };

   size_t cacheLineSize() override {
     return 64;
   };

  PhysReg rSp() override {
    return PhysReg(reg::rsp);
  };

  PhysReg rVmSp() override {
    return ppc64::rVmSp;
  };

  PhysReg rVmFp() override {
    return ppc64::rVmFp;
  };

   PhysReg rVmTl() override {
    return ppc64::rVmTl;
  }; 

//TODO PPC64 review this code, since it is duplicated
#if defined (__powerpc64__)
  #define CALLEE_SAVED_BARRIER()
#else
  #define CALLEE_SAVED_BARRIER()                                    \
      asm volatile("" : : : "rbx", "r12", "r13", "r14", "r15");
#endif

   void enterTCHelper(TCA start, ActRec* stashedAR) override {
      // We have to force C++ to spill anything that might be in a callee-saved
      // register (aside from rbp). enterTCHelper does not save them.
      CALLEE_SAVED_BARRIER();
      auto& regs = vmRegsUnsafe();
      jit::ppc64::enterTCHelper(regs.stack.top(), regs.fp, start,
                         vmFirstAR(), rds::tl_base, stashedAR);
      CALLEE_SAVED_BARRIER();
   };
   UniqueStubs emitUniqueStubs() override {
     return ppc64::emitUniqueStubs();
   };
   TCA emitServiceReqWork(
    CodeBlock& cb,
    TCA start,
    SRFlags flags,
    folly::Optional<FPInvOffset> spOff,
    ServiceRequest req,
    const ServiceReqArgVec& argv) override {
	   return ppc64::emitServiceReqWork(cb, start, flags, spOff, req, argv);
   };
   size_t reusableStubSize() const override { not_implemented(); };

   void emitInterpReq(CodeBlock& code,
                             SrcKey sk,
                             FPInvOffset spOff) override {
     not_implemented();
   };

   bool funcPrologueHasGuard(TCA prologue, const Func* func) override {
     not_implemented();
   };

   TCA funcPrologueToGuard(TCA prologue, const Func* func) override {
     not_implemented();
   };

   SrcKey emitFuncPrologue(TransID transID, Func* func, int argc,
                                  TCA& start) override { not_implemented(); };

   TCA emitCallArrayPrologue(Func* func, DVFuncletsVec& dvs) override {
     not_implemented();
   };

   void funcPrologueSmashGuard(TCA prologue, const Func* func) override {
     not_implemented();
   };

   void emitIncStat(CodeBlock& cb, intptr_t disp, int n) override {
     not_implemented();
   };

   void prepareForTestAndSmash(CodeBlock& cb, int testBytes,
                                      TestAndSmashFlags flags) override {
     not_implemented();
   };

   void smashJmp(TCA jmpAddr, TCA newDest) override { not_implemented(); };

   void smashCall(TCA callAddr, TCA newDest) override { not_implemented(); };

   void smashJcc(TCA jccAddr, TCA newDest) override { not_implemented(); };

   void emitSmashableJump(CodeBlock& cb, TCA dest, ConditionCode cc) override {
     not_implemented();
   };

   void emitSmashableCall(CodeBlock& cb, TCA dest) override {
     not_implemented();
   };

   TCA smashableCallFromReturn(TCA returnAddr) override {
     not_implemented();
   };

   TCA jmpTarget(TCA jmp) override { not_implemented(); };

   TCA jccTarget(TCA jmp) override { not_implemented(); };

   ConditionCode jccCondCode(TCA jmp) override { not_implemented(); };

   TCA callTarget(TCA call) override { not_implemented(); };

   void addDbgGuard(CodeBlock& codeMain, CodeBlock& codeCold,
                           SrcKey sk, size_t dbgOff) override {
     not_implemented();
   };


   void streamPhysReg(std::ostream& os, PhysReg reg) override {
     auto name = (reg.type() == PhysReg::GP) ? reg::regname(Reg64(reg)) :
       (reg.type() == PhysReg::SIMD) ? reg::regname(RegXMM(reg)) :
       /* (reg.type() == PhysReg::SF) ? */ reg::regname(RegSF(reg));
     os << name;
   };

   void disasmRange(std::ostream& os, int indent, bool dumpIR,
                           TCA begin, TCA end) override { not_implemented(); };

   void genCodeImpl(IRUnit& unit, CodeKind, AsmInfo*) override;

private:
   void do_moveToAlign(CodeBlock&, MoveToAlignFlags) override {
     not_implemented();
   };

   bool do_isSmashable(Address, int, int) override { not_implemented(); };

   void do_prepareForSmash(CodeBlock&, int, int) override {
     not_implemented();
   };


};


std::unique_ptr<jit::BackEnd> newBackEnd() {
  return folly::make_unique<BackEnd>();
}

static size_t genBlock(CodegenState& state, Vout& v, Vout& vc, Block* block) {
//  FTRACE(6, "genBlock: {}\n", block->id());
  HPHP::jit::x64::CodeGenerator cg(state, v, vc);
  size_t hhir_count{0};
  for (IRInstruction& inst : *block) {
    hhir_count++;
    if (inst.is(EndGuards)) state.pastGuards = true;
    v.setOrigin(&inst);
    vc.setOrigin(&inst);
    cg.cgInst(&inst);
  }
  return hhir_count;
};

void BackEnd::genCodeImpl(IRUnit& unit, CodeKind kind, AsmInfo* asmInfo) {
  Timer _t(Timer::codeGen);
  CodeBlock& mainCodeIn   = mcg->code.main();
  CodeBlock& coldCodeIn   = mcg->code.cold();
  CodeBlock* frozenCode   = &mcg->code.frozen();

  CodeBlock mainCode;
  CodeBlock coldCode;
  bool do_relocate = false;
  if (!mcg->useLLVM() &&
      RuntimeOption::EvalJitRelocationSize &&
      coldCodeIn.canEmit(RuntimeOption::EvalJitRelocationSize * 3)) {
    /*
     * This is mainly to exercise the relocator, and ensure that its
     * not broken by new non-relocatable code. Later, it will be
     * used to do some peephole optimizations, such as reducing branch
     * sizes.
     * Allocate enough space that the relocated cold code doesn't
     * overlap the emitted cold code.
     */

    static unsigned seed = 42;
    auto off = rand_r(&seed) & (cacheLineSize() - 1);
    coldCode.init(coldCodeIn.frontier() +
                   RuntimeOption::EvalJitRelocationSize + off,
                   RuntimeOption::EvalJitRelocationSize - off, "cgRelocCold");

    mainCode.init(coldCode.frontier() +
                  RuntimeOption::EvalJitRelocationSize + off,
                  RuntimeOption::EvalJitRelocationSize - off, "cgRelocMain");

    do_relocate = true;
  } else {
    /*
     * Use separate code blocks, so that attempts to use the mcg's
     * code blocks directly will fail (eg by overwriting the same
     * memory being written through these locals).
     */
    coldCode.init(coldCodeIn.frontier(), coldCodeIn.available(),
                  coldCodeIn.name().c_str());
    mainCode.init(mainCodeIn.frontier(), mainCodeIn.available(),
                  mainCodeIn.name().c_str());
  }

  if (frozenCode == &coldCodeIn) {
    frozenCode = &coldCode;
  }

  auto frozenStart = frozenCode->frontier();
  auto coldStart DEBUG_ONLY = coldCodeIn.frontier();
  auto mainStart DEBUG_ONLY = mainCodeIn.frontier();
  size_t hhir_count{0};

  {
    mcg->code.lock();
    mcg->cgFixups().setBlocks(&mainCode, &coldCode, frozenCode);

    SCOPE_EXIT {
      mcg->cgFixups().setBlocks(nullptr, nullptr, nullptr);
      mcg->code.unlock();
    };

    CodegenState state(unit, asmInfo, *frozenCode);
    auto const blocks = rpoSortCfg(unit);
    Vasm vasm;
    auto& vunit = vasm.unit();
    SCOPE_ASSERT_DETAIL("vasm unit") { return show(vunit); };
    // create the initial set of vasm numbered the same as hhir blocks.
    for (uint32_t i = 0, n = unit.numBlocks(); i < n; ++i) {
      state.labels[i] = vunit.makeBlock(AreaIndex::Main);
    }
    // create vregs for all relevant SSATmps
    assignRegs(unit, vunit, state, blocks);
    vunit.entry = state.labels[unit.entry()];
    vasm.main(mainCode);
    vasm.cold(coldCode);
    vasm.frozen(*frozenCode);

    for (auto block : blocks) {
      auto& v = block->hint() == Block::Hint::Unlikely ? vasm.cold() :
               block->hint() == Block::Hint::Unused ? vasm.frozen() :
               vasm.main();
//         FTRACE(6, "genBlock {} on {}\n", block->id(),
//                area_names[(unsigned)v.area()]);
      auto b = state.labels[block];
      vunit.blocks[b].area = v.area();
      v.use(b);
      hhir_count += genBlock(state, v, vasm.cold(), block);
      assertx(v.closed());
      assertx(vasm.main().empty() || vasm.main().closed());
      assertx(vasm.cold().empty() || vasm.cold().closed());
      assertx(vasm.frozen().empty() || vasm.frozen().closed());
    }
    printUnit(kInitialVasmLevel, "after initial vasm generation", vunit);
    assertx(check(vunit));

//       auto const& abi = kind == CodeKind::Trace ? ppc64::abi
//                                                 : ppc64::cross_trace_abi;

    if (mcg->useLLVM()) {
      auto& areas = vasm.areas();
      auto x64_unit = vunit;
      auto vasm_size = std::numeric_limits<size_t>::max();

      jit::vector<UndoMarker> undoAll = {UndoMarker(mcg->globalData())};
      for (auto const& area : areas) undoAll.emplace_back(area.code);
      auto resetCode = [&] {
        for (auto& marker : undoAll) marker.undo();
        mcg->cgFixups().clear();
      };
      auto optimized = false;

      // When EvalJitLLVMKeepSize is non-zero, we'll throw away the LLVM code
      // and use vasm's output instead if the LLVM code is more than x% the
      // size of the vasm code. First we generate and throw away code with
      // vasm, just to see how big it is. The cost of this is trivial compared
      // to the LLVM code generation.
      if (RuntimeOption::EvalJitLLVMKeepSize) {
//           optimizeX64(x64_unit, abi);
        optimized = true;
        emitX64(x64_unit, areas, nullptr);
        vasm_size = areas[0].code.frontier() - areas[0].start;
        resetCode();
      }

      try {
        genCodeLLVM(vunit, areas);

        auto const llvm_size = areas[0].code.frontier() - areas[0].start;
        if (llvm_size * 100 / vasm_size > RuntimeOption::EvalJitLLVMKeepSize) {
          throw FailedLLVMCodeGen("LLVM size {}, vasm size {}\n",
                                  llvm_size, vasm_size);
        }
      } catch (const FailedLLVMCodeGen& e) {
//           FTRACE_MOD(Trace::llvm,
//                      1, "LLVM codegen failed ({}); falling back to x64 backend\n",
//                      e.what());
        always_assert_flog(
          RuntimeOption::EvalJitLLVM < 3,
          "Mandatory LLVM codegen failed with reason `{}' on unit:\n{}",
          e.what(), show(vunit)
        );

        mcg->setUseLLVM(false);
        resetCode();
//           if (!optimized) optimizeX64(x64_unit, abi);
        emitX64(x64_unit, areas, state.asmInfo);

//           if (auto compare = dynamic_cast<const CompareLLVMCodeGen*>(&e)) {
//             printLLVMComparison(unit, vasm.unit(), areas, compare);
//           }
      }
    } else {
//         optimizeX64(vunit, abi);
      emitPPC64(vunit, vasm.areas(), state.asmInfo);
    }
  }

//  auto bcMap = &mcg->cgFixups().m_bcMap;
//  if (do_relocate && !bcMap->empty()) {
//    TRACE(1, "BCMAPS before relocation\n");
//    for (UNUSED auto& map : *bcMap) {
//      TRACE(1, "%s %-6d %p %p %p\n", map.md5.toString().c_str(),
//             map.bcStart, map.aStart, map.acoldStart, map.afrozenStart);
//    }
//  }

  assertx(coldCodeIn.frontier() == coldStart);
  assertx(mainCodeIn.frontier() == mainStart);

  if (do_relocate) {
    if (asmInfo) {
      printUnit(kRelocationLevel, unit, " before relocation ", asmInfo);
    }

    RelocationInfo rel;
    size_t asm_count{0};
    asm_count += relocate(rel, mainCodeIn,
                          mainCode.base(), mainCode.frontier(),
                          mcg->cgFixups(), nullptr);

    asm_count += relocate(rel, coldCodeIn,
                          coldCode.base(), coldCode.frontier(),
                          mcg->cgFixups(), nullptr);
//    TRACE(1, "hhir-inst-count %ld asm %ld\n", hhir_count, asm_count);

    if (frozenCode != &coldCode) {
      rel.recordRange(frozenStart, frozenCode->frontier(),
                      frozenStart, frozenCode->frontier());
    }
    adjustForRelocation(rel);
    adjustMetaDataForRelocation(rel, asmInfo, mcg->cgFixups());
    adjustCodeForRelocation(rel, mcg->cgFixups());

    if (asmInfo) {
      static int64_t mainDeltaTot = 0, coldDeltaTot = 0;
      int64_t mainDelta =
        (mainCodeIn.frontier() - mainStart) -
        (mainCode.frontier() - mainCode.base());
      int64_t coldDelta =
        (coldCodeIn.frontier() - coldStart) -
        (coldCode.frontier() - coldCode.base());

      mainDeltaTot += mainDelta;
      coldDeltaTot += coldDelta;
/*      if (HPHP::Trace::moduleEnabledRelease(HPHP::Trace::printir, 1)) {
        HPHP::Trace::traceRelease("main delta after relocation: "
                                  "%" PRId64 " (%" PRId64 ")\n",
                                  mainDelta, mainDeltaTot);
        HPHP::Trace::traceRelease("cold delta after relocation: "
                                  "%" PRId64 " (%" PRId64 ")\n",
                                  coldDelta, coldDeltaTot);*/
      }
    }
#ifndef NDEBUG
    auto& ip = mcg->cgFixups().m_inProgressTailJumps;
    for (size_t i = 0; i < ip.size(); ++i) {
      const auto& ib = ip[i];
      assertx(!mainCode.contains(ib.toSmash()));
      assertx(!coldCode.contains(ib.toSmash()));
    }
    memset(mainCode.base(), 0xcc, mainCode.frontier() - mainCode.base());
    memset(coldCode.base(), 0xcc, coldCode.frontier() - coldCode.base());
#endif
//  } else {
    coldCodeIn.skip(coldCode.frontier() - coldCodeIn.frontier());
    mainCodeIn.skip(mainCode.frontier() - mainCodeIn.frontier());
//  }

  if (asmInfo) {
    printUnit(kCodeGenLevel, unit, " after code gen ", asmInfo);
  }
}

//////////////////////////////////////////////////////////////////////

bool isSmashable(Address frontier, int nBytes, int offset /* = 0 */) {
  return false;
}

void prepareForSmashImpl(CodeBlock& cb, int nBytes, int offset) {
  not_implemented();
}

void smashJmp(TCA jmpAddr, TCA newDest) { not_implemented(); }

void smashCall(TCA callAddr, TCA newDest) { not_implemented(); }

//////////////////////////////////////////////////////////////////////

//void BackEnd::genCodeImpl(IRUnit& unit, AsmInfo* asmInfo) {
//  not_implemented();
//}

}}}

#pragma GCC diagnostic pop
