/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | Copyright (c) 2010-2013 Facebook, Inc. (http://www.facebook.com)     |
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

#include "hphp/runtime/vm/jit/vasm-emit.h"

#include "hphp/runtime/base/arch.h"
#include "hphp/runtime/vm/jit/abi-ppc64.h"
#include "hphp/runtime/vm/jit/back-end-ppc64.h"
#include "hphp/runtime/vm/jit/block.h"
#include "hphp/runtime/vm/jit/code-gen-helpers-ppc64.h"
#include "hphp/runtime/vm/jit/ir-instruction.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/print.h"
#include "hphp/runtime/vm/jit/reg-algorithms.h"
#include "hphp/runtime/vm/jit/service-requests-ppc64.h"
#include "hphp/runtime/vm/jit/service-requests-inline.h"
#include "hphp/runtime/vm/jit/timer.h"
#include "hphp/runtime/vm/jit/vasm.h"
#include "hphp/runtime/vm/jit/vasm-instr.h"
#include "hphp/runtime/vm/jit/vasm-print.h"
#include "hphp/runtime/vm/jit/vasm-unit.h"
#include "hphp/runtime/vm/jit/vasm-util.h"
#include "hphp/runtime/vm/jit/vasm-visit.h"

#include "hphp/ppc64-asm/asm-ppc64.h"

namespace HPHP { namespace jit {

///////////////////////////////////////////////////////////////////////////////

using namespace ppc64;
using namespace ppc64_asm;

namespace ppc64 { struct ImmFolder; }

namespace {
///////////////////////////////////////////////////////////////////////////////

struct Vgen {
  Vgen(const Vunit& u, Vasm::AreaList& areas, AsmInfo* asmInfo)
    : unit(u)
    , backend(mcg->backEnd())
    , areas(areas)
    , m_asmInfo(asmInfo) {
    addrs.resize(u.blocks.size());
    points.resize(u.next_point);
  }
  void emit(jit::vector<Vlabel>&);

 private:
  template<class Inst> void emit(const Inst& i) {
    always_assert_flog(false, "unimplemented instruction: {} in B{}\n",
                       vinst_names[Vinstr(i).op], size_t(current));
  }

  // intrinsics
  void emit(const bindaddr& i) { not_implemented(); }
  void emit(const bindcall& i) { not_implemented(); }
  void emit(const bindjcc1st& i) { not_implemented(); }
  void emit(const bindjcc& i) { not_implemented(); }
  void emit(const bindjmp& i) { not_implemented(); }
  void emit(const callstub& i) { not_implemented(); }
  void emit(const callfaststub& i) { not_implemented(); }
  void emit(const contenter& i) { not_implemented(); }
  void emit(const copy& i) { not_implemented(); }
  void emit(const copy2& i) { not_implemented(); }
  void emit(const debugtrap& i) { not_implemented(); }
  void emit(const fallthru& i) { not_implemented(); }
  void emit(const ldimmb& i) { not_implemented(); }
  void emit(const ldimml& i) { not_implemented(); }
  void emit(const ldimmq& i) { not_implemented(); }
  void emit(const ldimmqs& i) { not_implemented(); }
  void emit(const fallback& i) { not_implemented(); }
  void emit(const fallbackcc& i) { not_implemented(); }
  void emit(const load& i) { not_implemented(); }
  void emit(const mccall& i) { not_implemented(); }
  void emit(const mcprep& i) { not_implemented(); }
  void emit(const nothrow& i) { not_implemented(); }
  void emit(const store& i) { not_implemented(); }
  void emit(const syncpoint& i) { not_implemented(); }
  void emit(const unwind& i) { not_implemented(); }
  void emit(const landingpad& i) { not_implemented(); }
  void emit(const vretm& i) { not_implemented(); }
  void emit(const vret& i) { not_implemented(); }
  void emit(const leavetc&) { not_implemented(); }

  // instructions
  void emit(andb i) { not_implemented(); }
  void emit(andbi i) { not_implemented(); }
  void emit(const andbim& i) { not_implemented(); }
  void emit(andl i) { not_implemented(); }
  void emit(andli i) { not_implemented(); }
  void emit(andq i) { not_implemented(); }
  void emit(andqi i) { not_implemented(); }
  void emit(addli i) { not_implemented(); }
  void emit(const addlm& i) { not_implemented(); }
  void emit(addq i) { not_implemented(); }
  void emit(addqi i) { not_implemented(); }
  void emit(const addqim& i) { not_implemented(); }
  void emit(addsd i) { not_implemented(); }
  void emit(const call& i) { not_implemented(); }
  void emit(const callm& i) { not_implemented(); }
  void emit(const callr& i) { not_implemented(); }
  void emit(const cloadq& i) { not_implemented(); }
  void emit(const cmovq& i) { not_implemented(); }
  void emit(const cmpb& i) { not_implemented(); }
  void emit(const cmpbi& i) { not_implemented(); }
  void emit(const cmpbim& i) { not_implemented(); }
  void emit(const cmpl& i) { not_implemented(); }
  void emit(const cmpli& i) { not_implemented(); }
  void emit(const cmplim& i) { not_implemented(); }
  void emit(const cmplm& i) { not_implemented(); }
  void emit(const cmpq& i) { not_implemented(); }
  void emit(const cmpqi& i) { not_implemented(); }
  void emit(const cmpqim& i) { not_implemented(); }
  void emit(const cmpqims& i) { not_implemented(); }
  void emit(const cmpqm& i) { not_implemented(); }
  void emit(cmpsd i) { not_implemented(); }
  void emit(const cqo& i) { not_implemented(); }
  void emit(const cvttsd2siq& i) { not_implemented(); }
  void emit(const cvtsi2sd& i) { not_implemented(); }
  void emit(const cvtsi2sdm& i) { not_implemented(); }
  void emit(decl i) { not_implemented(); }
  void emit(const declm& i) { not_implemented(); }
  void emit(decq i) { not_implemented(); }
  void emit(const decqm& i) { not_implemented(); }
  void emit(divsd i) { not_implemented(); }
  void emit(imul i) { not_implemented(); }
  void emit(const idiv& i) { not_implemented(); }
  void emit(incl i) { not_implemented(); }
  void emit(const inclm& i) { not_implemented(); }
  void emit(incq i) { not_implemented(); }
  void emit(const incqm& i) { not_implemented(); }
  void emit(const incqmlock& i) { not_implemented(); }
  void emit(const incwm& i) { not_implemented(); }
  void emit(const jcc& i) { not_implemented(); }
  void emit(const jcci& i) { not_implemented(); }
  void emit(const jmp& i) { not_implemented(); }
  void emit(const jmpr& i) { not_implemented(); }
  void emit(const jmpm& i) { not_implemented(); }
  void emit(const jmpi& i) { not_implemented(); }
  void emit(const lea& i) { not_implemented(); }
  void emit(const leap& i) { not_implemented(); }
  void emit(const loadups& i) { not_implemented(); }
  void emit(const loadtqb& i) { not_implemented(); }
  void emit(const loadl& i) { not_implemented(); }
  void emit(const loadqp& i) { not_implemented(); }
  void emit(const loadsd& i) { not_implemented(); }
  void emit(const loadzbl& i) { not_implemented(); }
  void emit(const loadzbq& i) { not_implemented(); }
  void emit(const loadzlq& i) { not_implemented(); }
  void emit(const movb& i) { not_implemented(); }
  void emit(const movl& i) { not_implemented(); }
  void emit(const movzbl& i) { not_implemented(); }
  void emit(const movzbq& i) { not_implemented(); }
  void emit(mulsd i) { not_implemented(); }
  void emit(neg i) { not_implemented(); }
  void emit(const nop& i) { not_implemented(); }
  void emit(not i) { not_implemented(); }
  void emit(notb i) { not_implemented(); }
  void emit(const orwim& i) { not_implemented(); }
  void emit(orq i) { not_implemented(); }
  void emit(orqi i) { not_implemented(); }
  void emit(const orqim& i) { not_implemented(); }
  void emit(const pop& i) { not_implemented(); }
  void emit(const popm& i) { not_implemented(); }
  void emit(psllq i) { not_implemented(); }
  void emit(psrlq i) { not_implemented(); }
  void emit(const push& i) { not_implemented(); }
  void emit(const roundsd& i) { not_implemented(); }
  void emit(const ret& i) { not_implemented(); }
  void emit(const sarq& i) { not_implemented(); }
  void emit(sarqi i) { not_implemented(); }
  void emit(const setcc& i) { not_implemented(); }
  void emit(shlli i) { not_implemented(); }
  void emit(shlq i) { not_implemented(); }
  void emit(shlqi i) { not_implemented(); }
  void emit(shrli i) { not_implemented(); }
  void emit(shrqi i) { not_implemented(); }
  void emit(const sqrtsd& i) { not_implemented(); }
  void emit(const storeups& i) { not_implemented(); }
  void emit(const storeb& i) { not_implemented(); }
  void emit(const storebi& i) { not_implemented(); }
  void emit(const storel& i) { not_implemented(); }
  void emit(const storeli& i) { not_implemented(); }
  void emit(const storeqi& i) { not_implemented(); }
  void emit(const storesd& i) { not_implemented(); }
  void emit(const storew& i) { not_implemented(); }
  void emit(const storewi& i) { not_implemented(); }
  void emit(subbi i) { not_implemented(); }
  void emit(subl i) { not_implemented(); }
  void emit(subli i) { not_implemented(); }
  void emit(subq i) { not_implemented(); }
  void emit(subqi i) { not_implemented(); }
  void emit(subsd i) { not_implemented(); }
  void emit(const testb& i) { not_implemented(); }
  void emit(const testbi& i) { not_implemented(); }
  void emit(const testbim& i) { not_implemented(); }
  void emit(const testwim& i) { not_implemented(); }
  void emit(const testl& i) { not_implemented(); }
  void emit(const testli& i) { not_implemented(); }
  void emit(const testlim& i) { not_implemented(); }
  void emit(const testq& i) { not_implemented(); }
  void emit(const testqm& i) { not_implemented(); }
  void emit(const testqim& i) { not_implemented(); }
  void emit(const ucomisd& i) { not_implemented(); }
  void emit(const ud2& i) { not_implemented(); }
  void emit(unpcklpd i) { not_implemented(); }
  void emit(xorb i) { not_implemented(); }
  void emit(xorbi i) { not_implemented(); }
  void emit(xorl i) { not_implemented(); }
  void emit(xorq i) { not_implemented(); }
  void emit(xorqi i) { not_implemented(); }

  CodeAddress start(Vlabel b) {
    auto area = unit.blocks[b].area;
    return areas[(int)area].start;
  }
  CodeBlock& main() { return area(AreaIndex::Main).code; }
  CodeBlock& cold() { return area(AreaIndex::Cold).code; }
  CodeBlock& frozen() { return area(AreaIndex::Frozen).code; }

private:
  Vasm::Area& area(AreaIndex i) {
    assertx((unsigned)i < areas.size());
    return areas[(unsigned)i];
  }

private:
  struct LabelPatch { CodeAddress instr; Vlabel target; };
  struct PointPatch { CodeAddress instr; Vpoint pos; Vreg d; };
  const Vunit& unit;
  BackEnd& backend;
  jit::vector<Vasm::Area>& areas;
  AsmInfo* m_asmInfo;
  ppc64_asm::Assembler* a;
  CodeBlock* codeBlock;
  Vlabel current{0}, next{0}; // in linear order
  jit::vector<CodeAddress> addrs, points;
  jit::vector<LabelPatch> jccs, jmps, bccs, calls, catches;
  jit::vector<PointPatch> ldpoints;
};

// toplevel emitter
void Vgen::emit(jit::vector<Vlabel>& labels) {
  // Some structures here track where we put things just for debug printing.
  struct Snippet {
    const IRInstruction* origin;
    TcaRange range;
  };
  struct BlockInfo {
    jit::vector<Snippet> snippets;
  };

 // This is under the printir tracemod because it mostly shows you IR and
 // machine code, not vasm and machine code (not implemented).
  bool shouldUpdateAsmInfo = !!m_asmInfo
    && Trace::moduleEnabledRelease(HPHP::Trace::printir, kCodeGenLevel);

  std::vector<TransBCMapping>* bcmap = nullptr;
  if (mcg->tx().isTransDBEnabled() || RuntimeOption::EvalJitUseVtuneAPI) {
    bcmap = &mcg->cgFixups().m_bcMap;
  }

  jit::vector<jit::vector<BlockInfo>> areaToBlockInfos;
  if (shouldUpdateAsmInfo) {
    areaToBlockInfos.resize(areas.size());
    for (auto& r : areaToBlockInfos) {
      r.resize(unit.blocks.size());
    }
  }

  for (int i = 0, n = labels.size(); i < n; ++i) {
    assertx(checkBlockEnd(unit, labels[i]));

    auto b = labels[i];
    auto& block = unit.blocks[b];
    codeBlock = &area(block.area).code;
    //TODO(IBM): Pass codeblock as argument
    ppc64_asm::Assembler as { area(block.area).code };
    a = &as;
    auto blockStart = a->frontier();
    addrs[b] = blockStart;

    {
      // Compute the next block we will emit into the current area.
      auto cur_start = start(labels[i]);
      auto j = i + 1;
      while (j < labels.size() && cur_start != start(labels[j])) {
        j++;
      }
      next = j < labels.size() ? labels[j] : Vlabel(unit.blocks.size());
      //TODO(IBM): why arm code doesn't have current = b in this line?
    }

    const IRInstruction* currentOrigin = nullptr;
    auto blockInfo = shouldUpdateAsmInfo
      ? &areaToBlockInfos[unsigned(block.area)][b]
      : nullptr;
    auto start_snippet = [&](const Vinstr& inst) {
      if (!shouldUpdateAsmInfo) return;

      blockInfo->snippets.push_back(
        Snippet { inst.origin, TcaRange { codeBlock->frontier(), nullptr } }
      );
    };
    auto finish_snippet = [&] {
      if (!shouldUpdateAsmInfo) return;

      if (!blockInfo->snippets.empty()) {
        auto& snip = blockInfo->snippets.back();
        snip.range = TcaRange { snip.range.start(), codeBlock->frontier() };
      }
    };

    //TODO(IBM): check if (is_empty_catch(block)) continue; ??????
    for (auto& inst : block.code) {
      if (currentOrigin != inst.origin) {
        finish_snippet();
        start_snippet(inst);
        currentOrigin = inst.origin;
      }

      if (bcmap && inst.origin) {
        auto sk = inst.origin->marker().sk();
        if (bcmap->empty() ||
            bcmap->back().md5 != sk.unit()->md5() ||
            bcmap->back().bcStart != sk.offset()) {
          bcmap->push_back(TransBCMapping{sk.unit()->md5(), sk.offset(),
                                          main().frontier(), cold().frontier(),
                                          frozen().frontier()});
        }
      }

      switch (inst.op) {
#define O(name, imms, uses, defs) \
        case Vinstr::name: emit(inst.name##_); break;
        VASM_OPCODES
#undef O
      }
    }

    finish_snippet();
  }

  // TODO(IBM): Implement jump smashers
  // for (auto& p : jccs) {
  //   assertx(addrs[p.target]);
  // }

  // for (auto& p : jmps) {
  //   assertx(addrs[p.target]);
  // }

  // for (auto& p : calls) {
  //   assertx(addrs[p.target]);
  // }

  // for (auto& p : ldpoints) {}

  for (auto& p : catches) {
    mcg->registerCatchBlock(p.instr, addrs[p.target]);
  }

  if (!shouldUpdateAsmInfo) {
    return;
  }

  for (auto i = 0; i < areas.size(); ++i) {
    const IRInstruction* currentOrigin = nullptr;
    auto& blockInfos = areaToBlockInfos[i];
    for (auto const blockID : labels) {
      auto const& blockInfo = blockInfos[static_cast<size_t>(blockID)];
      if (blockInfo.snippets.empty()) continue;

      for (auto const& snip : blockInfo.snippets) {
        if (currentOrigin != snip.origin && snip.origin) {
          currentOrigin = snip.origin;
        }

        m_asmInfo->updateForInstruction(
          currentOrigin,
          static_cast<AreaIndex>(i),
          snip.range.start(),
          snip.range.end());
      }
    }
  }
}

/*
 Lower facilitate code generation. In some cases is used because some vasm opcodes
 doesn't have a 1:1 mapping to machine asm code.
*/
void lowerForX64(Vunit& unit, const Abi& abi) {
  //TODO(IBM) Implement function
  printUnit(kVasmARMFoldLevel, "after lower for PPC64", unit);
}
///////////////////////////////////////////////////////////////////////////////
} // anonymous namespace

void optimizePPC64(Vunit& unit, const Abi& abi) {
 //TODO(IBM) Implement function
}

void emitPPC64(const Vunit& unit, Vasm::AreaList& areas, AsmInfo* asmInfo) {
  //TODO(IBM) Implement function
}

///////////////////////////////////////////////////////////////////////////////
}}