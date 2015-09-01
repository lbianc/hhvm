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

#include "hphp/runtime/vm/jit/abi-ppc64.h"

#include "hphp/runtime/vm/jit/types.h"
#include "hphp/runtime/vm/jit/abi.h"

namespace HPHP { namespace jit { namespace ppc64 {

namespace {

const RegSet kGPCallerSaved =
    ppc64_asm::reg::r0  | ppc64_asm::reg::r3  | ppc64_asm::reg::r4
  | ppc64_asm::reg::r5  | ppc64_asm::reg::r6  | ppc64_asm::reg::r7
  | ppc64_asm::reg::r8  | ppc64_asm::reg::r9  | ppc64_asm::reg::r10
  | ppc64_asm::reg::r12;  // r11 is a scratch register

const RegSet kGPCalleeSaved =
    ppc64_asm::reg::r1  | ppc64_asm::reg::r20 | ppc64_asm::reg::r14
  | ppc64_asm::reg::r15 | ppc64_asm::reg::r16 | ppc64_asm::reg::r17
  | ppc64_asm::reg::r18 | ppc64_asm::reg::r19 | ppc64_asm::reg::r20
  | ppc64_asm::reg::r21 | ppc64_asm::reg::r22 | ppc64_asm::reg::r23
  | ppc64_asm::reg::r24 | ppc64_asm::reg::r25 | ppc64_asm::reg::r26
  | ppc64_asm::reg::r27 | ppc64_asm::reg::r28 | ppc64_asm::reg::r29
  | ppc64_asm::reg::r30 | ppc64_asm::reg::r31;

const RegSet kGPUnreserved = kGPCallerSaved | kGPCalleeSaved;

const RegSet kGPReserved = RegSet(ppc64_asm::reg::r13) | rvmtl();

const RegSet kGPRegs = kGPUnreserved | kGPReserved;

const RegSet kXMMCallerSaved =
    ppc64_asm::reg::v0  | ppc64_asm::reg::v1  | ppc64_asm::reg::v2
  | ppc64_asm::reg::v3  | ppc64_asm::reg::v4  | ppc64_asm::reg::v5
  | ppc64_asm::reg::v6  | ppc64_asm::reg::v7  | ppc64_asm::reg::v8
  | ppc64_asm::reg::v9  | ppc64_asm::reg::v10 | ppc64_asm::reg::v11
  | ppc64_asm::reg::v12 | ppc64_asm::reg::v13 | ppc64_asm::reg::v14
  | ppc64_asm::reg::v15 | ppc64_asm::reg::v16 | ppc64_asm::reg::v17
  | ppc64_asm::reg::v18 | ppc64_asm::reg::v19;

const RegSet kXMMCalleeSaved =
    ppc64_asm::reg::v20 | ppc64_asm::reg::v21 | ppc64_asm::reg::v22
  | ppc64_asm::reg::v23 | ppc64_asm::reg::v24 | ppc64_asm::reg::v25
  | ppc64_asm::reg::v26 | ppc64_asm::reg::v27 | ppc64_asm::reg::v28
  | ppc64_asm::reg::v29 | ppc64_asm::reg::v30 | ppc64_asm::reg::v31;

const RegSet kXMMUnreserved = kXMMCallerSaved | kXMMCalleeSaved;

const RegSet kXMMReserved;

const RegSet kXMMRegs = kXMMUnreserved | kXMMReserved;

const RegSet kCallerSaved = kGPCallerSaved | kXMMCallerSaved;

const RegSet kCalleeSaved = kGPCalleeSaved | kXMMCalleeSaved;

const RegSet kSF = RegSet(RegSF{0});

///////////////////////////////////////////////////////////////////////////////

/*
 * Registers that can safely be used for scratch purposes in-between traces.
 */
const RegSet kScratchCrossTraceRegs =
  kXMMCallerSaved | (kGPUnreserved - vm_regs_with_sp());

/*
 * Helper code ABI registers.
 */
const RegSet kGPHelperRegs; //TODO
const RegSet kXMMHelperRegs; // TODO

///////////////////////////////////////////////////////////////////////////////

const Abi trace_abi {
  kGPUnreserved,
  kGPReserved,
  kXMMUnreserved,
  kXMMReserved,
  kCalleeSaved,
  kSF,
  true,
};

const Abi cross_trace_abi {
  trace_abi.gp() & kScratchCrossTraceRegs,
  trace_abi.gp() - kScratchCrossTraceRegs,
  trace_abi.simd() & kScratchCrossTraceRegs,
  trace_abi.simd() - kScratchCrossTraceRegs,
  trace_abi.calleeSaved & kScratchCrossTraceRegs,
  trace_abi.sf,
  false
};

const Abi helper_abi {
  kGPHelperRegs,
  trace_abi.gp() - kGPHelperRegs,
  kXMMHelperRegs,
  trace_abi.simd() - kXMMHelperRegs,
  trace_abi.calleeSaved,
  trace_abi.sf,
  false

};

constexpr PhysReg gp_args[] = {
  ppc64_asm::reg::r3, ppc64_asm::reg::r4, ppc64_asm::reg::r5,
  ppc64_asm::reg::r6, ppc64_asm::reg::r7, ppc64_asm::reg::r8,
  ppc64_asm::reg::r9
};

constexpr PhysReg simd_args[] = { //TODO
    ppc64_asm::reg::v0, ppc64_asm::reg::v1,
    ppc64_asm::reg::v2 };

constexpr PhysReg svcreq_args[] = { //TODO
    ppc64_asm::reg::r8
};

}

///////////////////////////////////////////////////////////////////////////////

const Abi& abi(CodeKind kind) {
  switch (kind) {
    case CodeKind::Trace:
      return trace_abi;
    case CodeKind::CrossTrace:
      return cross_trace_abi;
    case CodeKind::Helper:
      return helper_abi;
  }
  not_reached();
}

///////////////////////////////////////////////////////////////////////////////

PhysReg rarg(size_t i) {
  assertx(i < num_arg_regs());
  return gp_args[i];
}
PhysReg rarg_simd(size_t i) {
  assertx(i < num_arg_regs_simd());
  return simd_args[i];
}

size_t num_arg_regs() {
  return sizeof(gp_args) / sizeof(PhysReg);
}
size_t num_arg_regs_simd() {
  return sizeof(simd_args) / sizeof(PhysReg);
}

PhysReg r_svcreq_sf() {
  return abi().sf.findFirst();
}
PhysReg r_svcreq_arg(size_t i) {
  return svcreq_args[i];
}

///////////////////////////////////////////////////////////////////////////////

}}}
