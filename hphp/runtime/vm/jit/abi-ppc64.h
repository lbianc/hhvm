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
 * Enumerations and constants defining the binary interface between
 * tracelets.
 *
 * Most changes here will likely require corresponding changes in
 * __enterTCHelper and other parts of mc-generator.cpp and the IR
 * translator.
 */

#ifndef incl_HPHP_VM_RUNTIME_TRANSLATOR_ABI_PPC64_H_
#define incl_HPHP_VM_RUNTIME_TRANSLATOR_ABI_PPC64_H_

#include "hphp/ppc64-asm/asm-ppc64.h"
#include "hphp/runtime/vm/jit/abi.h"
#include "hphp/runtime/vm/jit/phys-reg.h"

namespace HPHP { namespace jit { namespace ppc64 {

//////////////////////////////////////////////////////////////////////
/*
 * Principal reserved registers.
 *
 * These registers have special purposes both during and between
 * traces.
 */

/*
 * Frame pointer.  When mid-trace, points to the ActRec for the
 * function currently executing.
 */
//TODO
constexpr PhysReg rVmFp      = ppc64_asm::reg::r1;

/*
 * Stack pointer.  When mid-trace, points to the top of the eval stack
 * (lowest valid address) at the start of the current tracelet.
 */
//TODO
constexpr PhysReg rVmSp      = ppc64_asm::reg::r1;

/*
 * RDS base pointer.  Always points to the base of the RDS block for
 * the current request.
 */
//TODO
constexpr PhysReg rVmTl      = ppc64_asm::reg::r12;

/*
 * scratch register
 */
//TODO
constexpr ppc64_asm::Reg64 rAsm         = ppc64_asm::reg::r10;

//////////////////////////////////////////////////////////////////////
/*
 * Registers used during a tracelet for program locations.
 *
 * These are partitioned into caller-saved and callee-saved regs
 * according to the ppc64 C abi.  These are all the registers that the
 * translator manages via its RegMap.
 */

const RegSet kGPCallerSaved =
		ppc64_asm::reg::r2 | ppc64_asm::reg::r3 | ppc64_asm::reg::r4 | ppc64_asm::reg::r5 | ppc64_asm::reg::r6 | ppc64_asm::reg::r7 |
		ppc64_asm::reg::r8  | ppc64_asm::reg::r9  | ppc64_asm::reg::r10 | ppc64_asm::reg::r11 | ppc64_asm::reg::r12;

const RegSet kGPCalleeSaved =
		ppc64_asm::reg::r14 | ppc64_asm::reg::r15 | ppc64_asm::reg::r16 | ppc64_asm::reg::r17 | ppc64_asm::reg::r18 | ppc64_asm::reg::r19
  | ppc64_asm::reg::r20 | ppc64_asm::reg::r21 | ppc64_asm::reg::r22 | ppc64_asm::reg::r23 | ppc64_asm::reg::r24 | ppc64_asm::reg::r25
  | ppc64_asm::reg::r26 | ppc64_asm::reg::r27 | ppc64_asm::reg::r28 | ppc64_asm::reg::r29 | ppc64_asm::reg::r30 | ppc64_asm::reg::r31;

const RegSet kGPUnreserved = kGPCallerSaved | kGPCalleeSaved;

//TODO
const RegSet kGPReserved = ppc64_asm::reg::r1 | ppc64_asm::reg::r13;

const RegSet kGPRegs = kGPUnreserved | kGPReserved;

const RegSet kXMMCallerSaved/* =
  reg::v0 | reg::v1 | reg::v2 | reg::v3 | reg::v4 | reg::v5 | reg::v6 | reg::v7
  | reg::v8 | reg::v9 | reg::v10 | reg::v11 | reg::v12 | reg::v13 | reg::v14 | reg::v15
  | reg::v16 | reg::v17 | reg::v18 | reg::v19*/;

const RegSet kXMMCalleeSaved /*=  reg::v20 | reg::v21 | reg::v22 | reg::v23 | reg::v24
		 | reg::v25 | reg::v26 | reg::v27 | reg::v28 | reg::v29 | reg::v30 | reg::v31*/;

const RegSet kXMMUnreserved = kXMMCallerSaved | kXMMCalleeSaved;

const RegSet kXMMReserved;

const RegSet kXMMRegs = kXMMUnreserved | kXMMReserved;

const RegSet kCallerSaved = kGPCallerSaved | kXMMCallerSaved;

const RegSet kCalleeSaved = kGPCalleeSaved | kXMMCalleeSaved;

const RegSet kSF = RegSet(RegSF{0});

//////////////////////////////////////////////////////////////////////
/*
 * Registers reserved for cross-tracelet ABI purposes.
 *
 * These registers should not be used for scratch purposes between
 * tracelets, and have to be specially handled if we are returning to
 * the interpreter.
 */

/*
 * Registers that are live between tracelets, in two flavors, depending whether
 * we are between tracelets in a resumed function.
 */
const RegSet kCrossTraceRegs        = rVmFp | rVmTl;
const RegSet kCrossTraceRegsResumed = kCrossTraceRegs | rVmSp;

/*
 * Registers live on entry to the fcallArrayHelper.
 *
 * TODO(#2288359): we don't want this to include rVmSp eventually.
 */
const RegSet kCrossTraceRegsFCallArray = kCrossTraceRegs | rVmSp;

/*
 * Registers live on entry to an interpOneCFHelper.
 */
const RegSet kCrossTraceRegsInterpOneCF = kCrossTraceRegs | rVmSp | rAsm;

/*
 * Registers that are live after a PHP function return.
 *
 * TODO(#2288359): we don't want this to include rVmSp eventually.
 */
const RegSet kCrossTraceRegsReturn = kCrossTraceRegs | rVmSp;

/*
 * Registers that are live during a PHP function call, between the caller and
 * the callee.
 */
const RegSet kCrossCallRegs = kCrossTraceRegs;

/*
 * Registers that can safely be used for scratch purposes in-between
 * traces.
 *
 * Note: there are portions of the func prologue code that will hit
 * assertions if you remove rax, rdx, or rcx from this set without
 * modifying them.
 */
const RegSet kScratchCrossTraceRegs = kXMMCallerSaved |
  (kGPUnreserved - (kCrossTraceRegs | kCrossTraceRegsResumed));

//////////////////////////////////////////////////////////////////////
/*
 * Calling convention registers for service requests or calling C++.
 */

// ppc64 INTEGER class argument registers.
const PhysReg argNumToRegName[] = {
		ppc64_asm::reg::r3, ppc64_asm::reg::r4, ppc64_asm::reg::r5, ppc64_asm::reg::r6,
		ppc64_asm::reg::r7, ppc64_asm::reg::r8, ppc64_asm::reg::r9, ppc64_asm::reg::r10
};
const int kNumRegisterArgs = sizeof(argNumToRegName) / sizeof(PhysReg);

inline RegSet argSet(int n) {
  RegSet regs;
  for (int i = 0; i < n; i++) {
    regs.add(argNumToRegName[i]);
  }
  return regs;
}

// ppc64 SSE class argument registers.
const PhysReg argNumToSIMDRegName[] = {
  /*reg::v2, reg::v3, reg::v4, reg::v5, reg::v6, reg::v7, reg::v8,
  reg::v9, reg::v10, reg::v11, reg::v12, reg::v13*/
};
const int kNumSIMDRegisterArgs = sizeof(argNumToSIMDRegName) / sizeof(PhysReg);

/*
 * JIT'd code "reverse calls" the enterTC routine by returning to it,
 * with a service request number and arguments.
 */
constexpr PhysReg serviceReqArgRegs[] = {
  // rdi: contains request number
		//TODO
  /*reg::rsi, reg::rdx, reg::rcx, */reg::r8
};
constexpr int kNumServiceReqArgRegs =
  sizeof(serviceReqArgRegs) / sizeof(PhysReg);

/*
 * Some data structures are accessed often enough from translated code
 * that we have shortcuts for getting offsets into them.
 */
#define TVOFF(nm) int(offsetof(TypedValue, nm))
#define AROFF(nm) int(offsetof(ActRec, nm))
#define AFWHOFF(nm) int(offsetof(c_AsyncFunctionWaitHandle, nm))
#define CONTOFF(nm) int(offsetof(c_Generator, nm))

UNUSED const Abi abi {
  .gpUnreserved   = kGPUnreserved,
  .gpReserved     = kGPReserved,
  .simdUnreserved = kXMMUnreserved,
  .simdReserved   = kXMMReserved,
  .calleeSaved    = kCalleeSaved,
  .sf             = kSF,
  .canSpill       = true,
};

UNUSED const Abi cross_trace_abi {
  .gpUnreserved   = abi.gp() & kScratchCrossTraceRegs,
  .gpReserved     = abi.gp() - kScratchCrossTraceRegs,
  .simdUnreserved = abi.simd() & kScratchCrossTraceRegs,
  .simdReserved   = abi.simd() - kScratchCrossTraceRegs,
  .calleeSaved    = abi.calleeSaved & kScratchCrossTraceRegs,
  .sf             = abi.sf,
  .canSpill       = false
};

//////////////////////////////////////////////////////////////////////

}}}

#endif
