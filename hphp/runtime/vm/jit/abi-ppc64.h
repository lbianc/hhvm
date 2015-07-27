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

#ifndef incl_HPHP_JIT_ABI_PPC64_H
#define incl_HPHP_JIT_ABI_PPC64_H

#include "hphp/runtime/vm/jit/abi.h"
#include "hphp/runtime/vm/jit/phys-reg.h"
#include "hphp/ppc64-asm/asm-ppc64.h"

namespace HPHP { namespace jit { namespace ppc64 {

/*
  TODO(IBM):This is a draft for PPC64 abi, must validate all these informations

*/
// constexpr RegisterPhys rVmFp(regs::gpr::r1)        // Frame Pointer
// constexpr RegisterPhys rVmSp(regs::gpr::gpr1)      // Stack Pointer
// constexpr RegisterPhys rAsm(regs::gpr::grp10)      // Scratch Register 
// constexpr RegisterPhys rVmToC(regs::gpr::gpr2)     // Base of Stack 
// constexpr RegisterPhys rLinkReg(regs::lr)          // Link register
// constexpr RegisterPhys rReturnReg(regs::gpr::gpr3) // Return Register

// kGPReserved =  rVmSp | regs::gpr::gpr13 

// const RegSet kGPUnreserved = kGPCallerSaved | kGPCalleeSaved; 

// Calling conventions
// const RegSet kGPCallerSaved = regs::gpr::r2 | regs::gpr::r3 | regs::gpr::r4 | regs::gpr::r5 | regs::gpr::r6 | regs::gpr::r7 | 
//                               regs::gpr::r8  | regs::gpr::r9  | regs::gpr::r10 | regs::gpr::r11 | regs::gpr::r12;


// const RegSet kGPCalleeSaved = regs::gpr::r14 | regs::gpr::r15 | regs::gpr::r16 | regs::gpr::r17 | regs::gpr::r18 | regs::gpr::r19 | 
//                               regs::gpr::r20 | regs::gpr::r21 | regs::gpr::r22 | regs::gpr::r23 | regs::gpr::r24 | regs::gpr::r25 | 
//                               regs::gpr::r26 | regs::gpr::r27 | regs::gpr::r28 | regs::gpr::r29 | regs::gpr::r30 | regs::gpr::r31;

// const RegSet kGPUnreserved = kGPCallerSaved | kGPCalleeSaved;

// const RegSet kSIMDCallerSaved = reg::v20-v21 vrsave
// const RegSet kSIMDCalleSaved = v0-v13 ?????

 /*
   TODO(IBM) Need to check ABI for Vector Instructions.
 */

// UNUSED const Abi abi {
//   kGPUnreserved,
//   kGPReserved,
//   kSIMDUnreserved,
//   kSIMDReserved
//   kCalleeSaved,
//   kSF 
// };

}}}

#endif