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

namespace HPHP { namespace jit { namespace ppc64 {

/*
  TODO(IBM):This is a draft for PPC64 abi, must validate all these informations

*/
// constexpr RegisterPhys rVmFp(reg::gpr1) // Frame Pointer (PPC has no frame pointer?)
// constexpr RegisterPhys rVmSp(reg::gpr1) //Stack Pointer
// constexpr RegisterPhys rAsm(reg::grp10) //Scratch Register 
// constexpr PhysReg rVmToC(reg::gpr2) //Base of Stack 
// constexpr RegisterPhys rLinkReg(reg::rLK) //Link register
// constexpr RegisterPhys rReturnReg(reg::gpr3) //Return Register

//kGPReserved =  rLinkReg | rVmSp | reg::gpr13 

//const RegSet kGPUnreserved = kGPCallerSaved | kGPCalleeSaved; 

//calling conventions
// const RegSet kGPCallerSaved = reg::r2 | reg::r3 | reg::r4 | reg::r5 | reg::r6 | reg::r7 | 
//                               reg::r8  | reg::r9  | reg::r10 | reg::r11 | reg::r12;


// const RegSet kGPCalleeSaved = reg::r14 | reg::r15 | reg::r16 | reg::r17 | reg::r18 | reg::r19 | 
//                               reg::r20 | reg::r21 | reg::r22 | reg::r23 | reg::r24 | reg::r25 | 
//                               reg::r26 | reg::r27 | reg::r28 | reg::r29 | reg::r30 | reg::r31; + CR2-4 

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