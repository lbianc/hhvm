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
// constexpr RegisterPhys rAsm(reg::grp0) //Scratch Register 
// constexpr PhysReg rVmToC(reg::gpr2) //Base of Stack 
// constexpr RegisterPhys rLinkReg(reg::rLK) //Link register
// constexpr RegisterPhys rReturnReg(reg::gpr3) //Return Register

//kGPReserved =  rLinkReg | rVmSp //Maybe CR register here ?

//const RegSet kGPUnreserved = kGPCallerSaved | kGPCalleeSaved; 

//This is calling conventions
//const RegSet kGPCallerSaved = reg::gpr0 | reg::gpr3 | reg::gpr4 | reg::gpr5 | reg::gpr6 | reg::gpr7 | reg::gpr8 | reg::gpr9 | 
// reg::gpr10 | reg::gpr11 | reg::gpr12 | reg::gpr13 | + CTR, CR0, CR1, CR5-7  

// const RegSet kGPCalleSaved = Callee-save: r14-r31, CR2-4 


// const RegSet kSIMDCallerSaved = reg::v20-v21 vrsave
// const RegSet kSIMDCalleSaved = v0-v13 ?????

// kSIMDReserved 
//const RegSet kSF = RegSet(RegSF{0});

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