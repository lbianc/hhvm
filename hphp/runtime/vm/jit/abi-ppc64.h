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

#include "hphp/runtime/vm/jit/phys-reg.h"

#include "hphp/ppc64-asm/asm-ppc64.h"

namespace HPHP { namespace jit {

struct Abi;

namespace ppc64 {

///////////////////////////////////////////////////////////////////////////////

/*
 * Mirrors the API of abi.h.
 */

const Abi& abi(CodeKind kind = CodeKind::Trace);

constexpr PhysReg rvmfp()    { return ppc64_asm::reg::r28; }
constexpr PhysReg rvmsp()    { return ppc64_asm::reg::r29; }
constexpr PhysReg rvmtl()    { return ppc64_asm::reg::r30; }
constexpr PhysReg rvasmtmp() { return ppc64_asm::reg::r31; }
constexpr PhysReg rsp()      { return ppc64_asm::reg::r1; }

namespace detail {
  const RegSet kVMRegs      = rvmfp() | rvmtl() | rvmsp();
  const RegSet kVMRegsNoSP  = rvmfp() | rvmtl();
}

inline RegSet vm_regs_with_sp() { return detail::kVMRegs; }
inline RegSet vm_regs_no_sp()   { return detail::kVMRegsNoSP; }

constexpr PhysReg rret() { return ppc64_asm::reg::r3; }

PhysReg rarg(size_t i);
PhysReg rarg_simd(size_t i);

size_t num_arg_regs();
size_t num_arg_regs_simd();

constexpr PhysReg r_svcreq_req()  { return ppc64_asm::reg::r8; }
constexpr PhysReg r_svcreq_stub() { return ppc64_asm::reg::r8; }
PhysReg r_svcreq_sf();
PhysReg r_svcreq_arg(size_t i);

///////////////////////////////////////////////////////////////////////////////

/*
 * Scratch register.
 */
constexpr Reg64 rAsm = ppc64_asm::reg::r11;

///////////////////////////////////////////////////////////////////////////////

}}}

#endif
