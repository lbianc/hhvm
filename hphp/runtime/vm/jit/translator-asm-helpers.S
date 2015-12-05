/*
 * enterTCHelper
 *
 * This helper routine is written in assembly to take care of the details
 * when transferring control between jitted code and the translator.
 *
 * Note that MSVC uses translator-asm-helpers.asm instead of this.
 *
 * The columns are registers of:
 * Linux and Mac ABI / Windows ABI / ARM ABI / PPC64 ABI
 *   rdi / rcx   / x0 / r3:  Cell* vm_sp
 *   rsi / rdx   / x1 / r4:  Cell* vm_fp
 *   rdx / r8    / x2 / r5:  unsigned char* start
 *   rcx / r9    / x4 / r6:  ActRec* firstAR
 *   r8  / stack / x5 / r7:  uint8_t* targetCacheBase
 *   r9  / stack / x6 / r8:  ActRec* calleeAR
 *
 * Note that on Windows, ETCH_GET_ARG5/6 borrow r10/r11 respectively
 */

#include "hphp/util/etch-helpers.h"

///////////////////////////////////////////////////////////////////////////////
#if defined(__x86_64__)
  .byte 0
  ETCH_ALIGN16
  ETCH_SECTION(enterTCHelper)
  .globl ETCH_NAME(enterTCHelper)
ETCH_NAME(enterTCHelper):
  // Prologue
  CFI(startproc)             // amongst other things, cfa reg is now rsp, and offset is 8

  // On Windows, get the 5th and 6th arguments from the stack.
  ETCH_GET_ARG5
  ETCH_GET_ARG6

  push ETCH_R_VM_FP
  CFI2(adjust_cfa_offset, 8) // cfa is now 8 bytes further from rsp than it was before
  CFI3C(offset, rbp, -16)    // Where to find previous value of rbp, relative to cfa

  // Set firstAR->m_sfp to point to this frame.
  mov %rsp, (ETCH_ARG4)

  // Set up special registers used for translated code.
  mov ETCH_ARG1, ETCH_R_VM_SP
  mov ETCH_ARG5, ETCH_R_VM_TL
  mov ETCH_ARG2, ETCH_R_VM_FP

  sub $8, %rsp // align native stack
  CFI2(adjust_cfa_offset, 8)

  /*
   * If we're entering the TC at a function prologue, make it look like we got
   * there via a callphp{} by pushing return addresses, setting the callee
   * frame pointer, then jumping to the prologue. We leave the TC with a ret
   * instruction, so if we enter it with a jmp, that will unbalance the RSB and
   * cause tons of branch mispredictions in the frames above us. To avoid this,
   * we get to the prologue by calling a stub that pops the return address
   * pushed by the call and jumps to the prologue. This pushes a bogus address
   * on the RSB but the ret to callToExit always mispredicts anyway, and this
   * keeps the RSB balanced.
   */
  test ETCH_ARG6, ETCH_ARG6
  jz ETCH_LABEL(enterTCHelper$callTC)
  push ETCH_NAME_REL(enterTCExit)
  push 0x8(ETCH_ARG6)
  mov ETCH_ARG6, ETCH_R_VM_FP
  call ETCH_LABEL(enterTCHelper$prologue)

  /*
   * The translated code we are about to enter does not follow the
   * standard prologue of pushing rbp at entry, so we are purposely 8
   * bytes short of 16-byte alignment before this call instruction so
   * that the return address being pushed will make the native stack
   * 16-byte aligned.
   */
ETCH_LABEL(enterTCHelper$callTC):
  call *ETCH_ARG3

  /*
   * enterTCExit is never called directly; this exists to give the jit
   * access to the address of the expected return address while in the TC.
   */
  .globl ETCH_NAME(enterTCExit)
ETCH_NAME(enterTCExit):
  /*
   * Eager vm-reg save. Must match values in rds-header.h
   */
  mov ETCH_R_VM_SP, 0x10(ETCH_R_VM_TL)
  mov ETCH_R_VM_FP, 0x20(ETCH_R_VM_TL)
  add $8, %rsp
  CFI2(adjust_cfa_offset, -8)

  // Epilogue
  pop ETCH_R_VM_FP
  CFI2(restore, rbp)
  CFI2(adjust_cfa_offset, -8)
  ret

ETCH_LABEL(enterTCHelper$prologue):
  pop %rax
  jmp *ETCH_ARG3

  CFI(endproc)
  ETCH_SIZE(enterTCHelper)

/*
 * handleSRHelper
 *
 * Translated code will jump to this stub to perform all
 * service requests. It calls out to C++ to handle the request, then jumps
 * to the returned address (which may be the callToExit stub).
 *
 * The columns are registers of:
 * X64 ABI / PPC64 ABI
 *   rdi / r3:  ServiceRequest req (r_svcreq_req)
 *   r10 / r8:  TCA stub (r_svcreq_stub)
 *   rsi / r4:  args[0]
 *   rdx / r5:  args[1]
 *   rcx / r6:  args[2]
 *   r8  / r7:  args[3]
 */
  ETCH_ALIGN16
  ETCH_SECTION(handleSRHelper)
  .globl ETCH_NAME(handleSRHelper)
ETCH_NAME(handleSRHelper):
  CFI(startproc)

  // Sync vmsp & vmfp
  mov ETCH_R_VM_SP, 0x10(ETCH_R_VM_TL)
  mov ETCH_R_VM_FP, 0x20(ETCH_R_VM_TL)

  // Push a ReqInfo struct onto the stack and call handleServiceRequest.
  push ETCH_SVCREQ_ARG4
  push ETCH_SVCREQ_ARG3
  push ETCH_SVCREQ_ARG2
  push ETCH_SVCREQ_ARG1
  push ETCH_SVCREQ_STUB
  push ETCH_SVCREQ_REQ
  CFI2(adjust_cfa_offset, 0x30)

  // call mcg->handleServiceRequest(%rsp)
  mov ETCH_NAME(mcg)(%rip), ETCH_ARG1
  mov %rsp, ETCH_ARG2
  call MCGenerator_handleServiceRequest

  // Pop the ReqInfo struct off the stack.
  add $0x30, %rsp
  CFI2(adjust_cfa_offset, -0x30)

  // rVmTl was preserved by the callee, but vmsp and vmfp might've changed if
  // we interpreted anything. Reload them.
  mov 0x10(ETCH_R_VM_TL), ETCH_R_VM_SP
  mov 0x20(ETCH_R_VM_TL), ETCH_R_VM_FP

  jmp *ETCH_RET1

  CFI(endproc)
  ETCH_SIZE(handleSRHelper)

///////////////////////////////////////////////////////////////////////////////
#elif defined(__AARCH64EL__)
  .globl enterTCHelper
enterTCHelper:
  .globl enterTCServiceReq
enterTCServiceReq:
  brk 0

#elif defined(__powerpc64__)
// ==== enterTCHelper ====
// r3: regs.stack.top()
// r4: regs.fp
// r5: start
// r6: vmFirstAR()
// r7: rds::tl_base
// r8: stashedAR
  ETCH_ALIGN16
  ETCH_SECTION(enterTCHelper)
  .globl ETCH_NAME(enterTCHelper)
ETCH_NAME(enterTCHelper):
  // Prologue
  CFI(startproc)                // set debug information

  // PIC code setup
    // As this function is not called with $r12 pointing out to this function,
    // it's forbidden to set it. Why is the linker pointing out incorrectly?
  // addis %r2, %r12, .TOC.-enterTCHelper@ha
  // addi  %r2, %r2,  .TOC.-enterTCHelper@l

  /*
   * Manipulating the call stack in order to use the pthread memory section.
   *
   *                          pointer value : pointed value
   * -----------------------------------------------------------
   * current sp:              0x7fffffffcb90: 0x00007fffffffcbb0
   * current vmFirstAR():     0x7fffe187ffc0: 0x0000000000000000
   *
   * sp should be:            0x7fffffffcb70: 0x00007fffe187ffc0
   * vmFirstAR() should be:   0x7fffe187ffc0: 0x00007fffffffcb90
   */

  // Set firstAR->m_sfp to point to this frame.
  std %r1, 0(ETCH_ARG4)
  // Set backchain to rvmsp and save it on rbackchain to be used on vasm call
  // (see abi definition for more details)
  std ETCH_ARG4, -192(%r1)
  mr ETCH_R_BACKCHAIN, ETCH_ARG4

  mflr %r0
  std %r0, 16(%r1)              // LR on parent call frame
  la %r1, -192(%r1)             // backchain was taken care above on -192(%r1)
  CFI2(adjust_cfa_offset, 192)  // debug information about stack pointer change
  CFI3C(offset, 65, 16)         // LR is register #65 in DWARF format

  // Save non-volatile registers in order to restore it on Epilogue
  // also takes care of debug information

#define SAVE_REG(reg, pos)              \
  std %f##reg, pos(%r1); CFI3C(offset, 32 + reg, pos - 192)

  SAVE_REG(14, 48)   // floating point local variable #1
  SAVE_REG(15, 56)   // floating point local variable #2
  SAVE_REG(16, 64)   // floating point local variable #3
  SAVE_REG(17, 72)   // floating point local variable #4
  SAVE_REG(18, 80)   // floating point local variable #5
  SAVE_REG(19, 88)   // floating point local variable #6
  SAVE_REG(20, 96)   // floating point local variable #7
  SAVE_REG(21, 104)  // floating point local variable #8
  SAVE_REG(22, 112)  // floating point local variable #9
  SAVE_REG(23, 120)  // floating point local variable #10
  SAVE_REG(24, 128)  // floating point local variable #11
  SAVE_REG(25, 136)  // floating point local variable #12
  SAVE_REG(26, 144)  // floating point local variable #13
  SAVE_REG(27, 152)  // floating point local variable #14
  SAVE_REG(28, 160)  // floating point local variable #15
  SAVE_REG(29, 168)  // floating point local variable #16
  SAVE_REG(30, 176)  // floating point local variable #17
  SAVE_REG(31, 184)  // floating point local variable #18

#undef SAVE_REG

  // Set up special registers used for translated code.
  mr ETCH_R_VM_SP, ETCH_ARG1
  mr ETCH_R_VM_TL, ETCH_ARG5
  mr ETCH_R_VM_FP, ETCH_ARG2

  // Set up ETCH_R_ONE with value "1" (see abi definition for more details)
  li ETCH_R_ONE, ETCH_V_ONE

  cmpdi ETCH_ARG6, 0
  beq ETCH_LABEL(enterTCHelper$callTC)

  // Entering the TC at a function prologue: make it look like we got
  // there via a callphp{} by setting return addresses, the callee frame
  // pointer, then jumping to the prologue.
  ld  %r0, enterTCExit@got(%r2)
  std %r0, 32(%r1)              // being checked on emitCallToExit unique stub

  // set return address (LR)
  ld %r0, 16(ETCH_ARG6)         // m_savedRip of the stashedAR's call frame
  mtlr %r0

  // now manipulate the frame to go to prologue
  mr ETCH_R_VM_FP, ETCH_ARG6

  mr %r12, ETCH_ARG3            // %r12 is the function linkage
  mtctr %r12
  bctr                          // no 'l' so it returns to stashedAR->m_savedRip

ETCH_LABEL(enterTCHelper$callTC):
  // go to requested code
  mr %r12, ETCH_ARG3  // %r12 is the function linkage
  mtctr %r12
  bctrl               // mind the 'l' at the end so that it returns here later

  /*
   * enterTCExit is never called directly; this exists to give the jit
   * access to the address of the expected return address while in the TC.
   */
  .globl ETCH_NAME(enterTCExit)
ETCH_NAME(enterTCExit):
  /*
   * Eager vm-reg save. Must match values in rds-header.h
   */
  std ETCH_R_VM_SP, 0x10(ETCH_R_VM_TL)
  std ETCH_R_VM_FP, 0x20(ETCH_R_VM_TL)

  // Epilogue
  // Load non-volatile registers saved on Prologue
  ld %f14, 48(%r1)    // floating point local variable #1
  ld %f15, 56(%r1)    // floating point local variable #2
  ld %f16, 64(%r1)    // floating point local variable #3
  ld %f17, 72(%r1)    // floating point local variable #4
  ld %f18, 80(%r1)    // floating point local variable #5
  ld %f19, 88(%r1)    // floating point local variable #6
  ld %f20, 96(%r1)    // floating point local variable #7
  ld %f21, 104(%r1)   // floating point local variable #8
  ld %f22, 112(%r1)   // floating point local variable #9
  ld %f23, 120(%r1)   // floating point local variable #10
  ld %f24, 128(%r1)   // floating point local variable #11
  ld %f25, 136(%r1)   // floating point local variable #12
  ld %f26, 144(%r1)   // floating point local variable #13
  ld %f27, 152(%r1)   // floating point local variable #14
  ld %f28, 160(%r1)   // floating point local variable #15
  ld %f29, 168(%r1)   // floating point local variable #16
  ld %f30, 176(%r1)   // floating point local variable #17
  ld %f31, 184(%r1)   // floating point local variable #18

  addi %r1, %r1, 192            // aligned with Prologue: std %r1, -192(%r1)
  CFI2(adjust_cfa_offset, -192) // debug information about stack pointer change
  ld %r0, 16(%r1)
  mtlr %r0
  blr

  CFI(endproc)
  ETCH_SIZE(enterTCHelper)
// ==== enterTCHelper ====

// ==== handleSRHelper ====
  ETCH_ALIGN16
  ETCH_SECTION(handleSRHelper)
  .globl ETCH_NAME(handleSRHelper)
ETCH_NAME(handleSRHelper):
  CFI(startproc)

  // PIC code setup
  addis %r2, %r12, .TOC.-handleSRHelper@ha
  addi  %r2, %r2,  .TOC.-handleSRHelper@l

  // Sync vmsp & vmfp
  std ETCH_R_VM_SP, 0x10(ETCH_R_VM_TL)
  std ETCH_R_VM_FP, 0x20(ETCH_R_VM_TL)

  // Push a ReqInfo struct onto the stack and call handleServiceRequest.
  mflr %r0
  std %r0, 16(%r1)    // LR on parent call frame
  stdu %r1, -80(%r1)  // aligned with Epilogue: addi %r1, %r1, 80
  CFI2(adjust_cfa_offset, 80)   // debug information about stack pointer change
  CFI3C(offset, 65, 16)         // LR is register #65 in DWARF format

  std ETCH_SVCREQ_ARG4, 72(%r1)
  std ETCH_SVCREQ_ARG3, 64(%r1)
  std ETCH_SVCREQ_ARG2, 56(%r1)
  std ETCH_SVCREQ_ARG1, 48(%r1)
  std ETCH_SVCREQ_STUB, 40(%r1)
  std ETCH_SVCREQ_REQ,  32(%r1)

  // call mcg->handleServiceRequest()
    // get the address pointed by mcg@got to be the "this" pointer
  ld   ETCH_ARG1, mcg@got(%r2)
  ld   ETCH_ARG1, 0(ETCH_ARG1)
    // address of the ReqInfo structure
  addi ETCH_ARG2, %r1, 32

  ld   %r12, MCGenerator_handleServiceRequest@got(%r2)
  mtctr %r12
  bctrl

  // Pop the ReqInfo struct off the stack.
  addi %r1, %r1, 80   // aligned with Prologue: stdu %r1, -80(%r1)
  CFI2(adjust_cfa_offset, -80)
  ld %r0, 16(%r1)
  mtlr %r0

  // rVmTl was preserved by the callee, but vmsp and vmfp might've changed if
  // we interpreted anything. Reload them.
  ld ETCH_R_VM_SP, 0x10(ETCH_R_VM_TL)
  ld ETCH_R_VM_FP, 0x20(ETCH_R_VM_TL)

  mtctr ETCH_RET1
  bctr

  CFI(endproc)
  ETCH_SIZE(handleSRHelper)

#endif
