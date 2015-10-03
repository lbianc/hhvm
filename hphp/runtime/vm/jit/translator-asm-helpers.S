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
   * there via a bindcall by pushing return addresses, setting the callee frame
   * pointer, then jumping to the prologue. We leave the TC with a ret
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

  // handleSRHelper: Translated code will jump to this stub to perform all
  // service requests. It calls out to C++ to handle the request, then jumps
  // to the returned address (which may be the callToExit stub).
  ETCH_ALIGN16
  ETCH_SECTION(handleSRHelper)
  .globl ETCH_NAME(handleSRHelper)
ETCH_NAME(handleSRHelper):
  CFI(startproc)

  // Sync vmsp & vmfp
  mov ETCH_R_VM_SP, 0x10(ETCH_R_VM_TL)
  mov ETCH_R_VM_FP, 0x20(ETCH_R_VM_TL)

  // Push a ServiceReqInfo struct onto the stack and call handleServiceRequest.
  push %r8
  push %rcx
  push %rdx
  push %rsi
  push %r10
  push %rdi
  CFI2(adjust_cfa_offset, 0x30)

  // call mcg->handleServiceRequest(%rsp)
  mov ETCH_NAME(mcg)(%rip), ETCH_ARG1
  mov %rsp, ETCH_ARG2
  call MCGenerator_handleServiceRequest

  // Pop the ServiceReqInfo off the stack.
  add $0x30, %rsp
  CFI2(adjust_cfa_offset, -0x30)

  // rVmTl was preserved by the callee, but vmsp and vmfp might've changed if
  // we interpreted anything. Reload them.
  mov 0x10(ETCH_R_VM_TL), ETCH_R_VM_SP
  mov 0x20(ETCH_R_VM_TL), ETCH_R_VM_FP

  jmp *%rax
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

  mflr %r0
  std %r0, 16(%r1)    // LR on parent call frame
  CFI3C(offset, lr, -16)        // Where to find previous value of link register
  stdu %r1, -176(%r1) // aligned with Epilogue: addi %r1, %r1, 176
  CFI2(adjust_cfa_offset, 176)  // cfa is now adjusted

  // Save non-volatile registers in order to restore it on Epilogue
  std %r2, 24(%r1)    // TOC
  std %r14, 32(%r1)   // local variable #1
  std %r15, 40(%r1)   // local variable #2
  std %r16, 48(%r1)   // local variable #3
  std %r17, 56(%r1)   // local variable #4
  std %r18, 64(%r1)   // local variable #5
  std %r19, 72(%r1)   // local variable #6
  std %r20, 80(%r1)   // local variable #7
  std %r21, 88(%r1)   // local variable #8
  std %r22, 96(%r1)   // local variable #9
  std %r23, 104(%r1)  // local variable #10
  std %r24, 112(%r1)  // local variable #11
  std %r25, 120(%r1)  // local variable #12
  std %r26, 128(%r1)  // local variable #13
  std %r27, 136(%r1)  // local variable #14
  std %r28, 144(%r1)  // local variable #15
  std %r29, 152(%r1)  // local variable #16
  std %r30, 160(%r1)  // local variable #17
  std %r31, 168(%r1)  // local variable #18

  // Set firstAR->m_sfp to point to this frame.
  std %r1, 0(ETCH_ARG4)

  // Set up special registers used for translated code.
  mr ETCH_R_VM_SP, ETCH_ARG1
  mr ETCH_R_VM_TL, ETCH_ARG5
  mr ETCH_R_VM_FP, ETCH_ARG2

  cmpdi ETCH_ARG6, 0
  beq ETCH_LABEL(enterTCHelper$callTC)
  mr ETCH_R_VM_FP, ETCH_ARG6

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
  ld %r2, 24(%r1)     // TOC
  ld %r14, 32(%r1)    // local variable #1
  ld %r15, 40(%r1)    // local variable #2
  ld %r16, 48(%r1)    // local variable #3
  ld %r17, 56(%r1)    // local variable #4
  ld %r18, 64(%r1)    // local variable #5
  ld %r19, 72(%r1)    // local variable #6
  ld %r20, 80(%r1)    // local variable #7
  ld %r21, 88(%r1)    // local variable #8
  ld %r22, 96(%r1)    // local variable #9
  ld %r23, 104(%r1)   // local variable #10
  ld %r24, 112(%r1)   // local variable #11
  ld %r25, 120(%r1)   // local variable #12
  ld %r26, 128(%r1)   // local variable #13
  ld %r27, 136(%r1)   // local variable #14
  ld %r28, 144(%r1)   // local variable #15
  ld %r29, 152(%r1)   // local variable #16
  ld %r30, 160(%r1)   // local variable #17
  ld %r31, 168(%r1)   // local variable #18

  addi %r1, %r1, 176  // aligned with Prologue: stdu %r1, -176(%r1)
  ld %r0, 16(%r1)
  mtlr %r0
  blr

  CFI(endproc)
  ETCH_SIZE(enterTCHelper)
// ==== enterTCHelper ====

  // handleSRHelper: Translated code will jump to this stub to perform all
  // service requests. It calls out to C++ to handle the request, then jumps
  // to the returned address (which may be the callToExit stub).
  ETCH_ALIGN16
  ETCH_SECTION(handleSRHelper)
  .globl ETCH_NAME(handleSRHelper)
ETCH_NAME(handleSRHelper):
  CFI(startproc)

  trap // not yet implemented

  CFI(endproc)
  ETCH_SIZE(handleSRHelper)

#endif
