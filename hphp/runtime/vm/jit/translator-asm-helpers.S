/*
 * enterTCHelper
 *
 * This helper routine is written in assembly to take care of the details
 * when transferring control between jitted code and the translator.
 *   rdi / x0:  Cell* vm_sp
 *   rsi / x1:  Cell* vm_fp
 *   rdx / x2:  unsigned char* start
 *   rcx / x4:  ActRec* firstAR
 *   r8  / x5:  uint8_t* targetCacheBase
 *   r9  / x6:  ActRec* calleeAR
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
  push %rbp
  CFI2(adjust_cfa_offset, 8) // cfa is now 8 bytes further from rsp than it was before
  CFI3C(offset, rbp, -16)    // Where to find previous value of rbp, relative to cfa

  // Set firstAR->m_sfp to point to this frame.
  mov %rsp, (%rcx)

  // Set up special registers used for translated code.
  mov %rdi, %rbx          // rVmSp
  mov %r8,  %r12          // rVmTl
  mov %rsi, %rbp          // rVmFp

  sub $8, %rsp // align native stack
  CFI2(adjust_cfa_offset, 8)

  /*
   * If we're entering the TC at a function prologue, make it look like we got
   * there via a bindcall by pushing return addresses, setting the callee frame
   * pointer, then jumping to the prologue.
   */
  test %r9, %r9
  jz ETCH_LABEL(enterTCHelper$callTC)
  push ETCH_NAME_REL(enterTCExit)
  push 0x8(%r9)
  mov %r9, %rbp
  jmp *%rdx

  /*
   * The translated code we are about to enter does not follow the
   * standard prologue of pushing rbp at entry, so we are purposely 8
   * bytes short of 16-byte alignment before this call instruction so
   * that the return address being pushed will make the native stack
   * 16-byte aligned.
   */
ETCH_LABEL(enterTCHelper$callTC):
  call *%rdx

  /*
   * enterTCExit is never called directly; this exists to give the jit
   * access to the address of the expected return address while in the TC.
   */
  .globl ETCH_NAME(enterTCExit)
ETCH_NAME(enterTCExit):
  /*
   * Eager vm-reg save. Must match values in rds-header.h
   */
  mov %rbx, 0x10(%r12)
  mov %rbp, 0x20(%r12)
  add $8, %rsp
  CFI2(adjust_cfa_offset, -8)

  // Epilogue
  pop %rbp
  CFI2(restore, rbp)
  CFI2(adjust_cfa_offset, -8)
  ret
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
  mov %rbx, 0x10(%r12)
  mov %rbp, 0x20(%r12)

  // Push a ServiceReqInfo struct onto the stack and call handleServiceRequest.
  push %r8
  push %rcx
  push %rdx
  push %rsi
  push %r10
  push %rdi
  CFI2(adjust_cfa_offset, 0x30)

  // call mcg->handleServiceRequest(%rsp)
  mov ETCH_NAME(mcg)(%rip), %rdi
  mov %rsp, %rsi
  call MCGenerator_handleServiceRequest

  // Pop the ServiceReqInfo off the stack.
  add $0x30, %rsp
  CFI2(adjust_cfa_offset, -0x30)

  // rVmTl was preserved by the callee, but vmsp and vmfp might've changed if
  // we interpreted anything. Reload them.
  mov 0x10(%r12), %rbx
  mov 0x20(%r12), %rbp

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
ETCH_SECTION(enterTCHelper)
.globl ETCH_NAME(enterTCHelper)
ETCH_NAME(enterTCHelper):
  // Prologue
  mflr %r0
  std %r0, 16(%r1)    // LR on parent call frame
  stdu %r1, -192(%r1) // aligned with Epilogue: addi %r1, %r1, 192

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
  std %r3,  176(%r1)  // local variable #18 //TODO check if it will be always necessary

  mr %r3, %r4 // change fp to right register

  mr %r12, %r5        // %r12 is the function linkage
  mtctr %r12
  bctrl               // mind the 'l' at the end so that it returns here later
  b enterTCExit
ETCH_SIZE(enterTCHelper)
// ==== enterTCHelper ====

ETCH_SECTION(handleSRHelper)
.globl ETCH_NAME(handleSRHelper)
ETCH_NAME(handleSRHelper):
  blr
ETCH_SIZE(handleSRHelper)

ETCH_SECTION(handleSRResumeTC)
.globl ETCH_NAME(handleSRResumeTC)
ETCH_NAME(handleSRResumeTC):
  blr
ETCH_SIZE(handleSRResumeTC)

ETCH_SECTION(enterTCServiceReq)
.globl ETCH_NAME(enterTCServiceReq)
ETCH_NAME(enterTCServiceReq):
  blr
ETCH_SIZE(enterTCServiceReq)

ETCH_SECTION(enterTCExit)
.globl ETCH_NAME(enterTCExit)
ETCH_NAME(enterTCExit):
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
  ld %r3, 176(%r1)    // local variable #18 //TODO check if it will be always necessary

  addi %r1, %r1, 192  // aligned with Prologue: stdu %r1, -192(%r1)
  ld %r0, 16(%r1)
  mtlr %r0
  blr
ETCH_SIZE(enterTCExit)



#endif
