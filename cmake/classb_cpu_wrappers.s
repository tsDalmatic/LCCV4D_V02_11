/*
 * AAPCS wrappers for the prebuilt Keil Class-B CPU self-test routines.
 *
 * The supplied routines intentionally overwrite the CPU registers being
 * tested.  Keil's callers account for that, while GCC expects R4-R11 to be
 * preserved by every function call.  Preserve those registers here so that
 * GCC can safely continue to use (for example) R7 as its frame pointer.
 */

.syntax unified
.cpu cortex-m0
.thumb

.section .text.classb_cpu_wrappers,"ax",%progbits

.macro CLASSB_CPU_WRAPPER wrapper, test
    .global \wrapper
    .type \wrapper, %function
    .thumb_func
\wrapper:
    /* Six words plus four words keeps SP 8-byte aligned at the BL. */
    push    {r3-r7, lr}
    mov     r4, r8
    mov     r5, r9
    mov     r6, r10
    mov     r7, r11
    push    {r4-r7}

    bl      \test

    pop     {r4-r7}
    mov     r8, r4
    mov     r9, r5
    mov     r10, r6
    mov     r11, r7
    pop     {r3-r7, pc}
    .size   \wrapper, .-\wrapper
.endm

CLASSB_CPU_WRAPPER STL_StartUpCPUTest_GCC, STL_StartUpCPUTest
CLASSB_CPU_WRAPPER STL_RunTimeCPUTest_GCC, STL_RunTimeCPUTest
