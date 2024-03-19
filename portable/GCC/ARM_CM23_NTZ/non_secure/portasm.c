/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/* Standard includes. */
#include <stdint.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE ensures that PRIVILEGED_FUNCTION
 * is defined correctly and privileged functions are placed in correct sections. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Portasm includes. */
#include "portasm.h"

/* System call numbers includes. */
#include "mpu_syscall_numbers.h"

/* MPU_WRAPPERS_INCLUDED_FROM_API_FILE is needed to be defined only for the
 * header files. */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if ( configENABLE_FPU == 1 )
    #error Cortex-M0+ does not have a Floating Point Unit (FPU) and therefore configENABLE_FPU must be set to 0.
#endif /* configENABLE_FPU == 1 */

#if ( configRUN_FREERTOS_SECURE_ONLY == 1 )
    #error Cortex-M0+ Does not support the ARMv6-M Secure Extension
#endif /* configRUN_FREERTOS_SECURE_ONLY == 1 */

void vRestoreContextOfFirstTask( void ) /* __attribute__ (( naked )) PRIVILEGED_FUNCTION */
{
    __asm volatile
    (
        " .extern pxCurrentTCB                            \n"
        " .syntax unified                                 \n"
        "                                                 \n"
        " cpsie i                                         \n"
        " cpsie f                                         \n"
        " dsb                                             \n"
        " isb                                             \n"
        " program_mpu_first_task:                         \n"
        "    dmb                                          \n" /* Complete outstanding transfers before disabling MPU. */
        "    ldr r1, =0xe000ed94                          \n" /* MPU_CTRL register. */
        "    ldr r2, [r1]                                 \n" /* Read the value of MPU_CTRL. */
        "    movs r3, #0x1                                \n"
        "    bics r2, r2, r3                              \n" /* r2 = r2 & ~1 i.e. Clear the bit 0 in r2. */
        "    str r2, [r1]                                 \n" /* Disable MPU */
        "                                                 \n"
        "    ldr r3, =pxCurrentTCB                        \n" /* r3 = &pxCurrentTCB. */
        "    ldr r0, [r3]                                 \n" /* r0 = pxCurrentTCB.*/
        "    adds r0, #8                                  \n" /* r2 = Second item in the TCB which is xMPUSettings. */
        "                                                 \n"
        "    dmb                                          \n" /* Complete outstanding transfers before disabling MPU. */
        "    ldr r1, =0xe000ed94                          \n" /* r1 = 0xe000ed94 [Location of MPU_CTRL]. */
        "    ldr r2, [r1]                                 \n" /* Read the value of MPU_CTRL. */
        "    movs r3, #1                                  \n" /* r3 = 1. */
        "    bics r2, r3                                  \n" /* r2 = r2 & ~r3 i.e. Clear the bit 0 in r2. */
        "    str r2, [r1]                                 \n" /* Disable MPU. */
        "                                                 \n"
        "    ldr r2, =0xe000ed9c                          \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    ldr r3, =0xe000eda0                          \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "    ldmia r0!, {r5-r6}                           \n" /* Read first set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldmia r0!, {r5-r6}                           \n" /* Read second set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldmia r0!, {r5-r6}                           \n" /* Read third set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldmia r0!, {r5-r6}                           \n" /* Read fourth set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldmia r0!, {r5-r6}                           \n" /* Read fifth set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldr r1, =0xe000ed94                          \n" /* MPU_CTRL register. */
        "    ldr r2, [r1]                                 \n" /* Read the value of MPU_CTRL. */
        "    movs r3, #1                                  \n" /* r3 = 1. */
        "    orrs r2, r3                                  \n" /* r2 = r2 | r3 i.e. Set the bit 0 in r2. */
        "    str r2, [r1]                                 \n" /* Enable MPU. */
        "    dsb                                          \n" /* Force memory writes before continuing. */
        "                                                 \n"
        " restore_context_first_task:                     \n"
        "    ldr r2, =pxCurrentTCB                        \n" /* r2 = &pxCurrentTCB. */
        "    ldr r0, [r2]                                 \n" /* r0 = pxCurrentTCB.*/
        "    ldr r1, [r0]                                 \n" /* r1 = Location of saved context in TCB. */
        "                                                 \n"
        " restore_special_regs_first_task:                \n"
        "    subs r1, #16                                 \n"
        "    ldmia r1!, {r2-r5}                           \n" /* r2 = original PSP, r3 = PSPLIM, r4 = CONTROL, r5 = LR. */
        "    subs r1, #16                                 \n"
        "    msr psp, r2                                  \n"
        "    msr control, r4                              \n"
        "    mov lr, r5                                   \n"
        "                                                 \n"
        " restore_general_regs_first_task:                \n"
        "    subs r1, #32                                 \n"
        "    ldmia r1!, {r4-r7}                           \n" /* r4-r7 contain half of the hardware saved context. */
        "    stmia r2!, {r4-r7}                           \n" /* Copy half of the the hardware saved context on the task stack. */
        "    ldmia r1!, {r4-r7}                           \n" /* r4-r7 contain rest half of the hardware saved context. */
        "    stmia r2!, {r4-r7}                           \n" /* Copy rest half of the the hardware saved context on the task stack. */
        "    subs r1, #48                                 \n"
        "    ldmia r1!, {r4-r7}                           \n" /* Restore r8-r11. */
        "    mov r8, r4                                   \n" /* r8 = r4. */
        "    mov r9, r5                                   \n" /* r9 = r5. */
        "    mov r10, r6                                  \n" /* r10 = r6. */
        "    mov r11, r7                                  \n" /* r11 = r7. */
        "    subs r1, #32                                 \n"
        "    ldmia r1!, {r4-r7}                           \n" /* Restore r4-r7. */
        "    subs r1, #16                                 \n"
        "                                                 \n"
        " restore_context_done_first_task:                \n"
        "    str r1, [r0]                                 \n" /* Save the location where the context should be saved next as the first member of TCB. */
        "    bx lr                                        \n"
        ::"i" ( portSVC_START_SCHEDULER ) : "memory"
    );
}

/* ----------------------------------------------------------------------------------- */

BaseType_t xIsPrivileged( void ) /* __attribute__ (( naked )) */
{
    __asm volatile
    (
        "   .syntax unified                                 \n"
        "                                                   \n"
        "   mrs r0, control                                 \n" /* r0 = CONTROL. */
        "   movs r1, #1                                     \n" /* r1 = 1. */
        "   tst r0, r1                                      \n" /* Perform r0 & r1 (bitwise AND) and update the conditions flag. */
        "   beq running_privileged                          \n" /* If the result of previous AND operation was 0, branch. */
        "   movs r0, #0                                     \n" /* CONTROL[0]!=0. Return false to indicate that the processor is not privileged. */
        "   bx lr                                           \n" /* Return. */
        " running_privileged:                               \n"
        "   movs r0, #1                                     \n" /* CONTROL[0]==0. Return true to indicate that the processor is privileged. */
        "   bx lr                                           \n" /* Return. */
        "                                                   \n"
        "   .align 4                                        \n"
        ::: "r0", "r1", "memory"
    );
}
/* ----------------------------------------------------------------------------------- */

void vStartFirstTask( void ) /* __attribute__ (( naked )) PRIVILEGED_FUNCTION */
{
    /* Don't reset the MSP stack as is done on CM3/4 devices. The vector table
     * in some CM0 devices cannot be modified and thus may not hold the
     * application's initial MSP value. */
    __asm volatile
    (
        "   .syntax unified                                 \n"
        "   cpsie i                                         \n" /* Globally enable interrupts. */
        "   dsb                                             \n"
        "   isb                                             \n"
        "   svc %0                                          \n" /* System call to start the first task. */
        "   nop                                             \n"
        "                                                   \n"
        "   .align 4                                        \n"
        ::"i" ( portSVC_START_SCHEDULER ) : "memory"
    );
}
/* ----------------------------------------------------------------------------------- */

uint32_t ulSetInterruptMask( void ) /* __attribute__(( naked )) PRIVILEGED_FUNCTION */
{
    __asm volatile
    (
        "   .syntax unified                                 \n"
        "                                                   \n"
        "   mrs r0, PRIMASK                                 \n"
        "   cpsid i                                         \n"
        "   bx lr                                           \n"
        ::: "memory"
    );
}
/* ----------------------------------------------------------------------------------- */

void vClearInterruptMask( __attribute__( ( unused ) ) uint32_t ulMask ) /* __attribute__(( naked )) PRIVILEGED_FUNCTION */
{
    __asm volatile
    (
        "   .syntax unified                                 \n"
        "                                                   \n"
        "   msr PRIMASK, r0                                 \n"
        "   bx lr                                           \n"
        ::: "memory"
    );
}
/* ----------------------------------------------------------------------------------- */

void PendSV_Handler( void ) /* __attribute__ (( naked )) PRIVILEGED_FUNCTION */
{
    __asm volatile
    (
        " .syntax unified                                 \n"
        "                                                 \n"
        " ldr r2, =pxCurrentTCB                           \n" /* r2 = &( pxCurrentTCB ). */
        " ldr r0, [r2]                                    \n" /* r0 = pxCurrentTCB. */
        " ldr r1, [r0]                                    \n" /* r1 = Location in TCB where the context should be saved. */
        " mrs r2, psp                                     \n" /* r2 = PSP. */
        "                                                 \n"
        " save_general_regs:                              \n"
        "    stmia r1!, {r4-r7}                           \n" /* Store r4-r7. */
        "    mov r4, r8                                   \n" /* r4 = r8. */
        "    mov r5, r9                                   \n" /* r5 = r9. */
        "    mov r6, r10                                  \n" /* r6 = r10. */
        "    mov r7, r11                                  \n" /* r7 = r11. */
        "    stmia r1!, {r4-r7}                           \n" /* Store r8-r11. */
        "    ldmia r2!, {r4-r7}                           \n" /* Copy half of the  hardware saved context into r4-r7. */
        "    stmia r1!, {r4-r7}                           \n" /* Store the hardware saved context. */
        "    ldmia r2!, {r4-r7}                           \n" /* Copy rest half of the  hardware saved context into r4-r7. */
        "    stmia r1!, {r4-r7}                           \n" /* Store the hardware saved context. */
        "                                                 \n"
        " save_special_regs:                              \n"
        "    mrs r2, psp                                  \n" /* r2 = PSP. */
        "    movs r3, #0                                  \n" /* r3 = 0. 0 is stored in the PSPLIM slot. */
        "    mrs r4, control                              \n" /* r4 = CONTROL. */
        "    mov r5, lr                                   \n" /* r5 = LR. */
        "    stmia r1!, {r2-r5}                           \n" /* Store original PSP (after hardware has saved context), PSPLIM, CONTROL and LR. */
        "    str r1, [r0]                                 \n" /* Save the location from where the context should be restored as the first member of TCB. */
        "                                                 \n"
        " select_next_task:                               \n"
        "    cpsid i                                      \n"
        "    bl vTaskSwitchContext                        \n"
        "    cpsie i                                      \n"
        "                                                 \n"
        " ldr r2, =pxCurrentTCB                           \n" /* r2 = &( pxCurrentTCB ). */
        " ldr r0, [r2]                                    \n" /* r0 = pxCurrentTCB. */
        " ldr r1, [r0]                                    \n" /* r1 = Location in TCB where the context should be saved. */
        " adds r0, #8                                     \n" /* r2 = Second item in the TCB which is xMPUSettings. */
        "                                                 \n"
        " program_mpu:                                    \n"
        "    dmb                                          \n" /* Complete outstanding transfers before disabling MPU. */
        "    ldr r1, =0xe000ed94                          \n" /* r1 = 0xe000ed94 [Location of MPU_CTRL]. */
        "    ldr r2, [r1]                                 \n" /* Read the value of MPU_CTRL. */
        "    movs r3, #0x1                                \n" /* r3 = 1. */
        "    bics r2, r3                                  \n" /* r2 = r2 & ~1 i.e. Clear the bit 0 in r2. */
        "    str r2, [r1]                                 \n" /* Disable MPU */
        "                                                 \n"
        "    ldr r2, =0xe000ed9c                          \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    ldr r3, =0xe000eda0                          \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldmia r0!, {r5-r6}                           \n" /* Read first set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldmia r0!, {r5-r6}                           \n" /* Read second set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldmia r0!, {r5-r6}                           \n" /* Read third set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldmia r0!, {r5-r6}                           \n" /* Read fourth set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldmia r0!, {r5-r6}                           \n" /* Read fifth set of RBAR/RASR registers from TCB. */
        "    str r5, [r2]                                 \n" /* r2 = 0xe000ed9c [Location of RBAR]. */
        "    str r6, [r3]                                 \n" /* r3 = 0xe000eda0 [Location of RASR]. */
        "                                                 \n"
        "    ldr r2, [r1]                                 \n" /* Read the value of MPU_CTRL. */
        "    movs r3, #1                                  \n" /* r3 = 1. */
        "    orrs r2, r3                                  \n" /* r2 = r2 | r3 i.e. Set the bit 0 in r2. */
        "    str r2, [r1]                                 \n" /* Enable MPU. */
        "    dsb                                          \n" /* Force memory writes before continuing. */
        "                                                 \n"
        " restore_context:                                \n"
        "    ldr r2, =pxCurrentTCB                        \n" /* r2 = &pxCurrentTCB. */
        "    ldr r0, [r2]                                 \n" /* r0 = pxCurrentTCB.*/
        "    ldr r1, [r0]                                 \n" /* r1 = Location of saved context in TCB. */
        "                                                 \n"
        " restore_special_regs:                           \n"
        "    subs r1, #16                                 \n"
        "    ldmia r1!, {r2-r5}                           \n" /* r2 = original PSP, r3 = PSPLIM, r4 = CONTROL, r5 = LR. */
        "    subs r1, #16                                 \n"
        "    msr psp, r2                                  \n"
        "    msr control, r4                              \n"
        "    mov lr, r5                                   \n"
        "                                                 \n"
        " restore_general_regs:                           \n"
        "    subs r1, #32                                 \n"
        "    ldmia r1!, {r4-r7}                           \n" /* r4-r7 contain half of the hardware saved context. */
        "    stmia r2!, {r4-r7}                           \n" /* Copy half of the the hardware saved context on the task stack. */
        "    ldmia r1!, {r4-r7}                           \n" /* r4-r7 contain rest half of the hardware saved context. */
        "    stmia r2!, {r4-r7}                           \n" /* Copy rest half of the the hardware saved context on the task stack. */
        "    subs r1, #48                                 \n"
        "    ldmia r1!, {r4-r7}                           \n" /* Restore r8-r11. */
        "    mov r8, r4                                   \n" /* r8 = r4. */
        "    mov r9, r5                                   \n" /* r9 = r5. */
        "    mov r10, r6                                  \n" /* r10 = r6. */
        "    mov r11, r7                                  \n" /* r11 = r7. */
        "    subs r1, #32                                 \n"
        "    ldmia r1!, {r4-r7}                           \n" /* Restore r4-r7. */
        "    subs r1, #16                                 \n"
        "                                                 \n"
        " restore_context_done:                           \n"
        "    str r1, [r0]                                 \n" /* Save the location where the context should be saved next as the first member of TCB. */
        "    bx lr                                        \n"
        "                                                 \n"
        " .align 4                                        \n"
        " .extern pxCurrentTCB                            \n"
    );
}

/* ----------------------------------------------------------------------------------- */

void SVC_Handler( void ) /* __attribute__ (( naked )) PRIVILEGED_FUNCTION */
{
    __asm volatile
    (
        ".syntax unified                \n"
        ".extern vPortSVCHandler_C      \n"
        ".extern vSystemCallEnter       \n"
        ".extern vSystemCallExit        \n"
        ".extern pxCurrentTCB           \n"
        "                               \n"
        "movs r0, #4                    \n"
        "mov r1, lr                     \n"
        "tst r0, r1                     \n"
        "beq stack_on_msp               \n"
        "stack_on_psp:                  \n"
        "    mrs r0, psp                \n"
        "    b route_svc                \n"
        "stack_on_msp:                  \n"
        "    mrs r0, msp                \n"
        "    b route_svc                \n"
        "                               \n"
        "route_svc:                     \n"
        "    ldr r3, [r0, #24]          \n"
        "    subs r3, #2                \n"
        "    ldrb r2, [r3, #0]          \n"
        "    ldr r3, =%0                \n"
        "    cmp r2, r3                 \n"
        "    blt system_call_enter      \n"
        "    ldr r3, =%1                \n"
        "    cmp r2, r3                 \n"
        "    beq system_call_exit       \n"
        "    b vPortSVCHandler_C        \n"
        "                               \n"
        "system_call_enter:             \n"
        "   push {lr}                   \n"
        "   bl vSystemCallEnter         \n"
        "   pop {pc}                    \n"
        "system_call_exit:              \n"
        "   push {lr}                   \n"
        "   b vSystemCallExit           \n"
        "   pop {pc}                    \n"
        "                               \n"
        " .align 4                      \n"
        "                               \n"
        : /* No outputs. */
        : "i" ( NUM_SYSTEM_CALLS ), "i" ( portSVC_SYSTEM_CALL_EXIT )
        : "r0", "r1", "r2", "r3", "memory"
    );
}

/* ----------------------------------------------------------------------------------- */
