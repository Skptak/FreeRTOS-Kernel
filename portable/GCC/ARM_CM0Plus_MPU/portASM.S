/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2024 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the Software), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
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

    .thumb
    .cpu cortex-m0plus
    .syntax unified
    /* All code in the portASM.S file is intended to be run from a prvileged
     * operating mode, as such mark the entire file as privileged_functions */
    .section privileged_functions

    /* External FreeRTOS-Kernel Variables */
    .extern pxCurrentTCB
    .extern uxSystemCallImplementations
    .extern ulPortInterruptNesting
    .extern ulPortYieldRequired

    /* External Linker script variables that are needed by the port */
    .extern __privileged_functions_start__
    .extern __privileged_functions_end__
    .extern __privileged_stacks_start__
    .extern __privileged_stacks_end__
    .extern __syscalls_flash_length__
    .extern __syscalls_flash_start__
    .extern __syscalls_flash_end__

    /* External FreeRTOS-Kernel Functions */
    .extern vAssertCalled
    .extern vTaskSwitchContext
    .extern vApplicationIRQHandler

    .extern vSVCHandler_C
    .extern vSystemCallEnter
    .extern vSystemCallExit

    
    #define FREERTOS_ASSEMBLY
        #include "mpu_syscall_numbers.h"
    #undef FREERTOS_ASSEMBLY
/* ----------------------------------------------------------------------------------- */

.align 4
.global vPortStartFirstTask
.type vPortStartFirstTask, %function
vPortStartFirstTask:
    //ldr   r0, =0xE000ED08                      /* MPU_CTRL register. */
    ldr   r1, =#0xE000
    lsls  r1, r1, #0x16
    ldr   r1, =#0xED08                          /* Use the NVIC offset register to locate the stack. */
    ldr   r0, [r0]
    ldr   r0, [r0]
    msr   msp, r0                              /* Set the msp back to the start of the stack. */

    /* ------------ Program MPU. ------------ */
    ldr   r3, =pxCurrentTCB               /* r3 = pxCurrentTCBConst2. */
    ldr   r2, [r3]                             /* r2 = pxCurrentTCB. */
    adds  r2, r2, #4                           /* r2 = Second item in the TCB which is xMPUSettings. */

    dmb                                        /* Complete outstanding transfers before disabling MPU. */
    ldr   r0, =0xe000ed94                      /* MPU_CTRL register. */
    //ldr   r0, =#0xE000
    //lsls  r0, r0, #0x16
    //ldr   r0, =#0xED94                          /* MPU_CTRL register. */
    ldr   r3, [r0]                             /* Read the value of MPU_CTRL. */
    ldr   r1, =#1
    bics  r3, r3, r1                           /* r3 = r3 & ~1 i.e. Clear the bit 0 in r3. */
    str   r3, [r0]                             /* Disable MPU. */


    // ldr r0, =0xe000ed9c                      /* Region Base Address register. */
    ldr   r0, =0xE000
    lsls  r0, r0, #0x16
    ldr   r0, =0xED9C                          /* Region Base Address register. */
    ldmia r2!, {r4-r7}                         /* Read 2 sets of MPU registers [MPU Region # 0 - 1]. */
    stmia r0!, {r4-r7}                         /* Write 2 sets of MPU registers [MPU Region # 0 - 2]. */
    ldmia r2!, {r4-r7}                         /* Read 2 sets of MPU registers [MPU Region # 2 - 3]. */
    stmia r0!,  {r4-r7}                        /* Write 2 sets of MPU registers [MPU Region # 2 - 3]. */

    //ldr   r0, =0xe000ed94                      /* MPU_CTRL register. */
    ldr   r0, =0xE000
    lsls  r0, r0, #0x16
    ldr   r0, =0xED94                          /* MPU_CTRL register. */
    ldr   r3, [r0]                             /* Read the value of MPU_CTRL. */
    orrs  r3, r3, r1                           /* r3 = r3 | 1 i.e. Set the bit 0 in r3. */
    str   r3, [r0]                             /* Enable MPU. */
    dsb                                        /* Force memory writes before continuing. */

    /*---------- Restore Context. ---------- */
    ldr  r2, =pxCurrentTCB                 /* Obtain location of pxCurrentTCB. */
    ldr  r3, [r2]
    ldr  r0, [r3]                                /* The first item in pxCurrentTCB is the task top of stack. */
    adds r0, #32                                 /* Discard everything up to r0. */
    msr  psp, r0                                 /* This is now the new top of stack to use in the task. */
    movs r0, #2                                  /* Switch to the psp stack. */
    msr  CONTROL, r0
    isb
    pop  {r0-r5}                                 /* Pop the registers that are saved automatically. */
    mov  lr, r5                                  /* lr is now in r5. */
    pop  {r3}                                    /* Return address is now in r3. */
    pop  {r2}                                    /* Pop and discard XPSR. */
    cpsie i                                      /* The first task has its context and interrupts can be enabled. */
    bx   r3                                      /* Finally, jump to the user defined task code. */
    .align 4
/*-----------------------------------------------------------*/

/* Handler for Supervisor Calls (SVCs) when using this FreeRTOS Port */

/* Upon entering here the LR, or R14, will hold the address of the following
 * instruction. This then checks that instruction for the SVC # raised.
 * Checks:
 *    1. SVC is raised from the system call section (i.e. application is
 *       not raising SVC directly).
 *    2. pxMpuSettings->xSystemCallStackInfo.pulTaskStack must be NULL as
 *       it is non-NULL only during the execution of a system call (i.e.
 *       between system call enter and exit).
 *    3. System call is not for a kernel API disabled by the configuration
 *       in FreeRTOSConfig.h.
 *    4. We do not need to check that ucSystemCallNumber is within range
 *       because the assembly SVC handler checks that before calling
 *       this function.
 */
.align 4
.global vPortSVCHandler
.type vPortSVCHandler, %function
vPortSVCHandler:
     ldr r0, =#4
     mov r1, lr
     tst r1, r0
     beq load_msp
     mrs r0, psp
     b svc_num_load
     load_msp:
        mrs r0, msp

     svc_num_load:
       ldr r1, [r0, #24]
       subs r1, r1, #2
       ldr r2, [r1]
       ldr r3, =#NUM_SYSTEM_CALLS
       cmp r2, r3
       blt syscall_enter
       ldr  r3, =#104
       cmp r2, r3
       beq syscall_exit
       bl vSVCHandler_C

     syscall_enter:
         mov r1, lr
         bl vSystemCallEnter

     syscall_exit:
         mov r1, lr
         bl vSystemCallExit

.align 4
.global xPortPendSVHandler
.type xPortPendSVHandler, %function
xPortPendSVHandler:
     ldr r3, =pxCurrentTCB             /* r3 = pxCurrentTCBConst. */
     ldr r2, [r3]                           /* r2 = pxCurrentTCB. */
     ldr r1, [r2]                           /* r1 = Location where the context should be saved. */

    /*------------ Save Context. ----------- */
     mrs r3, control
     mrs r0, psp
     isb

     subs r0, r0, #32                     /* Make space for the remaining low registers. */
     str r0, [r2]                         /* Save the new top of stack. */
     stmia r0!, {r4-r7}                   /* Store the low registers that are not saved automatically. */
     mov r4, r8                           /* Store the high registers. */
     mov r5, r9
     mov r6, r10
     mov r7, r11
     stmia r0!, {r4-r7}

    /*---------- Select next task. --------- */
     push {r3, r14}
     cpsid i
     bl vTaskSwitchContext
     cpsie i
     pop {r2, r3}                         /* lr goes in r3. r2 now holds tcb pointer. */

    /*------------ Program MPU. ------------ */
     adds r2, r2, #4                         /* r2 = Second item in the TCB which is xMPUSettings. */

     dmb                                    /* Complete outstanding transfers before disabling MPU. */
    // ldr r0, =0xe000ed94                    /* MPU_CTRL register. */
     ldr   r0, =0xE000
     lsls  r0, r0, #0x16
     ldr   r0, =0xED94                          /* MPU_CTRL register. */
     ldr r3, [r0]                           /* Read the value of MPU_CTRL. */
     ldr r1, =#1
     bics r3, r3, r1                        /* r3 = r3 & ~1 i.e. Clear the bit 0 in r3. */
     str r3, [r0]                           /* Disable MPU. */

    // ldr r0, =0xe000ed9c                    /* Region Base Address register. */
     ldr   r0, =#0xE000
     lsls  r0, r0, #0x16
     ldr   r0, =#0xED9C                          /* Region Base Address register. */
     ldmia r2!, {r4-r7}                     /* Read 2 sets of MPU registers [MPU Region # 0 - 1]. */
     stmia r0!, {r4-r7}                     /* Write 2 sets of MPU registers [MPU Region # 0 - 2]. */
     ldmia r2!, {r4-r7}                    /* Read 2 sets of MPU registers [MPU Region # 2 - 3]. */
     stmia r0!,  {r4-r7}                    /* Write 2 sets of MPU registers [MPU Region # 2 - 3]. */

    // ldr r0, =0xe000ed94                    /* MPU_CTRL register. */
     ldr   r0, =#0xE000ED94
     lsls  r0, r0, #0x16
     ldr   r0, =#0xED94                          /* MPU_CTRL register. */
     ldr r3, [r0]                           /* Read the value of MPU_CTRL. */
     orrs r3, r3, r1                           /* r3 = r3 | 1 i.e. Set the bit 0 in r3. */
     str r3, [r0]                           /* Enable MPU. */
     dsb                                    /* Force memory writes before continuing. */

    /*---------- Restore Context. ---------- */
     ldr  r2, =pxCurrentTCB      /* Obtain location of pxCurrentTCB. */
     ldr  r3, [r2]
     ldr  r0, [r3]                    /* The first item in pxCurrentTCB is the task top of stack. */
     adds r0, #32                     /* Discard everything up to r0. */
     msr  psp, r0                     /* This is now the new top of stack to use in the task. */
     movs r0, #2                      /* Switch to the psp stack. */
     msr  CONTROL, r0
     isb
     pop  {r0-r5}                     /* Pop the registers that are saved automatically. */
     mov  lr, r5                      /* lr is now in r5. */
     pop  {r3}                        /* Return address is now in r3. */
     pop  {r2}                        /* Pop and discard XPSR. */
     cpsie i                          /* The first task has its context and interrupts can be enabled. */
     bx   r3                          /* Finally, jump to the user defined task code. */

    .align 4

.end
