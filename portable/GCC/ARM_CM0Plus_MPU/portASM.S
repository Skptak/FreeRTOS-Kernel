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

    /* External FreeRTOS-Kernel Functions */
    .extern vTaskSwitchContext

    .extern vSVCHandler_C
    .extern vSystemCallEnter
    .extern vSystemCallExit

    /* These must be in sync with portmacro.h. */
    #define portSVC_START_SCHEDULER        100
    #define portSVC_YIELD                  101
    #define portSVC_SYSTEM_CALL_EXIT       104

    #include "mpu_syscall_numbers.h"

/* ----------------------------------------------------------------------------------- */
/*
.align 4
.type portRESTORE_CONTEXT, %function
portRESTORE_CONTEXT:
*/
.macro portRESTORE_CONTEXT
    /* Don't reset the MSP stack as is done on CM3/4 devices. The vector table
     * in some CM0 devices cannot be modified and thus may not hold the
     * application's initial MSP value. */

    /* ------------ Program MPU. ------------ */
    /* r3 = &pxCurrentTCB. */
    ldr       r3, =pxCurrentTCB
    /* r0 = pxCurrentTCB. */
    ldr       r0, [r3]
    /* r2 = Second item in the TCB which is xMPUSettings. */
    adds      r0, #4

    /* Complete outstanding transfers before disabling MPU. */
    dmb
    /* MPU_CTRL register. */
    ldr     r1, =0xe000ed94
    /* Read the value of MPU_CTRL. */
    ldr     r2, [r1]
    movs    r3, #0x1
    /* r2 = r2 & ~1 i.e. Clear the bit 0 in r2. */
    bics    r2, r2, r3
    /* Disable MPU. */
    str     r2, [r1]


    /* Region Base Address register. */
    ldr   r1, =0xe000ed98   /* r1 = 0xe000ed98 [Location of RNR] */
    ldr	  r2, =0xe000ed9c   /* r2 = 0xe000ed9c [Location of RBAR]. */
    ldr   r3, =0xe000eda0   /* r3 = 0xe000eda0 [Location of RASR]. */

    ldmia r0!, {r5-r6}      /* Read first set of RBAR/RASR registers from TCB. */
    str r5, [r2]            /* r2 = 0xe000ed9c [Location of RBAR]. */
    str r6, [r3]            /* r3 = 0xe000eda0 [Location of RASR]. */

    ldmia r0!, {r5-r6}      /* Read second set of RBAR/RASR registers from TCB. */
    str r5, [r2]            /* r2 = 0xe000ed9c [Location of RBAR]. */
    str r6, [r3]            /* r3 = 0xe000eda0 [Location of RASR]. */

    ldmia r0!, {r5-r6}      /* Read third set of RBAR/RASR registers from TCB. */
    str r5, [r2]            /* r2 = 0xe000ed9c [Location of RBAR]. */
    str r6, [r3]            /* r3 = 0xe000eda0 [Location of RASR]. */

    ldmia r0!, {r5-r6}      /* Read fourth set of RBAR/RASR registers from TCB. */
    str r5, [r2]            /* r2 = 0xe000ed9c [Location of RBAR]. */
    str r6, [r3]            /* r3 = 0xe000eda0 [Location of RASR]. */

    ldmia r0!, {r5-r6}      /* Read fifth set of RBAR/RASR registers from TCB. */
    str r5, [r2]            /* r2 = 0xe000ed9c [Location of RBAR]. */
    str r6, [r3]            /* r3 = 0xe000eda0 [Location of RASR]. */

    /* MPU_CTRL register. */
    ldr   r1, =0xe000ed94
    /* Read the value of MPU_CTRL. */
    ldr     r2, [r1]
    movs    r3, #0x1
    /* r2 = r2 & ~1 i.e. Clear the bit 0 in r2. */
    orrs    r2, r2, r3
    /* Enable MPU. */
    str     r2, [r1]
    /* Force memory writes before continuing. */
    dsb

    /*---------- Restore Context. ---------- */
    /* R3 = = &pxCurrentTCB */
    LDR     R2, =pxCurrentTCB
    /* r0 = pxCurrentTCB. */
    LDR     r0, [r2]
    /* The first item in pxCurrentTCB is the location of the task context. */
    LDR     r1, [r0]

    subs r1, #0xC
    ldmia r1!, {r2-r4}                  /* r2 = original PSP, r3 = CONTROL, r4 = LR. */
    subs r1, #0xC

    msr psp, r2
    msr control, r3
    mov lr, r4

    subs r1, #32                        /* Move to the hardware saved context */
    ldmia r1!, {r4-r7}                  /* r4-r7 contain half of the hardware saved context. */
    stmia r2!, {r4-r7}                  /* Copy half of the the hardware saved context on the task stack. */
    ldmia r1!, {r4-r7}                  /* r4-r7 contain rest half of the hardware saved context. */
    stmia r2!, {r4-r7}                  /* Copy rest half of the the hardware saved context on the task stack. */
    subs r1, #48
    ldmia r1!, {r4-r7}                  /* Restore r8-r11. */
    mov r8, r4                          /* r8 = r4. */
    mov r9, r5                          /* r9 = r5. */
    mov r10, r6                         /* r10 = r6. */
    mov r11, r7                         /* r11 = r7. */
    subs r1, #32
    ldmia r1!, {r4-r7}                  /* Restore r4-r7. */
    subs r1, #16

    str r1, [r0]                        /* Save the location where the context should be saved next as the first member of TCB. */
    bx lr

.endm

.align 4
.global vPortStartFirstTask
.type vPortStartFirstTask, %function
vPortStartFirstTask:
    /* Don't reset the MSP stack as is done on CM3/4 devices. The vector table
     * in some CM0 devices cannot be modified and thus may not hold the
     * application's initial MSP value. */

    /* Perform a Data and Instruction Cache flush before making the SVC to
     * restore the context of the first task. */
    cpsie i
    cpsie f
    dsb
    isb
    svc #portSVC_START_SCHEDULER
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
.global SVC_Handler
.type SVC_Handler, %function
SVC_Handler:
    MOVS r0, #0x4
    mov r1, lr
    tst r1, r0
    beq load_msp
    mrs r0, psp
    b svc_num_load
    load_msp:
        mrs r0, msp

     svc_num_load:
        ldr r2, [r0, #24]
        subs r2, #2
        ldrb r1, [r2, #0]
        ldr r2, =#NUM_SYSTEM_CALLS
        cmp r1, r2
        blt syscall_enter

        /* System Call Exit SVC */
        ldr  r2, =#portSVC_SYSTEM_CALL_EXIT
        cmp r1, r2
        beq syscall_exit

        /* Start the first task */
        ldr     R2, =#portSVC_START_SCHEDULER
        cmp     R1, R2
        beq     start_first_task

        /* Handle a pending SVC */
        /* Last case, the SVC yield */
        //LDR     r3, =#portSVC_YIELD
        //CMP     r2, r3
        LDR     r3, =#0xe000ed04
        MOVS     r2, #1
        LSLS    r2, #28
        STR     r3, [r2]
        BLX     LR


    syscall_enter:
        PUSH	{ LR }
        bl vSystemCallEnter
        POP		{ PC }


    syscall_exit:
        PUSH	{ LR }
        bl vSystemCallExit
		POP		{ PC }


    start_first_task:
        portRESTORE_CONTEXT

.align 4
.global PendSV_Handler
.type PendSV_Handler, %function
PendSV_Handler:
    /*------------ Save Context. ----------- */
    ldr r2, =pxCurrentTCB                   /* Read the location of pxCurrentTCB i.e. &( pxCurrentTCB ). */
    ldr r0, [r2]                            /* r0 = pxCurrentTCB. */
    ldr r1, [r0]                            /* r1 = Location in TCB where the context should be saved. */
    mrs r2, psp                             /* r2 = PSP. */

    save_general_regs:
        stmia r1!, {r4-r7}                  /* Store r4-r7. */
        mov r4, r8                          /* r4 = r8. */
        mov r5, r9                          /* r5 = r9. */
        mov r6, r10                         /* r6 = r10. */
        mov r7, r11                         /* r7 = r11. */
        stmia r1!, {r4-r7}                  /* Store r8-r11. */
        ldmia r2!, {r4-r7}                  /* Copy half of the  hardware saved context into r4-r7. */
        stmia r1!, {r4-r7}                  /* Store the hardware saved context. */
        ldmia r2!, {r4-r7}                  /* Copy rest half of txhe  hardware saved context into r4-r7. */
        stmia r1!, {r4-r7}                  /* Store the hardware saved context. */

    save_special_regs:
        mrs r2, psp                         /* r2 = PSP. */
        mrs r3, control                     /* r3 = CONTROL. */
        mov r4, lr                          /* r4 = LR. */
        stmia r1!, {r2-r4}                  /* Store original PSP (after hardware has saved context), PSPLIM, CONTROL and LR. */
        str r1, [r0]                        /* Save the location from where the context should be restored as the first member of TCB. */

    /*---------- Select next task. --------- */
    //push   {r3, r14}
    cpsid  i
    blx     vTaskSwitchContext
    cpsie  i

    portRESTORE_CONTEXT

.end
