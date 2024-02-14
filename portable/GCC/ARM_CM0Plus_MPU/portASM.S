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

    /* These must be in sync with portmacro.h. */
    #define portSVC_START_SCHEDULER        100
    #define portSVC_SYSTEM_CALL_EXIT       103


    #define FREERTOS_ASSEMBLY
        #include "mpu_syscall_numbers.h"
    #undef FREERTOS_ASSEMBLY
/* ----------------------------------------------------------------------------------- */

//.macro portRESTORE_CONTEXT
.align 4
.type portRESTORE_CONTEXT, %function
portRESTORE_CONTEXT:
    /* Don't reset the MSP stack as is done on CM3/4 devices. The vector table
     * in some CM0 devices cannot be modified and thus may not hold the
     * application's initial MSP value. */

    /* ------------ Program MPU. ------------ */
    /* r3 = &pxCurrentTCB. */
    ldr       r3, =pxCurrentTCB
    /* r2 = pxCurrentTCB. */
    ldr       r2, [r3]
    /* r2 = Second item in the TCB which is xMPUSettings. */
    adds      r2, r2, #4
    /* Load the address of pulSystemCallStack */
    MOV     R3, R2
    ADDS    R3, R3, #0xB8
    LDR     R3, [R3]
    /* Set the Main Stack Pointer to be pulSystemCallStack */
    MSR     MSP, R3

    /* Complete outstanding transfers before disabling MPU. */
    dmb
    /* MPU_CTRL register. */
    ldr     r0, =0xe000ed94
    /* Read the value of MPU_CTRL. */
    ldr     r3, [r0]
    movs    r1, #0x1
    /* r3 = r3 & ~1 i.e. Clear the bit 0 in r3. */
    bics    r3, r3, r1
    /* Disable MPU. */
    str     r3, [r0]


    /* Region Base Address register. */
    ldr   r0, =0xE000ED9C

    /* Read 1 set of MPU registers [MPU Region # 0].*/
    ldmia r2!, {r4-r5}
    /* Write 1 set of MPU registers [MPU Region # 0 ]. */
    STR     R4, [R0]
    STR     R5, [R0, #0x4]

    /* Read 1 set of MPU registers [MPU Region # 1].*/
    ldmia r2!, {r4-r5}
    /* Write 1 set of MPU registers [MPU Region # 1 ]. */
    STR     R4, [R0]
    STR     R5, [R0, #0x4]
    /* Read 1 set of MPU registers [MPU Region # 2].*/
    ldmia r2!, {r4-r5}
    /* Write 1 set of MPU registers [MPU Region # 2 ]. */
    STR     R4, [R0]
    STR     R5, [R0, #0x4]

    /* Read 1 set of MPU registers [MPU Region # 3].*/
    ldmia r2!, {r4-r5}
    /* Write 1 set of MPU registers [MPU Region # 3 ]. */
    STR     R4, [R0]
    STR     R5, [R0, #0x4]

    /* MPU_CTRL register. */
    ldr   r0, =0xe000ed94
    /* Read the value of MPU_CTRL. */
    ldr   r3, [r0]
    /* r3 = r3 | 1 i.e. Set the bit 0 in r3. */
    orrs  r3, r3, r1
    /* Enable MPU. */
    str   r3, [r0]
    /* Force memory writes before continuing. */
    dsb
    /*---------- Restore Context. ---------- */
contextRestorePart:
    ldr r3, =pxCurrentTCB
    ldr r2, [r3]                           /* r2 = pxCurrentTCB. */
    /* The first item in pxCurrentTCB is the location of the task context. */
    LDR     R0, [R2]
   	/* Move to the hardware saved context */
    SUBS    R0, #0x48
    /* Load R8-R11 from ulContext */
    LDM     R0!, { R2-R5 }

    /* Move high registers up */
    MOV		R8, R2
    MOV		R9, R3
    MOV		R10, R4
    MOV		R11, R5

    /* Load EXEC return, the PSP from the stack, and R4-R7 */
    LDM    R0!, { R2-R7}
    /* Set the Program Stack Pointer (PSP) */
    MSR     PSP, R3
    /* Move the PSP into R1 to push to */
    MOV     R1, R3

    /* Pop the first 5 hardware-pushed registers from ulContext */
    LDM   	R0!, {R3-R7}
    /* Store the first 5 hardware-pushed registers onto the task stack */
    STMIA   R1!, {R3-R7}

    /* Pop the next 3 hardware-pushed registers */
    LDMIA   R0!, {R3-R5}
    /* Store the last two hardware-pushed registers onto the task stack */
    STMIA   R1!, {R3-R5}

    BX  R2
	/* Branch to the EXC_Return instruction */

#if 0

	/* Move R0 past the 4 popped high registers */
	SUBS	R0, #0x18
    /* Pop the remaining high registers */
    LDMIA   R0!, {R4-R5}
    MOV     R9, R4
    MOV     R8, R5

	/* Move R0 to the last remaining low registers */
	SUBS	R0, #0x20
    /* Pop the reamining low registers */
    LDMIA   R0!, {R4-R7}
    
    /* Branch to the EXC_Return instruction */
    BX      R12
/* Don't need this */
#endif

    #if 0
    /* Obtain location of pxCurrentTCB. */
    ldr  r2, =pxCurrentTCB
    ldr  r3, [r2]
    /* The first item in pxCurrentTCB is the location of the task context. */
    LDR     r0, [r3]
    /* Move up ulContext to where the hardware saved context is */
    SUBS    R0, #0x24

    /* Load the Task Stack Pointer into R1 */
    LDMIA    R0!, {R1}
    /* Increment the Task Stack Pointer past the buffering */
    ADDS    R1, #0x24
    /* Set R14 to be the task stack pointer */
    MOV     R14, R1
    /* Set the Program Stack Pointer to be the Task Stack Pointer */
    MSR     PSP, R1

    /* Set the SP to this location so we can pop 8 Registers at a time */
    MOV     SP, R0
    /* Set R12 to be the start of R4-R11 */
    SUBS    R0, #0x2C
    /* Save this location in R12 to load later */
    MOV     R12, R0

    /* Pop R0-R3, R12, LR, PC, and xPSR */
    POP     { R0-R7 }
    /* Set MSP to be the Task Stack, with room for the context */
    MSR     MSP, R14
    /* Push the hardware saved context to the task stack */
    PUSH    { R0-R7 }
    /* Set R14 to be the incremented value */
    MRS     R14, MSP

    /* Set MSP to be pointed to R4-R11 in ulContext */
    MSR     MSP, R12
    /* Pop R8-R11 into R0-R3, and then R4-R7 into R4-R7 */
    POP     { R0-R7 }

    /* Move R8-R10 into their correct registers */
    MOV     R8, R0
    MOV     R9, R1
    MOV     R10, R2
    /* Hold off on setting R11 so we can load the correct MSP */

    /* Load the initial control value from ulContext */
    POP     { R1 }

    /* Set the Main Stack Pointer to be pulSystemCallStack */
    MSR     MSP, R11

    /* Now set R11 to be the correct value */
    MOV     R11, R3

    /* Set R0 to be the bottom 3 hardware pushed values */
    MOV     R0, R14
    ADDS    R0, #0x10

    /* Set the Thread Stack Pointer to be the Task Stack */
    MSR     PSP, R0

    /* Set initial task Control Value */
    MSR     CONTROL, R1

    /* Pop off R12-R15 */
    LDMIA   R0!, {R1-R3}
    /* Restore the task's R12, and PC, no default LR is set */
    MOV     R12, R1
    MOV     LR, R3
    SUBS    R0, #0x1C
    /* Restore the task's R0-R3 */
    LDM     R0, {R0-R3}
    dsb
    isb
    BX      LR
#endif
//.endm

.align 4
.global vPortStartFirstTask
.type vPortStartFirstTask, %function
vPortStartFirstTask:
    /* Don't reset the MSP stack as is done on CM3/4 devices. The vector table
     * in some CM0 devices cannot be modified and thus may not hold the
     * application's initial MSP value. */

    /* Perform a Data and Instruction Cache flush before making the SVC to
     * restore the context of the first task. */
    #if 0
    ldr     r0, =0xE000ED1C
    ldr     r1, [r0]
    movs     r2, #15
    lsls    r2, #24
    orrs    r2, r2, r1
    str     r2, [r0]
    #endif
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
        /* Highest possible SVC on the M0+ is SVC #0xFF */
        MOVS	R1, #0XFF
        ANDS	R2, R2, R1
        ldr r3, =#NUM_SYSTEM_CALLS
        cmp r2, r3
        blt syscall_enter

        /* System Call Exit SVC */
        ldr  r3, =#portSVC_SYSTEM_CALL_EXIT
        cmp r2, r3
        beq syscall_exit

        /* Start the first task */
        ldr     R3, =#portSVC_START_SCHEDULER
        cmp     R2, R3
        beq     start_first_task

        /* If not the above go to vSVCHandler_C */
        bl vSVCHandler_C


    syscall_enter:
        mov r1, lr
        bl vSystemCallEnter

    syscall_exit:
        mov r1, lr
        bl vSystemCallExit

    start_first_task:
        b portRESTORE_CONTEXT

.align 4
.global PendSV_Handler
.type PendSV_Handler, %function
PendSV_Handler:
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
