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
	.extern vAssertCalled
    #define portSVC_SYSTEM_CALL_EXIT           104

/* ----------------------------------------------------------------------------------- */

/*
 * void vPortDisableInterrupts( void );
 */
.align 4
.global vPortDisableInterrupts
.type vPortDisableInterrupts, %function
vPortDisableInterrupts:
    CPSID   I
    BX     LR

/* ----------------------------------------------------------------------------------- */

/*
 * void vPortEnableInterrupts( void );
 */
.align 4
.global vPortEnableInterrupts
.type vPortEnableInterrupts, %function
vPortEnableInterrupts:
    CPSIE   I
    BX     LR

/* ----------------------------------------------------------------------------------- */

/*
 * void vRaisePrivilege( void );
 */
.align 4
.global vRaisePrivilege
.type vRaisePrivilege, %function
vRaisePrivilege:
    PUSH    { R0,R1 }
    MRS     R0, control     /* r0 = CONTROL. */
    MOVS    R1, #1          /* r1 = 1. */
    BICS    R0, R1          /* r0 = r0 & ~r1. */
    MSR     control, R0     /* CONTROL = r0. */
    POP     { R0,R1 }
    BX      LR              /* Return to the caller. */

/* ----------------------------------------------------------------------------------- */

/*
 * void vResetPrivilege( void );
 */
.align 4
.global vResetPrivilege
.type vResetPrivilege, %function
vResetPrivilege:
    PUSH    { R0, R1, LR }
    MRS     R0, control     /* r0 = CONTROL. */
    MOVS    R1, #1          /* r1 = 1. */
    ORRS    R0, R1          /* r0 = r0 | r1. */
    MSR     control, R0     /* CONTROL = r0. */
    POP     { R0, R1, PC }
    BX      LR              /* Return to the caller. */

/* ----------------------------------------------------------------------------------- */

/*
 * void vPortSetPSP( uint32_t * pulStack )
 */
.align 4
.global vPortSetPSP
.type vPortSetPSP, %function
vPortSetPSP:
    PUSH    { LR }
    MSR     PSP, R0
    POP     { PC }

/* ----------------------------------------------------------------------------------- */

/*
 * void vPortPipelineFlush( void );
 */
.align 4
.global vPortPipelineFlush
.type vPortPipelineFlush, %function
vPortPipelineFlush:
    PUSH    { LR }
    DSB
    ISB
    POP     { PC }

/* ----------------------------------------------------------------------------------- */

.end
