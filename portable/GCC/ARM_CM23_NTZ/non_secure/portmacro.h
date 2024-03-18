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

#ifndef PORTMACRO_H
#define PORTMACRO_H

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

/*------------------------------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the given hardware
 * and compiler.
 *
 * These settings should not be altered.
 *------------------------------------------------------------------------------
 */

/**
 * Architecture specifics.
 */
#define portARCH_NAME                    "Cortex-M0+"
#define portDONT_DISCARD                 __attribute__( ( used ) )
#define portHAS_ARMV8M_MAIN_EXTENSION   0
/*-----------------------------------------------------------*/

/* ARMv8-M common port configurations. */
#include "portmacrocommon.h"
/*-----------------------------------------------------------*/

#if ( configTOTAL_MPU_REGIONS != 8 )
    #error The ARM Cortex M0+ Only supports 8 MPU Regions
#endif

#ifndef configENABLE_MVE
    #define configENABLE_MVE    0
#elif( configENABLE_MVE != 0 )
    #error configENABLE_MVE must be left undefined, or defined to 0 for the Cortex-M0+.
#endif

#ifndef configENABLE_FPU
    #define configENABLE_FPU    0
#elif( configENABLE_FPU != 0 )
    #error configENABLE_FPU must be left undefined, or defined to 0 for the Cortex-M0+.
#endif
/*-----------------------------------------------------------*/

/**
 * @brief Critical section management.
 */
#define portDISABLE_INTERRUPTS()    __asm volatile ( " cpsid i " ::: "memory" )
#define portENABLE_INTERRUPTS()     __asm volatile ( " cpsie i " ::: "memory" )

/**
 * @brief Size in words to use for an exception stack, used when an assert is triggered.
 */
#ifndef configEXCEPTION_STACK_SIZE
    #define configEXCEPTION_STACK_SIZE 0x200
#endif /* configEXCEPTION_STACK_SIZE */


/**
 * @brief Info Pulled from from the CPU when a hardware exception occurs.
 */
typedef struct CORTEX_RX_SAFE_ASSERT_FAULT_INFO
{
    /* Cortex-R Series Exception Diagnostics. */
    uint32_t ulCPSR;    /* Current Program Status Register (CPSR). */
    uint32_t ulDFSR;    /* Data Fault Status Register (DFSR). */
    uint32_t ulIFSR;    /* Instruction Fault Status Register (IFSR). */
    uint32_t ulADFSR;   /* Auxiliary Data Fault Status Register (ADFSR). */
    uint32_t ulAIFSR;   /* Auxiliary Instruction Fault Status Register (AIFSR). */
    uint32_t ulDFAR;    /* Data Fault Address Register (DFAR). */
    uint32_t ulIFAR;    /* Instruction Fault Address Register (IFAR). */

    /* Arm-M Series Architecture Exception Stack. */
    uint32_t ulGPRZero;                 /* General Purpose Register 0 */
    uint32_t ulGPROne;                  /* General Purpose Register 1 */
    uint32_t ulGPRTwo;                  /* General Purpose Register 2 */
    uint32_t ulGPRThree;                /* General Purpose Register 3 */
    uint32_t ulGPRTwelve;               /* General Purpose Register 12 */
    uint32_t ulLinkRegister;            /* Link Register when Fault Happened. */
    uint32_t ulProgramCounter;          /* Program Counter when Fault Happened.  */
    uint32_t ulProgramStatusRegister;   /* Current Program Status Register when fault happened */
} xFrPFaultExceptionInfo;
/*-----------------------------------------------------------*/

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* PORTMACRO_H */
