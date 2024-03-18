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
 * @brief Enable interrupts.
 *
 * @ingroup Interrupt Management
 */
void vPortEnableInterrupts( void );

#define portENABLE_INTERRUPTS() vPortEnableInterrupts()

/**
 * @brief Disable interrupts.
 *
 * @ingroup Interrupt Management
 */
void vPortDisableInterrupts( void );

#define portDISABLE_INTERRUPTS() vPortDisableInterrupts()

/**
 * @brief Perform a DSB and ISB to flush the data and instruction caches.
 */
void vPortPipelineFlush( void );

#define portPIPELINE_FLUSH() vPortPipelineFlush()

/**
 * @brief Set the PSP to be the pulSystemCallStack.
 */
void vPortSetSystemCallStack( uint32_t * pulSystemCallStack );

/**
 * @brief Set the control bit high for the duration of a system call.
 */
void vPortRaisePrivilege( void );

/**
 * @brief Raise SVC for exiting from a system call.
 */
void vRequestSystemCallExit( void );


/**
 * @brief Set the Program Stack Pointer.
 */
void vPortSetPSP( uint32_t * pulTaskStack );

/**
 * @brief Obtain the number of the currently executing interrupt. 
 *
 * @note Interrupt Program Status Register (IPSR) holds the exception
 * number of the currently-executing exception or zero for Thread mode.
 */
uint32_t xPortGetIPSR( void );


/*-----------------------------------------------------------*/

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* PORTMACRO_H */
