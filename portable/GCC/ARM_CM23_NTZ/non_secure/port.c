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

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers. That should only be done when
 * task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* MPU includes. */
#include "mpu_wrappers.h"
#include "mpu_syscall_numbers.h"

/* Portasm includes. */
#include "portasm.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* ----------------------------------------------------------------------------------- */

/**
 * @brief Prototype of all Interrupt Service Routines (ISRs).
 */
typedef void ( * portISR_t )( void );
/* ----------------------------------------------------------------------------------- */

/**
 * @brief Constants required to manipulate the NVIC.
 */
#define portNVIC_SYSTICK_CTRL_REG             ( *( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG             ( *( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG    ( *( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SHPR3_REG                    ( *( ( volatile uint32_t * ) 0xe000ed20 ) )
#define portNVIC_SYSTICK_ENABLE_BIT           ( 1UL << 0UL )
#define portNVIC_SYSTICK_INT_BIT              ( 1UL << 1UL )
#define portNVIC_SYSTICK_CLK_BIT              ( 1UL << 2UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT       ( 1UL << 16UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT       ( 1UL << 25UL )
#define portNVIC_PEND_SYSTICK_SET_BIT         ( 1UL << 26UL )
#define portMIN_INTERRUPT_PRIORITY            ( 255UL )
#define portNVIC_PENDSV_PRI                   ( portMIN_INTERRUPT_PRIORITY << 16UL )
#define portNVIC_SYSTICK_PRI                  ( portMIN_INTERRUPT_PRIORITY << 24UL )
/* ----------------------------------------------------------------------------------- */

/**
 * @brief Constants required to manipulate the SCB.
 */
#define portSCB_VTOR_REG                      ( *( ( portISR_t ** ) 0xe000ed08 ) )
#define portSCB_SYS_HANDLER_CTRL_STATE_REG    ( *( ( volatile uint32_t * ) 0xe000ed24 ) )
#define portSCB_MEM_FAULT_ENABLE_BIT          ( 1UL << 16UL )
/* ----------------------------------------------------------------------------------- */

/**
 * @brief Constants used to check the installation of the FreeRTOS interrupt handlers.
 */
#define portVECTOR_INDEX_SVC       ( 11 )
#define portVECTOR_INDEX_PENDSV    ( 14 )
/* ----------------------------------------------------------------------------------- */

/**
 * @brief Constants required to check the validity of an interrupt priority.
 */
#define portNVIC_SHPR2_REG                 ( *( ( volatile uint32_t * ) 0xE000ED1C ) )
#define portFIRST_USER_INTERRUPT_NUMBER    ( 16 )
#define portNVIC_IP_REGISTERS_OFFSET_16    ( 0xE000E3F0 )
#define portAIRCR_REG                      ( *( ( volatile uint32_t * ) 0xE000ED0C ) )
#define portTOP_BIT_OF_BYTE                ( ( uint8_t ) 0x80 )
#define portMAX_PRIGROUP_BITS              ( ( uint8_t ) 7 )
#define portPRIORITY_GROUP_MASK            ( 0x07UL << 8UL )
#define portPRIGROUP_SHIFT                 ( 8UL )
/* ----------------------------------------------------------------------------------- */

/**
 * @brief Constants used during system call enter and exit.
 */
#define portPSR_STACK_PADDING_MASK              ( 1UL << 9UL )
#define portEXC_RETURN_STACK_FRAME_TYPE_MASK    ( 1UL << 4UL )
/* ----------------------------------------------------------------------------------- */

/**
 * @brief Offsets in the stack to the parameters when inside the SVC handler.
 */
#define portOFFSET_TO_LR     ( 5 )
#define portOFFSET_TO_PC     ( 6 )
#define portOFFSET_TO_PSR    ( 7 )
/* ----------------------------------------------------------------------------------- */

/**
 * @brief Constants required to manipulate the MPU.
 */
#define portMPU_TYPE_REG                        ( *( ( volatile uint32_t * ) 0xe000ed90 ) )
#define portMPU_CTRL_REG                        ( *( ( volatile uint32_t * ) 0xe000ed94 ) )
#define portMPU_RNR_REG                         ( *( ( volatile uint32_t * ) 0xe000ed98 ) )

#define portMPU_RBAR_REG                        ( *( ( volatile uint32_t * ) 0xe000ed9c ) )
#define portMPU_RASR_REG                        ( *( ( volatile uint32_t * ) 0xe000eda0 ) )

#define portMPU_RBAR_A1_REG                     ( *( ( volatile uint32_t * ) 0xe000eda4 ) )
#define portMPU_RLAR_A1_REG                     ( *( ( volatile uint32_t * ) 0xe000eda8 ) )

#define portMPU_RBAR_A2_REG                     ( *( ( volatile uint32_t * ) 0xe000edac ) )
#define portMPU_RLAR_A2_REG                     ( *( ( volatile uint32_t * ) 0xe000edb0 ) )

#define portMPU_RBAR_A3_REG                     ( *( ( volatile uint32_t * ) 0xe000edb4 ) )
#define portMPU_RLAR_A3_REG                     ( *( ( volatile uint32_t * ) 0xe000edb8 ) )

#define portMPU_MAIR0_REG                       ( *( ( volatile uint32_t * ) 0xe000edc0 ) )
#define portMPU_MAIR1_REG                       ( *( ( volatile uint32_t * ) 0xe000edc4 ) )

#define portMPU_RBAR_ADDRESS_MASK               ( 0xffffff00 ) /* Must be 32-byte aligned. */
#define portMPU_RLAR_ADDRESS_MASK               ( 0xffffffe0 ) /* Must be 32-byte aligned. */
#define portMPU_RASR_LENGTH_MASK                ( 0x1F )       /* Must be [7:31]. */

#define portMPU_RASR_ACCESS_PERMISSIONS_MASK    ( 7UL << 24UL )
#define portMPU_RBAR_ACCESS_PERMISSIONS_MASK    ( 3UL << 1UL )

#define portMPU_MAIR_ATTR0_POS                  ( 0UL )
#define portMPU_MAIR_ATTR0_MASK                 ( 0x000000ff )

#define portMPU_MAIR_ATTR1_POS                  ( 8UL )
#define portMPU_MAIR_ATTR1_MASK                 ( 0x0000ff00 )

#define portMPU_MAIR_ATTR2_POS                  ( 16UL )
#define portMPU_MAIR_ATTR2_MASK                 ( 0x00ff0000 )

#define portMPU_MAIR_ATTR3_POS                  ( 24UL )
#define portMPU_MAIR_ATTR3_MASK                 ( 0xff000000 )

#define portMPU_MAIR_ATTR4_POS                  ( 0UL )
#define portMPU_MAIR_ATTR4_MASK                 ( 0x000000ff )

#define portMPU_MAIR_ATTR5_POS                  ( 8UL )
#define portMPU_MAIR_ATTR5_MASK                 ( 0x0000ff00 )

#define portMPU_MAIR_ATTR6_POS                  ( 16UL )
#define portMPU_MAIR_ATTR6_MASK                 ( 0x00ff0000 )

#define portMPU_MAIR_ATTR7_POS                  ( 24UL )
#define portMPU_MAIR_ATTR7_MASK                 ( 0xff000000 )

#define portMPU_RLAR_ATTR_INDEX0                ( 0UL << 1UL )
#define portMPU_RLAR_ATTR_INDEX1                ( 1UL << 1UL )
#define portMPU_RLAR_ATTR_INDEX2                ( 2UL << 1UL )
#define portMPU_RLAR_ATTR_INDEX3                ( 3UL << 1UL )
#define portMPU_RLAR_ATTR_INDEX4                ( 4UL << 1UL )
#define portMPU_RLAR_ATTR_INDEX5                ( 5UL << 1UL )
#define portMPU_RLAR_ATTR_INDEX6                ( 6UL << 1UL )
#define portMPU_RLAR_ATTR_INDEX7                ( 7UL << 1UL )

#define portMPU_REGION_ENABLE              ( 1UL )

/* Enable privileged access to unmapped region. */
#define portMPU_PRIV_BACKGROUND_ENABLE_BIT      ( 1UL << 2UL )

/* Enable MPU. */
#define portMPU_ENABLE_BIT                      ( 1UL << 0UL )

/* Expected value of the portMPU_TYPE register. */
#define portEXPECTED_MPU_TYPE_VALUE             ( configTOTAL_MPU_REGIONS << 8UL )

/* Extract first address of the MPU region as encoded in the
 * RBAR (Region Base Address Register) value. */
#define portEXTRACT_FIRST_ADDRESS_FROM_RBAR( rbar ) \
    ( ( rbar ) & portMPU_RBAR_ADDRESS_MASK )

/* Extract last address of the MPU region as encoded in the
 * RASR (Region Attribute and Size Register) value. */
#define portEXTRACT_REGION_LENGTH_FROM_RASR( rasr ) \
    ( 1 << ( ( ( rasr >> 1 ) & portMPU_RASR_LENGTH_MASK ) + 1 ) )

/* Does addr lies within [start, end] address range? */
#define portIS_ADDRESS_WITHIN_RANGE( addr, start, end ) \
    ( ( ( addr ) >= ( start ) ) && ( ( addr ) <= ( end ) ) )

/* Is the access request satisfied by the available permissions? */
#define portIS_AUTHORIZED( accessRequest, permissions ) \
    ( ( ( permissions ) & ( accessRequest ) ) == accessRequest )

/* Max value that fits in a uint32_t type. */
#define portUINT32_MAX    ( ~( ( uint32_t ) 0 ) )

/* Check if adding a and b will result in overflow. */
#define portADD_UINT32_WILL_OVERFLOW( a, b )    ( ( a ) > ( portUINT32_MAX - ( b ) ) )
/* ----------------------------------------------------------------------------------- */

/**
 * @brief The maximum 24-bit number.
 *
 * It is needed because the systick is a 24-bit counter.
 */
#define portMAX_24_BIT_NUMBER       ( 0xffffffUL )

/**
 * @brief A fiddle factor to estimate the number of SysTick counts that would
 * have occurred while the SysTick counter is stopped during tickless idle
 * calculations.
 */
#define portMISSED_COUNTS_FACTOR    ( 94UL )
/* ----------------------------------------------------------------------------------- */

/**
 * @brief Constants required to set up the initial stack.
 */
#define portINITIAL_XPSR    ( 0x01000000 )

/**
 * @brief Initial EXC_RETURN value.
 *
 *     FF         FF         FF         FD
 * 1111 1111  1111 1111  1111 1111  1111 1101
 *
 * Bit[3] - 1 --> Return to the Thread mode.
 * Bit[2] - 1 --> Restore registers from the process stack.
 * Bit[1] - 0 --> Reserved, 0.
 * Bit[0] - 0 --> Reserved, 1.
 */
    #define portINITIAL_EXC_RETURN    ( 0xfffffffdUL )

/**
 * @brief CONTROL register privileged bit mask.
 *
 * Bit[0] in CONTROL register tells the privilege:
 *  Bit[0] = 0 ==> The task is privileged.
 *  Bit[0] = 1 ==> The task is not privileged.
 */
#define portCONTROL_PRIVILEGED_MASK         ( 1UL << 0UL )

/**
 * @brief Initial CONTROL register values.
 */
#define portINITIAL_CONTROL_UNPRIVILEGED    ( 0x3 )
#define portINITIAL_CONTROL_PRIVILEGED      ( 0x2 )

/**
 * @brief Let the user override the default SysTick clock rate.  If defined by the
 * user, this symbol must equal the SysTick clock rate when the CLK bit is 0 in the
 * configuration register.
 */
#ifndef configSYSTICK_CLOCK_HZ
    #define configSYSTICK_CLOCK_HZ             ( configCPU_CLOCK_HZ )
    /* Ensure the SysTick is clocked at the same frequency as the core. */
    #define portNVIC_SYSTICK_CLK_BIT_CONFIG    ( portNVIC_SYSTICK_CLK_BIT )
#else
    /* Select the option to clock SysTick not at the same frequency as the core. */
    #define portNVIC_SYSTICK_CLK_BIT_CONFIG    ( 0 )
#endif

/**
 * @brief Let the user override the pre-loading of the initial LR with the
 * address of prvTaskExitError() in case it messes up unwinding of the stack
 * in the debugger.
 */
#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS    configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS    prvTaskExitError
#endif

/**
 * @brief If portPRELOAD_REGISTERS then registers will be given an initial value
 * when a task is created. This helps in debugging at the cost of code size.
 */
#define portPRELOAD_REGISTERS    1

/**
 * @brief A task is created without a secure context, and must call
 * portALLOCATE_SECURE_CONTEXT() to give itself a secure context before it makes
 * any secure calls.
 */
#define portNO_SECURE_CONTEXT    0
/* ----------------------------------------------------------------------------------- */

/**
 * @brief Used to catch tasks that attempt to return from their implementing
 * function.
 */
static void prvTaskExitError( void );

/**
 * @brief Extract MPU region's access permissions from the Region Attribute and Size
 * Register (RASR) value.
 *
 * @param ulRASRValue RASR value for the MPU region.
 *
 * @return uint32_t Access permissions.
 */
    static uint32_t prvGetRegionAccessPermissions( uint32_t ulRASRValue ) PRIVILEGED_FUNCTION;

/**
 * @brief Setup the Memory Protection Unit (MPU).
 */
    static void prvSetupMPU( void ) PRIVILEGED_FUNCTION;

/**
 * @brief Setup the timer to generate the tick interrupts.
 *
 * The implementation in this file is weak to allow application writers to
 * change the timer used to generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void ) PRIVILEGED_FUNCTION;

/**
 * @brief Checks whether the current execution context is interrupt.
 *
 * @return pdTRUE if the current execution context is interrupt, pdFALSE
 * otherwise.
 */
BaseType_t xPortIsInsideInterrupt( void );

/**
 * @brief Yield the processor.
 */
void vPortYield( void ) PRIVILEGED_FUNCTION;

/**
 * @brief Enter critical section.
 */
void vPortEnterCritical( void ) PRIVILEGED_FUNCTION;

/**
 * @brief Exit from critical section.
 */
void vPortExitCritical( void ) PRIVILEGED_FUNCTION;

/**
 * @brief SysTick handler.
 */
void SysTick_Handler( void ) PRIVILEGED_FUNCTION;

/**
 * @brief C part of SVC handler.
 */
portDONT_DISCARD void vPortSVCHandler_C( uint32_t * pulCallerStackAddress ) PRIVILEGED_FUNCTION;

/**
 * @brief Sets up the system call stack so that upon returning from
 * SVC, the system call stack is used.
 *
 * @param pulTaskStack The current SP when the SVC was raised.
 * @param ulLR The value of Link Register (EXC_RETURN) in the SVC handler.
 * @param ucSystemCallNumber The system call number of the system call.
 */
void vSystemCallEnter( uint32_t * pulTaskStack,
                        uint32_t ulLR,
                        uint8_t ucSystemCallNumber ) PRIVILEGED_FUNCTION;

/**
 * @brief Raise SVC for exiting from a system call.
 */
void vRequestSystemCallExit( void ) __attribute__( ( naked ) ) PRIVILEGED_FUNCTION;

/**
 * @brief Sets up the task stack so that upon returning from
 * SVC, the task stack is used again.
 *
 * @param pulSystemCallStack The current SP when the SVC was raised.
 * @param ulLR The value of Link Register (EXC_RETURN) in the SVC handler.
 */
void vSystemCallExit( uint32_t * pulSystemCallStack,
                        uint32_t ulLR ) PRIVILEGED_FUNCTION;

/**
 * @brief Checks whether or not the calling task is privileged.
 *
 * @return pdTRUE if the calling task is privileged, pdFALSE otherwise.
 */
BaseType_t xPortIsTaskPrivileged( void ) PRIVILEGED_FUNCTION;

/* ----------------------------------------------------------------------------------- */

/**
 * @brief This variable is set to pdTRUE when the scheduler is started.
 */
    PRIVILEGED_DATA static BaseType_t xSchedulerRunning = pdFALSE;

/**
 * @brief Each task maintains its own interrupt status in the critical nesting
 * variable.
 */
PRIVILEGED_DATA static volatile uint32_t ulCriticalNesting = 0xaaaaaaaaUL;

/**
 * @brief Used by the portASSERT_IF_INTERRUPT_PRIORITY_INVALID() macro to ensure
 * FreeRTOS API functions are not called from interrupts that have been assigned
 * a priority above configMAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#if ( ( configASSERT_DEFINED == 1 ) && ( portHAS_ARMV8M_MAIN_EXTENSION == 1 ) )

    static uint8_t ucMaxSysCallPriority = 0;
    static uint32_t ulMaxPRIGROUPValue = 0;
    static const volatile uint8_t * const pcInterruptPriorityRegisters = ( const volatile uint8_t * ) portNVIC_IP_REGISTERS_OFFSET_16;

#endif /* #if ( ( configASSERT_DEFINED == 1 ) && ( portHAS_ARMV8M_MAIN_EXTENSION == 1 ) ) */

/* ----------------------------------------------------------------------------------- */

__attribute__( ( weak ) ) void vPortSetupTimerInterrupt( void ) /* PRIVILEGED_FUNCTION */
{
    /* Calculate the constants required to configure the tick interrupt. */
    /* Stop and reset SysTick.
     *
     * QEMU versions older than 7.0.0 contain a bug which causes an error if we
     * enable SysTick without first selecting a valid clock source. We trigger
     * the bug if we change clock sources from a clock with a zero clock period
     * to one with a nonzero clock period and enable Systick at the same time.
     * So we configure the CLKSOURCE bit here, prior to setting the ENABLE bit.
     * This workaround avoids the bug in QEMU versions older than 7.0.0. */
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT_CONFIG;
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

    /* Configure SysTick to interrupt at the requested rate. */
    portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
}
/* ----------------------------------------------------------------------------------- */

static void prvTaskExitError( void )
{
    volatile uint32_t ulDummy = 0UL;

    /* A function that implements a task must not exit or attempt to return to
     * its caller as there is nothing to return to. If a task wants to exit it
     * should instead call vTaskDelete( NULL ). Artificially force an assert()
     * to be triggered if configASSERT() is defined, then stop here so
     * application writers can catch the error. */
    configASSERT( ulCriticalNesting == ~0UL );
    portDISABLE_INTERRUPTS();

    while( ulDummy == 0 )
    {
        /* This file calls prvTaskExitError() after the scheduler has been
         * started to remove a compiler warning about the function being
         * defined but never called.  ulDummy is used purely to quieten other
         * warnings about code appearing after this function is called - making
         * ulDummy volatile makes the compiler think the function could return
         * and therefore not output an 'unreachable code' warning for code that
         * appears after it. */
    }
}
/* ----------------------------------------------------------------------------------- */

static uint32_t prvGetRegionAccessPermissions( uint32_t ulRASRValue ) /* PRIVILEGED_FUNCTION */
{
    uint32_t ulAccessPermissions = 0;

    if( ( ulRASRValue & portMPU_RASR_ACCESS_PERMISSIONS_MASK ) == portMPU_REGION_READ_ONLY )
    {
        ulAccessPermissions = tskMPU_READ_PERMISSION;
    }

    if( ( ulRASRValue & portMPU_RASR_ACCESS_PERMISSIONS_MASK ) == portMPU_REGION_READ_WRITE )
    {
        ulAccessPermissions = ( tskMPU_READ_PERMISSION | tskMPU_WRITE_PERMISSION );
    }

    return ulAccessPermissions;
}
/* ----------------------------------------------------------------------------------- */
  static uint32_t prvGetMPURegionSizeSetting( uint32_t ulActualSizeInBytes )
{
    uint32_t ulRegionSize, ulReturnValue = 7UL;

    /* 256 is the smallest region size, 31 is the largest valid value for
        * ulReturnValue. */
    for( ulRegionSize = 256UL; ulReturnValue < 31UL; ( ulRegionSize <<= 1UL ) )
    {
        if( ulActualSizeInBytes <= ulRegionSize )
        {
            break;
        }
        else
        {
            ulReturnValue++;
        }
    }

    /* Shift the code by one before returning so it can be written directly
    * into the the correct bit position of the attribute register. */
    return( ulReturnValue << 1UL );
}

static void prvSetupMPU( void ) /* PRIVILEGED_FUNCTION */
{
    #if defined( __ARMCC_VERSION )

        /* Declaration when these variable are defined in code instead of being
        * exported from linker scripts. */
        extern uint32_t * __privileged_functions_start__;
        extern uint32_t * __privileged_functions_end__;
        extern uint32_t * __FLASH_segment_start__;
        extern uint32_t * __FLASH_segment_end__;
        extern uint32_t * __privileged_sram_start__;
        extern uint32_t * __privileged_sram_end__;
    #else /* if defined( __ARMCC_VERSION ) */
        /* Declaration when these variable are exported from linker scripts. */
        extern uint32_t __privileged_functions_start__[];
        extern uint32_t __privileged_functions_end__[];
        extern uint32_t __FLASH_segment_start__[];
        extern uint32_t __FLASH_segment_end__[];
        extern uint32_t __privileged_sram_start__[];
        extern uint32_t __privileged_sram_end__[];
    #endif /* defined( __ARMCC_VERSION ) */

    /* The only permitted number of regions is 8. */
    configASSERT( configTOTAL_MPU_REGIONS == 8 );

    /* Ensure that the configTOTAL_MPU_REGIONS is configured correctly. */
    configASSERT( portMPU_TYPE_REG == portEXPECTED_MPU_TYPE_VALUE );

    /* Check that the MPU is present. */
    if( portMPU_TYPE_REG == portEXPECTED_MPU_TYPE_VALUE )
    {
        /* Setup privileged flash as Read Only so that privileged tasks can
         * read it but not modify. */
        portMPU_RBAR_REG = ( ( uint32_t ) __privileged_functions_start__ ) | /* Base address. */
                                        ( portMPU_REGION_VALID ) |
                                        ( portPRIVILEGED_FLASH_REGION );

        portMPU_RASR_REG = ( portMPU_REGION_PRIVILEGED_READ_ONLY ) |
                                    ( ( configTEX_S_C_B_FLASH & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
                                    ( prvGetMPURegionSizeSetting( ( uint32_t ) __privileged_functions_end__ - ( uint32_t ) __privileged_functions_start__ ) ) |
                                    ( portMPU_REGION_ENABLE );

        /* Setup unprivileged flash as Read Only by both privileged and
         * unprivileged tasks. All tasks can read it but no-one can modify. */
        portMPU_RBAR_REG = ( ( uint32_t ) __FLASH_segment_start__ ) | /* Base address. */
                                        ( portMPU_REGION_VALID ) |
                                        ( portUNPRIVILEGED_FLASH_REGION );

        portMPU_RASR_REG = ( portMPU_REGION_READ_ONLY ) |
                                    ( ( configTEX_S_C_B_FLASH & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
                                    ( prvGetMPURegionSizeSetting( ( uint32_t ) __FLASH_segment_end__ - ( uint32_t ) __FLASH_segment_start__ ) ) |
                                    ( portMPU_REGION_ENABLE );

        /* Setup RAM containing kernel data for privileged access only. */
        portMPU_RBAR_REG = ( ( uint32_t ) __privileged_sram_start__ ) | /* Base address. */
                                        ( portMPU_REGION_VALID ) |
                                        ( portPRIVILEGED_RAM_REGION );

        portMPU_RASR_REG = ( portMPU_REGION_PRIVILEGED_READ_WRITE ) |
                                    ( portMPU_REGION_EXECUTE_NEVER ) |
                                    ( ( configTEX_S_C_B_SRAM & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
                                    prvGetMPURegionSizeSetting( ( uint32_t ) __privileged_sram_end__ - ( uint32_t ) __privileged_sram_start__ ) |
                                    ( portMPU_REGION_ENABLE );

        /* Enable MPU with privileged background access i.e. unmapped
         * regions have privileged access. */
        portMPU_CTRL_REG |= ( portMPU_PRIV_BACKGROUND_ENABLE_BIT | portMPU_ENABLE_BIT );
    }
}
/* ----------------------------------------------------------------------------------- */

void vPortYield( void ) /* PRIVILEGED_FUNCTION */
{
    /* Set a PendSV to request a context switch. */
    portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

    /* Barriers are normally not required but do ensure the code is
     * completely within the specified behaviour for the architecture. */
    portCACHE_FLUSH();
}
/* ----------------------------------------------------------------------------------- */

void vPortEnterCritical( void ) /* PRIVILEGED_FUNCTION */
{
    portDISABLE_INTERRUPTS();
    ulCriticalNesting++;

    /* Barriers are normally not required but do ensure the code is
     * completely within the specified behaviour for the architecture. */
    portCACHE_FLUSH();
}
/* ----------------------------------------------------------------------------------- */

void vPortExitCritical( void ) /* PRIVILEGED_FUNCTION */
{
    configASSERT( ulCriticalNesting );
    ulCriticalNesting--;

    if( ulCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/* ----------------------------------------------------------------------------------- */

void SysTick_Handler( void ) /* PRIVILEGED_FUNCTION */
{
    uint32_t ulPreviousMask;

    ulPreviousMask = portSET_INTERRUPT_MASK_FROM_ISR();
    traceISR_ENTER();
    {
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            traceISR_EXIT_TO_SCHEDULER();
            /* Pend a context switch. */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
        else
        {
            traceISR_EXIT();
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulPreviousMask );
}
/* ----------------------------------------------------------------------------------- */

void vPortSVCHandler_C( uint32_t * pulCallerStackAddress ) /* PRIVILEGED_FUNCTION portDONT_DISCARD */
{
    uint32_t ulPC;

    uint8_t ucSVCNumber;

    /* Register are stored on the stack in the following order - R0, R1, R2, R3,
     * R12, LR, PC, xPSR. */
    ulPC = pulCallerStackAddress[ portOFFSET_TO_PC ];
    ucSVCNumber = ( ( uint8_t * ) ulPC )[ -2 ];

    switch( ucSVCNumber )
    {
        case portSVC_START_SCHEDULER:
            /* Setup the context of the first task so that the first task starts
             * executing. */
            vRestoreContextOfFirstTask();
            break;

        case portSVC_YIELD:
            vPortYield();
            break;

        default:
            /* Incorrect SVC call. */
            configASSERT( pdFALSE );
    }
}
/* ----------------------------------------------------------------------------------- */

void vSystemCallEnter( uint32_t * pulTaskStack,
                        uint32_t ulLR,
                        uint8_t ucSystemCallNumber ) /* PRIVILEGED_FUNCTION */
{
    extern TaskHandle_t pxCurrentTCB;
    extern UBaseType_t uxSystemCallImplementations[ NUM_SYSTEM_CALLS ];
    xMPU_SETTINGS * pxMpuSettings;
    uint32_t * pulSystemCallStack;
    uint32_t ulStackFrameSize, ulSystemCallLocation, i;

    #if defined( __ARMCC_VERSION )
        /* Declaration when these variable are defined in code instead of being
            * exported from linker scripts. */
        extern uint32_t * __syscalls_flash_start__;
        extern uint32_t * __syscalls_flash_end__;
    #else
        /* Declaration when these variable are exported from linker scripts. */
        extern uint32_t __syscalls_flash_start__[];
        extern uint32_t __syscalls_flash_end__[];
    #endif /* #if defined( __ARMCC_VERSION ) */

    ulSystemCallLocation = pulTaskStack[ portOFFSET_TO_PC ];
    pxMpuSettings = xTaskGetMPUSettings( pxCurrentTCB );

    /* Checks:
     * 1. SVC is raised from the system call section (i.e. application is
     *    not raising SVC directly).
     * 2. pxMpuSettings->xSystemCallStackInfo.pulTaskStack must be NULL as
     *    it is non-NULL only during the execution of a system call (i.e.
     *    between system call enter and exit).
     * 3. System call is not for a kernel API disabled by the configuration
     *    in FreeRTOSConfig.h.
     * 4. We do not need to check that ucSystemCallNumber is within range
     *    because the assembly SVC handler checks that before calling
     *    this function.
     */
    if( ( ulSystemCallLocation >= ( uint32_t ) __syscalls_flash_start__ ) &&
        ( ulSystemCallLocation <= ( uint32_t ) __syscalls_flash_end__ ) &&
        ( pxMpuSettings->xSystemCallStackInfo.pulTaskStack == NULL ) &&
        ( uxSystemCallImplementations[ ucSystemCallNumber ] != ( UBaseType_t ) 0 ) )
    {
        pulSystemCallStack = pxMpuSettings->xSystemCallStackInfo.pulSystemCallStack;

        ulStackFrameSize = 8;

        /* Make space on the system call stack for the stack frame. */
        pulSystemCallStack = pulSystemCallStack - ulStackFrameSize;

        /* Copy the stack frame. */
        for( i = 0; i < ulStackFrameSize; i++ )
        {
            pulSystemCallStack[ i ] = pulTaskStack[ i ];
        }

        /* Store the value of the Link Register before the SVC was raised.
         * It contains the address of the caller of the System Call entry
         * point (i.e. the caller of the MPU_<API>). We need to restore it
         * when we exit from the system call. */
        pxMpuSettings->xSystemCallStackInfo.ulLinkRegisterAtSystemCallEntry = pulTaskStack[ portOFFSET_TO_LR ];

        /* Use the pulSystemCallStack in thread mode. */
        portSET_PSP(pulSystemCallStack);

        /* Start executing the system call upon returning from this handler. */
        pulSystemCallStack[ portOFFSET_TO_PC ] = uxSystemCallImplementations[ ucSystemCallNumber ];

        /* Raise a request to exit from the system call upon finishing the
         * system call. */
        pulSystemCallStack[ portOFFSET_TO_LR ] = ( uint32_t ) vRequestSystemCallExit;

        /* Remember the location where we should copy the stack frame when we exit from
         * the system call. */
        pxMpuSettings->xSystemCallStackInfo.pulTaskStack = pulTaskStack + ulStackFrameSize;

        /* Record if the hardware used padding to force the stack pointer
         * to be double word aligned. */
        if( ( pulTaskStack[ portOFFSET_TO_PSR ] & portPSR_STACK_PADDING_MASK ) == portPSR_STACK_PADDING_MASK )
        {
            pxMpuSettings->ulTaskFlags |= portSTACK_FRAME_HAS_PADDING_FLAG;
        }
        else
        {
            pxMpuSettings->ulTaskFlags &= ( ~portSTACK_FRAME_HAS_PADDING_FLAG );
        }

        /* We ensure in pxPortInitialiseStack that the system call stack is
         * double word aligned and therefore, there is no need of padding.
         * Clear the bit[9] of stacked xPSR. */
        pulSystemCallStack[ portOFFSET_TO_PSR ] &= ( ~portPSR_STACK_PADDING_MASK );

        /* Raise the privilege for the duration of the system call. */
        vRaisePrivilege();
    }
}

/* ----------------------------------------------------------------------------------- */

void vRequestSystemCallExit( void ) /* __attribute__( ( naked ) ) PRIVILEGED_FUNCTION */
{
	portREQUEST_SYS_CALL_EXIT();
}

/* ----------------------------------------------------------------------------------- */

void vSystemCallExit( uint32_t * pulSystemCallStack,
                        uint32_t ulLR ) /* PRIVILEGED_FUNCTION */
{
    extern TaskHandle_t pxCurrentTCB;
    xMPU_SETTINGS * pxMpuSettings;
    uint32_t * pulTaskStack;
    uint32_t ulStackFrameSize, ulSystemCallLocation, i;

    #if defined( __ARMCC_VERSION )
        /* Declaration when these variable are defined in code instead of being
            * exported from linker scripts. */
        extern uint32_t * __privileged_functions_start__;
        extern uint32_t * __privileged_functions_end__;
    #else
        /* Declaration when these variable are exported from linker scripts. */
        extern uint32_t __privileged_functions_start__[];
        extern uint32_t __privileged_functions_end__[];
    #endif /* #if defined( __ARMCC_VERSION ) */

    ulSystemCallLocation = pulSystemCallStack[ portOFFSET_TO_PC ];
    pxMpuSettings = xTaskGetMPUSettings( pxCurrentTCB );

    /* Checks:
     * 1. SVC is raised from the privileged code (i.e. application is not
     *    raising SVC directly). This SVC is only raised from
     *    vRequestSystemCallExit which is in the privileged code section.
     * 2. pxMpuSettings->xSystemCallStackInfo.pulTaskStack must not be NULL -
     *    this means that we previously entered a system call and the
     *    application is not attempting to exit without entering a system
     *    call.
     */
    if( ( ulSystemCallLocation >= ( uint32_t ) __privileged_functions_start__ ) &&
        ( ulSystemCallLocation <= ( uint32_t ) __privileged_functions_end__ ) &&
        ( pxMpuSettings->xSystemCallStackInfo.pulTaskStack != NULL ) )
    {
        pulTaskStack = pxMpuSettings->xSystemCallStackInfo.pulTaskStack;

        ulStackFrameSize = 8;

        /* Make space on the task stack for the stack frame. */
        pulTaskStack = pulTaskStack - ulStackFrameSize;

        /* Copy the stack frame. */
        for( i = 0; i < ulStackFrameSize; i++ )
        {
            pulTaskStack[ i ] = pulSystemCallStack[ i ];
        }

        /* Use the pulTaskStack in thread mode. */
        portSET_PSP( pulTaskStack );

        /* Return to the caller of the System Call entry point (i.e. the
         * caller of the MPU_<API>). */
        pulTaskStack[ portOFFSET_TO_PC ] = pxMpuSettings->xSystemCallStackInfo.ulLinkRegisterAtSystemCallEntry;

        /* Ensure that LR has a valid value.*/
        pulTaskStack[ portOFFSET_TO_LR ] = pxMpuSettings->xSystemCallStackInfo.ulLinkRegisterAtSystemCallEntry;

        /* If the hardware used padding to force the stack pointer
         * to be double word aligned, set the stacked xPSR bit[9],
         * otherwise clear it. */
        if( ( pxMpuSettings->ulTaskFlags & portSTACK_FRAME_HAS_PADDING_FLAG ) == portSTACK_FRAME_HAS_PADDING_FLAG )
        {
            pulTaskStack[ portOFFSET_TO_PSR ] |= portPSR_STACK_PADDING_MASK;
        }
        else
        {
            pulTaskStack[ portOFFSET_TO_PSR ] &= ( ~portPSR_STACK_PADDING_MASK );
        }

        /* This is not NULL only for the duration of the system call. */
        pxMpuSettings->xSystemCallStackInfo.pulTaskStack = NULL;

        /* Drop the privilege before returning to the thread mode. */
        vResetPrivilege();
    }
}

/* ----------------------------------------------------------------------------------- */

BaseType_t xPortIsTaskPrivileged( void ) /* PRIVILEGED_FUNCTION */
{
    BaseType_t xTaskIsPrivileged = pdFALSE;
    const xMPU_SETTINGS * xTaskMpuSettings = xTaskGetMPUSettings( NULL ); /* Calling task's MPU settings. */

    if( ( xTaskMpuSettings->ulTaskFlags & portTASK_IS_PRIVILEGED_FLAG ) == portTASK_IS_PRIVILEGED_FLAG )
    {
        xTaskIsPrivileged = pdTRUE;
    }

    return xTaskIsPrivileged;
}

/* ----------------------------------------------------------------------------------- */

StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                        StackType_t * pxEndOfStack,
                                        TaskFunction_t pxCode,
                                        void * pvParameters,
                                        BaseType_t xRunPrivileged,
                                        xMPU_SETTINGS * xMPUSettings ) /* PRIVILEGED_FUNCTION */
{
    uint32_t ulIndex = 0;

    xMPUSettings->ulContext[ ulIndex ] = 0x04040404; /* r4. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x05050505; /* r5. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x06060606; /* r6. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x07070707; /* r7. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x08080808; /* r8. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x09090909; /* r9. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x10101010; /* r10. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x11111111; /* r11. */
    ulIndex++;

    xMPUSettings->ulContext[ ulIndex ] = ( uint32_t ) pvParameters;            /* r0. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x01010101;                           /* r1. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x02020202;                           /* r2. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x03030303;                           /* r3. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = 0x12121212;                           /* r12. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = ( uint32_t ) portTASK_RETURN_ADDRESS; /* LR. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = ( uint32_t ) pxCode;                  /* PC. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = portINITIAL_XPSR;                     /* xPSR. */
    ulIndex++;

    xMPUSettings->ulContext[ ulIndex ] = ( uint32_t ) ( pxTopOfStack - 8 ); /* PSP with the hardware saved stack. */
    ulIndex++;
    xMPUSettings->ulContext[ ulIndex ] = ( uint32_t ) pxEndOfStack;         /* PSPLIM. */
    ulIndex++;

    if( xRunPrivileged == pdTRUE )
    {
        xMPUSettings->ulTaskFlags |= portTASK_IS_PRIVILEGED_FLAG;
        xMPUSettings->ulContext[ ulIndex ] = ( uint32_t ) portINITIAL_CONTROL_PRIVILEGED; /* CONTROL. */
        ulIndex++;
    }
    else
    {
        xMPUSettings->ulTaskFlags &= ( ~portTASK_IS_PRIVILEGED_FLAG );
        xMPUSettings->ulContext[ ulIndex ] = ( uint32_t ) portINITIAL_CONTROL_UNPRIVILEGED; /* CONTROL. */
        ulIndex++;
    }

    xMPUSettings->ulContext[ ulIndex ] = portINITIAL_EXC_RETURN; /* LR (EXC_RETURN). */
    ulIndex++;

    /* Ensure that the system call stack is double word aligned. */
    xMPUSettings->xSystemCallStackInfo.pulSystemCallStack = &( xMPUSettings->xSystemCallStackInfo.ulSystemCallStackBuffer[ configSYSTEM_CALL_STACK_SIZE - 1 ] );
    xMPUSettings->xSystemCallStackInfo.pulSystemCallStack = ( uint32_t * ) ( ( uint32_t ) ( xMPUSettings->xSystemCallStackInfo.pulSystemCallStack ) &
                                                                                ( uint32_t ) ( ~( portBYTE_ALIGNMENT_MASK ) ) );

    xMPUSettings->xSystemCallStackInfo.pulSystemCallStackLimit = &( xMPUSettings->xSystemCallStackInfo.ulSystemCallStackBuffer[ 0 ] );
    xMPUSettings->xSystemCallStackInfo.pulSystemCallStackLimit = ( uint32_t * ) ( ( ( uint32_t ) ( xMPUSettings->xSystemCallStackInfo.pulSystemCallStackLimit ) +
                                                                                    ( uint32_t ) ( portBYTE_ALIGNMENT - 1 ) ) &
                                                                                    ( uint32_t ) ( ~( portBYTE_ALIGNMENT_MASK ) ) );

    /* This is not NULL only for the duration of a system call. */
    xMPUSettings->xSystemCallStackInfo.pulTaskStack = NULL;

    return &( xMPUSettings->ulContext[ ulIndex ] );
}

/* ----------------------------------------------------------------------------------- */

BaseType_t xPortStartScheduler( void ) /* PRIVILEGED_FUNCTION */
{
    /* An application can install FreeRTOS interrupt handlers in one of the
     * folllowing ways:
     * 1. Direct Routing - Install the functions SVC_Handler and PendSV_Handler
     *    for SVCall and PendSV interrupts respectively.
     * 2. Indirect Routing - Install separate handlers for SVCall and PendSV
     *    interrupts and route program control from those handlers to
     *    SVC_Handler and PendSV_Handler functions.
     *
     * Applications that use Indirect Routing must set
     * configCHECK_HANDLER_INSTALLATION to 0 in their FreeRTOSConfig.h. Direct
     * routing, which is validated here when configCHECK_HANDLER_INSTALLATION
     * is 1, should be preferred when possible. */
    #if ( configCHECK_HANDLER_INSTALLATION == 1 )
    {
        const portISR_t * const pxVectorTable = portSCB_VTOR_REG;

        /* Validate that the application has correctly installed the FreeRTOS
         * handlers for SVCall and PendSV interrupts. We do not check the
         * installation of the SysTick handler because the application may
         * choose to drive the RTOS tick using a timer other than the SysTick
         * timer by overriding the weak function vPortSetupTimerInterrupt().
         *
         * Assertion failures here indicate incorrect installation of the
         * FreeRTOS handlers. For help installing the FreeRTOS handlers, see
         * https://www.FreeRTOS.org/FAQHelp.html.
         *
         * Systems with a configurable address for the interrupt vector table
         * can also encounter assertion failures or even system faults here if
         * VTOR is not set correctly to point to the application's vector table. */
        configASSERT( pxVectorTable[ portVECTOR_INDEX_SVC ] == SVC_Handler );
        configASSERT( pxVectorTable[ portVECTOR_INDEX_PENDSV ] == PendSV_Handler );
    }
    #endif /* configCHECK_HANDLER_INSTALLATION */

    #if ( ( configASSERT_DEFINED == 1 ) && ( portHAS_ARMV8M_MAIN_EXTENSION == 1 ) )
    {
        volatile uint32_t ulImplementedPrioBits = 0;
        volatile uint8_t ucMaxPriorityValue;

        /* Determine the maximum priority from which ISR safe FreeRTOS API
         * functions can be called. ISR safe functions are those that end in
         * "FromISR". FreeRTOS maintains separate thread and ISR API functions to
         * ensure interrupt entry is as fast and simple as possible.
         *
         * First, determine the number of priority bits available. Write to all
         * possible bits in the priority setting for SVCall. */
        portNVIC_SHPR2_REG = 0xFF000000;

        /* Read the value back to see how many bits stuck. */
        ucMaxPriorityValue = ( uint8_t ) ( ( portNVIC_SHPR2_REG & 0xFF000000 ) >> 24 );

        /* Use the same mask on the maximum system call priority. */
        ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;

        /* Check that the maximum system call priority is nonzero after
         * accounting for the number of priority bits supported by the
         * hardware. A priority of 0 is invalid because setting the BASEPRI
         * register to 0 unmasks all interrupts, and interrupts with priority 0
         * cannot be masked using BASEPRI.
         * See https://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
        configASSERT( ucMaxSysCallPriority );

        /* Check that the bits not implemented in hardware are zero in
         * configMAX_SYSCALL_INTERRUPT_PRIORITY. */
        configASSERT( ( configMAX_SYSCALL_INTERRUPT_PRIORITY & ( uint8_t ) ( ~( uint32_t ) ucMaxPriorityValue ) ) == 0U );

        /* Calculate the maximum acceptable priority group value for the number
         * of bits read back. */
        while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
        {
            ulImplementedPrioBits++;
            ucMaxPriorityValue <<= ( uint8_t ) 0x01;
        }

        if( ulImplementedPrioBits == 8 )
        {
            /* When the hardware implements 8 priority bits, there is no way for
             * the software to configure PRIGROUP to not have sub-priorities. As
             * a result, the least significant bit is always used for sub-priority
             * and there are 128 preemption priorities and 2 sub-priorities.
             *
             * This may cause some confusion in some cases - for example, if
             * configMAX_SYSCALL_INTERRUPT_PRIORITY is set to 5, both 5 and 4
             * priority interrupts will be masked in Critical Sections as those
             * are at the same preemption priority. This may appear confusing as
             * 4 is higher (numerically lower) priority than
             * configMAX_SYSCALL_INTERRUPT_PRIORITY and therefore, should not
             * have been masked. Instead, if we set configMAX_SYSCALL_INTERRUPT_PRIORITY
             * to 4, this confusion does not happen and the behaviour remains the same.
             *
             * The following assert ensures that the sub-priority bit in the
             * configMAX_SYSCALL_INTERRUPT_PRIORITY is clear to avoid the above mentioned
             * confusion. */
            configASSERT( ( configMAX_SYSCALL_INTERRUPT_PRIORITY & 0x1U ) == 0U );
            ulMaxPRIGROUPValue = 0;
        }
        else
        {
            ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS - ulImplementedPrioBits;
        }

        /* Shift the priority group value back to its position within the AIRCR
         * register. */
        ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
        ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;
    }
    #endif /* #if ( ( configASSERT_DEFINED == 1 ) && ( portHAS_ARMV8M_MAIN_EXTENSION == 1 ) ) */

    /* Make PendSV and SysTick the lowest priority interrupts, and make SVCall
     * the highest priority. */
    portNVIC_SHPR3_REG |= portNVIC_PENDSV_PRI;
    portNVIC_SHPR3_REG |= portNVIC_SYSTICK_PRI;
    portNVIC_SHPR2_REG = 0;

    /* Setup the Memory Protection Unit (MPU). */
    prvSetupMPU();

    /* Start the timer that generates the tick ISR. Interrupts are disabled
     * here already. */
    vPortSetupTimerInterrupt();

    /* Initialize the critical nesting count ready for the first task. */
    ulCriticalNesting = 0;

    xSchedulerRunning = pdTRUE;

    /* Start the first task. */
    vStartFirstTask();

    /* Should never get here as the tasks will now be executing. Call the task
     * exit error function to prevent compiler warnings about a static function
     * not being called in the case that the application writer overrides this
     * functionality by defining configTASK_RETURN_ADDRESS. Call
     * vTaskSwitchContext() so link time optimization does not remove the
     * symbol. */
    vTaskSwitchContext();
    prvTaskExitError();

    /* Should not get here. */
    return 0;
}
/* ----------------------------------------------------------------------------------- */

void vPortEndScheduler( void ) /* PRIVILEGED_FUNCTION */
{
    /* Not implemented in ports where there is nothing to return to.
     * Artificially force an assert. */
    configASSERT( ulCriticalNesting == 1000UL );
}
/* ----------------------------------------------------------------------------------- */


void vPortStoreTaskMPUSettings( xMPU_SETTINGS * xMPUSettings,
                            const struct xMEMORY_REGION * const xRegions,
                            StackType_t * pxBottomOfStack,
                            configSTACK_DEPTH_TYPE uxStackDepth )
{
    #if defined( __ARMCC_VERSION )

        /* Declaration when these variable are defined in code instead of being
        * exported from linker scripts. */
        extern uint32_t * __SRAM_segment_start__;
        extern uint32_t * __SRAM_segment_end__;
        extern uint32_t * __privileged_sram_start__;
        extern uint32_t * __privileged_sram_end__;
    #else
        /* Declaration when these variable are exported from linker scripts. */
        extern uint32_t __SRAM_segment_start__[];
        extern uint32_t __SRAM_segment_end__[];
        extern uint32_t __privileged_sram_start__[];
        extern uint32_t __privileged_sram_end__[];
    #endif /* defined( __ARMCC_VERSION ) */

    int32_t lIndex;
    uint32_t ul;

    if( xRegions == NULL )
    {
        /* No MPU regions are specified so allow access to all RAM. */
        xMPUSettings->xRegionsSettings[ 0 ].ulRBAR =
            ( ( uint32_t ) __SRAM_segment_start__ ) | /* Base address. */
            ( portMPU_REGION_VALID ) |
            ( portSTACK_REGION );                     /* Region number. */

        xMPUSettings->xRegionsSettings[ 0 ].ulRASR =
            ( portMPU_REGION_READ_WRITE ) |
            ( portMPU_REGION_EXECUTE_NEVER ) |
            ( ( configTEX_S_C_B_SRAM & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
            ( prvGetMPURegionSizeSetting( ( uint32_t ) __SRAM_segment_end__ - ( uint32_t ) __SRAM_segment_start__ ) ) |
            ( portMPU_REGION_ENABLE );


        /* Invalidate user configurable regions. */
        for( ul = 1UL; ul <= portNUM_CONFIGURABLE_REGIONS; ul++ )
        {
            xMPUSettings->xRegionsSettings[ ul ].ulRBAR = ( ( ul - 1UL ) | portMPU_REGION_VALID );
            xMPUSettings->xRegionsSettings[ ul ].ulRASR = 0UL;
        }
    }
    else
    {
        /* This function is called automatically when the task is created - in
        * which case the stack region parameters will be valid.  At all other
        * times the stack parameters will not be valid and it is assumed that the
        * stack region has already been configured. */
        if( uxStackDepth > 0 )
        {
            /* Define the region that allows access to the stack. */
            xMPUSettings->xRegionsSettings[ 0 ].ulRBAR =
                ( ( uint32_t ) pxBottomOfStack ) |
                ( portMPU_REGION_VALID ) |
                ( portSTACK_REGION ); /* Region number. */

            xMPUSettings->xRegionsSettings[ 0 ].ulRASR =
                ( portMPU_REGION_READ_WRITE ) |
                ( portMPU_REGION_EXECUTE_NEVER ) |
                ( prvGetMPURegionSizeSetting( uxStackDepth * ( uint32_t ) sizeof( StackType_t ) ) ) |
                ( ( configTEX_S_C_B_SRAM & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
                ( portMPU_REGION_ENABLE );

        }

        lIndex = 0;

        for( ul = 1UL; ul <= portNUM_CONFIGURABLE_REGIONS; ul++ )
        {
            if( ( xRegions[ lIndex ] ).ulLengthInBytes > 0UL )
            {
                /* Translate the generic region definition contained in
                * xRegions into the CM4 specific MPU settings that are then
                * stored in xMPUSettings. */
                xMPUSettings->xRegionsSettings[ ul ].ulRBAR =
                    ( ( uint32_t ) xRegions[ lIndex ].pvBaseAddress ) |
                    ( portMPU_REGION_VALID ) |
                    ( ul - 1UL ); /* Region number. */

                xMPUSettings->xRegionsSettings[ ul ].ulRASR =
                    ( prvGetMPURegionSizeSetting( xRegions[ lIndex ].ulLengthInBytes ) ) |
                    ( xRegions[ lIndex ].ulParameters ) |
                    ( portMPU_REGION_ENABLE );
            }
            else
            {
                /* Invalidate the region. */
                xMPUSettings->xRegionsSettings[ ul ].ulRBAR = ( ( ul - 1UL ) | portMPU_REGION_VALID );
                xMPUSettings->xRegionsSettings[ ul ].ulRASR = 0UL;
            }

            lIndex++;
        }
    }
}

/* ----------------------------------------------------------------------------------- */

BaseType_t xPortIsAuthorizedToAccessBuffer( const void * pvBuffer,
                                            uint32_t ulBufferLength,
                                            uint32_t ulAccessRequested ) /* PRIVILEGED_FUNCTION */

{
    uint32_t i, ulBufferStartAddress, ulBufferEndAddress;
    uint32_t ulRegionStart, ulRegionLength, ulRegionEnd;
    BaseType_t xAccessGranted = pdFALSE;
    const xMPU_SETTINGS * xTaskMpuSettings = xTaskGetMPUSettings( NULL ); /* Calling task's MPU settings. */

    if( xSchedulerRunning == pdFALSE )
    {
        /* Grant access to all the kernel objects before the scheduler
        * is started. It is necessary because there is no task running
        * yet and therefore, we cannot use the permissions of any
        * task. */
        xAccessGranted = pdTRUE;
    }
    else if( ( xTaskMpuSettings->ulTaskFlags & portTASK_IS_PRIVILEGED_FLAG ) == portTASK_IS_PRIVILEGED_FLAG )
    {
        xAccessGranted = pdTRUE;
    }
    else
    {
        if( portADD_UINT32_WILL_OVERFLOW( ( ( uint32_t ) pvBuffer ), ( ulBufferLength - 1UL ) ) == pdFALSE )
        {
            ulBufferStartAddress = ( uint32_t ) pvBuffer;
            ulBufferEndAddress = ( ( ( uint32_t ) pvBuffer ) + ulBufferLength - 1UL );

            for( i = 0; i < portTOTAL_NUM_REGIONS; i++ )
            {
                /* Is the MPU region enabled? */
                if( ( xTaskMpuSettings->xRegionsSettings[ i ].ulRASR & portMPU_REGION_ENABLE ) == portMPU_REGION_ENABLE )
                {
                    ulRegionStart = portEXTRACT_FIRST_ADDRESS_FROM_RBAR( xTaskMpuSettings->xRegionsSettings[ i ].ulRBAR );
                    ulRegionLength = portEXTRACT_REGION_LENGTH_FROM_RASR( xTaskMpuSettings->xRegionsSettings[ i ].ulRASR );
                    ulRegionEnd = ulRegionStart + ulRegionLength;

                    if( portIS_ADDRESS_WITHIN_RANGE( ulBufferStartAddress,
                                                        ulRegionStart,
                                                        ulRegionEnd ) &&
                        portIS_ADDRESS_WITHIN_RANGE( ulBufferEndAddress,
                                                        ulRegionStart,
                                                        ulRegionEnd ) &&
                        portIS_AUTHORIZED( ulAccessRequested,
                                            prvGetRegionAccessPermissions( xTaskMpuSettings->xRegionsSettings[ i ].ulRASR ) ) )
                    {
                        xAccessGranted = pdTRUE;
                        break;
                    }
                }
            }
        }
    }

    return xAccessGranted;
}

/* ----------------------------------------------------------------------------------- */

BaseType_t xPortIsInsideInterrupt( void )
{
    uint32_t ulCurrentInterrupt;
    BaseType_t xReturn;

    /* Obtain the number of the currently executing interrupt. Interrupt Program
     * Status Register (IPSR) holds the exception number of the currently-executing
     * exception or zero for Thread mode.*/
    portGET_IPSR( ulCurrentInterrupt );

    if( ulCurrentInterrupt == 0 )
    {
        xReturn = pdFALSE;
    }
    else
    {
        xReturn = pdTRUE;
    }

    return xReturn;
}
/* ----------------------------------------------------------------------------------- */

#if ( ( configASSERT_DEFINED == 1 ) && ( portHAS_ARMV8M_MAIN_EXTENSION == 1 ) )

    void vPortValidateInterruptPriority( void )
    {
        uint32_t ulCurrentInterrupt;
        uint8_t ucCurrentPriority;

        /* Obtain the number of the currently executing interrupt. */
        __asm volatile ( "mrs %0, ipsr" : "=r" ( ulCurrentInterrupt )::"memory" );

        /* Is the interrupt number a user defined interrupt? */
        if( ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER )
        {
            /* Look up the interrupt's priority. */
            ucCurrentPriority = pcInterruptPriorityRegisters[ ulCurrentInterrupt ];

            /* The following assertion will fail if a service routine (ISR) for
             * an interrupt that has been assigned a priority above
             * configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
             * function.  ISR safe FreeRTOS API functions must *only* be called
             * from interrupts that have been assigned a priority at or below
             * configMAX_SYSCALL_INTERRUPT_PRIORITY.
             *
             * Numerically low interrupt priority numbers represent logically high
             * interrupt priorities, therefore the priority of the interrupt must
             * be set to a value equal to or numerically *higher* than
             * configMAX_SYSCALL_INTERRUPT_PRIORITY.
             *
             * Interrupts that  use the FreeRTOS API must not be left at their
             * default priority of  zero as that is the highest possible priority,
             * which is guaranteed to be above configMAX_SYSCALL_INTERRUPT_PRIORITY,
             * and  therefore also guaranteed to be invalid.
             *
             * FreeRTOS maintains separate thread and ISR API functions to ensure
             * interrupt entry is as fast and simple as possible.
             *
             * The following links provide detailed information:
             * https://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html
             * https://www.FreeRTOS.org/FAQHelp.html */
            configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
        }

        /* Priority grouping:  The interrupt controller (NVIC) allows the bits
         * that define each interrupt's priority to be split between bits that
         * define the interrupt's pre-emption priority bits and bits that define
         * the interrupt's sub-priority.  For simplicity all bits must be defined
         * to be pre-emption priority bits.  The following assertion will fail if
         * this is not the case (if some bits represent a sub-priority).
         *
         * If the application only uses CMSIS libraries for interrupt
         * configuration then the correct setting can be achieved on all Cortex-M
         * devices by calling NVIC_SetPriorityGrouping( 0 ); before starting the
         * scheduler.  Note however that some vendor specific peripheral libraries
         * assume a non-zero priority group setting, in which cases using a value
         * of zero will result in unpredictable behaviour. */
        configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );
    }

#endif /* #if ( ( configASSERT_DEFINED == 1 ) && ( portHAS_ARMV8M_MAIN_EXTENSION == 1 ) ) */
/* ----------------------------------------------------------------------------------- */

    void vPortGrantAccessToKernelObject( TaskHandle_t xInternalTaskHandle,
                                         int32_t lInternalIndexOfKernelObject ) /* PRIVILEGED_FUNCTION */
    {
        uint32_t ulAccessControlListEntryIndex, ulAccessControlListEntryBit;
        xMPU_SETTINGS * xTaskMpuSettings;

        ulAccessControlListEntryIndex = ( ( uint32_t ) lInternalIndexOfKernelObject / portACL_ENTRY_SIZE_BITS );
        ulAccessControlListEntryBit = ( ( uint32_t ) lInternalIndexOfKernelObject % portACL_ENTRY_SIZE_BITS );

        xTaskMpuSettings = xTaskGetMPUSettings( xInternalTaskHandle );

        xTaskMpuSettings->ulAccessControlList[ ulAccessControlListEntryIndex ] |= ( 1U << ulAccessControlListEntryBit );
    }

/* ----------------------------------------------------------------------------------- */

    void vPortRevokeAccessToKernelObject( TaskHandle_t xInternalTaskHandle,
                                          int32_t lInternalIndexOfKernelObject ) /* PRIVILEGED_FUNCTION */
    {
        uint32_t ulAccessControlListEntryIndex, ulAccessControlListEntryBit;
        xMPU_SETTINGS * xTaskMpuSettings;

        ulAccessControlListEntryIndex = ( ( uint32_t ) lInternalIndexOfKernelObject / portACL_ENTRY_SIZE_BITS );
        ulAccessControlListEntryBit = ( ( uint32_t ) lInternalIndexOfKernelObject % portACL_ENTRY_SIZE_BITS );

        xTaskMpuSettings = xTaskGetMPUSettings( xInternalTaskHandle );

        xTaskMpuSettings->ulAccessControlList[ ulAccessControlListEntryIndex ] &= ~( 1U << ulAccessControlListEntryBit );
    }

/* ----------------------------------------------------------------------------------- */

BaseType_t xPortIsAuthorizedToAccessKernelObject( int32_t lInternalIndexOfKernelObject ) /* PRIVILEGED_FUNCTION */
{
    uint32_t ulAccessControlListEntryIndex, ulAccessControlListEntryBit;
    BaseType_t xAccessGranted = pdFALSE;
    const xMPU_SETTINGS * xTaskMpuSettings;

    if( xSchedulerRunning == pdFALSE )
    {
        /* Grant access to all the kernel objects before the scheduler
         * is started. It is necessary because there is no task running
         * yet and therefore, we cannot use the permissions of any
         * task. */
        xAccessGranted = pdTRUE;
    }
    else
    {
        xTaskMpuSettings = xTaskGetMPUSettings( NULL ); /* Calling task's MPU settings. */

        ulAccessControlListEntryIndex = ( ( uint32_t ) lInternalIndexOfKernelObject / portACL_ENTRY_SIZE_BITS );
        ulAccessControlListEntryBit = ( ( uint32_t ) lInternalIndexOfKernelObject % portACL_ENTRY_SIZE_BITS );

        if( ( xTaskMpuSettings->ulTaskFlags & portTASK_IS_PRIVILEGED_FLAG ) == portTASK_IS_PRIVILEGED_FLAG )
        {
            xAccessGranted = pdTRUE;
        }
        else
        {
            if( ( xTaskMpuSettings->ulAccessControlList[ ulAccessControlListEntryIndex ] & ( 1U << ulAccessControlListEntryBit ) ) != 0 )
            {
                xAccessGranted = pdTRUE;
            }
        }
    }

    return xAccessGranted;
}

/* ----------------------------------------------------------------------------------- */
