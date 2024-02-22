/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

/*-----------------------------------------------------------
* Implementation of functions defined in portable.h for the ARM CM4 MPU port.
*----------------------------------------------------------*/

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers.  That should only be done when
 * task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "mpu_syscall_numbers.h"
#include "mpu_wrappers_v2_asm.h"

#ifndef __VFP_FP__
    //#error This port can only be used when the project options are configured to enable hardware floating point support.
#endif

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#ifndef configSYSTICK_CLOCK_HZ
    #define configSYSTICK_CLOCK_HZ    configCPU_CLOCK_HZ
    /* Ensure the SysTick is clocked at the same frequency as the core. */
    #define portNVIC_SYSTICK_CLK      ( 1UL << 2UL )
#else

/* The way the SysTick is clocked is not modified in case it is not the same
 * as the core. */
    #define portNVIC_SYSTICK_CLK    ( 0 )
#endif

#if ( configALLOW_UNPRIVILEGED_CRITICAL_SECTIONS == 1)
    #error "configALLOW_UNPRIVILEGED_CRITICAL_SECTIONS is not allowed"
#endif



#ifndef configTOTAL_MPU_REGIONS
	#define configTOTAL_MPU_REGIONS 8U
#elif ( configTOTAL_MPU_REGIONS != 8U )
	#error "The Cortex-M0+ Only supports 8 MPU regions"
#endif /* configTOTAL_MPU_REGIONS */
/* Constants used to detect Cortex-M7 r0p0 and r0p1 cores, and ensure
 * that a work around is active for errata 837070. */
#define portCPUID                                 ( *( ( volatile uint32_t * ) 0xE000ED00 ) )

/* Constants required to access and manipulate the MPU. */
#define portMPU_TYPE_REG                          ( *( ( volatile uint32_t * ) 0xE000ED90 ) )
#define portMPU_REGION_BASE_ADDRESS_REG           ( *( ( volatile uint32_t * ) 0xE000ED9C ) )
#define portMPU_REGION_ATTRIBUTE_REG              ( *( ( volatile uint32_t * ) 0xE000EDA0 ) )
#define portMPU_CTRL_REG                          ( *( ( volatile uint32_t * ) 0xE000ED94 ) )
#define portEXPECTED_MPU_TYPE_VALUE               ( configTOTAL_MPU_REGIONS << 8UL )
#define portMPU_ENABLE                            ( 0x01UL )
#define portMPU_BACKGROUND_ENABLE                 ( 1UL << 2UL )
#define portPRIVILEGED_EXECUTION_START_ADDRESS    ( 0UL )
#define portMPU_REGION_VALID                      ( 0x10UL )
#define portMPU_REGION_ENABLE                     ( 0x01UL )
#define portPERIPHERALS_START_ADDRESS             0x40000000UL
#define portPERIPHERALS_END_ADDRESS               0x5FFFFFFFUL

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR                          ( 0x01000000UL )
#define portINITIAL_EXC_RETURN                    ( 0xfffffffdUL )
#define portINITIAL_CONTROL_UNPRIVILEGED       ( 0x03 )
#define portINITIAL_CONTROL_PRIVILEGED         ( 0x02 )

/* Constants required to check the validity of an interrupt priority. */
#define portFIRST_USER_INTERRUPT_NUMBER           ( 16 )
#define portNVIC_IP_REGISTERS_OFFSET_16           ( 0xE000E3F0 )
#define portAIRCR_REG                             ( *( ( volatile uint32_t * ) 0xE000ED0C ) )
#define portMAX_8_BIT_VALUE                       ( ( uint8_t ) 0xff )
#define portTOP_BIT_OF_BYTE                       ( ( uint8_t ) 0x80 )
#define portMAX_PRIGROUP_BITS                     ( ( uint8_t ) 7 )
#define portPRIORITY_GROUP_MASK                   ( 0x07UL << 8UL )
#define portPRIGROUP_SHIFT                        ( 8UL )

/* Constants used during system call enter and exit. */
#define portPSR_STACK_PADDING_MASK                ( 1UL << 9UL )
#define portEXC_RETURN_STACK_FRAME_TYPE_MASK      ( 1UL << 4UL )

/* Offsets in the stack to the parameters when inside the SVC handler. */
#define portOFFSET_TO_LR                          ( 5 )
#define portOFFSET_TO_PC                          ( 6 )
#define portOFFSET_TO_PSR                         ( 7 )


/* For strict compliance with the Cortex-M spec the task start address should
 * have bit-0 clear, as it is loaded into the PC on exit from an ISR. */
#define portSTART_ADDRESS_MASK    ( ( StackType_t ) 0xfffffffeUL )

/* Does addr lie within [start, end] address range? */
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

/*
 * Configure a number of standard MPU regions that are used by all tasks.
 */
static void prvSetupMPU( void ) PRIVILEGED_FUNCTION;

/*
 * Return the smallest MPU region size that a given number of bytes will fit
 * into.  The region size is returned as the value that should be programmed
 * into the region attribute register for that region.
 */
static uint32_t prvGetMPURegionSizeSetting( uint32_t ulActualSizeInBytes ) PRIVILEGED_FUNCTION;

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void ) PRIVILEGED_FUNCTION;

void SysTick_Handler( void ) PRIVILEGED_FUNCTION;

/*
 * C portion of the SVC handler.  The SVC handler is split between an asm entry
 * and a C wrapper for simplicity of coding and maintenance.
 */
void vSVCHandler_C( uint32_t * pulRegisters ) __attribute__( ( noinline ) ) PRIVILEGED_FUNCTION;

/**
 * @brief Checks whether or not the processor is privileged.
 *
 * @return 1 if the processor is already privileged, 0 otherwise.
 */
BaseType_t xIsPrivileged( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

/**
 * @brief Lowers the privilege level by setting the bit 0 of the CONTROL
 * register.
 *
 * Bit 0 of the CONTROL register defines the privilege level of Thread Mode.
 *  Bit[0] = 0 --> The processor is running privileged
 *  Bit[0] = 1 --> The processor is running unprivileged.
 */
void vResetPrivilege( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;


/**
 * @brief Make a task unprivileged.
 */
void vPortSwitchToUserMode( void ) FREERTOS_SYSTEM_CALL;

/**
 * @brief Enter critical section.
 */
void vPortEnterCritical( void ) PRIVILEGED_FUNCTION;

/**
 * @brief Exit from critical section.
 */
void vPortExitCritical( void ) PRIVILEGED_FUNCTION;

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

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

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


//extern void MPU_vTaskDelayImpl( TickType_t xTicksToDelay ) PRIVILEGED_FUNCTION;
/* ----------------------------------------------------------------------------------- */

/* Each task maintains its own interrupt status in the critical nesting
 * variable.  Note this is not saved as part of the task context as context
 * switches can only occur when uxCriticalNesting is zero. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*
 * This variable is set to pdTRUE when the scheduler is started.
 */
    PRIVILEGED_DATA static BaseType_t xSchedulerRunning = pdFALSE;

/*
 * Used by the portASSERT_IF_INTERRUPT_PRIORITY_INVALID() macro to ensure
 * FreeRTOS API functions are not called from interrupts that have been assigned
 * a priority above configMAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#if ( configASSERT_DEFINED == 1 )
    static uint8_t ucMaxSysCallPriority = 0;
    static uint32_t ulMaxPRIGROUPValue = 0;
    static const volatile uint8_t * const pcInterruptPriorityRegisters = ( const volatile uint8_t * const ) portNVIC_IP_REGISTERS_OFFSET_16;
#endif /* configASSERT_DEFINED */

/* ----------------------------------------------------------------------------------- */

/*
 * See header file for description.
 */
StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                     TaskFunction_t pxCode,
                                     void * pvParameters,
                                     BaseType_t xRunPrivileged,
                                     xMPU_SETTINGS * xMPUSettings )
{
    uint32_t ulIndex = 0UL;   
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
    
    if( xRunPrivileged == pdTRUE )
    {
        xMPUSettings->ulTaskFlags |= portTASK_IS_PRIVILEGED_FLAG;
        xMPUSettings->ulContext[ ulIndex ] = ( uint32_t ) portINITIAL_CONTROL_PRIVILEGED; /* CONTROL. */
    }
    else
    {
        xMPUSettings->ulTaskFlags &= ( ~portTASK_IS_PRIVILEGED_FLAG );
        xMPUSettings->ulContext[ ulIndex ] = ( uint32_t ) portINITIAL_CONTROL_UNPRIVILEGED; /* CONTROL. */

    }
    ulIndex++;

    xMPUSettings->ulContext[ ulIndex ] = portINITIAL_EXC_RETURN; /* LR (EXC_RETURN). */
    ulIndex++;

#if 0

    xMPUSettings->ulContext[ 5 ] = portINITIAL_EXC_RETURN;              /* EXC_RETURN. */
    xMPUSettings->ulContext[ 6 ] = ( uint32_t ) ( pxTopOfStack - 8 );  /* PSP with the hardware saved stack. */
	xMPUSettings->ulContext[ 7 ] = 0x04040404;  /* r4. */
    xMPUSettings->ulContext[ 8 ] = 0x05050505;  /* r5. */
    xMPUSettings->ulContext[ 9 ] = 0x06060606;  /* r6. */
    xMPUSettings->ulContext[ 10 ] = 0x07070707; /* r7. */

    /* Auto-Pushed Registers */
    xMPUSettings->ulContext[ 11 ] = ( uint32_t ) pvParameters;                        /* r0. */
    xMPUSettings->ulContext[ 12 ] = 0x01010101;                                       /* r1. */
    xMPUSettings->ulContext[ 13 ] = 0x02020202;                                       /* r2. */
    xMPUSettings->ulContext[ 14 ] = 0x03030303;                                       /* r3. */
    xMPUSettings->ulContext[ 15 ] = 0x12121212;                                       /* r12. */
    xMPUSettings->ulContext[ 16 ] = ( uint32_t ) prvTaskExitError;                    /* LR. */
    xMPUSettings->ulContext[ 17 ] = ( ( uint32_t ) pxCode ) & portSTART_ADDRESS_MASK; /* PC. */
    xMPUSettings->ulContext[ 18 ] = portINITIAL_XPSR;                                 /* xPSR. */

#endif

    /* Ensure that the system call stack is double word aligned. */
    xMPUSettings->xSystemCallStackInfo.pulSystemCallStack = &( xMPUSettings->xSystemCallStackInfo.ulSystemCallStackBuffer[ configSYSTEM_CALL_STACK_SIZE - 1 ] );
    xMPUSettings->xSystemCallStackInfo.pulSystemCallStack = ( uint32_t * ) ( ( uint32_t ) ( xMPUSettings->xSystemCallStackInfo.pulSystemCallStack ) &
                                                                                ( uint32_t ) ( ~( portBYTE_ALIGNMENT_MASK ) ) );

    /* This is not NULL only for the duration of a system call. */
    xMPUSettings->xSystemCallStackInfo.pulTaskStack = NULL;

    return &( xMPUSettings->ulContext[ ulIndex] );
}

/* ----------------------------------------------------------------------------------- */

void vSVCHandler_C( uint32_t * pulParam ) /* PRIVILEGED_FUNCTION */
{
    uint8_t ucSVCNumber;
    uint32_t ulPC;

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

    /* The stack contains: r0, r1, r2, r3, r12, LR, PC and xPSR.  The first
     * argument (r0) is pulParam[ 0 ]. */
    ulPC = pulParam[ portOFFSET_TO_PC ];
    ucSVCNumber = ( ( uint8_t * ) ulPC )[ -2 ];

    switch( ucSVCNumber )
    {

        case portSVC_YIELD:
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

            /* Barriers are normally not required
             * but do ensure the code is completely
             * within the specified behaviour for the
             * architecture. */
            __asm volatile ( "dsb" ::: "memory" );
            __asm volatile ( "isb" );

            break;

        default: /* Unknown SVC call. */
            break;
    }
}
/* ----------------------------------------------------------------------------------- */

void vSystemCallEnter(  uint32_t * pulTaskStack,
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
    //configASSERT(uxSystemCallImplementations[ SYSTEM_CALL_vTaskDelay ] == MPU_vTaskDelayImpl );

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

        /* Standard frame i.e. FPU not in use. */
        ulStackFrameSize = 8UL;

        /* Make space on the system call stack for the stack frame. */
        pulSystemCallStack = pulSystemCallStack - ulStackFrameSize;

        /* Copy the stack frame. */
        for( i = 0; i < ulStackFrameSize; i++ )
        {
            pulSystemCallStack[ i ] = pulTaskStack[ i ];
        }

        /* Use the pulSystemCallStack in thread mode. */
        __asm volatile ( "msr psp, %0" : : "r" ( pulSystemCallStack ) );

        /* Raise the privilege for the duration of the system call. */
        __asm volatile (
            " .syntax unified       \n"
        	" PUSH	{ R0-R1 }		\n" /* Save registers before use */
            " movs  r0, #1          \n"
            " mrs   r1, control     \n" /* Obtain current control value. */
            " bics  r1, r0          \n" /* Clear nPRIV bit. */
            " msr   control, r1     \n" /* Write back new control value. */
			" POP	{ R0-R1 }		\n" /* Restore registers after use */
            ::: "r1", "memory"
            );

        /* Remember the location where we should copy the stack frame when we exit from
            * the system call. */
        pxMpuSettings->xSystemCallStackInfo.pulTaskStack = pulTaskStack + ulStackFrameSize;

        /* Store the value of the Link Register before the SVC was raised.
            * It contains the address of the caller of the System Call entry
            * point (i.e. the caller of the MPU_<API>). We need to restore it
            * when we exit from the system call. */
        pxMpuSettings->xSystemCallStackInfo.ulLinkRegisterAtSystemCallEntry = pulTaskStack[ portOFFSET_TO_LR ];

        /* Start executing the system call upon returning from this handler. */
        pulSystemCallStack[ portOFFSET_TO_PC ] = uxSystemCallImplementations[ ucSystemCallNumber ];
        /* Raise a request to exit from the system call upon finishing the
            * system call. */
        pulSystemCallStack[ portOFFSET_TO_LR ] = ( uint32_t ) vRequestSystemCallExit;

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
    }
}

/* ----------------------------------------------------------------------------------- */

void vRequestSystemCallExit( void ) /* __attribute__( ( naked ) ) PRIVILEGED_FUNCTION */
{
    __asm volatile ( "svc %0 \n" ::"i" ( portSVC_SYSTEM_CALL_EXIT ) : "memory" );
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

        /* Standard frame i.e. FPU not in use. */
        ulStackFrameSize = 8UL;

        /* Make space on the task stack for the stack frame. */
        pulTaskStack = pulTaskStack - ulStackFrameSize;

        /* Copy the stack frame. */
        for( i = 0; i < ulStackFrameSize; i++ )
        {
            pulTaskStack[ i ] = pulSystemCallStack[ i ];
        }

        /* Use the pulTaskStack in thread mode. */
        __asm volatile ( "msr psp, %0" : : "r" ( pulTaskStack ) );

        /* Drop the privilege before returning to the thread mode. */
        __asm volatile (
            " .syntax unified       \n"
            " movs   r0, #1         \n"
            " mrs   r1, control     \n" /* Obtain current control value. */
            " orrs  r1, r0          \n" /* Set nPRIV bit. */
            " msr   control, r1     \n" /* Write back new control value. */
            ::: "r1", "memory"
            );

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
    }
}

/* ----------------------------------------------------------------------------------- */

BaseType_t xPortIsTaskPrivileged( void ) /* PRIVILEGED_FUNCTION */
{
    BaseType_t xTaskIsPrivileged = pdFALSE;
    /* Calling task's MPU settings. */
    const xMPU_SETTINGS * xTaskMpuSettings = xTaskGetMPUSettings( NULL );

    if( ( xTaskMpuSettings->ulTaskFlags & portTASK_IS_PRIVILEGED_FLAG ) == portTASK_IS_PRIVILEGED_FLAG )
    {
        xTaskIsPrivileged = pdTRUE;
    }

    return xTaskIsPrivileged;
}

/* ----------------------------------------------------------------------------------- */

/*
 * See header file for description.
 */
BaseType_t xPortStartScheduler( void )
{
    /* An application can install FreeRTOS interrupt handlers in one of the
     * folllowing ways:
     * 1. Direct Routing - Install the function xPortPendSVHandler for PendSV
     *    interrupt.
     * 2. Indirect Routing - Install separate handler for PendSV interrupt and
     *    route program control from that handler to xPortPendSVHandler function.
     *
     * Applications that use Indirect Routing must set
     * configCHECK_HANDLER_INSTALLATION to 0 in their FreeRTOSConfig.h. Direct
     * routing, which is validated here when configCHECK_HANDLER_INSTALLATION
     * is 1, should be preferred when possible. */
    #if ( configCHECK_HANDLER_INSTALLATION == 1 )
    {
        /* Point pxVectorTable to the interrupt vector table. Systems without
         * a VTOR register provide the value zero in the VTOR register and
         * the vector table itself is located at the address 0x00000000. */
        const portISR_t * const pxVectorTable = portSCB_VTOR_REG;

        /* Validate that the application has correctly installed the FreeRTOS
         * handler for PendSV interrupt. We do not check the installation of the
         * SysTick handler because the application may choose to drive the RTOS
         * tick using a timer other than the SysTick timer by overriding the
         * weak function vPortSetupTimerInterrupt().
         *
         * Assertion failures here indicate incorrect installation of the
         * FreeRTOS handler. For help installing the FreeRTOS handler, see
         * https://www.FreeRTOS.org/FAQHelp.html.
         *
         * Systems with a configurable address for the interrupt vector table
         * can also encounter assertion failures or even system faults here if
         * VTOR is not set correctly to point to the application's vector table. */
        configASSERT( pxVectorTable[ portVECTOR_INDEX_PENDSV ] == PendSV_Handler );
        configASSERT( pxVectorTable[ portVECTOR_INDEX_PENDSV - 3U ] == SVC_Handler );
    }
    #endif /* configCHECK_HANDLER_INSTALLATION */

    /* Make PendSV and SysTick the lowest priority interrupts. */
    portNVIC_SHPR3_REG |= portNVIC_PENDSV_PRI;
    portNVIC_SHPR3_REG |= portNVIC_SYSTICK_PRI;

    /* Start the timer that generates the tick ISR.  Interrupts are disabled
     * here already. */
    vPortSetupTimerInterrupt();

    /* Initialise the critical nesting count ready for the first task. */
    uxCriticalNesting = 0;
    
    /* Setup default MPU regions */
    prvSetupMPU();

    /* Start the first task. */
    vPortStartFirstTask();

    /* Should never get here as the tasks will now be executing!  Call the task
     * exit error function to prevent compiler warnings about a static function
     * not being called in the case that the application writer overrides this
     * functionality by defining configTASK_RETURN_ADDRESS.  Call
     * vTaskSwitchContext() so link time optimisation does not remove the
     * symbol. */
    vTaskSwitchContext();

    /* Should not get here! */
    return 0;
}

/* ----------------------------------------------------------------------------------- */

void vPortEndScheduler( void )
{
    /* Not implemented in ports where there is nothing to return to.
     * Artificially force an assert. */
    configASSERT( uxCriticalNesting == 1000UL );
}
/* ----------------------------------------------------------------------------------- */

void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;
}
/* ----------------------------------------------------------------------------------- */

void vPortExitCritical( void )
{
    configASSERT( uxCriticalNesting );
    uxCriticalNesting--;

    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/* ----------------------------------------------------------------------------------- */

void SysTick_Handler( void )
{
    uint32_t ulCurrentInterruptMask;

    ulCurrentInterruptMask = portSET_INTERRUPT_MASK_FROM_ISR();
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
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulCurrentInterruptMask );
}
/* ----------------------------------------------------------------------------------- */

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
__attribute__( ( weak ) ) void vPortSetupTimerInterrupt( void )
{
    /* Calculate the constants required to configure the tick interrupt. */
    #if ( configUSE_TICKLESS_IDLE == 1 )
    {
        ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
        xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
        ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
    }
    #endif /* configUSE_TICKLESS_IDLE */

    /* Stop and reset the SysTick. */
    portNVIC_SYSTICK_CTRL_REG = 0UL;
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

    /* Configure SysTick to interrupt at the requested rate. */
    portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
    portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
}
/* ----------------------------------------------------------------------------------- */

static void prvSetupMPU( void )
{
    #if defined( __ARMCC_VERSION )

        /* Declaration when these variable are defined in code instead of being
         * exported from linker scripts. */
        extern uint32_t * __privileged_functions_start__;
        extern uint32_t * __privileged_functions_end__;
        extern uint32_t * __FLASH_segment_start__;
        extern uint32_t * __FLASH_segment_end__;
        extern uint32_t * __privileged_data_start__;
        extern uint32_t * __privileged_data_end__;
    #else
        /* Declaration when these variable are exported from linker scripts. */
        extern uint32_t __privileged_functions_start__[];
        extern uint32_t __privileged_functions_end__[];
        extern uint32_t __FLASH_segment_start__[];
        extern uint32_t __FLASH_segment_end__[];
        extern uint32_t __privileged_data_start__[];
        extern uint32_t __privileged_data_end__[];
    #endif /* if defined( __ARMCC_VERSION ) */

    /* Ensure that the configTOTAL_MPU_REGIONS is configured correctly. */
    configASSERT( portMPU_TYPE_REG == portEXPECTED_MPU_TYPE_VALUE );

    /* Check the expected MPU is present. */
    if( portMPU_TYPE_REG == portEXPECTED_MPU_TYPE_VALUE )
    {
        /* First setup the unprivileged flash for unprivileged read only access. */
        portMPU_REGION_BASE_ADDRESS_REG = ( ( uint32_t ) __FLASH_segment_start__ ) | /* Base address. */
                                          ( portMPU_REGION_VALID ) |
                                          ( portUNPRIVILEGED_FLASH_REGION );

        portMPU_REGION_ATTRIBUTE_REG = ( portMPU_REGION_READ_ONLY ) |
                                       ( ( configTEX_S_C_B_FLASH & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
                                       ( prvGetMPURegionSizeSetting( ( uint32_t ) __FLASH_segment_end__ - ( uint32_t ) __FLASH_segment_start__ ) ) |
                                       ( portMPU_REGION_ENABLE );

        /* Setup the privileged flash for privileged only access.  This is where
         * the kernel code is placed. */
        portMPU_REGION_BASE_ADDRESS_REG = ( ( uint32_t ) __privileged_functions_start__ ) | /* Base address. */
                                          ( portMPU_REGION_VALID ) |
                                          ( portPRIVILEGED_FLASH_REGION );

        portMPU_REGION_ATTRIBUTE_REG = ( portMPU_REGION_PRIVILEGED_READ_ONLY ) |
                                       ( ( configTEX_S_C_B_FLASH & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
                                       ( prvGetMPURegionSizeSetting( ( uint32_t ) __privileged_functions_end__ - ( uint32_t ) __privileged_functions_start__ ) ) |
                                       ( portMPU_REGION_ENABLE );

        /* Setup the privileged data RAM region.  This is where the kernel data
         * is placed. */
        portMPU_REGION_BASE_ADDRESS_REG = ( ( uint32_t ) __privileged_data_start__ ) | /* Base address. */
                                          ( portMPU_REGION_VALID ) |
                                          ( portPRIVILEGED_RAM_REGION );

        portMPU_REGION_ATTRIBUTE_REG = ( portMPU_REGION_PRIVILEGED_READ_WRITE ) |
                                       ( portMPU_REGION_EXECUTE_NEVER ) |
                                       ( ( configTEX_S_C_B_SRAM & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
                                       prvGetMPURegionSizeSetting( ( uint32_t ) __privileged_data_end__ - ( uint32_t ) __privileged_data_start__ ) |
                                       ( portMPU_REGION_ENABLE );

        /* Enable the memory fault exception. */
        portNVIC_SYS_CTRL_STATE_REG |= portNVIC_MEM_FAULT_ENABLE;

        /* Enable the MPU with the background region configured. */
        portMPU_CTRL_REG |= ( portMPU_ENABLE | portMPU_BACKGROUND_ENABLE );
    }
}
/* ----------------------------------------------------------------------------------- */

static uint32_t prvGetMPURegionSizeSetting( uint32_t ulActualSizeInBytes )
{
    uint32_t ulRegionSize, ulReturnValue = 4;

    /* 32 is the smallest region size, 31 is the largest valid value for
     * ulReturnValue. */
    for( ulRegionSize = 32UL; ulReturnValue < 31UL; ( ulRegionSize <<= 1UL ) )
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
/* ----------------------------------------------------------------------------------- */

BaseType_t xIsPrivileged( void ) /* __attribute__ (( naked )) */
{
    __asm volatile
    (
        " .syntax unified                           \n"
        " mrs   r0, control                         \n" /* r0 = CONTROL. */
        " movs  r1, #1                              \n"
        " ands  r0, r1                              \n" /* Perform r0 & 1 (bitwise AND) and update the conditions flag. */
        " bx lr                                     \n" /* Return. */
        "                                           \n"
        " .align 4                                  \n"
        ::: "r0", "memory"
    );
}
/* ----------------------------------------------------------------------------------- */

void vResetPrivilege( void ) /* __attribute__ (( naked )) */
{
    __asm volatile
    (
        " .syntax unified                           \n"
        " movs  r1, #1                              \n"
        " mrs   r0, control                         \n" /* r0 = CONTROL. */
        " orrs  r0, r1                              \n" /* r0 = r0 | 1. */
        " msr   control, r0                         \n" /* CONTROL = r0. */
        " bx    lr                                  \n" /* Return to the caller. */
        ::: "r0", "memory"
    );
}
/* ----------------------------------------------------------------------------------- */

void vPortSwitchToUserMode( void )
{
    /* Load the current task's MPU settings from its TCB. */
    xMPU_SETTINGS * xTaskMpuSettings = xTaskGetMPUSettings( NULL );

    /* Mark the task as unprivileged. */
    xTaskMpuSettings->ulTaskFlags &= ( ~( portTASK_IS_PRIVILEGED_FLAG ) );

    /* Lower the processor's privilege level. */
    vResetPrivilege();
}
/* ----------------------------------------------------------------------------------- */

void vPortStoreTaskMPUSettings( xMPU_SETTINGS * xMPUSettings,
                                const struct xMEMORY_REGION * const xRegions,
                                StackType_t * pxBottomOfStack,
                                uint32_t ulStackDepth )
{
    #if defined( __ARMCC_VERSION )

        /* Declaration when these variable are defined in code instead of being
         * exported from linker scripts. */
        extern uint32_t * __SRAM_segment_start__;
        extern uint32_t * __SRAM_segment_end__;
        extern uint32_t * __privileged_data_start__;
        extern uint32_t * __privileged_data_end__;
    #else
        /* Declaration when these variable are exported from linker scripts. */
        extern uint32_t __SRAM_segment_start__[];
        extern uint32_t __SRAM_segment_end__[];
        extern uint32_t __privileged_data_start__[];
        extern uint32_t __privileged_data_end__[];
    #endif /* if defined( __ARMCC_VERSION ) */

    int32_t lIndex;
    uint32_t ul;

    if( xRegions == NULL )
    {
        /* No MPU regions are specified so allow access to all RAM. */
        xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress =
            ( ( uint32_t ) __SRAM_segment_start__ ) | /* Base address. */
            ( portMPU_REGION_VALID ) |
            ( portSTACK_REGION );                     /* Region number. */

        xMPUSettings->xRegion[ 0 ].ulRegionAttribute =
            ( portMPU_REGION_READ_WRITE ) |
            ( portMPU_REGION_EXECUTE_NEVER ) |
            ( ( configTEX_S_C_B_SRAM & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
            ( prvGetMPURegionSizeSetting( ( uint32_t ) __SRAM_segment_end__ - ( uint32_t ) __SRAM_segment_start__ ) ) |
            ( portMPU_REGION_ENABLE );

        xMPUSettings->xRegionSettings[ 0 ].ulRegionStartAddress = ( uint32_t ) __SRAM_segment_start__;
        xMPUSettings->xRegionSettings[ 0 ].ulRegionEndAddress = ( uint32_t ) __SRAM_segment_end__;
        xMPUSettings->xRegionSettings[ 0 ].ulRegionPermissions = ( tskMPU_READ_PERMISSION |
                                                                   tskMPU_WRITE_PERMISSION );

        /* Invalidate user configurable regions. */
        for( ul = 1UL; ul <= portNUM_CONFIGURABLE_REGIONS; ul++ )
        {
            xMPUSettings->xRegion[ ul ].ulRegionBaseAddress = ( ( ul - 1UL ) | portMPU_REGION_VALID );
            xMPUSettings->xRegion[ ul ].ulRegionAttribute = 0UL;
            xMPUSettings->xRegionSettings[ ul ].ulRegionStartAddress = 0UL;
            xMPUSettings->xRegionSettings[ ul ].ulRegionEndAddress = 0UL;
            xMPUSettings->xRegionSettings[ ul ].ulRegionPermissions = 0UL;
        }
    }
    else
    {
        /* This function is called automatically when the task is created - in
         * which case the stack region parameters will be valid.  At all other
         * times the stack parameters will not be valid and it is assumed that the
         * stack region has already been configured. */
        if( ulStackDepth > 0 )
        {
            /* Define the region that allows access to the stack. */
            xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress =
                ( ( uint32_t ) pxBottomOfStack ) |
                ( portMPU_REGION_VALID ) |
                ( portSTACK_REGION ); /* Region number. */

            xMPUSettings->xRegion[ 0 ].ulRegionAttribute =
                ( portMPU_REGION_READ_WRITE ) |
                ( portMPU_REGION_EXECUTE_NEVER ) |
                ( prvGetMPURegionSizeSetting( ulStackDepth * ( uint32_t ) sizeof( StackType_t ) ) ) |
                ( ( configTEX_S_C_B_SRAM & portMPU_RASR_TEX_S_C_B_MASK ) << portMPU_RASR_TEX_S_C_B_LOCATION ) |
                ( portMPU_REGION_ENABLE );

            xMPUSettings->xRegionSettings[ 0 ].ulRegionStartAddress = ( uint32_t ) pxBottomOfStack;
            xMPUSettings->xRegionSettings[ 0 ].ulRegionEndAddress = ( uint32_t ) ( ( uint32_t ) ( pxBottomOfStack ) +
                                                                                   ( ulStackDepth * ( uint32_t ) sizeof( StackType_t ) ) - 1UL );
            xMPUSettings->xRegionSettings[ 0 ].ulRegionPermissions = ( tskMPU_READ_PERMISSION |
                                                                       tskMPU_WRITE_PERMISSION );
        }

        lIndex = 0;

        for( ul = 1UL; ul <= portNUM_CONFIGURABLE_REGIONS; ul++ )
        {
            if( ( xRegions[ lIndex ] ).ulLengthInBytes > 0UL )
            {
                /* Translate the generic region definition contained in
                 * xRegions into the CM4 specific MPU settings that are then
                 * stored in xMPUSettings. */
                xMPUSettings->xRegion[ ul ].ulRegionBaseAddress =
                    ( ( uint32_t ) xRegions[ lIndex ].pvBaseAddress ) |
                    ( portMPU_REGION_VALID ) |
                    ( ul - 1UL ); /* Region number. */

                xMPUSettings->xRegion[ ul ].ulRegionAttribute =
                    ( prvGetMPURegionSizeSetting( xRegions[ lIndex ].ulLengthInBytes ) ) |
                    ( xRegions[ lIndex ].ulParameters ) |
                    ( portMPU_REGION_ENABLE );

                xMPUSettings->xRegionSettings[ ul ].ulRegionStartAddress = ( uint32_t ) xRegions[ lIndex ].pvBaseAddress;
                xMPUSettings->xRegionSettings[ ul ].ulRegionEndAddress = ( uint32_t ) ( ( uint32_t ) xRegions[ lIndex ].pvBaseAddress + xRegions[ lIndex ].ulLengthInBytes - 1UL );
                xMPUSettings->xRegionSettings[ ul ].ulRegionPermissions = 0UL;

                if( ( ( xRegions[ lIndex ].ulParameters & portMPU_REGION_READ_ONLY ) == portMPU_REGION_READ_ONLY ) ||
                    ( ( xRegions[ lIndex ].ulParameters & portMPU_REGION_PRIVILEGED_READ_WRITE_UNPRIV_READ_ONLY ) == portMPU_REGION_PRIVILEGED_READ_WRITE_UNPRIV_READ_ONLY ) )
                {
                    xMPUSettings->xRegionSettings[ ul ].ulRegionPermissions = tskMPU_READ_PERMISSION;
                }

                if( ( xRegions[ lIndex ].ulParameters & portMPU_REGION_READ_WRITE ) == portMPU_REGION_READ_WRITE )
                {
                    xMPUSettings->xRegionSettings[ ul ].ulRegionPermissions = ( tskMPU_READ_PERMISSION | tskMPU_WRITE_PERMISSION );
                }
            }
            else
            {
                /* Invalidate the region. */
                xMPUSettings->xRegion[ ul ].ulRegionBaseAddress = ( ( ul - 1UL ) | portMPU_REGION_VALID );
                xMPUSettings->xRegion[ ul ].ulRegionAttribute = 0UL;
                xMPUSettings->xRegionSettings[ ul ].ulRegionStartAddress = 0UL;
                xMPUSettings->xRegionSettings[ ul ].ulRegionEndAddress = 0UL;
                xMPUSettings->xRegionSettings[ ul ].ulRegionPermissions = 0UL;
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

            for( i = 0; i < portTOTAL_NUM_REGIONS_IN_TCB; i++ )
            {
                if( portIS_ADDRESS_WITHIN_RANGE( ulBufferStartAddress,
                                                 xTaskMpuSettings->xRegionSettings[ i ].ulRegionStartAddress,
                                                 xTaskMpuSettings->xRegionSettings[ i ].ulRegionEndAddress ) &&
                    portIS_ADDRESS_WITHIN_RANGE( ulBufferEndAddress,
                                                 xTaskMpuSettings->xRegionSettings[ i ].ulRegionStartAddress,
                                                 xTaskMpuSettings->xRegionSettings[ i ].ulRegionEndAddress ) &&
                    portIS_AUTHORIZED( ulAccessRequested, xTaskMpuSettings->xRegionSettings[ i ].ulRegionPermissions ) )
                {
                    xAccessGranted = pdTRUE;
                    break;
                }
            }
        }
    }

    return xAccessGranted;
}
/* ----------------------------------------------------------------------------------- */

#if ( configASSERT_DEFINED == 1 )

    static void prvTaskExitError( void )
    {
        /* A function that implements a task must not exit or attempt to return to
        * its caller as there is nothing to return to.  If a task wants to exit it
        * should instead call vTaskDelete( NULL ).
        *
        * Artificially force an assert() to be triggered if configASSERT() is
        * defined, then stop here so application writers can catch the error. */
        configASSERT( uxCriticalNesting == ~0UL );
        portDISABLE_INTERRUPTS();

        for( ; ; )
        {
        }
    }

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
             * Interrupts that use the FreeRTOS API must not be left at their
             * default priority of zero as that is the highest possible priority,
             * which is guaranteed to be above configMAX_SYSCALL_INTERRUPT_PRIORITY,
             * and therefore also guaranteed to be invalid.
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

#endif /* configASSERT_DEFINED */
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
