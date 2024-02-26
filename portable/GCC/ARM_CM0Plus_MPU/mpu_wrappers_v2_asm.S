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

/*---------------------------------------------------------------------------*/

    .thumb
    .cpu cortex-m0plus
    .syntax unified
    .section freertos_system_calls
#define FREERTOS_ASSEMBLY
    #include "FreeRTOSConfig.h"
    #include "mpu_syscall_numbers.h"
#undef FREERTOS_ASSEMBLY

/* ------------------ Start of Port Specific System Calls ------------------ */

/**
 * Function: void vPortYield
 * Inputs:      VOID
*/
.align 4
.global vPortYield
.type vPortYield, %function
vPortYield:
    /* Make the SVC to swap tasks */
    SVC     #100
    /* After yielding to another task, resume executing the calling task */
    BX  LR

/*-------------------------------------------------------------------------------*/
/* vPortSystemCallExit */
.align 4
.global vPortSystemCallExit
.type vPortSystemCallExit, %function
vPortSystemCallExit:
    /* Make the SVC to exit a system call */
    SVC     #104
    /* After performing the requested FreeRTOS API, return to the calling task */
    BX      LR

/*-------------------------------------------------------------------------------*/

/**
 * Function: BaseType_t xPortIsPrivileged
 * Inputs:      VOID
*/
.align 4
.global xPortIsPrivileged
.type xPortIsPrivileged, %function
xPortIsPrivileged:
    /* Push R1 before using it */
    PUSH    { R1 }
    /* Load value of CONTROL into R0 */
    MRS     R0, CONTROL
    /* CONTROL bit is high if unprivileged */
    MOVS    r0, 0x1
    /* If CONTROL is high, set R0 to 0 */
    EORS     R0, R1
    /* Restore the pushed register */
    POP     { R1 }
    /* Return to the caller */
    BLX      LR


/*-------------------------------------------------------------------------------*/
#if 0
/**
 * Function: UBaseType_t ulPortCountLeadingZeros
 * Inputs:      UBaseType_t ulBitmap
*/
.align 4
.weak ulPortCountLeadingZeros
.type ulPortCountLeadingZeros, %function
ulPortCountLeadingZeros:
    /* Count the leading zeros and return in R0 */
    CLZ     R0, R0
    BX      LR
#endif
/* ------------------- End of Port Specific System Calls ------------------- */

.macro processorIsPrivileged
    /* Push R0 and R1 before using them */
    PUSH    { R0-R1 }
    MRS     R0, CONTROL
    MOVS    R1, #1
    TST     R0, R1
    /* Restore the pushed register */
    POP     { R0-R1 }
.endm

/*---------------------------------------------------------------------------*/

.extern MPU_xTaskGetTickCountImpl
/**
 * Function: TickType_t MPU_xTaskGetTickCount
 * Inputs:     void - No Inputs
*/
.align 4
.global MPU_xTaskGetTickCount
.type MPU_xTaskGetTickCount, function
MPU_xTaskGetTickCount:
    processorIsPrivileged
    BEQ     MPU_xTaskGetTickCount_Privileged
        SVC     #SYSTEM_CALL_xTaskGetTickCount
    MPU_xTaskGetTickCount_Privileged:
        PUSH    { LR }
        BLX     MPU_xTaskGetTickCountImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_uxTaskGetNumberOfTasksImpl
/**
 * Function: UBaseType_t MPU_uxTaskGetNumberOfTasks
 * Inputs:     void - No Inputs
*/
.align 4
.global MPU_uxTaskGetNumberOfTasks
.type MPU_uxTaskGetNumberOfTasks, function
MPU_uxTaskGetNumberOfTasks:
    processorIsPrivileged
    BEQ     MPU_uxTaskGetNumberOfTasks_Privileged
        SVC     #SYSTEM_CALL_uxTaskGetNumberOfTasks
    MPU_uxTaskGetNumberOfTasks_Privileged:
        PUSH    { LR }
        BLX     MPU_uxTaskGetNumberOfTasksImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_vTaskSetTimeOutStateImpl
/**
 * Function: void MPU_vTaskSetTimeOutState
 * Inputs:     TimeOut_t * const pxTimeOut
*/
.align 4
.global MPU_vTaskSetTimeOutState
.type MPU_vTaskSetTimeOutState, function
MPU_vTaskSetTimeOutState:
    processorIsPrivileged
    BEQ     MPU_vTaskSetTimeOutState_Privileged
        SVC     #SYSTEM_CALL_vTaskSetTimeOutState
    MPU_vTaskSetTimeOutState_Privileged:
        PUSH    { LR }
        BLX      MPU_vTaskSetTimeOutStateImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xTaskCheckForTimeOutImpl
/**
 * Function: BaseType_t MPU_xTaskCheckForTimeOut
 * Inputs:     TimeOut_t * const pxTimeOut
 * Inputs:     TickType_t * const pxTicksToWait
*/
.align 4
.global MPU_xTaskCheckForTimeOut
.type MPU_xTaskCheckForTimeOut, function
MPU_xTaskCheckForTimeOut:
    processorIsPrivileged
    BEQ     MPU_xTaskCheckForTimeOut_Privileged
        SVC     #SYSTEM_CALL_xTaskCheckForTimeOut
    MPU_xTaskCheckForTimeOut_Privileged:
        PUSH    { LR }
        BLX      MPU_xTaskCheckForTimeOutImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xQueueGenericSendImpl
/**
 * Function: BaseType_t MPU_xQueueGenericSend
 * Inputs:     QueueHandle_t xQueue
 * Inputs:     const void * const pvItemToQueue
 * Inputs:     TickType_t xTicksToWait
 * Inputs:     constBaseType_t xCopyPosition
*/
.align 4
.global MPU_xQueueGenericSend
.type MPU_xQueueGenericSend, function
MPU_xQueueGenericSend:
    processorIsPrivileged
    BEQ     MPU_xQueueGenericSend_Privileged
        SVC     #SYSTEM_CALL_xQueueGenericSend
    MPU_xQueueGenericSend_Privileged:
        PUSH    { LR }
        BLX      MPU_xQueueGenericSendImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_uxQueueMessagesWaitingImpl
/**
 * Function: UBaseType_t MPU_uxQueueMessagesWaiting
 * Inputs:     const QueueHandle_t xQueue
*/
.align 4
.global MPU_uxQueueMessagesWaiting
.type MPU_uxQueueMessagesWaiting, function
MPU_uxQueueMessagesWaiting:
    processorIsPrivileged
    BEQ     MPU_uxQueueMessagesWaiting_Privileged
        SVC     #SYSTEM_CALL_uxQueueMessagesWaiting
    MPU_uxQueueMessagesWaiting_Privileged:
        PUSH    { LR }
        BLX      MPU_uxQueueMessagesWaitingImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_uxQueueSpacesAvailableImpl
/**
 * Function: UBaseType_t MPU_uxQueueSpacesAvailable
 * Inputs:     const QueueHandle_t xQueue
*/
.align 4
.global MPU_uxQueueSpacesAvailable
.type MPU_uxQueueSpacesAvailable, function
MPU_uxQueueSpacesAvailable:
    processorIsPrivileged
    BEQ     MPU_uxQueueSpacesAvailable_Privileged
        SVC     #SYSTEM_CALL_uxQueueSpacesAvailable
    MPU_uxQueueSpacesAvailable_Privileged:
        PUSH    { LR }
        BLX      MPU_uxQueueSpacesAvailableImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xQueueReceiveImpl
/**
 * Function: BaseType_t MPU_xQueueReceive
 * Inputs:     QueueHandle_t xQueue
 * Inputs:     void * const pvBuffer
 * Inputs:     TickType_t xTicksToWait
*/
.align 4
.global MPU_xQueueReceive
.type MPU_xQueueReceive, function
MPU_xQueueReceive:
    processorIsPrivileged
    BEQ     MPU_xQueueReceive_Privileged
        SVC     #SYSTEM_CALL_xQueueReceive
    MPU_xQueueReceive_Privileged:
        PUSH    { LR }
        BLX     MPU_xQueueReceiveImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xQueuePeekImpl
/**
 * Function: BaseType_t MPU_xQueuePeek
 * Inputs:     QueueHandle_t xQueue
 * Inputs:     void * const pvBuffer
 * Inputs:     TickType_t xTicksToWait
*/
.align 4
.global MPU_xQueuePeek
.type MPU_xQueuePeek, function
MPU_xQueuePeek:
    processorIsPrivileged
    BEQ     MPU_xQueuePeek_Privileged
        SVC     #SYSTEM_CALL_xQueuePeek
    MPU_xQueuePeek_Privileged:
        PUSH    { LR }
        BLX     MPU_xQueuePeekImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xQueueSemaphoreTakeImpl
/**
 * Function: BaseType_t MPU_xQueueSemaphoreTake
 * Inputs:     QueueHandle_t xQueue
 * Inputs:     TickType_t xTicksToWait
*/
.align 4
.global MPU_xQueueSemaphoreTake
.type MPU_xQueueSemaphoreTake, function
MPU_xQueueSemaphoreTake:
    processorIsPrivileged
    BEQ     MPU_xQueueSemaphoreTake_Privileged
        SVC     #SYSTEM_CALL_xQueueSemaphoreTake
    MPU_xQueueSemaphoreTake_Privileged:
        PUSH    { LR }
        BLX     MPU_xQueueSemaphoreTakeImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xEventGroupWaitBitsImpl
/**
 * Function: EventBits_t MPU_xEventGroupWaitBitsEntry
 * Inputs:     const xEventGroupWaitBitsParams_t * pxParams
*/
.align 4
.global MPU_xEventGroupWaitBitsEntry
.type MPU_xEventGroupWaitBitsEntry, function
MPU_xEventGroupWaitBitsEntry:
    processorIsPrivileged
    BEQ     MPU_xEventGroupWaitBits_Privileged
        SVC     #SYSTEM_CALL_xEventGroupWaitBits
    MPU_xEventGroupWaitBits_Privileged:
        PUSH    { LR }
        BLX     MPU_xEventGroupWaitBitsImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xEventGroupClearBitsImpl
/**
 * Function: EventBits_t MPU_xEventGroupClearBits
 * Inputs:     EventGroupHandle_t xEventGroup
 * Inputs:     constEventBits_t uxBitsToClear
*/
.align 4
.global MPU_xEventGroupClearBits
.type MPU_xEventGroupClearBits, function
MPU_xEventGroupClearBits:
    processorIsPrivileged
    BEQ     MPU_xEventGroupClearBits_Privileged
        SVC     #SYSTEM_CALL_xEventGroupClearBits
    MPU_xEventGroupClearBits_Privileged:
        PUSH    { LR }
        BLX     MPU_xEventGroupClearBitsImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xEventGroupSetBitsImpl
/**
 * Function: EventBits_t MPU_xEventGroupSetBits
 * Inputs:     EventGroupHandle_t xEventGroup
 * Inputs:     constEventBits_t uxBitsToSet
*/
.align 4
.global MPU_xEventGroupSetBits
.type MPU_xEventGroupSetBits, function
MPU_xEventGroupSetBits:
    processorIsPrivileged
    BEQ     MPU_xEventGroupSetBits_Privileged
        SVC     #SYSTEM_CALL_xEventGroupSetBits
    MPU_xEventGroupSetBits_Privileged:
        PUSH    { LR }
        BLX     MPU_xEventGroupSetBitsImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xEventGroupSyncImpl
/**
 * Function: EventBits_t MPU_xEventGroupSync
 * Inputs:     EventGroupHandle_t xEventGroup
 * Inputs:     constEventBits_t uxBitsToSet
 * Inputs:     constEventBits_t uxBitsToWaitFor
 * Inputs:     TickType_t xTicksToWait
*/
.align 4
.global MPU_xEventGroupSync
.type MPU_xEventGroupSync, function
MPU_xEventGroupSync:
    processorIsPrivileged
    BEQ     MPU_xEventGroupSync_Privileged
        SVC     #SYSTEM_CALL_xEventGroupSync
    MPU_xEventGroupSync_Privileged:
        PUSH    { LR }
        BLX     MPU_xEventGroupSyncImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xStreamBufferSendImpl
/**
 * Function: size_t MPU_xStreamBufferSend
 * Inputs:     StreamBufferHandle_t xStreamBuffer
 * Inputs:     const void * pvTxData
 * Inputs:     size_t xDataLengthBytes
 * Inputs:     TickType_t xTicksToWait
*/
.align 4
.global MPU_xStreamBufferSend
.type MPU_xStreamBufferSend, function
MPU_xStreamBufferSend:
    processorIsPrivileged
    BEQ     MPU_xStreamBufferSend_Privileged
        SVC     #SYSTEM_CALL_xStreamBufferSend
    MPU_xStreamBufferSend_Privileged:
        PUSH    { LR }
        BLX     MPU_xStreamBufferSendImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xStreamBufferReceiveImpl
/**
 * Function: size_t MPU_xStreamBufferReceive
 * Inputs:     StreamBufferHandle_t xStreamBuffer
 * Inputs:     void * pvRxData
 * Inputs:     size_t xBufferLengthBytes
 * Inputs:     TickType_t xTicksToWait
*/
.align 4
.global MPU_xStreamBufferReceive
.type MPU_xStreamBufferReceive, function
MPU_xStreamBufferReceive:
    processorIsPrivileged
    BEQ     MPU_xStreamBufferReceive_Privileged
        SVC     #SYSTEM_CALL_xStreamBufferReceive
    MPU_xStreamBufferReceive_Privileged:
        PUSH    { LR }
        BLX     MPU_xStreamBufferReceiveImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xStreamBufferIsFullImpl
/**
 * Function: BaseType_t MPU_xStreamBufferIsFull
 * Inputs:     StreamBufferHandle_t xStreamBuffer
*/
.align 4
.global MPU_xStreamBufferIsFull
.type MPU_xStreamBufferIsFull, function
MPU_xStreamBufferIsFull:
    processorIsPrivileged
    BEQ     MPU_xStreamBufferIsFull_Privileged
        SVC     #SYSTEM_CALL_xStreamBufferIsFull
    MPU_xStreamBufferIsFull_Privileged:
        PUSH    { LR }
        BLX     MPU_xStreamBufferIsFullImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xStreamBufferIsEmptyImpl
/**
 * Function: BaseType_t MPU_xStreamBufferIsEmpty
 * Inputs:     StreamBufferHandle_t xStreamBuffer
*/
.align 4
.global MPU_xStreamBufferIsEmpty
.type MPU_xStreamBufferIsEmpty, function
MPU_xStreamBufferIsEmpty:
    processorIsPrivileged
    BEQ     MPU_xStreamBufferIsEmpty_Privileged
        SVC     #SYSTEM_CALL_xStreamBufferIsEmpty
    MPU_xStreamBufferIsEmpty_Privileged:
        PUSH    { LR }
        BLX     MPU_xStreamBufferIsEmptyImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xStreamBufferSpacesAvailableImpl
/**
 * Function: size_t MPU_xStreamBufferSpacesAvailable
 * Inputs:     StreamBufferHandle_t xStreamBuffer
*/
.align 4
.global MPU_xStreamBufferSpacesAvailable
.type MPU_xStreamBufferSpacesAvailable, function
MPU_xStreamBufferSpacesAvailable:
    processorIsPrivileged
    BEQ     MPU_xStreamBufferSpacesAvailable_Privileged
        SVC     #SYSTEM_CALL_xStreamBufferSpacesAvailable
    MPU_xStreamBufferSpacesAvailable_Privileged:
        PUSH    { LR }
        BLX     MPU_xStreamBufferSpacesAvailableImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xStreamBufferBytesAvailableImpl
/**
 * Function: size_t MPU_xStreamBufferBytesAvailable
 * Inputs:     StreamBufferHandle_t xStreamBuffer
*/
.align 4
.global MPU_xStreamBufferBytesAvailable
.type MPU_xStreamBufferBytesAvailable, function
MPU_xStreamBufferBytesAvailable:
    processorIsPrivileged
    BEQ     MPU_xStreamBufferBytesAvailable_Privileged
        SVC     #SYSTEM_CALL_xStreamBufferBytesAvailable
    MPU_xStreamBufferBytesAvailable_Privileged:
        PUSH    { LR }
        BLX     MPU_xStreamBufferBytesAvailableImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xStreamBufferSetTriggerLevelImpl
/**
 * Function: BaseType_t MPU_xStreamBufferSetTriggerLevel
 * Inputs:     StreamBufferHandle_t xStreamBuffer
 * Inputs:     size_t xTriggerLevel
*/
.align 4
.global MPU_xStreamBufferSetTriggerLevel
.type MPU_xStreamBufferSetTriggerLevel, function
MPU_xStreamBufferSetTriggerLevel:
    processorIsPrivileged
    BEQ     MPU_xStreamBufferSetTriggerLevel_Privileged
        SVC     #SYSTEM_CALL_xStreamBufferSetTriggerLevel
    MPU_xStreamBufferSetTriggerLevel_Privileged:
        PUSH    { LR }
        BLX     MPU_xStreamBufferSetTriggerLevelImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

.extern MPU_xStreamBufferNextMessageLengthBytesImpl
/**
 * Function: size_t MPU_xStreamBufferNextMessageLengthBytes
 * Inputs:     StreamBufferHandle_t xStreamBuffer
*/
.align 4
.global MPU_xStreamBufferNextMessageLengthBytes
.type MPU_xStreamBufferNextMessageLengthBytes, function
MPU_xStreamBufferNextMessageLengthBytes:
    processorIsPrivileged
    BEQ     MPU_xStreamBufferNextMessageLengthBytes_Privileged
        SVC     #SYSTEM_CALL_xStreamBufferNextMessageLengthBytes
    MPU_xStreamBufferNextMessageLengthBytes_Privileged:
        PUSH    { LR }
        BLX     MPU_xStreamBufferNextMessageLengthBytesImpl
        POP     { PC }

/*---------------------------------------------------------------------------*/

#if ( INCLUDE_xTaskDelayUntil == 1 ) | ( INCLUDE_vTaskDelayUntil == 1 )
    .extern MPU_xTaskDelayUntilImpl
    /**
     * Function: TaskHandle_t MPU_xTaskDelayUntil
     * Inputs:     TickType_t * const pxPreviousWakeTime
     * Inputs:     constTickType_t xTimeIncrement
    */
    .align 4
    .global MPU_xTaskDelayUntil
    .type MPU_xTaskDelayUntil, function
    MPU_xTaskDelayUntil:
        processorIsPrivileged
        BEQ     MPU_xTaskDelayUntil_Privileged
            SVC     #SYSTEM_CALL_xTaskDelayUntil
        MPU_xTaskDelayUntil_Privileged:
            PUSH    { LR }
            BLX     MPU_xTaskDelayUntilImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* ( INCLUDE_xTaskDelayUntil == 1 ) | ( INCLUDE_vTaskDelayUntil == 1 ) */

#if INCLUDE_xTaskAbortDelay == 1

/*---------------------------------------------------------------------------*/

    .extern MPU_xTaskAbortDelayImpl
    /**
     * Function: TaskHandle_t MPU_xTaskAbortDelay
     * Inputs:     TaskHandle_t xTask
    */
    .align 4
    .global MPU_xTaskAbortDelay
    .type MPU_xTaskAbortDelay, function
    MPU_xTaskAbortDelay:
        processorIsPrivileged
        BEQ     MPU_xTaskAbortDelay_Privileged
            SVC     #SYSTEM_CALL_xTaskAbortDelay
        MPU_xTaskAbortDelay_Privileged:
            PUSH    { LR }
            BLX     MPU_xTaskAbortDelayImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( INCLUDE_xTaskAbortDelay == 1 ) */

#if INCLUDE_vTaskDelay == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_vTaskDelayImpl
    /**
     * Function: void MPU_vTaskDelay
     * Inputs:     const TickType_t xTicksToDelay
    */
    .align 4
    .global MPU_vTaskDelay
    .type MPU_vTaskDelay, function
    MPU_vTaskDelay:
        processorIsPrivileged
        BEQ     MPU_vTaskDelay_Privileged
            SVC     #SYSTEM_CALL_vTaskDelay
        MPU_vTaskDelay_Privileged:
            PUSH    { LR }
            BLX     MPU_vTaskDelayImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( INCLUDE_vTaskDelay == 1 ) */

#if INCLUDE_uxTaskPriorityGet == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_uxTaskPriorityGetImpl
    /**
     * Function: UBaseType_t MPU_uxTaskPriorityGet
     * Inputs:     const TaskHandle_t xTask
    */
    .align 4
    .global MPU_uxTaskPriorityGet
    .type MPU_uxTaskPriorityGet, function
    MPU_uxTaskPriorityGet:
        processorIsPrivileged
        BEQ     MPU_uxTaskPriorityGet_Privileged
            SVC     #SYSTEM_CALL_uxTaskPriorityGet
        MPU_uxTaskPriorityGet_Privileged:
            PUSH    { LR }
            BLX     MPU_uxTaskPriorityGetImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( INCLUDE_uxTaskPriorityGet == 1 ) */

#if INCLUDE_eTaskGetState == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_eTaskGetStateImpl
    /**
     * Function: eTaskState MPU_eTaskGetState
     * Inputs:     TaskHandle_t xTask
    */
    .align 4
    .global MPU_eTaskGetState
    .type MPU_eTaskGetState, function
    MPU_eTaskGetState:
        processorIsPrivileged
        BEQ     MPU_eTaskGetState_Privileged
            SVC     #SYSTEM_CALL_eTaskGetState
        MPU_eTaskGetState_Privileged:
            PUSH    { LR }
            BLX     MPU_eTaskGetStateImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( INCLUDE_eTaskGetState == 1 ) */

#if configUSE_TRACE_FACILITY == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_vTaskGetInfoImpl
    /**
     * Function: void MPU_vTaskGetInfo
     * Inputs:     TaskHandle_t xTask
     * Inputs:     TaskStatus_t* pxTaskStatus
     * Inputs:     TaskHandle_t xGetFreeStackSpace
     * Inputs:     eTaskState eState
    */
    .align 4
    .global MPU_vTaskGetInfo
    .type MPU_vTaskGetInfo, function
    MPU_vTaskGetInfo:
        processorIsPrivileged
        BEQ     MPU_vTaskGetInfo_Privileged
            SVC     #SYSTEM_CALL_vTaskGetInfo
        MPU_vTaskGetInfo_Privileged:
            PUSH    { LR }
            BLX     MPU_vTaskGetInfoImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_uxTaskGetSystemStateImpl
    /**
     * Function: UBaseType_t MPU_uxTaskGetSystemState
     * Inputs:     TaskStatus_t * const pxTaskStatusArray
     * Inputs:     constUBaseType_t uxArraySize
     * Inputs:     configRUN_TIME_COUNTER_TYPE* const pulTotalRunTime
    */
    .align 4
    .global MPU_uxTaskGetSystemState
    .type MPU_uxTaskGetSystemState, function
    MPU_uxTaskGetSystemState:
        processorIsPrivileged
        BEQ     MPU_uxTaskGetSystemState_Privileged
            SVC     #SYSTEM_CALL_uxTaskGetSystemState
        MPU_uxTaskGetSystemState_Privileged:
            PUSH    { LR }
            BLX     MPU_uxTaskGetSystemStateImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_uxEventGroupGetNumberImpl
    /**
     * Function: UBaseType_t MPU_uxEventGroupGetNumber
     * Inputs:     void * xEventGroup
    */
    .align 4
    .global MPU_uxEventGroupGetNumber
    .type MPU_uxEventGroupGetNumber, function
    MPU_uxEventGroupGetNumber:
    processorIsPrivileged
        BEQ     MPU_uxEventGroupGetNumber_Privileged
            SVC     #SYSTEM_CALL_uxEventGroupGetNumber
        MPU_uxEventGroupGetNumber_Privileged:
            PUSH    { LR }
            BLX     MPU_uxEventGroupGetNumberImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_vEventGroupSetNumberImpl
    /**
     * Function: void MPU_vEventGroupSetNumber
     * Inputs:     void * xEventGroup
     * Inputs:     UBaseType_t uxEventGroupNumber
    */
    .align 4
    .global MPU_vEventGroupSetNumber
    .type MPU_vEventGroupSetNumber, function
    MPU_vEventGroupSetNumber:
        processorIsPrivileged
        BEQ     MPU_vEventGroupSetNumber_Privileged
            SVC     #SYSTEM_CALL_vEventGroupSetNumber
        MPU_vEventGroupSetNumber_Privileged:
            PUSH    { LR }
            BLX     MPU_vEventGroupSetNumberImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/
#endif /* if ( configUSE_TRACE_FACILITY == 1 ) */

#if INCLUDE_xTaskGetIdleTaskHandle == 1

    /*-----------------------------------------------------------------------*/
    .extern MPU_xTaskGetIdleTaskHandleImpl
    /**
     * Function: TaskHandle_t MPU_xTaskGetIdleTaskHandle
     * Inputs:     void - No Inputs
    */
    .align 4
    .global MPU_xTaskGetIdleTaskHandle
    .type MPU_xTaskGetIdleTaskHandle, function
    MPU_xTaskGetIdleTaskHandle:
        processorIsPrivileged
        BEQ     MPU_xTaskGetIdleTaskHandle_Privileged
            SVC     #SYSTEM_CALL_xTaskGetIdleTaskHandle
        MPU_xTaskGetIdleTaskHandle_Privileged:
            PUSH    { LR }
            BLX     MPU_xTaskGetIdleTaskHandleImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) */

#if INCLUDE_vTaskSuspend == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_vTaskSuspendImpl
    /**
     * Function: void MPU_vTaskSuspend
     * Inputs:     TaskHandle_t xTaskToSuspend
    */
    .align 4
    .global MPU_vTaskSuspend
    .type MPU_vTaskSuspend, function
    MPU_vTaskSuspend:
        processorIsPrivileged
        BEQ     MPU_vTaskSuspend_Privileged
            SVC     #SYSTEM_CALL_vTaskSuspend
        MPU_vTaskSuspend_Privileged:
            PUSH    { LR }
            BLX     MPU_vTaskSuspendImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_vTaskResumeImpl
    /**
     * Function: void MPU_vTaskResume
     * Inputs:     TaskHandle_t xTaskToResume
    */
    .align 4
    .global MPU_vTaskResume
    .type MPU_vTaskResume, function
    MPU_vTaskResume:
        processorIsPrivileged
        BEQ     MPU_vTaskResume_Privileged
            SVC     #SYSTEM_CALL_vTaskResume
        MPU_vTaskResume_Privileged:
            PUSH    { LR }
            BLX     MPU_vTaskResumeImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( INCLUDE_vTaskSuspend == 1 ) */

#if configGENERATE_RUN_TIME_STATS == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_ulTaskGetRunTimeCounterImpl
    /**
     * Function: configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetRunTimeCounter
     * Inputs:     const TaskHandle_t xTask
    */
    .align 4
    .global MPU_ulTaskGetRunTimeCounter
    .type MPU_ulTaskGetRunTimeCounter, function
    MPU_ulTaskGetRunTimeCounter:
        processorIsPrivileged
        BEQ     MPU_ulTaskGetRunTimeCounter_Privileged
            SVC     #SYSTEM_CALL_ulTaskGetRunTimeCounter
        MPU_ulTaskGetRunTimeCounter_Privileged:
            PUSH    { LR }
            BLX     MPU_ulTaskGetRunTimeCounterImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_ulTaskGetRunTimePercentImpl
    /**
     * Function: configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetRunTimePercent
     * Inputs:     const TaskHandle_t xTask
    */
    .align 4
    .global MPU_ulTaskGetRunTimePercent
    .type MPU_ulTaskGetRunTimePercent, function
    MPU_ulTaskGetRunTimePercent:
        processorIsPrivileged
        BEQ     MPU_ulTaskGetRunTimePercent_Privileged
            SVC     #SYSTEM_CALL_ulTaskGetRunTimePercent
        MPU_ulTaskGetRunTimePercent_Privileged:
            PUSH    { LR }
            BLX     MPU_ulTaskGetRunTimePercentImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    #if INCLUDE_xTaskGetIdleTaskHandle == 1

        /*-------------------------------------------------------------------*/

        .extern MPU_ulTaskGetIdleRunTimePercentImpl
        /**
         * Function: configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetIdleRunTimePercent
         * Inputs:     void
        */
        .align 4
        .global MPU_ulTaskGetIdleRunTimePercent
        .type MPU_ulTaskGetIdleRunTimePercent, function
        MPU_ulTaskGetIdleRunTimePercent:
            processorIsPrivileged
            BEQ     MPU_ulTaskGetIdleRunTimePercent_Privileged
                SVC     #SYSTEM_CALL_ulTaskGetIdleRunTimePercent
            MPU_ulTaskGetIdleRunTimePercent_Privileged:
                PUSH    { LR }
                BLX     MPU_ulTaskGetIdleRunTimePercentImpl
                POP     { PC }

        /*-------------------------------------------------------------------*/

        .extern MPU_ulTaskGetIdleRunTimeCounterImpl
        /**
         * Function: configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetIdleRunTimeCounter
         * Inputs:     void - No Inputs
        */
        .align 4
        .global MPU_ulTaskGetIdleRunTimeCounter
        .type MPU_ulTaskGetIdleRunTimeCounter, function
        MPU_ulTaskGetIdleRunTimeCounter:
            processorIsPrivileged
            BEQ     MPU_ulTaskGetIdleRunTimeCounter_Privileged
                SVC     #SYSTEM_CALL_ulTaskGetIdleRunTimeCounter
            MPU_ulTaskGetIdleRunTimeCounter_Privileged:
                PUSH    { LR }
                BLX     MPU_ulTaskGetIdleRunTimeCounterImpl
                POP     { PC }

        /*-------------------------------------------------------------------*/

    #endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) ) */
#endif /* configGENERATE_RUN_TIME_STATS */

#if configUSE_APPLICATION_TASK_TAG == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_vTaskSetApplicationTaskTagImpl
    /**
     * Function: void MPU_vTaskSetApplicationTaskTag
     * Inputs:     TaskHandle_t xTask
     * Inputs:     TaskHookFunction_tpxHookFunction
    */
    .align 4
    .global MPU_vTaskSetApplicationTaskTag
    .type MPU_vTaskSetApplicationTaskTag, function
    MPU_vTaskSetApplicationTaskTag:
        processorIsPrivileged
        BEQ     MPU_vTaskSetApplicationTaskTag_Privileged
            SVC     #SYSTEM_CALL_vTaskSetApplicationTaskTag
        MPU_vTaskSetApplicationTaskTag_Privileged:
            PUSH    { LR }
            BLX     MPU_vTaskSetApplicationTaskTagImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTaskGetApplicationTaskTagImpl
    /**
     * Function: TaskHookFunction_t MPU_xTaskGetApplicationTaskTag
     * Inputs:     TaskHandle_t xTask
    */
    .align 4
    .global MPU_xTaskGetApplicationTaskTag
    .type MPU_xTaskGetApplicationTaskTag, function
    MPU_xTaskGetApplicationTaskTag:
        processorIsPrivileged
        BEQ     MPU_xTaskGetApplicationTaskTag_Privileged
            SVC     #SYSTEM_CALL_xTaskGetApplicationTaskTag
        MPU_xTaskGetApplicationTaskTag_Privileged:
            PUSH    { LR }
            BLX     MPU_xTaskGetApplicationTaskTagImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/
#endif /* if ( configUSE_APPLICATION_TASK_TAG == 1 ) */

#if configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0

    /*-----------------------------------------------------------------------*/

    .extern MPU_vTaskSetThreadLocalStoragePointerImpl
    /**
     * Function: void MPU_vTaskSetThreadLocalStoragePointer
     * Inputs:     TaskHandle_t xTaskToSet
     * Inputs:     TaskHandle_t xIndex
     * Inputs:     void * pvValue
    */
    .align 4
    .global MPU_vTaskSetThreadLocalStoragePointer
    .type MPU_vTaskSetThreadLocalStoragePointer, function
    MPU_vTaskSetThreadLocalStoragePointer:
        processorIsPrivileged
        BEQ     MPU_vTaskSetThreadLocalStoragePointer_Privileged
            SVC     #SYSTEM_CALL_vTaskSetThreadLocalStoragePointer
        MPU_vTaskSetThreadLocalStoragePointer_Privileged:
            PUSH    { LR }
            BLX     MPU_vTaskSetThreadLocalStoragePointerImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_pvTaskGetThreadLocalStoragePointerImpl
    /**
     * Function: void * MPU_pvTaskGetThreadLocalStoragePointer
     * Inputs:     TaskHandle_t xTaskToQuery
     * Inputs:     TaskHandle_t xIndex
    */
    .align 4
    .global MPU_pvTaskGetThreadLocalStoragePointer
    .type MPU_pvTaskGetThreadLocalStoragePointer, function
    MPU_pvTaskGetThreadLocalStoragePointer:
        processorIsPrivileged
        BEQ     MPU_pvTaskGetThreadLocalStoragePointer_Privileged
            SVC     #SYSTEM_CALL_pvTaskGetThreadLocalStoragePointer
        MPU_pvTaskGetThreadLocalStoragePointer_Privileged:
            PUSH    { LR }
            BLX     MPU_pvTaskGetThreadLocalStoragePointerImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 ) */

#if INCLUDE_uxTaskGetStackHighWaterMark == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_uxTaskGetStackHighWaterMarkImpl
    /**
     * Function: UBaseType_t MPU_uxTaskGetStackHighWaterMark
     * Inputs:     TaskHandle_t xTask
    */
    .align 4
    .global MPU_uxTaskGetStackHighWaterMark
    .type MPU_uxTaskGetStackHighWaterMark, function
    MPU_uxTaskGetStackHighWaterMark:
        processorIsPrivileged
        BEQ     MPU_uxTaskGetStackHighWaterMark_Privileged
            SVC     #SYSTEM_CALL_uxTaskGetStackHighWaterMark
        MPU_uxTaskGetStackHighWaterMark_Privileged:
            PUSH    { LR }
            BLX     MPU_uxTaskGetStackHighWaterMarkImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) */


#if INCLUDE_uxTaskGetStackHighWaterMark == 2

    /*-----------------------------------------------------------------------*/

    .extern MPU_uxTaskGetStackHighWaterMark2Impl
    /**
     * Function: configSTACK_DEPTH_TYPE MPU_uxTaskGetStackHighWaterMark2
     * Inputs:     TaskHandle_t xTask
    */
    .align 4
    .global MPU_uxTaskGetStackHighWaterMark2
    .type MPU_uxTaskGetStackHighWaterMark2, function
    MPU_uxTaskGetStackHighWaterMark2:
        processorIsPrivileged
        BEQ     MPU_uxTaskGetStackHighWaterMark2_Privileged
            SVC     #SYSTEM_CALL_uxTaskGetStackHighWaterMark2
        MPU_uxTaskGetStackHighWaterMark2_Privileged:
            PUSH    { LR }
            BLX     MPU_uxTaskGetStackHighWaterMark2Impl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( INCLUDE_uxTaskGetStackHighWaterMark2 == 1 ) */


#if INCLUDE_xTaskGetCurrentTaskHandle == 1 || configUSE_MUTEXES == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTaskGetCurrentTaskHandleImpl
    /**
     * Function: TaskHandle_t MPU_xTaskGetCurrentTaskHandle
     * Inputs:     void - No Inputs
    */
    .align 4
    .global MPU_xTaskGetCurrentTaskHandle
    .type MPU_xTaskGetCurrentTaskHandle, function
    MPU_xTaskGetCurrentTaskHandle:
        processorIsPrivileged
        BEQ     MPU_xTaskGetCurrentTaskHandle_Privileged
            SVC     #SYSTEM_CALL_xTaskGetCurrentTaskHandle
        MPU_xTaskGetCurrentTaskHandle_Privileged:
            PUSH    { LR }
            BLX     MPU_xTaskGetCurrentTaskHandleImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) || ( configUSE_MUTEXES == 1 ) ) */

#if INCLUDE_xTaskGetSchedulerState == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTaskGetSchedulerStateImpl
    /**
     * Function: TaskHandle_t MPU_xTaskGetSchedulerState
     * Inputs:     void - No Inputs
    */
    .align 4
    .global MPU_xTaskGetSchedulerState
    .type MPU_xTaskGetSchedulerState, function
    MPU_xTaskGetSchedulerState:
        processorIsPrivileged
        BEQ     MPU_xTaskGetSchedulerState_Privileged
            SVC     #SYSTEM_CALL_xTaskGetSchedulerState
        MPU_xTaskGetSchedulerState_Privileged:
            PUSH    { LR }
            BLX     MPU_xTaskGetSchedulerStateImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( INCLUDE_xTaskGetSchedulerState == 1 ) */

#if configUSE_MUTEXES == 1 && INCLUDE_xSemaphoreGetMutexHolder

    /*-----------------------------------------------------------------------*/

    .extern MPU_xQueueGetMutexHolderImpl
    /**
     * Function: TaskHandle_t MPU_xQueueGetMutexHolder
     * Inputs:     QueueHandle_t xSemaphore
    */
    .align 4
    .global MPU_xQueueGetMutexHolder
    .type MPU_xQueueGetMutexHolder, function
    MPU_xQueueGetMutexHolder:
        processorIsPrivileged
        BEQ     MPU_xQueueGetMutexHolder_Privileged
            SVC     #SYSTEM_CALL_xQueueGetMutexHolder
        MPU_xQueueGetMutexHolder_Privileged:
            PUSH    { LR }
            BLX     MPU_xQueueGetMutexHolderImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) ) */

#if configUSE_RECURSIVE_MUTEXES == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_xQueueTakeMutexRecursiveImpl
    /**
     * Function: TaskHandle_t MPU_xQueueTakeMutexRecursive
     * Inputs:     QueueHandle_t xMutex
     * Inputs:     TickType_t xTicksToWait
    */
    .align 4
    .global MPU_xQueueTakeMutexRecursive
    .type MPU_xQueueTakeMutexRecursive, function
    MPU_xQueueTakeMutexRecursive:
        processorIsPrivileged
        BEQ     MPU_xQueueTakeMutexRecursive_Privileged
            SVC     #SYSTEM_CALL_xQueueTakeMutexRecursive
        MPU_xQueueTakeMutexRecursive_Privileged:
            PUSH    { LR }
            BLX     MPU_xQueueTakeMutexRecursiveImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xQueueGiveMutexRecursiveImpl
    /**
     * Function: TaskHandle_t MPU_xQueueGiveMutexRecursive
     * Inputs:     QueueHandle_t pxMutex
    */
    .align 4
    .global MPU_xQueueGiveMutexRecursive
    .type MPU_xQueueGiveMutexRecursive, function
    MPU_xQueueGiveMutexRecursive:
        processorIsPrivileged
        BEQ     MPU_xQueueGiveMutexRecursive_Privileged
            SVC     #SYSTEM_CALL_xQueueGiveMutexRecursive
        MPU_xQueueGiveMutexRecursive_Privileged:
            PUSH    { LR }
            BLX     MPU_xQueueGiveMutexRecursiveImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( configUSE_RECURSIVE_MUTEXES == 1 ) */

#if configUSE_QUEUE_SETS == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_xQueueSelectFromSetImpl
    /**
     * Function: QueueSetMemberHandle_t MPU_xQueueSelectFromSet
     * Inputs:     QueueSetHandle_t xQueueSet
     * Inputs:     constTickType_t xTicksToWait
    */
    .align 4
    .global MPU_xQueueSelectFromSet
    .type MPU_xQueueSelectFromSet, function
    MPU_xQueueSelectFromSet:
        processorIsPrivileged
        BEQ     MPU_xQueueSelectFromSet_Privileged
            SVC     #SYSTEM_CALL_xQueueSelectFromSet
        MPU_xQueueSelectFromSet_Privileged:
            PUSH    { LR }
            BLX     MPU_xQueueSelectFromSetImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xQueueAddToSetImpl
    /**
     * Function: TaskHandle_t MPU_xQueueAddToSet
     * Inputs:     QueueSetMemberHandle_t xQueueOrSemaphore
     * Inputs:     QueueSetHandle_t xQueueSet
    */
    .align 4
    .global MPU_xQueueAddToSet
    .type MPU_xQueueAddToSet, function
    MPU_xQueueAddToSet:
        processorIsPrivileged
        BEQ     MPU_xQueueAddToSet_Privileged
            SVC     #SYSTEM_CALL_xQueueAddToSet
        MPU_xQueueAddToSet_Privileged:
            PUSH    { LR }
            BLX     MPU_xQueueAddToSetImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( configUSE_QUEUE_SETS == 1 ) */

#if configQUEUE_REGISTRY_SIZE != 0

    /*-----------------------------------------------------------------------*/

    .extern MPU_vQueueAddToRegistryImpl
    /**
     * Function: void MPU_vQueueAddToRegistry
     * Inputs:     QueueHandle_t xQueue
     * Inputs:     constchar * pcName
    */
    .align 4
    .global MPU_vQueueAddToRegistry
    .type MPU_vQueueAddToRegistry, function
    MPU_vQueueAddToRegistry:
        processorIsPrivileged
        BEQ     MPU_vQueueAddToRegistry_Privileged
            SVC     #SYSTEM_CALL_vQueueAddToRegistry
        MPU_vQueueAddToRegistry_Privileged:
            PUSH    { LR }
            BLX     MPU_vQueueAddToRegistryImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_vQueueUnregisterQueueImpl
    /**
     * Function: void MPU_vQueueUnregisterQueue
     * Inputs:     QueueHandle_t xQueue
    */
    .align 4
    .global MPU_vQueueUnregisterQueue
    .type MPU_vQueueUnregisterQueue, function
    MPU_vQueueUnregisterQueue:
        processorIsPrivileged
        BEQ     MPU_vQueueUnregisterQueue_Privileged
            SVC     #SYSTEM_CALL_vQueueUnregisterQueue
        MPU_vQueueUnregisterQueue_Privileged:
            PUSH    { LR }
            BLX     MPU_vQueueUnregisterQueueImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_pcQueueGetNameImpl
    /**
     * Function: const char * MPU_pcQueueGetName
     * Inputs:     QueueHandle_t xQueue
    */
    .align 4
    .global MPU_pcQueueGetName
    .type MPU_pcQueueGetName, function
    MPU_pcQueueGetName:
        processorIsPrivileged
        BEQ     MPU_pcQueueGetName_Privileged
            SVC     #SYSTEM_CALL_pcQueueGetName
        MPU_pcQueueGetName_Privileged:
            PUSH    { LR }
            BLX     MPU_pcQueueGetNameImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( configQUEUE_REGISTRY_SIZE > 0 ) */

#if configUSE_TIMERS == 1

    /*-----------------------------------------------------------------------*/

    .extern MPU_pvTimerGetTimerIDImpl
    /**
     * Function: void * MPU_pvTimerGetTimerID
     * Inputs:     const TimerHandle_t xTimer
    */
    .align 4
    .global MPU_pvTimerGetTimerID
    .type MPU_pvTimerGetTimerID, function
    MPU_pvTimerGetTimerID:
        processorIsPrivileged
        BEQ     MPU_pvTimerGetTimerID_Privileged
            SVC     #SYSTEM_CALL_pvTimerGetTimerID
        MPU_pvTimerGetTimerID_Privileged:
            PUSH    { LR }
            BLX     MPU_pvTimerGetTimerIDImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_vTimerSetTimerIDImpl
    /**
     * Function: void MPU_vTimerSetTimerID
     * Inputs:     TimerHandle_t xTimer
     * Inputs:     void * pvNewID
    */
    .align 4
    .global MPU_vTimerSetTimerID
    .type MPU_vTimerSetTimerID, function
    MPU_vTimerSetTimerID:
        processorIsPrivileged
        BEQ     MPU_vTimerSetTimerID_Privileged
            SVC     #SYSTEM_CALL_vTimerSetTimerID
        MPU_vTimerSetTimerID_Privileged:
            PUSH    { LR }
            BLX     MPU_vTimerSetTimerIDImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTimerIsTimerActiveImpl
    /**
     * Function: TaskHandle_t MPU_xTimerIsTimerActive
     * Inputs:     TimerHandle_t xTimer
    */
    .align 4
    .global MPU_xTimerIsTimerActive
    .type MPU_xTimerIsTimerActive, function
    MPU_xTimerIsTimerActive:
        processorIsPrivileged
        BEQ     MPU_xTimerIsTimerActive_Privileged
            SVC     #SYSTEM_CALL_xTimerIsTimerActive
        MPU_xTimerIsTimerActive_Privileged:
            PUSH    { LR }
            BLX     MPU_xTimerIsTimerActiveImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTimerGetTimerDaemonTaskHandleImpl
    /**
     * Function: TaskHandle_t MPU_xTimerGetTimerDaemonTaskHandle
     * Inputs:     void - No Inputs
    */
    .align 4
    .global MPU_xTimerGetTimerDaemonTaskHandle
    .type MPU_xTimerGetTimerDaemonTaskHandle, function
    MPU_xTimerGetTimerDaemonTaskHandle:
        processorIsPrivileged
        BEQ     MPU_xTimerGetTimerDaemonTaskHandle_Privileged
            SVC     #SYSTEM_CALL_xTimerGetTimerDaemonTaskHandle
        MPU_xTimerGetTimerDaemonTaskHandle_Privileged:
            PUSH    { LR }
            BLX     MPU_xTimerGetTimerDaemonTaskHandleImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTimerGenericCommandFromTaskImpl
    /**
    * Function: TaskHandle_t MPU_xTimerGenericCommandFromTaskEntry
    * Inputs:     const xTimerGenericCommandParams_t * pxParams
    */
    .align 4
    .global MPU_xTimerGenericCommandFromTaskEntry
    .type MPU_xTimerGenericCommandFromTaskEntry, function
    MPU_xTimerGenericCommandFromTaskEntry:
        processorIsPrivileged
        BEQ     MPU_xTimerGenericCommandFromTask_Privileged
            SVC     #SYSTEM_CALL_xTimerGenericCommandFromTask
        MPU_xTimerGenericCommandFromTask_Privileged:
            PUSH    { LR }
            BLX     MPU_xTimerGenericCommandFromTaskImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/


    .extern MPU_pcTimerGetNameImpl
    /**
     * Function: const char * MPU_pcTimerGetName
     * Inputs:     TimerHandle_t xTimer
    */
    .align 4
    .global MPU_pcTimerGetName
    .type MPU_pcTimerGetName, function
    MPU_pcTimerGetName:
        processorIsPrivileged
        BEQ     MPU_pcTimerGetName_Privileged
            SVC     #SYSTEM_CALL_pcTimerGetName
        MPU_pcTimerGetName_Privileged:
            PUSH    { LR }
            BLX     MPU_pcTimerGetNameImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_vTimerSetReloadModeImpl
    /**
     * Function: void MPU_vTimerSetReloadMode
     * Inputs:     TimerHandle_t xTimer
     * Inputs:     constBaseType_t uxAutoReload
    */
    .align 4
    .global MPU_vTimerSetReloadMode
    .type MPU_vTimerSetReloadMode, function
    MPU_vTimerSetReloadMode:
        processorIsPrivileged
        BEQ     MPU_vTimerSetReloadMode_Privileged
            SVC     #SYSTEM_CALL_vTimerSetReloadMode
        MPU_vTimerSetReloadMode_Privileged:
            PUSH    { LR }
            BLX     MPU_vTimerSetReloadModeImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTimerGetReloadModeImpl
    /**
     * Function: TaskHandle_t MPU_xTimerGetReloadMode
     * Inputs:     TimerHandle_t xTimer
    */
    .align 4
    .global MPU_xTimerGetReloadMode
    .type MPU_xTimerGetReloadMode, function
    MPU_xTimerGetReloadMode:
        processorIsPrivileged
        BEQ     MPU_xTimerGetReloadMode_Privileged
            SVC     #SYSTEM_CALL_xTimerGetReloadMode
        MPU_xTimerGetReloadMode_Privileged:
            PUSH    { LR }
            BLX     MPU_xTimerGetReloadModeImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_uxTimerGetReloadModeImpl
    /**
     * Function: UBaseType_t MPU_uxTimerGetReloadMode
     * Inputs:     TimerHandle_t xTimer
    */
    .align 4
    .global MPU_uxTimerGetReloadMode
    .type MPU_uxTimerGetReloadMode, function
    MPU_uxTimerGetReloadMode:
        processorIsPrivileged
        BEQ     MPU_uxTimerGetReloadMode_Privileged
            SVC     #SYSTEM_CALL_uxTimerGetReloadMode
        MPU_uxTimerGetReloadMode_Privileged:
            PUSH    { LR }
            BLX     MPU_uxTimerGetReloadModeImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTimerGetPeriodImpl
    /**
     * Function: TickType_t MPU_xTimerGetPeriod
     * Inputs:     TimerHandle_t xTimer
    */
    .align 4
    .global MPU_xTimerGetPeriod
    .type MPU_xTimerGetPeriod, function
    MPU_xTimerGetPeriod:
        processorIsPrivileged
        BEQ     MPU_xTimerGetPeriod_Privileged
            SVC     #SYSTEM_CALL_xTimerGetPeriod
        MPU_xTimerGetPeriod_Privileged:
            PUSH    { LR }
            BLX     MPU_xTimerGetPeriodImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTimerGetExpiryTimeImpl
    /**
     * Function: TickType_t MPU_xTimerGetExpiryTime
     * Inputs:     TimerHandle_t xTimer
    */
    .align 4
    .global MPU_xTimerGetExpiryTime
    .type MPU_xTimerGetExpiryTime, function
    MPU_xTimerGetExpiryTime:
        processorIsPrivileged
        BEQ     MPU_xTimerGetExpiryTime_Privileged
            SVC     #SYSTEM_CALL_xTimerGetExpiryTime
        MPU_xTimerGetExpiryTime_Privileged:
            PUSH    { LR }
            BLX     MPU_xTimerGetExpiryTimeImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( configUSE_TIMERS == 1 ) */

#if configUSE_TASK_NOTIFICATIONS == 1
    .extern MPU_xTaskGenericNotifyImpl
    /**
    * Function: TaskHandle_t MPU_xTaskGenericNotifyEntry
    * Inputs:      const xTaskGenericNotifyParams_t * pxParams
    */
    .align 4
    .global MPU_xTaskGenericNotifyEntry
    .type MPU_xTaskGenericNotifyEntry, function
    MPU_xTaskGenericNotifyEntry:
        processorIsPrivileged
        BEQ     MPU_xTaskGenericNotify_Privileged
            SVC     #SYSTEM_CALL_xTaskGenericNotify
        MPU_xTaskGenericNotify_Privileged:
            PUSH    { LR }
            BLX     MPU_xTaskGenericNotifyImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTaskGenericNotifyWaitImpl
    /**
     * Function: TaskHandle_t MPU_xTaskGenericNotifyWaitEntry
     * Inputs:     const xTaskGenericNotifyWaitParams_t * pxParams
    */
    .align 4
    .global MPU_xTaskGenericNotifyWaitEntry
    .type MPU_xTaskGenericNotifyWaitEntry, function
    MPU_xTaskGenericNotifyWaitEntry:
        processorIsPrivileged
        BEQ     MPU_xTaskGenericNotifyWait_Privileged
            SVC     #SYSTEM_CALL_xTaskGenericNotifyWait
        MPU_xTaskGenericNotifyWait_Privileged:
            PUSH    { LR }
            BLX     MPU_xTaskGenericNotifyWaitImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_ulTaskGenericNotifyTakeImpl
    /**
     * Function: uint32_t MPU_ulTaskGenericNotifyTake
     * Inputs:     UBaseType_t uxIndexToWaitOn
     * Inputs:     TaskHandle_t xClearCountOnExit
     * Inputs:     TickType_t xTicksToWait
    */
    .align 4
    .global MPU_ulTaskGenericNotifyTake
    .type MPU_ulTaskGenericNotifyTake, function
    MPU_ulTaskGenericNotifyTake:
        processorIsPrivileged
        BEQ     MPU_ulTaskGenericNotifyTake_Privileged
            SVC     #SYSTEM_CALL_ulTaskGenericNotifyTake
        MPU_ulTaskGenericNotifyTake_Privileged:
            PUSH    { LR }
            BLX     MPU_ulTaskGenericNotifyTakeImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_xTaskGenericNotifyStateClearImpl
    /**
     * Function: TaskHandle_t MPU_xTaskGenericNotifyStateClear
     * Inputs:     TaskHandle_t xTask
     * Inputs:     UBaseType_t uxIndexToClear
    */
    .align 4
    .global MPU_xTaskGenericNotifyStateClear
    .type MPU_xTaskGenericNotifyStateClear, function
    MPU_xTaskGenericNotifyStateClear:
        processorIsPrivileged
        BEQ     MPU_xTaskGenericNotifyStateClear_Privileged
            SVC     #SYSTEM_CALL_xTaskGenericNotifyStateClear
        MPU_xTaskGenericNotifyStateClear_Privileged:
            PUSH    { LR }
            BLX     MPU_xTaskGenericNotifyStateClearImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

    .extern MPU_ulTaskGenericNotifyValueClearImpl
    /**
     * Function: uint32_t MPU_ulTaskGenericNotifyValueClear
     * Inputs:     TaskHandle_t xTask
     * Inputs:     UBaseType_t uxIndexToClear
     * Inputs:     uint32_t ulBitsToClear
    */
    .align 4
    .global MPU_ulTaskGenericNotifyValueClear
    .type MPU_ulTaskGenericNotifyValueClear, function
    MPU_ulTaskGenericNotifyValueClear:
        processorIsPrivileged
        BEQ     MPU_ulTaskGenericNotifyValueClear_Privileged
            SVC     #SYSTEM_CALL_ulTaskGenericNotifyValueClear
        MPU_ulTaskGenericNotifyValueClear_Privileged:
            PUSH    { LR }
            BLX     MPU_ulTaskGenericNotifyValueClearImpl
            POP     { PC }

    /*-----------------------------------------------------------------------*/

#endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
