/**
  ******************************************************************************
  * @file    exception_handlers.c
  * @brief   Custom HardFault handler implementation for advanced diagnostics
  *          and error reporting.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "sys_conf.h"
#include "common_memory_symbols.h"
#include "main.h"
#include "utilities_conf.h"

#include "sid_pal_log_like.h"
#include <sid_stm32_common_utils.h>

#include <cmsis_compiler.h>

#include "stm32wlxx.h"
#include "stm32wlxx_hal.h"
#include "stm32_adv_trace.h"
#if defined(NUCLEO_WL55_BOARD)
#  include "stm32wlxx_nucleo.h"
#endif

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief Stack frame register indices for analysis
 */
enum {
    STACK_R0  = 0,
    STACK_R1  = 1,
    STACK_R2  = 2,
    STACK_R3  = 3,
    STACK_R12 = 4,
    STACK_LR  = 5,
    STACK_PC  = 6,
    STACK_PSR = 7,
    STACK_MAX = 8, /* Total number of registers */
};

/**
 * @brief System Control Block (SCB) fault register indices
 */
enum {
    REG_HFSR = 0, /*!< Hard Fault Status Register */
    REG_CFSR = 1, /*!< Configurable Fault Status Register */
    REG_MAX  = 2, /* Total number of registers */
};

/* Private defines -----------------------------------------------------------*/

#define EXHA_HARD_FAULT_INFO_VALID_KEY1 ((uint32_t)0xDEADBEEFu)
#define EXHA_HARD_FAULT_INFO_VALID_KEY2 ((uint32_t)0x82DFA017u)

#ifndef EXHA_LOGS_PRINTOUT_TIMEOUT_MS
#  define EXHA_LOGS_PRINTOUT_TIMEOUT_MS (500u) /*!< Time limit (in milliseconds) to flush the loging UART buffer before proceeding with error reaction */
#endif /* EXHA_LOGS_PRINTOUT_TIMEOUT_MS */

/* Private macro -------------------------------------------------------------*/

#define EXHA_INVALIDATE_HARDFAULT_INFO_VALIDITY()   do {\
                                                        hard_fault_info_validity_markers[0] = 0u;\
                                                        hard_fault_info_validity_markers[1] = 0u;\
                                                    } while (0)

/* Private variables ---------------------------------------------------------*/

/**
 * @brief Global variable to store the stack frame during a Hard Fault
 */
UTIL_MEM_PLACE_IN_SECTION(".no_init")
static uint32_t hard_fault_stack_frame[STACK_MAX];

/**
 * @brief Global variable to store relevant SCB fault registers
 */
UTIL_MEM_PLACE_IN_SECTION(".no_init")
static uint32_t hard_fault_status_regs[REG_MAX];

UTIL_MEM_PLACE_IN_SECTION(".no_init")
static uint32_t hard_fault_info_validity_markers[2];

/* Private constants ---------------------------------------------------------*/

/**
 * @brief Addresses of SCB fault registers for simplified access
 */
static volatile uint32_t const * const regsAddr[REG_MAX] = {
    &(SCB->HFSR), /*!< HFSR */
    &(SCB->CFSR), /*!< CFSR */
};

/* Private function prototypes -----------------------------------------------*/

#if APP_LOG_ENABLED
/**
 * @brief Invokes an ISR handler based on the supplied IRQ number
 * @param irq_num IRQ number in NVIC
 */
static void _call_irq_handler_by_number(const int32_t irq_num);

/**
 * @brief Ensure all pending logs are printed out
 */
static inline void _flush_log_buffer(void);
#endif /* APP_LOG_ENABLED */

/**
 * @brief Dumps the stack frame to the logging interface
 * @param stack Pointer to the stack frame
 */
static inline void _print_stack_frame(const uint32_t * const stack);

#if defined __CORTEX_M && (__CORTEX_M == 4U)
/**
 * @brief Provides a detailed fault report over the logging interface
 * @param hard_fault_status_regs Pointer to the array containing fault register values
 */
static inline void _print_hardfault_report(const uint32_t * const hard_fault_status_regs);
#endif /* defined __CORTEX_M && (__CORTEX_M == 4U) */

/* Private function definitions ----------------------------------------------*/

#if APP_LOG_ENABLED
static void _call_irq_handler_by_number(const int32_t irq_num)
{
    if (irq_num < 0)
    {
        /* Negative IRQn is a system exception and it's not supposed to be handled here */
        return;
    }

    /* Get ISR function address from the vector table */
#if !defined(__VTOR_PRESENT) || (__VTOR_PRESENT == 0u)
#  error "This code relies on VTOR register to get the base address of the ISR vector table, but VTOR is not implemented on the target MCU"
#endif /* !defined(__VTOR_PRESENT) || (__VTOR_PRESENT == 0u) */
    const uint32_t * const vect_table = (uint32_t *)SCB->VTOR;
    const uint32_t         isr_index  = 16u + (uint32_t)irq_num;
    const uint32_t         isr_entry  = vect_table[isr_index];

    /* Sanity checks */
    if ((isr_entry < APP_FLASH_START) /* Entry is located before the code area in the flash */
     || (isr_entry >= APP_FLASH_END)  /* Entry is located past the code area in the flash */
     || ((isr_entry & 0x01u) == 0u))  /* Vector table entries for Thumb functions normally have LSB == 1 */
    {
        /* ISR address does not look valid */
        return;
    }

    /* Cast to a no-arg ISR handler function pointer */
    void (*irq_handler)(void) = (void (*)(void))(uintptr_t)isr_entry;

    /* Call the ISR handler function */
    irq_handler();
}
#endif /* APP_LOG_ENABLED */

/*----------------------------------------------------------------------------*/

#if APP_LOG_ENABLED
static inline void _flush_log_buffer(void)
{
    /**
     * IMPORTANT NOTE: using HAL_GetTick() works here because HAL tick in the reference implementation is sourced from RTC and does not require
     * an IRQ to be incremented. If you switch HAL Tick to any other source that needs an IRQ (e.g., TIM timer), you need to modify this code
     * to either use RTC or artificially call an ISR to handle HAL Tick increments.
     */
    const uint32_t start_timestamp = HAL_GetTick();

    while ((UTIL_ADV_TRACE_IsBufferEmpty() == 0u) && ((HAL_GetTick() - start_timestamp) < EXHA_LOGS_PRINTOUT_TIMEOUT_MS))
    {
        /* Buffer is not empty, meaning a transfer is ongoing. Wait for UART or DMA IRQ to be indicated */
        while ((HAL_NVIC_GetPendingIRQ(APP_LOG_DMA_TX_CHANNEL_IRQn) == 0u)
            && (HAL_NVIC_GetPendingIRQ(APP_LOG_UART_IRQn) == 0u)
            && ((HAL_GetTick() - start_timestamp) < EXHA_LOGS_PRINTOUT_TIMEOUT_MS))
        {
            __NOP();
        }

        /* Artificially call DMA ISR if a DMA IRQ is pending */
        if (HAL_NVIC_GetPendingIRQ(APP_LOG_DMA_TX_CHANNEL_IRQn) != 0u)
        {
            _call_irq_handler_by_number(APP_LOG_DMA_TX_CHANNEL_IRQn);
        }

        /* Artificially call UART ISR if a UART IRQ is pending */
        if (HAL_NVIC_GetPendingIRQ(APP_LOG_UART_IRQn) != 0u)
        {
            _call_irq_handler_by_number(APP_LOG_UART_IRQn);
        }
    }
}
#endif /* APP_LOG_ENABLED */

/*----------------------------------------------------------------------------*/

static inline void _print_stack_frame(const uint32_t * const stack)
{
    if (NULL == stack)
    {
        return;
    }

    SID_PAL_LOG_ERROR("------ STACK FRAME DUMP ------");
    SID_PAL_LOG_ERROR("r0  = 0x%08x", stack[STACK_R0]);
    SID_PAL_LOG_ERROR("r1  = 0x%08x", stack[STACK_R1]);
    SID_PAL_LOG_ERROR("r2  = 0x%08x", stack[STACK_R2]);
    SID_PAL_LOG_ERROR("r3  = 0x%08x", stack[STACK_R3]);
    SID_PAL_LOG_ERROR("r12 = 0x%08x", stack[STACK_R12]);
    SID_PAL_LOG_ERROR("lr  = 0x%08x", stack[STACK_LR]);
    SID_PAL_LOG_ERROR("pc  = 0x%08x", stack[STACK_PC]);
    SID_PAL_LOG_ERROR("psr = 0x%08x", stack[STACK_PSR]);
    SID_PAL_LOG_ERROR("------------------------------");
}

/*----------------------------------------------------------------------------*/

#if defined __CORTEX_M && (__CORTEX_M == 4U)
static inline void _print_hardfault_report(const uint32_t * const hard_fault_status_regs)
{
    if(NULL == hard_fault_status_regs)
    {
        return;
    }

    SID_PAL_LOG_ERROR("------- FAULT DETAILS --------");

    SID_PAL_LOG_ERROR("SCB->HFSR = 0x%08x", hard_fault_status_regs[REG_HFSR]);

    /* Print if Debug Event flag is set */
    if ((hard_fault_status_regs[REG_HFSR] & SCB_HFSR_DEBUGEVT_Msk) != 0u)
    {
        SID_PAL_LOG_ERROR("    Debug event");
    }

    /* Checks if Hard Fault is caused by a forced exception */
    if ((hard_fault_status_regs[REG_HFSR] & SCB_HFSR_FORCED_Msk) != 0u)
    {
        SID_PAL_LOG_ERROR("    Forced HardFault");

        SID_PAL_LOG_ERROR("SCB->CFSR = 0x%08x", hard_fault_status_regs[REG_CFSR]);

        /* Check usage fault status */
        if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_USGFAULTSR_Msk) != 0u)
        {
            SID_PAL_LOG_ERROR("    Usage fault: ");

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_DIVBYZERO_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Div by zero");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_UNALIGNED_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Unaligned access");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_NOCP_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        No coprocessor");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_INVPC_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Exception return with a bad value in the EXC_RETURN number");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_INVSTATE_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        An attempt to switch to an invalid state");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_UNDEFINSTR_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Undefined instruction");
            }
        }

        /* Check Bus Fault Status */
        if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_BUSFAULTSR_Msk) != 0u)
        {
            SID_PAL_LOG_ERROR("    Bus Fault: ");

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_BFARVALID_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Bus Fault Address Valid");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_LSPERR_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Floating-Point lazy stacking error");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_STKERR_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Stack fault");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_UNSTKERR_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Unstacking error");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_PRECISERR_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Precise data access error");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_IBUSERR_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Instruction access error");
            }
        }

        /* Check memory fault status */
        if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_MEMFAULTSR_Msk) != 0u)
        {
            SID_PAL_LOG_ERROR("    Memory Fault: ");

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_MMARVALID_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Memory Fault Address Valid");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_MLSPERR_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Floating-Point lazy memory error");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_MSTKERR_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Memory stack fault");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_MUNSTKERR_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Memory unstacking error");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_DACCVIOL_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Data access violation");
            }

            if ((hard_fault_status_regs[REG_CFSR] & SCB_CFSR_IACCVIOL_Msk) != 0u)
            {
                SID_PAL_LOG_ERROR("        Instruction access violation");
            }
        }
    }

    SID_PAL_LOG_ERROR("------------------------------");
}
#endif /* defined __CORTEX_M && (__CORTEX_M == 4U) */

/* Global function definitions -----------------------------------------------*/

void Fatal_Error_Report_Handler(void)
{
    /* Temporarily disable IRQs to avoid interruptions of this routine */
    UTILS_ENTER_CRITICAL_SECTION();

    if ((EXHA_HARD_FAULT_INFO_VALID_KEY1 == hard_fault_info_validity_markers[0]) && (EXHA_HARD_FAULT_INFO_VALID_KEY2 == hard_fault_info_validity_markers[1]))
    {
        /* Invalidate HardFault info validity to avoid printout loops */
        EXHA_INVALIDATE_HARDFAULT_INFO_VALIDITY();

        if (hard_fault_status_regs[REG_HFSR] == SCB_HFSR_DEBUGEVT_Msk)
        {
            /* If only the DEBUGEVT is present it's not actually a fault, but rather normal debugger operation - do nothing */
        }
        else
        {
            /* Print out HardFault info */
            SID_PAL_LOG_ERROR("Hard Fault occurred");

            /* Dump the stack frame */
            _print_stack_frame(hard_fault_stack_frame);

#if defined __CORTEX_M && (__CORTEX_M == 4U)
            /* Reporting the fault reason */
            _print_hardfault_report(hard_fault_status_regs);
#endif /* defined __CORTEX_M && (__CORTEX_M == 4U) */

            /* Point of no return - call app's error handler to finalize the error processing */
            Error_Handler();
        }
    }

    /* Re-enable interrupts before returning */
    UTILS_EXIT_CRITICAL_SECTION();
}

/*----------------------------------------------------------------------------*/

/**
 * @brief  Handles the Hard Fault interrupt.
 * @details  Captures the stack frame, system fault registers, disables interrupts,
 *           and passes control for post-fault actions.
 * @note     This is the primary entry point for Hard Fault exceptions. This function
 *           is naked to prevent the compiler from optimizing the code and inserting
 *           epilogue and prologue instructions that would modify the stack pointer.
 */
__NO_RETURN SID_STM32_STACKLESS_FUNCTION void HardFault_Handler(void)
{
    /* Check if the microcontroller is Cortex-M4 */
#if defined __CORTEX_M && ((__CORTEX_M == 4U) || (__CORTEX_M == 0U))
    /* This code saves the stack frame to a local variable */
    __ASM volatile(
            "TST lr, #4           \n"  // Test SPSEL bit in EXC_RETURN value to find out stack mode
            "ITE eq               \n"  // Use ITE (If-Then-Else) instruction for conditional execution
            "MRSEQ R8, msp        \n"  // Move Main Stack Pointer to R8 if stack is in thread mode
            "MRSNE R8, psp        \n"  // Move Process Stack Pointer to R8 if stack is in handler mode
            "LDMIA R8!, {r0-r7}   \n"  // Load multiple registers from the stack address in R8, then increment R8
            "STMIA %0!, {r0-r7}   \n"  // Store multiple registers
            :
            : "r" (hard_fault_stack_frame)
              : "r0", "r1", "r2", "r3","r4","r5","r6","r7","r8", "memory"
    );

    /* Saves the fault registers to a local variable */
    __ASM volatile(
            "MOV R0, %0       \n"   // Move the destination address to R1
            "MOV R1, %1       \n"   // Load the source address register
            "MOV R2, %2       \n"   // Load the quantity
            "copy_loop:       \n"
            "LDR R3, [R1], #4 \n"   // Load the value from the source address and increment source address
            "LDR R3, [R3]     \n"   // Dereference the value at the address loaded from source
            "STR R3, [%0], #4 \n"   // Store the dereferenced value to the destination address and increment destination address
            "SUBS R2, R2, #1  \n"   // Decrement counter
            "BNE copy_loop    \n"   // Branch back to the beginning if the loop counter is not zero
            :
            : "r" (hard_fault_status_regs), "r" (regsAddr), "r" (REG_MAX)
              : "r0", "r1", "r2", "r3", "memory"
    );
#else
#warning "Hard Fault handler code is not implemented for this MCU."
#endif /* #if defined __CORTEX_M && ((__CORTEX_M == 4U) || (__CORTEX_M == 0U)) */

    /**
     * IMPORTANT NOTE: HardFault means a critical exception and system integrity is not guaranteed at this point. Due to this
     * it's not safe to let the UART logs to be printed out in full before proceeding. The below log flushing code is for
     * debugging purposes only and shall not be used in production.
     */
#if defined(DEBUG) && (APP_LOG_ENABLED != 0)
#  warning "It's not safe to let the UART logs to be printed out in HardFault handler. DO NOT USE THIS FUNCTIONALITY IN RELEASE BUILDS. Feel free to ignore this warning if log buffer flush is used intentionally"
    /* Ensure all the logs are printed out */
    _flush_log_buffer();
#endif /* defined(DEBUG) && (APP_LOG_ENABLED != 0) */

    /* Since HardFault is generally considered non-recoverable the best option is to reset the MCU from here */
    hard_fault_info_validity_markers[0] = EXHA_HARD_FAULT_INFO_VALID_KEY1; /* Set exception info validity marker */
    hard_fault_info_validity_markers[1] = EXHA_HARD_FAULT_INFO_VALID_KEY2; /* Set exception info validity marker */
    NVIC_SystemReset(); /* Trigger MCU reset, exception info will be printed out on boot */
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
__NO_RETURN void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */

    /* Ensure Error_Handler won't be interrupted */
    __disable_irq();

    /* Provide visual error indication if possible */
#if CFG_LED_SUPPORTED
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_BLUE);
    BSP_LED_On(LED_RED);
#endif /* CFG_LED_SUPPORTED */

#if APP_LOG_ENABLED
#  ifdef DEBUG
    SID_PAL_LOG_ERROR("\e[1;31m" "FATAL ERROR. System halted" "\e[0m");
#  else
    SID_PAL_LOG_ERROR("\e[1;31m" "FATAL ERROR. System will be reset" "\e[0m");
#  endif /* DEBUG */

    /* Ensure all the logs are printed out */
    _flush_log_buffer();
#endif /* APP_LOG_ENABLED */

#ifdef DEBUG
    /* Halt the system */
    while (1)
    {
        __NOP();
    }
#else
    /* For release builds reset the MCU to recover from fault */
    NVIC_SystemReset();
    //TODO: consider counting consecutive resets triggered from here and halting the MCU at some threshold to avoid infinite reset loop
#endif /* DEBUG */
    /* USER CODE END Error_Handler_Debug */
}
