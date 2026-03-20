/**
  ******************************************************************************
  * @file           : delay.c
  * @brief          : Implementation of the Sidewalk's sid_pal_delay module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Private macro -------------------------------------------------------------*/

#if defined(STM32WBA50xx) || defined(STM32WBA52xx) || defined(STM32WBA54xx) || defined(STM32WBA55xx) || defined(STM32WBA5Mxx) || \
    defined(STM32WBA62xx) || defined(STM32WBA63xx) || defined(STM32WBA64xx) || defined(STM32WBA65xx) || defined(STM32WBA6Mxx)
#  define STM32WBAxx_FAMILY
#elif defined(STM32WL55xx) || defined(STM32WL54xx) || defined(STM32WLE5xx) || defined(STM32WLE4xx) || defined(STM32WL5Mxx)
#  define STM32WLxx_FAMILY
#  else
#    error "Unable to identify the target STM32 MCU. Please check if you have specified the MCU type in compile definitions and that your selected MCU is supported"
#endif

#ifndef SID_PAL_DELAY_DYNAMIC_CLOCK_CHANGE_SUPPORT
#  define SID_PAL_DELAY_DYNAMIC_CLOCK_CHANGE_SUPPORT (0)
#endif /* SID_PAL_DELAY_DYNAMIC_CLOCK_CHANGE_SUPPORT */

/* Includes ------------------------------------------------------------------*/

#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
#  include <cmsis_os.h>
#endif

#if defined(STM32WBAxx_FAMILY)
#  include "stm32wbaxx.h"
#elif defined(STM32WLxx_FAMILY)
#  include "stm32wlxx.h"
#endif

#include <sid_stm32_common_utils.h>

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
#  define SID_PAL_DELAY_CALC_SYSTICKS_PER_MS(__SYSTICK_PERIOD_TICKS__)           ((((__SYSTICK_PERIOD_TICKS__) * osKernelGetTickFreq()) + 500u) / 1000u)
#  define SID_PAL_DELAY_CALC_DELAY_TICKS(__SYSTICK_PERIOD_TICKS__, __DELAY_US__) (((SID_PAL_DELAY_CALC_SYSTICKS_PER_MS(__SYSTICK_PERIOD_TICKS__) * (__DELAY_US__)) + 500u) / 1000u)
#else
#  define SID_PAL_DELAY_CALC_DELAY_TICKS(__SYSTICK_PERIOD_TICKS__, __DELAY_US__) ((((__SYSTICK_PERIOD_TICKS__) * (__DELAY_US__)) + 500u) / 1000u)
#endif /* SID_PAL_ENABLE_SCHEDULER_DELAY */

/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function definitions ----------------------------------------------*/

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_pal_delay_us(uint32_t delay)
{
    /* Get the current timestamp ASAP - this is crucial for the short delays */
    const register uint32_t ticks_start = READ_REG(SysTick->VAL);
    /* Ensure ticks_start is captured before any further actions */
    __COMPILER_BARRIER();

    /**
     * SysTick is configred to trigger an interrupt each RTOS kernel tick (or 1 ms if used as HAL Tick
     * timebase). LOAD register contains the reload number of ticks and counts till zero thus we need to
     * add one tick to determine the timer period in ticks
     *
     * WARNING: Even though this implementation supports SysTick clock frequency alterations on the fly,
     *          it is still recommended to avoid such situations and call sid_pal_delay_us() only after
     *          the system clock reaches a stable state. For example, if your system uses low power mode
     *          the system clock (and SysTick subsequently) is typically stopped in LPM and restarted on
     *          wakeup. The recommendation is to ensure clock configuration is fully restored on wake up
     *          (XTAL/PLL is started, locked, and system clock switch to XTAL/PLL completed) before the
     *          application code resumes. However, in certain circumstances, like switching from PLL to
     *          XTAL to save power when full performance is not needed, the SysTick frequency change is
     *          unavoidable.
     */
    register uint32_t systick_load_reg_effective = READ_REG(SysTick->LOAD);
    register uint32_t systick_period_ticks = (systick_load_reg_effective + 1u); /* systick_period_ticks is the number of ticks per one RTOS kernel tick (or per 1ms if SysTick is used as HAL Tick timebase */

    /* Convert the requested delay into SysTick ticks */
    register uint32_t delay_ticks = SID_PAL_DELAY_CALC_DELAY_TICKS(systick_period_ticks, delay);
    register uint32_t elapsed = 0u;

    /* Now delay for the specified amount of ticks */
    register uint32_t ticks_prev = ticks_start;
    do
    {
        /* Get the current reading of the SysTick timer */
        const register uint32_t ticks_now = READ_REG(SysTick->VAL);

        /* SysTick counts downwards, if the current reading is higher that the ticks_prev it means an underflow has happened */
        if (ticks_now > ticks_prev)
        {
            elapsed += (ticks_prev + (systick_period_ticks - ticks_now));
        }
        else
        {
            elapsed += (ticks_prev - ticks_now);
        }

        /* Update ticks_prev for the next iteration */
        ticks_prev = ticks_now;

#if SID_PAL_DELAY_DYNAMIC_CLOCK_CHANGE_SUPPORT
        /* Monitor for SysTick clock frequency changes */
        register uint32_t systick_load_reg_current = READ_REG(SysTick->LOAD); /* Store to a local variable because LOAD is volatile */
        if (systick_load_reg_effective != systick_load_reg_current)
        {
            /* SysTick frequency has changed, we must recalculate wait time */

            systick_load_reg_effective                 = systick_load_reg_current;      /* Store new LOAD register value for which delay calculations are made */
            register uint32_t new_systick_period_ticks = systick_load_reg_current + 1u; /* Compute new SyTick timer period */

            delay_ticks = ((delay_ticks * new_systick_period_ticks) + (systick_period_ticks >> 1)) / systick_period_ticks; /* Recalculate expected delay time in ticks */
            elapsed     = ((elapsed * new_systick_period_ticks) + (systick_period_ticks >> 1)) / systick_period_ticks;     /* Convert elapsed time */

            /* Store new period */
            systick_period_ticks = new_systick_period_ticks;
        }
#endif /* SID_PAL_DELAY_DYNAMIC_CLOCK_CHANGE_SUPPORT */
    } while (elapsed < delay_ticks);
}

/*----------------------------------------------------------------------------*/

#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
SID_STM32_SPEED_OPTIMIZED void sid_pal_scheduler_delay_ms(uint32_t delay)
{
    const uint32_t tick_freq = osKernelGetTickFreq();
    const uint32_t delay_ticks = (uint32_t)(((uint64_t)delay * (uint64_t)tick_freq) / (uint64_t)1000u) + 1u; /* Add 1 OS tick since osDelay() provides a delay somewhere in [(delay_ticks - 1)...delay_ticks] range - due to that one more tick is needed to guarantee that the actual delay will be not less than requested */

    const osStatus_t ret = osDelay(delay_ticks);

    /* Protect from a systematic SW failure - calling sid_pal_scheduler_delay_ms from IRQ context */
    assert_param(osOK == ret);
    (void)ret;
}
#endif
