/**
  ******************************************************************************
  * @file    reset.c
  * @brief   MCU reset interface for Sidewalk SDK
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

/* Includes ------------------------------------------------------------------*/

#include <sid_hal_reset_ifc.h>
#include <sid_pal_log_ifc.h>
#include <stm32wbaxx_hal.h>
#include <stm32wbaxx_ll_rcc.h>

#include <sid_stm32_common_utils.h>

#ifdef AIM_OTA_SUPPORT
#  include "sid_application_install_manager_ifc.h"
#endif /* AIM_OTA_SUPPORT */

/* Global function definitions -----------------------------------------------*/

sid_error_t sid_hal_reset(sid_hal_reset_type_t type)
{
    sid_error_t err = SID_ERROR_GENERIC;

    switch (type)
    {
        case SID_HAL_RESET_NORMAL:
#ifdef HAL_ICACHE_MODULE_ENABLED
            /* Invalidate ICache to ensure all the potentially cached store operations (e.g., flash write) are completed */
            (void)HAL_ICACHE_Invalidate();
#endif /* HAL_ICACHE_MODULE_ENABLED */
            if (SID_STM32_UTIL_IS_IRQ() == FALSE)
            {
                /* Ensure all log messages are printed out */
                SID_PAL_LOG_FLUSH();
            }
            /* Clear reset flags in RCC to ensure the reset reason can be reliably detected on the next boot */
            LL_RCC_ClearResetFlags();
            /* Initiate software reset */
            HAL_NVIC_SystemReset();
            /* No return from here */
            break;

        case SID_HAL_RESET_DFU:
#ifdef AIM_OTA_SUPPORT
#  ifdef HAL_ICACHE_MODULE_ENABLED
            /* Invalidate ICache to ensure all the potentially cached store operations (e.g., flash write) are completed */
            (void)HAL_ICACHE_Invalidate();
#  endif /* HAL_ICACHE_MODULE_ENABLED */
            if (SID_STM32_UTIL_IS_IRQ() == FALSE)
            {
                /* Ensure all log messages are printed out */
                SID_PAL_LOG_FLUSH();
            }
            /* Clear reset flags in RCC to ensure the reset reason can be reliably detected on the next boot */
            LL_RCC_ClearResetFlags();
            /* Initiate software reset */
            sid_aim_ifc_boot_in_dfu_mode();
            /* No return from here */
#else
            err = SID_ERROR_NOSUPPORT;
#endif /* AIM_OTA_SUPPORT */
            break;

        default:
            err = SID_ERROR_INVALID_ARGS;
            break;
    }

    return err;
}
