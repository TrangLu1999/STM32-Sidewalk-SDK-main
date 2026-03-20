/**
  ******************************************************************************
  * @file    sid_stm32wba_sys_timer.h
  * @brief   System timer HAL for Sidewalk on STM32WBAxx MCUs
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

#ifndef __SID_STM32WBA_SYS_TIMER_H_
#define __SID_STM32WBA_SYS_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Sidewalk interfaces */
#include <sid_error.h>
#include <sid_time_types.h>

/* Platform interfaces */
#include <stm32_systime.h>

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize platform-specific system timer
 *
 * @retval SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_sys_timer_init(void);

/**
 * @brief De-nitialize platform-specific system timer
 *
 * @retval SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_sys_timer_deinit(void);

/**
 * @brief Get current system time since reset
 *
 * @param [out] result Storage for the time value
 * @retval SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_sys_timer_get(struct sid_timespec * const result);

/**
 * @brief Set system timer clock PPM to compensate for temperature drift
 *
 * @param [in] ppm PPM value to apply for the compensation
 */
void sid_stm32wba_sys_timer_set_xtal_ppm(int16_t ppm);

/**
 * @brief Get currently apple system clock drift PPM value
 *
 * @retval PPM value
 */
int16_t sid_stm32wba_sys_timer_get_xtal_ppm(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __SID_STM32WBA_SYS_TIMER_H_ */
