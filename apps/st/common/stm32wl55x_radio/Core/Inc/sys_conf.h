/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sys_conf.h
  * @author  MCD Application Team
  * @brief   Applicative configuration, e.g. : debug, trace, low power, sensors
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYS_CONF_H__
#define __SYS_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/**
  * @brief Temperature and pressure values are retrieved from sensors shield
  *        (instead of sending dummy values). It requires MEMS IKS shield
  */
#define SENSOR_ENABLED                       0

/**
  * @brief  Verbose level for all trace logs
  */
#define VERBOSE_LEVEL                        VLEVEL_M

/**
  * @brief Enable trace logs
  */
#define APP_LOG_ENABLED                      1

/**
  * @brief Activate monitoring (probes) of some internal RF signals for debug purpose
  */
#define DEBUG_SUBGHZSPI_MONITORING_ENABLED   0

#define DEBUG_RF_NRESET_ENABLED              0

#define DEBUG_RF_HSE32RDY_ENABLED            0

#define DEBUG_RF_SMPSRDY_ENABLED             0

#define DEBUG_RF_LDORDY_ENABLED              0

#define DEBUG_RF_DTB1_ENABLED                0

#define DEBUG_RF_BUSY_ENABLED                0

/**
  * @brief Enable/Disable MCU Debugger pins (dbg serial wires)
  * @note  by HW serial wires are ON by default, need to put them OFF to save power
  */
#ifndef DEBUGGER_ENABLED
#  define DEBUGGER_ENABLED                   0
#endif /* DEBUGGER_ENABLED */

/**
  * @brief Disable Low Power mode
  * @note  0: LowPowerMode enabled. MCU enters stop1 mode, 1: LowPowerMode disabled. MCU enters sleep mode only
  */
#ifndef LOW_POWER_DISABLE
#  define LOW_POWER_DISABLE                  0
#endif /* LOW_POWER_DISABLE */

/* Defines time to wake up from standby before radio event to meet timings */
#define CFG_LPM_STDBY_WAKEUP_TIME          (1000u)

/* Wait time (in microseconds) right after the HSE startup to allow the oscillator frequency to stabilize */
#define CFG_LPM_HSE_STABILIZATION_DELAY_US (200u)

/* Resume time (in microseconds) for RTOS to perform post-wakeup actions */
#define CFG_LPM_RTOS_RESUME_DELAY_US       (150u)

/* Defines time to wake up from Sleep LPM before radio event to meet timings */
#define CFG_LPM_SLEEP_WAKEUP_DELAY_US      (CFG_LPM_RTOS_RESUME_DELAY_US)

/* Defines time to wake up from Stop LPM before radio event to meet timings */
#define CFG_LPM_STOP_WAKEUP_DELAY_US       ((CFG_LPM_HSE_STABILIZATION_DELAY_US) + (CFG_LPM_RTOS_RESUME_DELAY_US))

/* The minimum LPM duration (in microseconds) for which using the Stop mode makes sense. Any shorted idle time will result in spending more energy on wake from Stop than it would be saved by residing in Stop mode */
#define CFG_LPM_STOP_MIN_DURATION_US       (700u)

/* The minimum LPM duration (in milliseconds) for which using the Standby mode makes sense. Any shorted idle time will result in spending more energy on wake from Standby than it would be saved by residing in Standby mode */
#define CFG_LPM_STDBY_MIN_DURATION_MS      (350u)

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __SYS_CONF_H__ */
