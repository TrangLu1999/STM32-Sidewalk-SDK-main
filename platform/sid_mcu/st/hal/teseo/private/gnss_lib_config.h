/**
  ******************************************************************************
  * @file    gnss_lib_config.h
  * @brief   Header file for gnss_lib_config.c
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

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __STM32_SID_TESEO_GNSS_LIB_CONFIG_H_
#define __STM32_SID_TESEO_GNSS_LIB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>

#include <sid_pal_log_ifc.h>
#include <cmsis_compiler.h>

/* Exported constants --------------------------------------------------------*/

#define OS_Delay             GNSS_Wrapper_Delay

/* Exported macro ------------------------------------------------------------*/

#if (GNSS_DEBUG != 0)
#  define PRINT_DBG(pBuffer) teseo_gnss_print(pBuffer, SID_PAL_LOG_SEVERITY_DEBUG)
#else
#  define PRINT_DBG(pBuffer)
#endif

#define PRINT_INFO(pBuffer)  teseo_gnss_print(pBuffer, SID_PAL_LOG_SEVERITY_INFO)

/* Exported functions prototypes ---------------------------------------------*/

/** @addtogroup MIDDLEWARES
 *  @{
 */

/** @addtogroup ST
 *  @{
 */

/** @addtogroup LIB_GNSS
 *  @{
 */

/** @addtogroup LibGNSS
 *  @{
 */

/** @defgroup GNSS_DATA_FUNCTIONS GNSS DATA FUNCTIONS
 *  @brief Prototypes of the API allowing the application to interface the driver
 *  and interact with GNSS module (sending commands, retrieving parsed NMEA info, etc.).
 *  The implementation is up to the application according to specific needs.
 *  @{
 */

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief  This function puts a string on the console (via UART).
 * @param  pBuffer The string that contains the data to be written on the console
 * @retval None
 */
int32_t GNSS_Wrapper_Send(uint8_t * buffer, uint16_t length);
int32_t GNSS_Wrapper_Reset(void);
void    GNSS_Wrapper_Delay(uint32_t Delay);
int     GNSS_PRINT(char * pBuffer);
int     teseo_gnss_print(const char * const pBuffer, const sid_pal_log_severity_t severity);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __STM32_SID_TESEO_GNSS_LIB_CONFIG_H_ */
