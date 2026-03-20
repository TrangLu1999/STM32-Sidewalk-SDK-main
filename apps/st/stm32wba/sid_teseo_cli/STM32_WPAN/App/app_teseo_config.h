/**
  ******************************************************************************
  * @file    app_teseo_config.h
  * @brief   Teseo GNSS receiver configuration for Sidewalk application
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

#ifndef __APP_TESEO_CONFIG_H_
#define __APP_TESEO_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <teseo_gnss_config.h>

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Returns a pointer to the Teseo hardware configuration
 */
const teseo_gnss_device_config_t * get_teseo_cfg(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __APP_900_CONFIG_H_ */
