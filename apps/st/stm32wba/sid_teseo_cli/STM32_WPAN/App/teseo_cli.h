/**
  ******************************************************************************
  * @file    teseo_cli.h
  * @brief   CLI for driving Teseo GNSS receivers
  *
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

#ifndef __SID_STM32_TESEO_CLI_H_
#define __SID_STM32_TESEO_CLI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_error.h>

/* Exported functions --------------------------------------------------------*/

sid_error_t teseo_cli_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_TESEO_CLI_H_ */
