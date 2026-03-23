/**
  ******************************************************************************
  * @file    app_sidewalk.h
  * @brief   Sidewalk SubGHz link sample app implementation
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_SIDEWALK_H_
#define __APP_SIDEWALK_H_

/* Global function prototypes ------------------------------------------------*/
void SID_APP_Init(void);
void SID_APP_StandbyExit(void);
void app_sidewalk_forward_uart2_data(const uint8_t *data, uint8_t len);

#endif /*__APP_SIDEWALK_H_ */
