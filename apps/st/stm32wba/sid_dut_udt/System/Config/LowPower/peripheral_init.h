/**
  ******************************************************************************
  * @file    peripheral_init.h
  * @brief   Peripheral re-initialization handling on leaving Standby LPM
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

#ifndef __PERIPHERAL_INIT_H_
#define __PERIPHERAL_INIT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Configure the SoC peripherals at Standby mode exit.
  * @param  None
  * @retval None
  */
void MX_StandbyExit_PeripheralInit(void);

/**
  * @brief  Backup the SoC peripheral states before entring Standby mode.
  * @param  None
  * @retval None
  */
void MX_StandbyEntry_PeripheralBackup(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __PERIPHERAL_INIT_H_ */
