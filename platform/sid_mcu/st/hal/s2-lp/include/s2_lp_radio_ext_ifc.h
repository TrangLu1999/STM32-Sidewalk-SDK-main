/**
  ******************************************************************************
  * @file    s2_lp_radio_ext_ifc.h
  * @brief   Extended API for the S2-LP Sidewalk radio driver
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

#ifndef __SID_S2_LP_RADIO_EXT_IFC_H_
#define __SID_S2_LP_RADIO_EXT_IFC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_radio_ifc.h>
#include "s2_lp_radio.h"

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Perform the actions required after the host MCU leaves Standby low power mode (e.g., restore GPIO config)
 */
void sid_pal_radio_notify_host_standby_exit(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __SID_S2_LP_RADIO_EXT_IFC_H_ */
