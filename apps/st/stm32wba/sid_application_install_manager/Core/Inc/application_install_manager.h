/**
  ******************************************************************************
  * @file    application_install_manager.h
  * @brief   Application Install Manager for Sidewalk applications
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

#ifndef __SID_APPLICATION_INSTALL_MANAGER_H_
#define __SID_APPLICATION_INSTALL_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void AIM_BootUserApp(void);
void AIM_Init(void);
void AIM_Entry(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __SID_APPLICATION_INSTALL_MANAGER_H_ */
