/**
  ******************************************************************************
  * @file    app_ble_config.c
  * @brief   BLE radio configuration for Sidewalk application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_ble_adapter_stm32wba_config.h>
#include <sid_pal_ble_adapter_stm32wba_ext_ifc.h>
#include <sid_stm32_common_utils.h>

#include <app_conf.h>
#include <ble_defs.h>

#include "app_ble_config.h"

/* Private defines -----------------------------------------------------------*/

#if (SID_STM32_BLE_COEXISTENCE_MODE != SID_STM32_BLE_COEXISTENCE_MODE_NONE)
#  error "This configuration is not designed to be used in BLE coexistence applications"
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

#define APP_BLE_CFG_DEFAULT_TX_POWER            (SID_STM32_BLE_RADIO_MAX_TX_POWER_EU)  /*!< Default targeted EIRP. This vaue can later be adjusted by the application if required */
#define APP_BLE_CFG_NUCLEO_ANTENNA_GAIN         SID_STM32_BLE_POWER_DBM_TO_FIXED(1.95) /*!< Gain of the PCB antenna on the NUCLEO-WBAxx board */
#define APP_BLE_CFG_NUCLEO_RF_PATH_LOSS         SID_STM32_BLE_POWER_DBM_TO_FIXED(1.45) /*!< Estimated signal loss for the matching network on the NUCLEO-WBAxx boards */
#define APP_BLE_CFG_RADIO_RX_LNA_GAIN           SID_STM32_BLE_POWER_DBM_TO_FIXED(0)    /*!< External LNA gain in dB. The reference projects do no use external LNA, but feel free to adjust this value if your PCB has the LNA for 2.4GHz radio */
#define APP_BLE_CFG_RADIO_TX_EXT_PA_GAIN        SID_STM32_BLE_POWER_DBM_TO_FIXED(0)    /*!< External PA gain in dB. The reference projects do no use external PA, but feel free to adjust this value if your PCB has the external PA for 2.4GHz radio */

/* Private constants ---------------------------------------------------------*/

static const sid_ble_adapter_ext_cfg_t ble_ext_config = {
    .pa_config = {
        .max_tx_power_dbm  = APP_BLE_CFG_DEFAULT_TX_POWER,
#if defined NUCLEO_WBA52_BOARD || defined NUCLEO_WBA55_BOARD || defined NUCLEO_WBA65_BOARD
        .ant_gain          = APP_BLE_CFG_NUCLEO_ANTENNA_GAIN,
        .tx_insertion_loss = APP_BLE_CFG_NUCLEO_RF_PATH_LOSS,
        .rx_insertion_loss = APP_BLE_CFG_NUCLEO_RF_PATH_LOSS,
#else
#  warning "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the 2.4GHz RF frontend configuration here if you are using another (e.g. custom) board"
        .ant_gain          = SID_STM32_BLE_POWER_DBM_TO_FIXED(0),
        .tx_insertion_loss = SID_STM32_BLE_POWER_DBM_TO_FIXED(0),
        .rx_insertion_loss = SID_STM32_BLE_POWER_DBM_TO_FIXED(0),
#  if SID_STM32_BLE_CFG_USE_EXTERNAL_PA /* This part is relevant only when an external RF Power Amplifier (PA) is mounted on PCB */
        .ext_pa_gain       = APP_BLE_CFG_RADIO_TX_EXT_PA_GAIN,
#  endif /* SID_STM32_BLE_CFG_USE_EXTERNAL_PA */
#  if SID_STM32_BLE_CFG_USE_EXTERNAL_LNA /* This part is relevant only when an external Low Noise Amplifier (LNA) is mounted on PCB */
        .lna_gain          = APP_BLE_CFG_RADIO_RX_LNA_GAIN,
#  endif /* SID_STM32_BLE_CFG_USE_EXTERNAL_LNA */
#endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
    },
    .sidewalk_profile = {
#if defined(NUCLEO_WBA52_BOARD)
        .device_name       = "sid_lr11xx_geo_nucleo-wba52",
#elif defined(NUCLEO_WBA55_BOARD)
        .device_name       = "sid_lr11xx_geo_nucleo-wba55",
#elif defined(NUCLEO_WBA65_BOARD)
        .device_name       = "sid_lr11xx_geo_nucleo-wba65",
#elif defined(STM32WBA5x)
        .device_name       = "sid_lr11xx_geo_stm32wba5x",
#elif defined(STM32WBA6x)
        .device_name       = "sid_lr11xx_geo_stm32wba5x",
#else
#  error "Unknown MCU platform"
#endif
        .max_att_mtu = SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX,
    },
};

/* Global function definitions -----------------------------------------------*/

const sid_ble_adapter_ext_cfg_t * sid_pal_ble_adapter_ext_get_link_config(void)
{
    return &ble_ext_config;
}
