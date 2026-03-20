/**
  ******************************************************************************
  * @file    teseo_gnss_private.h
  * @brief   Private API of the Sidewalk Teseo GNSS driver
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

#ifndef __STM32_SID_TESEO_GNSS_PRIVATE_H_
#define __STM32_SID_TESEO_GNSS_PRIVATE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "teseo_gnss.h"
#include "teseo_gnss_config.h"
#include "teseo_gnss_defs.h"

/* Sidewalk interfaces */
#include <sid_error.h>
#include <sid_pal_timer_ifc.h>

/* X-CUBE-GNSS1 middleware */
#include <gnss_parser.h>

#include <cmsis_os.h>

/* Exported types ------------------------------------------------------------*/

typedef union {
    struct {
        uint8_t nmea_port_baudrate: 1;
        uint8_t app_cfg1          : 1;
        uint8_t app_cfg2          : 1;
        uint8_t msg_list_low      : 1;
        uint8_t msg_list_high     : 1;
        uint8_t startup_msg       : 1;
    };
    uint32_t raw;
} teseo_cdb_cache_validity_t;

typedef struct {
    teseo_cdb_cache_validity_t          validity;
    teseo_cdb_102_nmea_port_baudrate_t  nmea_port_baudrate;
    teseo_cdb_200_application_onoff_t   app_cfg1;
    teseo_cdb_227_application_onoff_2_t app_cfg2;
    teseo_cdb_201_uart_msg_list_low_t   msg_list_low;
    teseo_cdb_228_uart_msg_list_high_t  msg_list_high;
    char                                startup_msg[32];
} teseo_cdb_cache_t;

typedef struct {
    uint8_t                               init_done;
    uint8_t                               in_standby;                     /*!< Indicates whether Teseo is in a standby mode and won't react on any commands or provide output */
    uint8_t                               echo_to_logs;                   /*!< If enabled, all Teseo messages are forwarded to Sidewalk log console */
    uint8_t                               gnss_fix_obtained;
    const teseo_gnss_device_config_t *    config;
    teseo_gnss_event_callbacks_t          user_callbacks;
    const sid_pal_uart_ext_ifc_t *        uart_ifc;
    sid_pal_timer_t                       standby_wakeup_timer;
    GNSSParser_Data_t                     gnss_data;                      /*!< Parsed GNSS data */
    osSemaphoreId_t                       gnss_data_locks[NMEA_MSGS_NUM]; /*!< Synchronization locks for Teseo messages. Lock is released every time the corresponding message arrives from Teseo */

    teseo_gnss_version_info_t             version_info;
    teseo_cdb_cache_t                     cdb_cache;

    uint8_t                               last_command[TESEO_GNSS_NMEA_MESSAGE_SIZE_LIMIT]; /*!< Most recently send command. Used to filter out Teseo command mirroring messages */
    osSemaphoreId_t                       cmd_completion_lock;
    osSemaphoreId_t                       gnss_fix_obtained_sem;
    osSemaphoreId_t                       gnss_fix_lost_sem;
    osSemaphoreId_t                       gnss_engine_restart_sem;

    teseo_gnss_position_data_t            latest_position;                /*!< Most recent position data */
} teseo_drv_ctx_t;

/* Exported functions --------------------------------------------------------*/

teseo_drv_ctx_t * teseo_gnss_prv_get_drv_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_SID_TESEO_GNSS_PRIVATE_H_ */
