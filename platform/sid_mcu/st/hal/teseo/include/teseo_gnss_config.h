/**
  ******************************************************************************
  * @file    teseo_gnss_config.h
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

#ifndef __STM32_SID_TESEO_GNSS_CONFIG_H_
#define __STM32_SID_TESEO_GNSS_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_uart_client.h>

/* Exported constants --------------------------------------------------------*/

#define TESEO_GNSS_NMEA_MESSAGE_SIZE_LIMIT         (82u) /*!< As per NMEA 0183 specification message size cannot exceed 82 bytes */

#ifndef TESEO_GNSS_CFG_ENFORCE_COMPATIBLE_CONFIG
#  define TESEO_GNSS_CFG_ENFORCE_COMPATIBLE_CONFIG (1u)  /*!< When enabled, the driver will proactively write to Teseo's CDBs to configure it for the use with this specific driver */
#endif /* TESEO_GNSS_CFG_ENFORCE_COMPATIBLE_CONFIG */

#ifndef TESEO_GNSS_CFG_WAIT_FOR_ACURACY_DATA
#  define TESEO_GNSS_CFG_WAIT_FOR_ACURACY_DATA     (1u)  /*!< When enabled, GNSS fix availability is reported after both the position and related accuracy data is received. When disabled, GNSS fix is reported right after the coordinates are received, but accuracy data may still be missing */
#endif /* TESEO_GNSS_CFG_WAIT_FOR_ACURACY_DATA */

/* Exported types ------------------------------------------------------------*/

typedef struct {
    uint8_t use_gps    : 1; /*!< Enable GPS constellation and use it for positioning */
    uint8_t use_galileo: 1; /*!< Enable Galileo constellation and use it for positioning */
    uint8_t use_beidou : 1; /*!< Enable BeiDou constellation and use it for positioning */
    uint8_t use_qzss   : 1; /*!< Enable QZSS constellation and use it for positioning */
    uint8_t use_glonass: 1; /*!< Enable Glonass */
} teseo_gnss_constellations_cfg_t;

typedef struct {
    const sid_pal_uart_config_t * const uart_cfg;   /*!< USART config for Teseo communication */

    const struct {
        uint32_t                            reset;  /*!< MCU pin that is connected to the nRESET line of the Teseo unit */
        uint32_t                            wakeup; /*!< MCU pin that is connected to the Wakeup line of the Teseo unit */
    }                                   gpio;       /*!< GPIO lines required to drive the Teseo GNSS receiver */

    teseo_gnss_constellations_cfg_t     constellations_to_use;
} teseo_gnss_device_config_t;

typedef struct teseo_gnss_position_data_s teseo_gnss_position_data_t; /*!< Forward declaration of teseo_gnss_position_data_t */

/**
 * @brief User-defined event callbacks to be called by Teseo driver
 */
typedef struct {
    /**
     * @brief User-defined argument to be passed to every callback
     */
    void * user_arg;

    /**
     * @brief GNSS Engine Restarted event callback. Invoked every time Teseo's GNSS engine is restarted
     *
     * @warning This callback blocks the whole system (it runs in a critical section), it is highly advisable not to perform any extensive processing directly in the context of this callback
     * @note GNSS engine can be restarted by $PSTMGPSRESET, $PSTMCOLD, $PSTWARM, or $PSTMHOT commands
     *
     * @param [in] user_arg User-defined argument
     * @return None
     */
    void (*on_gnss_engine_restarted)(void * user_arg);

    /**
     * @brief GNSS Fix Obtained event callback. Invoked every time GNSS Fix changes its status from invalid to valid
     *
     * @warning This callback blocks the whole system (it runs in a critical section), it is highly advisable not to perform any extensive processing directly in the context of this callback
     * @note GNSS fix is considered valid when all the relevant NMEA messages are received from Teseo, these messages indicate valid positioning data, and all of them share the same timestamp
     *
     * @param [in] user_arg User-defined argument
     * @return None
     */
    void (*on_gnss_fix_obtained)(void * user_arg);

    /**
     * @brief GNSS Fix Lost event callback. Invoked every time GNSS Fix changes its status from valid to invalid
     *
     * @warning This callback blocks the whole system (it runs in a critical section), it is highly advisable not to perform any extensive processing directly in the context of this callback
     * @note GNSS fix is considered invalid when at least one relevant NMEA message contains invalid positioning information
     *
     * @param [in] user_arg User-defined argument
     * @return None
     */
    void (*on_gnss_fix_lost)(void * user_arg);

    /**
     * @brief GNSS Position Update event callback. Invoked every time there's a new valid position reported by Teseo
     *
     * @warning This callback blocks the whole system (it runs in a critical section), it is highly advisable not to perform any extensive processing directly in the context of this callback
     * @note It is ensured by the driver that all the relevant NMEA messages were received and all of them share the same timestamp, so the coherency of positioning data is guaranteed
     *
     * @param [in] position Pointer to the new positioning data
     * @param [in] user_arg User-defined argument
     * @return None
     */
    void (*on_gnss_position_update)(const teseo_gnss_position_data_t * const position, void * user_arg);
} teseo_gnss_event_callbacks_t;

#ifdef __cplusplus
}
#endif

#endif /* __STM32_SID_TESEO_GNSS_CONFIG_H_ */
