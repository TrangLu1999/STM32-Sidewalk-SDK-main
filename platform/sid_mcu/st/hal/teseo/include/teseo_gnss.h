/**
  ******************************************************************************
  * @file    teseo_gnss.h
  * @brief   Handling of the Teseo GNSS receiver for Sidewalk application
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

#ifndef __STM32_SID_TESEO_GNSS_H_
#define __STM32_SID_TESEO_GNSS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <time.h>

/* Sidewalk interfaces */
#include <sid_error.h>

#include "teseo_gnss_config.h"
#include "teseo_gnss_defs.h"

/* Exported constants --------------------------------------------------------*/

/** @defgroup TESEO_GNSS_Start_Flags Start flags for GNSS cold start
  * @{
  */
#define TESEO_GNSS_START_FLAGS_NONE            (0x0u)
#define TESEO_GNSS_START_FLAGS_CLEAR_ALMANAC   (0x1u)
#define TESEO_GNSS_START_FLAGS_CLEAR_EPHEMERIS (0x2u)
#define TESEO_GNSS_START_FLAGS_CLEAR_POSITION  (0x4u)
#define TESEO_GNSS_START_FLAGS_CLEAR_TIME      (0x8u)
#define TESEO_GNSS_START_FLAGS_ALL             (  TESEO_GNSS_START_FLAGS_CLEAR_ALMANAC   \
                                                | TESEO_GNSS_START_FLAGS_CLEAR_EPHEMERIS \
                                                | TESEO_GNSS_START_FLAGS_CLEAR_POSITION  \
                                                | TESEO_GNSS_START_FLAGS_CLEAR_TIME      \
                                               )
/**
  * @}
  */

#define TESEO_GNSS_STANDBY_DURATION_INFINITE   (0u)

/* Exported types ------------------------------------------------------------*/

typedef enum {
    TESEO_RESET_SYS_HARDWARE     = 0u, /*!< Hardware reset of the Teseo module using nRESET pin */
    TESEO_RESET_SYS_SOFTWAREWARE = 1u, /*!< Software reset of the Teseo module using $PSTMSRR command */
    TESEO_RESET_GNSS_ENGINE      = 2u, /*!< Reset of the Teseo GNSS engine only using $PSTMGPSRESET command */
} teseo_gnss_reset_type_t;

typedef enum {
    TESEO_COLD_START = 0u, /*!< Perform Teseo cold start using $PSTMCOLD command */
    TESEO_WARM_START = 1u, /*!< Perform Teseo warm start using $PSTWARM command */
    TESEO_HOT_START  = 2u, /*!< Perform Teseo hot start using $PSTMHOT command */
} teseo_gnss_start_type_t;

typedef enum {
    TESEO_VER_TYPE_GNSS_LIB       = 0u,   /*!< GNSS Library Version */
    TESEO_VER_TYPE_OS20           = 1u,   /*!< OS20 Version */
    TESEO_VER_TYPE_SDK_APP        = 2u,   /*!< SDK App Version */
    TESEO_VER_TYPE_FW             = 6u,   /*!< Binary Image Version */
    TESEO_VER_TYPE_HW             = 7u,   /*!< STA8088 HW version */
    TESEO_VER_TYPE_SW_CFG_ID      = 11u,  /*!< SW configuration ID */
    TESEO_VER_TYPE_PRODUCT_ID     = 12u,  /*!< Product ID */
    TESEO_VER_TYPE_UCODE          = 13u,  /*!< Unique code that is reported by $PSTMGETUCODE command */
    TESEO_VER_TYPE_CFG_DATA_BLOCK = 254u, /*!< configuration data block */
    TESEO_VER_TYPE_ALL_STRINGS    = 255u, /*!< all versions strings (as reported at the NMEA startup) */

    TESEO_VER_TYPE_ALL_SUPPORTED  = 128u, /*!< Request all the version information strings supported by this driver */
} teseo_gnss_version_info_type_t;

typedef struct teseo_gnss_position_data_s {
    float     latitude;            /*!< Latitude value in degrees */
    float     longitude;           /*!< Longitude value in degrees */
    float     elevation;           /*!< Elevation above WGS84 ellipsoid in meters */
    float     horizontal_accuracy; /*!< Horizontal accuracy reported by Teseo unit. All the calculations are done using 1-sigma standard deviation, so the confidence level for this accuracy is fixed at approx. 0.68 and there's no added value to store confidence level separately */
    float     vertical_accuracy;   /*!< Vertical accuracy computed using 1-sigma standard deviation. Confidence level is fixed to 0.68 due to 1-sigma */
    struct tm utc_time;            /*!< UTC timestamp of the position scan */
    uint32_t  posix_time;          /*!< The same timestamp of the position data expressed in Unix epoch (number of seconds from 1970-01-01T00:00:00Z) */
} teseo_gnss_position_data_t;

typedef union {
    struct {
      uint8_t build;
      uint8_t patch;
      uint8_t minor;
      uint8_t major;
    };
    uint32_t raw;
} teseo_gnss_version_num_t;

typedef struct {
    teseo_gnss_version_num_t bin_image;
    teseo_gnss_version_num_t gnss_lib;
    teseo_gnss_version_num_t os20_lib;
    teseo_gnss_version_num_t gps_app;
    char                     dev_name[16];
    char                     ucode[15];
} teseo_gnss_version_info_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Performs the requested reset procedure of the Teseo unit
 *
 * @note This command issues the requested reset type and wait for Teseo to return back online. The call will block until Teseo responds or timeout happens
 *
 * @param [in] reset_type Type of the reset to execute: hardware (using the physical nRESET pin), software via $PSTMSRR command, or GNSS App-only via th $PSTMGPSRESET
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_reset(const teseo_gnss_reset_type_t reset_type);

/**
 * @brief Performs the requested (re)start of the Teseo's GNSS engine
 *
 * @note This command issues the requested start type and wait for Teseo to return back online. The call will block until Teseo responds or timeout happens
 *
 * @param [in] start_type Type of the start to perform: cold, warm, or hot
 * @param [in] flags Additional flags to configure restart. Valid for cold start only, see @ref TESEO_GNSS_Start_Flags
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_engine_restart(const teseo_gnss_start_type_t start_type, const uint32_t flags);

/**
 * @brief Restores factory default configuration by invalidating the current configuration in NVM and RAM
 *
 * @note This methods restores the factory default configuration and resets the Teseo module to make the restore settings effective
 *
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_restore_factory_defaults(void);

/**
 * @brief Enable forwarding of the raw Teseo output to the Sidewalk log console
 *
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_enable_log_mirroring(void);

/**
 * @brief Disable forwarding of the raw Teseo output to the Sidewalk log console
 *
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_disable_log_mirroring(void);

/**
 * @brief Query the status of the raw Teseo output forwarding to the Sidewalk log console
 *
 * @param [out] is_enabled Set to TRUE if forwarding is currently enabled, set to FALSE otherwise
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_get_log_mirroring_status(uint8_t * const is_enabled);

/**
 * @brief Sends a raw command string to Teseo module
 *
 * @param [in] cmd Command string. It may or may not start with '$' and end with '\r\n', proper start and line endings will be applied automatically.
 *                 It may or may not contain the checksum - it will be calculated and added automatically if needed. @warning: if provided command string contains an invalid checksum it will be sent to Teseo as is
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_send_command(const char * const cmd);

/**
 * @brief Waits for the last issued command (via @ref teseo_gnss_send_command) is acknowledged by Teseo
 *
 * @note Command completion monitoring is based on the command echo sent back by Teseo. Thus, it is essential to have echoing enabled in Application On/Off 2 CDB parameter,
 *       otherwise monitoring of the command execution status is not technically possible. If command echoing is disabled this method will return SID_ERROR_INVALID_STATE
 *       immediately. This does not necessarily mean the command execution failed, but rather shows that monitoring is not possible. It is safe to proceed after SID_ERROR_INVALID_STATE
 *       is reported
 *
 * @param [in] timeout_ms Time to wait for command completion
 * @return SID_ERROR_NONE upon success, SID_ERROR_TIMEOUT upon timeout, SID_ERROR_INVALID_STATE if current Teseo configuration does not allow to monitor command completion, other values are possible for systematic failures
 */
sid_error_t teseo_gnss_wait_command_completion(const uint32_t timeout_ms);

/**
 * @brief Proactively requests version information readout from Teseo unit
 *
 * @note Normally you don't need to call this API since version information is retrieved automatically upon initialization. It is safe to go with @ref teseo_gnss_get_version_info directly.
 *       This method can be useful to facilitate an update after the version information may have changed (e.g., after a firmware update)
 * @param [in] version_type Type of version information to request from Teseo
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_request_version_info(const teseo_gnss_version_info_type_t version_type);

/**
 * @brief Provides Teseo version information from the local cache
 *
 * @note This method does not request any data from Teseo module. If the version info in the local cache is not available or is incomplete, the undefined values will be populated into the output
 * @param [out] version_info_storage Pointer to the storage location of the version information to be provided
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_get_version_info(teseo_gnss_version_info_t * const version_info_storage);

/**
 * @brief Provides Teseo Unique Code information from the local cache
 *
 * @note This method does not request any data from Teseo module. If the unique code in the local cache is not available or is incomplete, the undefined values will be populated into the output
 *       Use teseo_gnss_request_version_info(TESEO_VER_TYPE_UCODE) to explicitly request redout of the unique code from Teseo unit
 * @param [out] buffer Pointer to the storage location for the unique code string
 * @param [in]  buffer_size Size of the storage buffer for the unique code string
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_get_ucode(char * const buffer, const uint32_t buffer_size);

/**
 * @brief Requests readout of NMEA UART Port Baudrate setting (CDB 102) from Teseo's current configuration
 *
 * @note This methods performs direct reading from Teseo, cached values are not used
 *
 * @param [out] nmea_port_baudrate Storage for NMEA UART Port Baudrate (CDB 102)
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_request_nmea_port_baudrate_config(teseo_cdb_102_nmea_port_baudrate_t * const nmea_port_baudrate);

/**
 * @brief Writes NMEA UART Port Baudrate setting (CDB 102) into Teseo's current configuration (RAM)
 *
 * @note This methods performs direct write to Teseo, CDB values are fully overwritten with the new ones
 *
 * @param [in] nmea_port_baudrate Storage for NMEA UART Port Baudrate (CDB 102)
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_send_nmea_port_baudrate_config(const teseo_cdb_102_nmea_port_baudrate_t nmea_port_baudrate);

/**
 * @brief Requests readout of Application On/Off (CDB 200) and Application On/Off 2 (CDB 227) from Teseo's current configuration
 *
 * @note This methods performs direct reading from Teseo, cached values are not used
 *
 * @param [out] app_cfg1 Storage for Application On/Off (CDB 200). Can be set to NULL to skip the readout of this particular CDB
 * @param [out] app_cfg2 Storage for Application On/Off 2 (CDB 227). Can be set to NULL to skip the readout of this particular CDB
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_request_app_config(teseo_cdb_200_application_onoff_t * const app_cfg1, teseo_cdb_227_application_onoff_2_t * const app_cfg2);

/**
 * @brief Writes Application On/Off (CDB 200) and Application On/Off 2 (CDB 227) into Teseo's current configuration (RAM)
 *
 * @note This methods performs direct write to Teseo, CDB values are fully overwritten with the new ones
 *
 * @param [in] app_cfg1 Application On/Off (CDB 200) value to send to Teseo
 * @param [in] app_cfg2 Application On/Off 2 (CDB 227) value to send to Teseo
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_send_app_config(const teseo_cdb_200_application_onoff_t app_cfg1, const teseo_cdb_227_application_onoff_2_t app_cfg2);

/**
 * @brief Requests readout of NMEA UART Port Message List Low (CDB 201) and NMEA UART Port Message List High (CDB 228) from Teseo's current configuration
 *
 * @note This methods performs direct reading from Teseo, cached values are not used
 *
 * @param [out] msg_list_low Storage for NMEA UART Port Message List Low (CDB 201). Can be set to NULL to skip the readout of this particular CDB
 * @param [out] msg_list_high Storage for NMEA UART Port Message List High (CDB 228). Can be set to NULL to skip the readout of this particular CDB
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_request_message_config(teseo_cdb_201_uart_msg_list_low_t * const msg_list_low, teseo_cdb_228_uart_msg_list_high_t * const msg_list_high);

/**
 * @brief Writes NMEA UART Port Message List Low (CDB 201) and NMEA UART Port Message List High (CDB 228) into Teseo's current configuration (RAM)
 *
 * @note This methods performs direct write to Teseo, CDB values are fully overwritten with the new ones
 *
 * @param [in] msg_list_low NMEA UART Port Message List Low (CDB 201) value to send to Teseo
 * @param [in] msg_list_high NMEA UART Port Message List High (CDB 228) value to send to Teseo
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_send_message_config(const teseo_cdb_201_uart_msg_list_low_t msg_list_low, const teseo_cdb_228_uart_msg_list_high_t msg_list_high, const uint8_t message_rate);

/**
 * @brief Requests readout of Startup Message Text (CDB 500) from Teseo's current configuration
 *
 * @note This methods performs direct reading from Teseo, cached values are not used
 *
 * @param [out] msg Storage for Startup Message Text (CDB 500)
 * @param [out] max_size Size limit for Startup Message Text (CDB 500) storage. This also includes the '\0' terminator
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_request_startup_message_text(char * const msg, const uint32_t max_size);

/**
 * @brief Stores Teseo configuration in RAM to NVM and ensures the new configuration is fully applied by performing a soft-reset of Teseo
 *
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_apply_configuration(void);

/**
 * @brief Reports if the GNSS fix is available and position information can be queried from this driver
 *
 * @param [out] available Set to TRUE if GNSS fix is obtained and positioning data can be provide, set to FALSE otherwise
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_is_gnss_fix_available(uint8_t * const available);

/**
 * @brief Provides the most recent positioning data available
 *
 * @param [out] postion_storage Storage for the positioning data
 * @return SID_ERROR_NONE upon success, SID_ERROR_NOT_FOUND if position is not available (e.g., GNSS fix is not obtained yet)
 */
sid_error_t teseo_gnss_get_position(teseo_gnss_position_data_t * const postion_storage);

sid_error_t teseo_gnss_suspend_gnss_engine(void);

sid_error_t teseo_gnss_standby(const uint32_t duration_s);

sid_error_t teseo_gnss_wakeup(void);

/**
 * @brief Performs initialization of the Teseo module and underlying hardware (e.g., UART, DMA, GPIO, etc.)
 *
 * @param [in] config User-defined configuration of the Teseo module and the driver
 * @param [in] event_callbacks User-defined callbacks that will be triggered upon corresponding events take place
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_init(const teseo_gnss_device_config_t * const config, teseo_gnss_event_callbacks_t * const event_callbacks);

/**
 * @brief Performs full de-initialization of Teseo module and release all associated hardware and software resources
 *
 * @return SID_ERROR_NONE upon success
 */
sid_error_t teseo_gnss_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_SID_TESEO_GNSS_H_ */
