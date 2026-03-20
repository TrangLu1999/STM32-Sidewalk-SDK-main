/**
  ******************************************************************************
  * @file    teseo_gnss_defs.h
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

#ifndef __STM32_SID_TESEO_GNSS_DEFS_H_
#define __STM32_SID_TESEO_GNSS_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <cmsis_compiler.h>
#include <sid_stm32_common_utils.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Configuration data block (CDB) identifier
 */
typedef enum {
    TESEO_CDB_CURRENT_CONFIG    = 1, /*!< Current (active) Configuration */
    TESEO_CDB_DEFAULT_CONFIG    = 2, /*!< Default Configuration */
    TESEO_CDB_NVM_STORED_CONFIG = 3, /*!< NVM Stored configuration */
} teseo_cdb_block_id_t;

typedef enum {
    TESEO_CDB_PARAM_NMEA_PORT_BAUDRATE  = 102u,
    TESEO_CDB_PARAM_APPLICATION_ONOFF   = 200u,
    TESEO_CDB_PARAM_UART_MSG_LIST_LOW   = 201u,
    TESEO_CDB_PARAM_APPLICATION_ONOFF_2 = 227u,
    TESEO_CDB_PARAM_UART_MSG_LIST_HIGH  = 228u,
    TESEO_CDB_PARAM_STARTUP_MESSAGE     = 500u,
} teseo_cdb_param_id_t;

typedef enum {
    TESEO_NMEA_BAUDRATE_300    = 0x0,
    TESEO_NMEA_BAUDRATE_600    = 0x1,
    TESEO_NMEA_BAUDRATE_1200   = 0x2,
    TESEO_NMEA_BAUDRATE_2400   = 0x3,
    TESEO_NMEA_BAUDRATE_4800   = 0x4,
    TESEO_NMEA_BAUDRATE_9600   = 0x5,
    TESEO_NMEA_BAUDRATE_14400  = 0x6,
    TESEO_NMEA_BAUDRATE_19200  = 0x7,
    TESEO_NMEA_BAUDRATE_38400  = 0x8,
    TESEO_NMEA_BAUDRATE_57600  = 0x9,
    TESEO_NMEA_BAUDRATE_115200 = 0xA,
    TESEO_NMEA_BAUDRATE_230400 = 0xB,
    TESEO_NMEA_BAUDRATE_460800 = 0xC,
    TESEO_NMEA_BAUDRATE_921600 = 0xD,
} teseo_nmea_port_baudrate_t;

typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint32_t                             : 24;
        teseo_nmea_port_baudrate_t br_cfg_val:  8; /*!< Index value that defines the baud rate used by Teseo UART. See CDB 102 definition for mapping between this index and the actual baud rate */
#else
        teseo_nmea_port_baudrate_t br_cfg_val:  8; /*!< Index value that defines the baud rate used by Teseo UART. See CDB 102 definition for mapping between this index and the actual baud rate */
        uint32_t                             : 24;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint32_t raw;
} teseo_cdb_102_nmea_port_baudrate_t;

typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t LOW_POWER_ON_OFF                : 1;
        uint8_t NMEA_RAW_ON_OFF                 : 1;
        uint8_t HIGH_DYNAMICS_ON_OFF            : 1;
        uint8_t                                 : 1; /*!< RESERVED */
        uint8_t TIMING_TRAIM_ON_OFF             : 1;
        uint8_t POSITION_HOLD_ENABLE            : 1;
        uint8_t PPS_POLARITY_INVERSION          : 1;
        uint8_t PPS_ENABLE                      : 1;

        uint8_t QZSS_USE_ENABLE                 : 1;
        uint8_t GPS_USE_ENABLE                  : 1;
        uint8_t GLONASS_USE_ENABLE              : 1;
        uint8_t NMEA_GNGSA_ENABLE               : 1;
        uint8_t NMEA_GNGSV_ENABLE               : 1;
        uint8_t QZSS_ENABLE                     : 1;
        uint8_t GLONASS_ENABLE                  : 1;
        uint8_t GPS_ENABLE                      : 1;

        uint8_t STOP_DETECTION_ENABLE           : 1;
        uint8_t WALKING_MODE_ENABLE             : 1;
        uint8_t                                 : 1; /*!< RESERVED */
        uint8_t FDE_ENABLE                      : 1;
        uint8_t RTCM_ENABLE                     : 1;
        uint8_t ST_HEADERS_ENABLE               : 1;
        uint8_t CONFIG_TXT_HEADER_EN            : 1; /*!< Allow a text message at startup over the NMEA port. The user is free to use this text as product name or as specific configuration marker */
        uint8_t                                 : 1; /*!< RESERVED */

        uint8_t QZSS_DISTRIBUTED_ACQ_MODE_ENABLE: 1;
        uint8_t                                 : 1; /*!< RESERVED */
        uint8_t TCXO_2_5_PPM_ENABLE             : 1;
        uint8_t STAGPS_ENABLE                   : 1;
        uint8_t SBAS_SAT_ON_GSV_MSG_ENABLE      : 1;
        uint8_t SBAS_ENABLE                     : 1;
        uint8_t GPS_2D_FIX_ENABLE               : 1;
        uint8_t                                 : 1; /*!< RESERVED */
#else
        uint8_t                                 : 1; /*!< RESERVED */
        uint8_t GPS_2D_FIX_ENABLE               : 1; //FIXME: reserved?
        uint8_t SBAS_ENABLE                     : 1;
        uint8_t SBAS_SAT_ON_GSV_MSG_ENABLE      : 1;
        uint8_t STAGPS_ENABLE                   : 1;
        uint8_t TCXO_2_5_PPM_ENABLE             : 1;
        uint8_t                                 : 1; /*!< RESERVED */
        uint8_t QZSS_DISTRIBUTED_ACQ_MODE_ENABLE: 1;

        uint8_t                                 : 1; /*!< RESERVED */
        uint8_t CONFIG_TXT_HEADER_EN            : 1; /*!< Allow a text message at startup over the NMEA port. The user is free to use this text as product name or as specific configuration marker */
        uint8_t ST_HEADERS_ENABLE               : 1; //FIXME: reserved?
        uint8_t RTCM_ENABLE                     : 1;
        uint8_t FDE_ENABLE                      : 1;
        uint8_t                                 : 1; /*!< RESERVED */
        uint8_t WALKING_MODE_ENABLE             : 1;
        uint8_t STOP_DETECTION_ENABLE           : 1;

        uint8_t GPS_ENABLE                      : 1;
        uint8_t GLONASS_ENABLE                  : 1;
        uint8_t QZSS_ENABLE                     : 1;
        uint8_t NMEA_GNGSV_ENABLE               : 1;
        uint8_t NMEA_GNGSA_ENABLE               : 1;
        uint8_t GLONASS_USE_ENABLE              : 1;
        uint8_t GPS_USE_ENABLE                  : 1;
        uint8_t QZSS_USE_ENABLE                 : 1;

        uint8_t PPS_ENABLE                      : 1;
        uint8_t PPS_POLARITY_INVERSION          : 1;
        uint8_t POSITION_HOLD_ENABLE            : 1;
        uint8_t TIMING_TRAIM_ON_OFF             : 1;
        uint8_t                                 : 1; /*!< RESERVED */
        uint8_t HIGH_DYNAMICS_ON_OFF            : 1; //FIXME: reserved?
        uint8_t NMEA_RAW_ON_OFF                 : 1; //FIXME: reserved?
        uint8_t LOW_POWER_ON_OFF                : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint32_t raw;
} teseo_cdb_200_application_onoff_t;

typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                               : 5; /*!< RESERVED */
        uint8_t RTC_CALIBRATION_ENABLE        : 1; /*!< Enable/disable the RTC calibration feature. When enabled the RTC counter is calibrated using the accurate GNSS internal time reference */
        uint8_t                               : 2; /*!< RESERVED */

        uint8_t                               : 8; /*!< RESERVED */

        uint8_t                               : 1; /*!< RESERVED */
        uint8_t EXCLUDED_SATS_REPORTING_ENABLE: 1; /*!< Enable/disable the excluded satellites reporting in the GGA, GSA, GNS and PSTMTG NMEA messages */
        uint8_t                               : 1; /*!< RESERVED */
        uint8_t FAST_SATELLITE_DROP_ENABLE    : 1; /*!< Enable/disable the Fast Satellite Drop feature. When fast satellite drop is enabled, the GNSS software reports NO FIX status immediately after the tunnel entrance; the position update is no more propagated for some seconds inside the tunnel */
        uint8_t RTC_USAGE_DISABLING           : 1; /*!< Enable/disable the usage of RTC from the GNSS engine. It is recommended to have RTC usage disabled (Bit12 set to 1) is the RTC crystal is not mounted */
        uint8_t                               : 1; /*!< RESERVED */
        uint8_t BEIDOU_USAGE_ENABLE           : 1; /*!< Enable/disable the usage of BEIDOU satellite for the GNSS position fix. If this bit is disabled and BEIDOU constellation is enabled, the BEIDOU satellites are only tracked */
        uint8_t BEIDOU_ENABLE                 : 1; /*!< Enable/disable the BEIDOU constellation. When this bit is enabled BEIDOU satellites are enabled to be tracked and used for positioning */

        uint8_t GALILEO_USAGE_ENABLE          : 1; /*!< Enable/disable the usage of Galileo satellite for the GNSS position fix. If this bit is disabled and Galileo constellation is enabled, the Galileo satellites are only tracked */
        uint8_t GALILEO_ENABLE                : 1; /*!< Enable/disable the Galileo constellation. When this bit is enabled Galileo satellites are enabled to be tracked and used for positioning */
        uint8_t                               : 1; /*!< RESERVED */
        uint8_t NMEA_IN_OUT_INTERFACE_SELECT  : 1;
        uint8_t                               : 1; /*!< RESERVED */
        uint8_t FEW_SATS_POS_ESTIMATION_ENABLE: 1; /*!< Enable/disable the position estimation algorithm when tracked satellites are less than 3 */
        uint8_t NMEA_TTFF_MESSAGE_ENABLE      : 1; /*!< Enable/disable the Time To First Fix message on the NMEA port. If enabled, the TTFF message is sent only one time as soon as the GNSS position fix is achieved */
        uint8_t NMEA_COMMAND_ECHO_ENABLE      : 1; /*!< Enable/disable the command echo on the NMEA port */
#else
        uint8_t NMEA_COMMAND_ECHO_ENABLE      : 1; /*!< Enable/disable the command echo on the NMEA port */
        uint8_t NMEA_TTFF_MESSAGE_ENABLE      : 1; /*!< Enable/disable the Time To First Fix message on the NMEA port. If enabled, the TTFF message is sent only one time as soon as the GNSS position fix is achieved */
        uint8_t FEW_SATS_POS_ESTIMATION_ENABLE: 1; /*!< Enable/disable the position estimation algorithm when tracked satellites are less than 3 */
        uint8_t                               : 1; /*!< RESERVED */
        uint8_t NMEA_IN_OUT_INTERFACE_SELECT  : 1;
        uint8_t                               : 1; /*!< RESERVED */
        uint8_t GALILEO_ENABLE                : 1; /*!< Enable/disable the Galileo constellation. When this bit is enabled Galileo satellites are enabled to be tracked and used for positioning */
        uint8_t GALILEO_USAGE_ENABLE          : 1; /*!< Enable/disable the usage of Galileo satellite for the GNSS position fix. If this bit is disabled and Galileo constellation is enabled, the Galileo satellites are only tracked */

        uint8_t BEIDOU_ENABLE                 : 1; /*!< Enable/disable the BEIDOU constellation. When this bit is enabled BEIDOU satellites are enabled to be tracked and used for positioning */
        uint8_t BEIDOU_USAGE_ENABLE           : 1; /*!< Enable/disable the usage of BEIDOU satellite for the GNSS position fix. If this bit is disabled and BEIDOU constellation is enabled, the BEIDOU satellites are only tracked */
        uint8_t                               : 1; /*!< RESERVED */
        uint8_t RTC_USAGE_DISABLING           : 1; /*!< Enable/disable the usage of RTC from the GNSS engine. It is recommended to have RTC usage disabled (Bit12 set to 1) is the RTC crystal is not mounted */
        uint8_t FAST_SATELLITE_DROP_ENABLE    : 1; /*!< Enable/disable the Fast Satellite Drop feature. When fast satellite drop is enabled, the GNSS software reports NO FIX status immediately after the tunnel entrance; the position update is no more propagated for some seconds inside the tunnel */
        uint8_t                               : 1; /*!< RESERVED */
        uint8_t EXCLUDED_SATS_REPORTING_ENABLE: 1; /*!< Enable/disable the excluded satellites reporting in the GGA, GSA, GNS and PSTMTG NMEA messages */
        uint8_t                               : 1; /*!< RESERVED */

        uint8_t                               : 8; /*!< RESERVED */

        uint8_t                               : 2; /*!< RESERVED */
        uint8_t RTC_CALIBRATION_ENABLE        : 1; /*!< Enable/disable the RTC calibration feature. When enabled the RTC counter is calibrated using the accurate GNSS internal time reference */
        uint8_t                               : 5; /*!< RESERVED */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint32_t raw;
} teseo_cdb_227_application_onoff_2_t;

typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t PSTMTM          : 1;
        uint8_t PSTMNOTCHSTATUS : 1;
        uint8_t PSTMLOWPOWERDATA: 1;
        uint8_t PSTMAGPS        : 1;
        uint8_t PSTMKFCOV       : 1;
        uint8_t PSTMPOSHOLD     : 1;
        uint8_t PSTMTRAIMSTATUS : 1;
        uint8_t GPZDA           : 1;

        uint8_t PSTMCPU         : 1;
        uint8_t                 : 1; /*!< RESERVED */
        uint8_t PSTMPPSDATA_EN  : 1;
        uint8_t GPGLL_EN        : 1;
        uint8_t GPGSV_EN        : 1;
        uint8_t PSTMTESTRF_EN   : 1;
        uint8_t PSTMSBAS_EN     : 1;
        uint8_t PSTMCORR_EN     : 1;

        uint8_t PSTMDIFF_EN     : 1;
        uint8_t PSTMWAAS_EN     : 1;
        uint8_t PSTMTIM_EN      : 1;
        uint8_t PSTMRES_EN      : 1;
        uint8_t PSTMSAT_EN      : 1;
        uint8_t PSTMPA_EN       : 1;
        uint8_t PSTMTS_EN       : 1;
        uint8_t PSTMTG_EN       : 1;

        uint8_t PSTMRF_EN       : 1;
        uint8_t GPRMC_EN        : 1;
        uint8_t PSTMNOISE_EN    : 1;
        uint8_t GPVTG_EN        : 1;
        uint8_t GPGST_EN        : 1;
        uint8_t GPGSA_EN        : 1;
        uint8_t GPGGA_EN        : 1;
        uint8_t GPGNS_EN        : 1;
#else
        uint8_t GPGNS_EN        : 1;
        uint8_t GPGGA_EN        : 1;
        uint8_t GPGSA_EN        : 1;
        uint8_t GPGST_EN        : 1;
        uint8_t GPVTG_EN        : 1;
        uint8_t PSTMNOISE_EN    : 1;
        uint8_t GPRMC_EN        : 1;
        uint8_t PSTMRF_EN       : 1;

        uint8_t PSTMTG_EN       : 1;
        uint8_t PSTMTS_EN       : 1;
        uint8_t PSTMPA_EN       : 1;
        uint8_t PSTMSAT_EN      : 1;
        uint8_t PSTMRES_EN      : 1;
        uint8_t PSTMTIM_EN      : 1;
        uint8_t PSTMWAAS_EN     : 1;
        uint8_t PSTMDIFF_EN     : 1;

        uint8_t PSTMCORR_EN     : 1;
        uint8_t PSTMSBAS_EN     : 1;
        uint8_t PSTMTESTRF_EN   : 1;
        uint8_t GPGSV_EN        : 1;
        uint8_t GPGLL_EN        : 1;
        uint8_t PSTMPPSDATA_EN  : 1;
        uint8_t                 : 1; /*!< RESERVED */
        uint8_t PSTMCPU         : 1;

        uint8_t GPZDA           : 1;
        uint8_t PSTMTRAIMSTATUS : 1;
        uint8_t PSTMPOSHOLD     : 1;
        uint8_t PSTMKFCOV       : 1;
        uint8_t PSTMAGPS        : 1;
        uint8_t PSTMLOWPOWERDATA: 1;
        uint8_t PSTMNOTCHSTATUS : 1;
        uint8_t PSTMTM          : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint32_t raw;
} teseo_cdb_201_uart_msg_list_low_t;

typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t xxRLM_EN          : 1;
        uint8_t                   : 7; /*!< RESERVED */

        uint8_t                   : 2; /*!< RESERVED */
        uint8_t PSTMGNSSINTEGRITY : 1;
        uint8_t PSTMLOGSTATUS     : 1;
        uint8_t PSTMGEOFENCESTATUS: 1;
        uint8_t PSTMODO           : 1;
        uint8_t                   : 1; /*!< RESERVED */
        uint8_t PSTMFEDATA        : 1;

        uint8_t                   : 1; /*!< RESERVED */
        uint8_t PSTMPVRAW         : 1;
        uint8_t GPGBS             : 1;
        uint8_t PSTMBIASDATA      : 1;
        uint8_t                   : 1; /*!< RESERVED */
        uint8_t PSTMIONOPARAMS    : 1;
        uint8_t PSTMALMANAC       : 1;
        uint8_t PSTMEPHEM         : 1;

        uint8_t GPDTM             : 1;
        uint8_t PSTMUSEDSATS      : 1;
        uint8_t                   : 2; /*!< RESERVED */
        uint8_t PSTMADCDATA       : 1;
        uint8_t PSTMUTC           : 1;
        uint8_t PSTMPVQ           : 1;
        uint8_t PSTMPV            : 1;
#else
        uint8_t PSTMPV            : 1;
        uint8_t PSTMPVQ           : 1;
        uint8_t PSTMUTC           : 1;
        uint8_t PSTMADCDATA       : 1;
        uint8_t                   : 2; /*!< RESERVED */
        uint8_t PSTMUSEDSATS      : 1;
        uint8_t GPDTM             : 1;

        uint8_t PSTMEPHEM         : 1;
        uint8_t PSTMALMANAC       : 1;
        uint8_t PSTMIONOPARAMS    : 1;
        uint8_t                   : 1; /*!< RESERVED */
        uint8_t PSTMBIASDATA      : 1;
        uint8_t GPGBS             : 1;
        uint8_t PSTMPVRAW         : 1;
        uint8_t                   : 1; /*!< RESERVED */

        uint8_t PSTMFEDATA        : 1;
        uint8_t                   : 1; /*!< RESERVED */
        uint8_t PSTMODO           : 1;
        uint8_t PSTMGEOFENCESTATUS: 1;
        uint8_t PSTMLOGSTATUS     : 1;
        uint8_t PSTMGNSSINTEGRITY : 1;
        uint8_t                   : 2; /*!< RESERVED */

        uint8_t                   : 7; /*!< RESERVED */
        uint8_t xxRLM_EN          : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint32_t raw;
} teseo_cdb_228_uart_msg_list_high_t;

#ifdef __cplusplus
}
#endif

#endif /* __STM32_SID_TESEO_GNSS_DEFS_H_ */
