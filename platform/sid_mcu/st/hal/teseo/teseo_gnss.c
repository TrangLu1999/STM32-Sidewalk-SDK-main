/**
  ******************************************************************************
  * @file    teseo_gnss.c
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

/* Includes ------------------------------------------------------------------*/

#include <math.h>
#include <stdlib.h>

#include "teseo_gnss.h"
#include "teseo_gnss_config.h"
#include "teseo_gnss_private.h"

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_timer_ifc.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>

/* GNSS lib */
#include "gnss_lib_config.h"
#include <gnss_data.h>
#include <gnss_parser.h>

#include "sid_stm32_common_utils.h"

#ifdef TESEO_GNSS_USE_STDLIB_PRINT
#  include <stdio.h>
#else
#  include "stm32_tiny_vsnprintf.h"
#endif /* TESEO_GNSS_USE_STDLIB_PRINT */

/* Private defines -----------------------------------------------------------*/

#define TESEO_GNSS_CMD_TO_CMD_DELAY_MS         (20u)    /*!< Delay (in milliseconds) between two consecutive commands (20ms minimum as per UM2229) */
#define TESEO_GNSS_RESET_HOLD_TIME_MS          (20u)    /*!< Time to keep reset signal active to trigger the module reset */
#define TESEO_GNSS_WAKEUP_HOLD_TIME_MS         (1u)     /*!< Time to keep wakeup signal active to trigger the module wakeup */
#define TESEO_GNSS_BOOT_TIMEOUT_MS             (5000u)  /*!< Maximum wait time to receive a first message from Teseo after Reset is release. If no message arrives within this time a fault will be indicated */

#define TESEO_GNSS_DEFAULT_CMD_TIMEOUT_MS      (1500u)  /*!< Default wait time for a command response message from Teseo */
#define TESEO_GNSS_GPSSUSPEND_CMD_TIMEOUT_MS   (20000u) /*!< Timeout for $PSTMGPSSUSPEND command */
#define TESEO_GNSS_FORCESTANDBY_CMD_TIMEOUT_MS (2500u)  /*!< Timeout for $PSTMFORCESTANDBY command */
#define TESEO_GNSS_PSTMGETUCODE_CMD_TIMEOUT_MS (400u)   /*!< Timeout for $PSTMGETUCODE command - using shorter wait time because this command is used for auto baud rate detection */
#define TESEO_GNSS_SAVEPAR_CMD_TIMEOUT_MS      (10000u) /*!< Timeout for $PSTMSAVEPAR command - it writes to NVM so the execution may take long time */
#define TESEO_GNSS_ENGINE_RESET_CMD_TIMEOUT_MS (5000u)  /*!< Default wait time for a command response message from Teseo */ /* FIXME: compute it in runtime based on the GNSS fix rate and message rate */

#define TESEO_GNSS_UART_LOOKAHEAD_WAKEUP_MS    (50u)    /*!< Whenever Teseo is put into a Standby mode with timeout, the UART will be reactivated for this amount of milliseconds ahead of anticipated Teseo wakeup */

#define TESEO_GNSS_SWVER_REQUEST_ATTEMPTS_MAX  (5u)     /*!< Attempt limit for requesting a specific type of version information */

#define TESEO_GNSS_TIMEOUT_INFINITE            (0xFFFFFFFFu)

#define TESEO_GNSS_GPTXT_LOG_PREFIX            "\e[1;34m" "[TESEO] " "\e[0m"

#define TESEO_GNSS_NMEA_CHECKSUM_CHARS         (3u)     /*!< Checksum in NMEA sentence requires 3 characters: '*' as a delimiter and two chars to represent checksum byte */
#define TESEO_GNSS_NMEA_LINE_ENDING            "\r\n"

#ifdef TESEO_GNSS_USE_STDLIB_PRINT
#  define TESEO_GNSS_VSNPRINTF(...)            vsnprintf(__VA_ARGS__)
#  define TESEO_GNSS_SNPRINTF(...)             snprintf(__VA_ARGS__)
#else
#  define TESEO_GNSS_VSNPRINTF(...)            tiny_vsnprintf_like(__VA_ARGS__)
#  define TESEO_GNSS_SNPRINTF(...)             _prv_snprintf_like(__VA_ARGS__)
#endif /* TESEO_GNSS_USE_STDLIB_PRINT */

/* Private macro -------------------------------------------------------------*/

#define TESEO_GNSS_MS_TO_OS_TICKS(__MS__)      ((uint32_t)(((uint64_t)(__MS__) * (uint64_t)osKernelGetTickFreq()) / (uint64_t)1000u) + 1u) /* Add 1 OS tick since osDelay() provides a delay somewhere in (delay_ticks - 1)...delay_ticks range - 1 more tick is needed to guarantee that the actual delay will be not less than requested */

/* Private types -------------------------------------------------------------*/

typedef struct {
    uint32_t                   baud_rate; /* UART baud rate expressed in bps */
    teseo_nmea_port_baudrate_t cfg_val;   /* Corresponding Teseo configuration value for CDB 102  */
} teseo_baudrate_lut_entry_t;

/* Private variables ---------------------------------------------------------*/

static teseo_drv_ctx_t drv_ctx = {0};

/* Private function prototypes -----------------------------------------------*/

static inline uint32_t    _prv_is_leap_year(const uint32_t year);
static        uint32_t    _prv_timegm_like(const struct tm * const tm);

#ifndef TESEO_GNSS_USE_STDLIB_PRINT
static inline int _prv_snprintf_like(char * buf, const size_t size, const char *fmt, ...);
#endif /* TESEO_GNSS_USE_STDLIB_PRINT */

static        sid_error_t _prv_parse_version_number_string(const char * const version_string, const char * prefix, const uint32_t prefix_len, teseo_gnss_version_num_t * const version_num_storage);
static        sid_error_t _prv_parse_bin_image_version_string(const char * const version_string);
static        sid_error_t _prv_parse_gnss_lib_version_string(const char * const version_string);
static        sid_error_t _prv_parse_gps_app_version_string(const char * const version_string);
static        sid_error_t _prv_parse_os20_lib_version_string(const char * const version_string);
static        sid_error_t _prv_parse_hw_version_string(const char * const version_string);
static inline float       _prv_nmea_degree_minutes_to_degrees(const float64_t degrees_minutes_val);

static        sid_error_t _prv_assert_teseo_reset(void);
static        sid_error_t _prv_release_teseo_reset(void);
static        sid_error_t _prv_assert_teseo_wakeup(void);
static        sid_error_t _prv_release_teseo_wakeup(void);
static        sid_error_t _prv_sys_hw_reset(void);
static        sid_error_t _prv_sys_wakeup(void);
static        void        _prv_wakeup_timer_event_handler(void * arg, sid_pal_timer_t * originator);
static        sid_error_t _prv_gpio_init(void);
static        sid_error_t _prv_gpio_deinit(void);
static        sid_error_t _prv_uart_init(void);
static        sid_error_t _prv_uart_change_baud_rate(const uint32_t new_baud_rate);
static        sid_error_t _prv_uart_try_autodetect_baud_rate(void);
static        sid_error_t _prv_uart_baudrate_to_teseo_param(const uint32_t baud_rate, teseo_nmea_port_baudrate_t * const out_cfg_val);
static        sid_error_t _prv_uart_teseo_param_to_baudrate(const teseo_nmea_port_baudrate_t cfg_val, uint32_t * const out_baud_rate);

static        sid_error_t _prv_uart_tx_done(void * user_ctx);
static        sid_error_t _prv_uart_rx_done(void * user_ctx);

static        void        _prv_invalidate_gnss_data_storage(const eNMEAMsg data_message_type);
static inline uint32_t    _prv_is_teseo_data_available(const eNMEAMsg data_message_type);
static        sid_error_t _prv_wait_teseo_data_availability(const eNMEAMsg data_message_type, const uint32_t timeout_ms);
static        sid_error_t _prv_request_cdb_param(const teseo_cdb_param_id_t param_id, const teseo_cdb_block_id_t config_block);
static        sid_error_t _prv_set_cdb_int_param(const teseo_cdb_param_id_t param_id, const uint32_t value);
static inline void        _prv_invalidate_cdb_cache(void);
static        sid_error_t _prv_load_cdb_cache(teseo_cdb_cache_t * const cache_storage);

static inline uint32_t    _prv_are_all_positioning_timestamps_in_sync(void);
static inline uint32_t    _prv_are_all_positioning_messages_valid(void);
static inline void        _prv_invalidate_gnss_fix(void);
static inline void        _prv_trigger_gnss_fix_loss(void);
static inline void        _prv_on_gnss_fix_obtained(void);
static inline void        _prv_on_gnss_fix_lost(void);
static inline void        _prv_on_gnss_position_update(void);
static inline void        _prv_on_gnss_engine_restarted(void);

/* Private constants ---------------------------------------------------------*/

static const sid_pal_serial_callbacks_t teseo_uart_callbacks = {
    .tx_done_cb     = _prv_uart_tx_done,
    .new_rx_done_cb = _prv_uart_rx_done,
};

static sid_pal_serial_params_t teseo_uart_client_params = {
    .user_ctx  = NULL,
    .callbacks = &teseo_uart_callbacks,
};

/* Days per month (non-leap year) */
static const uint8_t days_in_month[] = {
    31u, 28u, 31u, 30u, 31u, 30u,
    31u, 31u, 30u, 31u, 30u, 31u,
};

/* List of the baud rates that are supported by Teseo. The order is form most probable ones to less probable ones */
static const teseo_baudrate_lut_entry_t teseo_supported_baud_rates[] = {
    { .baud_rate =   9600u, .cfg_val = TESEO_NMEA_BAUDRATE_9600   },
    { .baud_rate = 115200u, .cfg_val = TESEO_NMEA_BAUDRATE_115200 },
    { .baud_rate =  14400u, .cfg_val = TESEO_NMEA_BAUDRATE_14400  },
    { .baud_rate =  19200u, .cfg_val = TESEO_NMEA_BAUDRATE_19200  },
    { .baud_rate =  38400u, .cfg_val = TESEO_NMEA_BAUDRATE_38400  },
    { .baud_rate =  57600u, .cfg_val = TESEO_NMEA_BAUDRATE_57600  },
    { .baud_rate = 230400u, .cfg_val = TESEO_NMEA_BAUDRATE_230400 },
    { .baud_rate = 460800u, .cfg_val = TESEO_NMEA_BAUDRATE_460800 },
    { .baud_rate = 921600u, .cfg_val = TESEO_NMEA_BAUDRATE_921600 },
    { .baud_rate =   4800u, .cfg_val = TESEO_NMEA_BAUDRATE_4800   },
    { .baud_rate =   2400u, .cfg_val = TESEO_NMEA_BAUDRATE_2400   },
    { .baud_rate =   1200u, .cfg_val = TESEO_NMEA_BAUDRATE_1200   },
    { .baud_rate =    600u, .cfg_val = TESEO_NMEA_BAUDRATE_600    },
    { .baud_rate =    300u, .cfg_val = TESEO_NMEA_BAUDRATE_300    },
};

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _prv_is_leap_year(const uint32_t year)
{
    const uint32_t is_leap_year = (year % 4u == 0u) && ((year % 100u != 0u) || (year % 400u == 0u)) ? TRUE : FALSE;
    return is_leap_year;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static uint32_t _prv_timegm_like(const struct tm * const tm)
{
    uint32_t year  = (uint32_t)tm->tm_year + 1900u;
    uint32_t month = (uint32_t)tm->tm_mon;
    uint32_t day   = (uint32_t)tm->tm_mday - 1u; /* days start at 0 for calculation */
    uint32_t days  = 0u;

    /* Count full days in previous years */
    const uint32_t first_leap_year = 1972u;
    const uint32_t total_leap_years = ((year - 1u - first_leap_year) / 4u) + 1u;
    days = (total_leap_years * 366u) + (((year - 1970u) - total_leap_years) * 365u);

    /* Count full days in current year, up to current month */
    for (uint32_t m = 0u; m < month; m++)
    {
        days += days_in_month[m];
        if ((1u == m) && (_prv_is_leap_year(year) != FALSE))
        {
            days++; /* Add leap day in Feb */
        }
    }

    /* Add current month's days */
    days += day;

    /* Calculate total seconds */
    uint32_t seconds = days * 86400u +
                       (uint32_t)tm->tm_hour * 3600u +
                       (uint32_t)tm->tm_min * 60u +
                       (uint32_t)tm->tm_sec;

    return seconds;
}

/*----------------------------------------------------------------------------*/

#ifndef TESEO_GNSS_USE_STDLIB_PRINT
SID_STM32_SPEED_OPTIMIZED static inline int _prv_snprintf_like(char * buf, const size_t size, const char *fmt, ...)
{
    va_list args;
    int print_len;

    va_start(args, fmt);
    print_len = TESEO_GNSS_VSNPRINTF(buf, size, fmt, args);
    va_end(args);

    return print_len;
}
#endif /* TESEO_GNSS_USE_STDLIB_PRINT */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_parse_version_number_string(const char * const version_string, const char * prefix, const uint32_t prefix_len, teseo_gnss_version_num_t * const version_num_storage)
{
    sid_error_t err;

    do
    {
        uint32_t major, minor, patch, build;
        const char * parse_ptr;
        const char * post_parse_ptr;

        /* detect prefix */
        if (strncmp(version_string, prefix, prefix_len) != 0)
        {
            /* Prefix mismatch, abort parsing */
            err = SID_ERROR_INVALID_RESPONSE;
            break;
        }

        /* Parse major number */
        parse_ptr = post_parse_ptr = &version_string[prefix_len];
        major = strtol(parse_ptr, (char **)&post_parse_ptr, 10);
        if (parse_ptr == post_parse_ptr)
        {
            /* parsing error, abort */
            err = SID_ERROR_INVALID_RESPONSE;
            break;
        }

        /* Parse minor number */
        post_parse_ptr++;
        parse_ptr = post_parse_ptr;
        minor = strtol(parse_ptr, (char **)&post_parse_ptr, 10);
        if (parse_ptr == post_parse_ptr)
        {
            /* parsing error, abort */
            err = SID_ERROR_INVALID_RESPONSE;
            break;
        }

        /* Parse patch number */
        post_parse_ptr++;
        parse_ptr = post_parse_ptr;
        patch = strtol(parse_ptr, (char **)&post_parse_ptr, 10);
        if (parse_ptr == post_parse_ptr)
        {
            /* parsing error, abort */
            err = SID_ERROR_INVALID_RESPONSE;
            break;
        }

        /* Parse build number */
        post_parse_ptr++;
        parse_ptr = post_parse_ptr;
        build = strtol(parse_ptr, (char **)&post_parse_ptr, 10);
        if (parse_ptr == post_parse_ptr)
        {
            /* Build version may not always be present, it's fine  */
            build = 0u;
        }

        sid_pal_enter_critical_region();
        version_num_storage->major = (uint8_t)major;
        version_num_storage->minor = (uint8_t)minor;
        version_num_storage->patch = (uint8_t)patch;
        version_num_storage->build = (uint8_t)build;
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_parse_bin_image_version_string(const char * const version_string)
{
    sid_error_t err;

    /* Expected version string example: "BINIMG_4.6.8.2_ARM" */
    const char prefix[] = "BINIMG_";
    const uint32_t prefix_len = sizeof(prefix) - 1u;

    err = _prv_parse_version_number_string(version_string, prefix, prefix_len, &drv_ctx.version_info.bin_image);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_parse_gnss_lib_version_string(const char * const version_string)
{
    sid_error_t err;

    /* Expected version string example: "GNSSLIB_8.4.10.16.3_ARM" */
    const char prefix[] = "GNSSLIB_";
    const uint32_t prefix_len = sizeof(prefix) - 1u;

    err = _prv_parse_version_number_string(version_string, prefix, prefix_len, &drv_ctx.version_info.gnss_lib);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_parse_gps_app_version_string(const char * const version_string)
{
    sid_error_t err;

    /* Expected version string example: "GPSAPP_2.4.0.2_ARM" */
    const char prefix[] = "GPSAPP_";
    const uint32_t prefix_len = sizeof(prefix) - 1u;

    err = _prv_parse_version_number_string(version_string, prefix, prefix_len, &drv_ctx.version_info.gps_app);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_parse_os20_lib_version_string(const char * const version_string)
{
    sid_error_t err;

    /* Expected version string example: "OS20LIB_4.3.0_ARM" */
    const char prefix[] = "OS20LIB_";
    const uint32_t prefix_len = sizeof(prefix) - 1u;

    err = _prv_parse_version_number_string(version_string, prefix, prefix_len, &drv_ctx.version_info.os20_lib);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_parse_hw_version_string(const char * const version_string)
{
    sid_error_t err;

    do
    {
        /* Expected version string example: "STA8090_822bc043" */

        /* Search delimiter position */
        const char * end_ptr = strstr((char *)version_string, "_");
        if (NULL == end_ptr)
        {
            err = SID_ERROR_INVALID_RESPONSE;
            break;
        }

        /* Copy device name */
        uint32_t bytes_to_copy = (uint32_t)(void *)end_ptr - (uint32_t)(void *)version_string;
        if (bytes_to_copy >= sizeof(drv_ctx.version_info.dev_name))
        {
            bytes_to_copy = sizeof(drv_ctx.version_info.dev_name) - 1u;
        }

        /* Copy as many bytes as fits into storage */
        sid_pal_enter_critical_region();
        SID_STM32_UTIL_fast_memcpy(drv_ctx.version_info.dev_name, version_string, bytes_to_copy);
        drv_ctx.version_info.dev_name[bytes_to_copy] = '\0';
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline float _prv_nmea_degree_minutes_to_degrees(const float64_t degrees_minutes_val)
{
    const float64_t nmea_degrees = trunc(degrees_minutes_val / 100.0);
    const float64_t nmea_minutes = fmod(degrees_minutes_val, 100.0);
    const float degrees = (float32_t)(nmea_degrees + (nmea_minutes / 60.0));

    return degrees;
}


/*----------------------------------------------------------------------------*/

static sid_error_t _prv_assert_teseo_reset(void)
{
    sid_error_t err;

    err = sid_pal_gpio_write(drv_ctx.config->gpio.reset, 0u);
    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.reset);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_release_teseo_reset(void)
{
    sid_error_t err;

    err = sid_pal_gpio_write(drv_ctx.config->gpio.reset, 1u);
    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.reset);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_assert_teseo_wakeup(void)
{
    sid_error_t err;

    err = sid_pal_gpio_write(drv_ctx.config->gpio.wakeup, 1u);
    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.wakeup);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_release_teseo_wakeup(void)
{
    sid_error_t err;

    err = sid_pal_gpio_write(drv_ctx.config->gpio.wakeup, 0u);
    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.wakeup);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_sys_hw_reset(void)
{
    sid_error_t err;

    SID_PAL_ASSERT(drv_ctx.config != NULL);

    do
    {
        /* Trigger reset */
        err = _prv_assert_teseo_reset();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Wait for reset to take effect */
        sid_pal_scheduler_delay_ms(TESEO_GNSS_RESET_HOLD_TIME_MS);

        /* Release reset */
        err = _prv_release_teseo_reset();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_sys_wakeup(void)
{
    sid_error_t err;

    SID_PAL_ASSERT(drv_ctx.config != NULL);

    do
    {
        /* Trigger wakeup pin */
        err = _prv_assert_teseo_wakeup();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Keep Wakeup signal asserted for the predefined time */
        sid_pal_scheduler_delay_ms(TESEO_GNSS_WAKEUP_HOLD_TIME_MS);

        /* Wakeup pin reset */
        err = _prv_release_teseo_wakeup();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _prv_wakeup_timer_event_handler(void * arg, sid_pal_timer_t * originator)
{
    sid_error_t err;

    do
    {
        SID_PAL_ASSERT(drv_ctx.init_done                != FALSE);
        SID_PAL_ASSERT(drv_ctx.uart_ifc                 != NULL);
        SID_PAL_ASSERT((*drv_ctx.uart_ifc)->resume_uart != NULL);
        SID_PAL_ASSERT((*drv_ctx.uart_ifc)->start_rx    != NULL);

        /* Re-enable UART peripheral */
        err = (*drv_ctx.uart_ifc)->resume_uart(drv_ctx.uart_ifc);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Restart continuous UART Rx */
        err = (*drv_ctx.uart_ifc)->start_rx(drv_ctx.uart_ifc);
        if (err != SID_ERROR_NONE)
        {
            break;
        }
    } while (0);
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_gpio_init(void)
{
    sid_error_t err;

    do
    {
        /* Configure Teseo reset pin -------------------------------------------------*/
        err = _prv_assert_teseo_reset();
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to assert Teseo reset line. error %d", (int32_t)err);
            break;
        }

        err = sid_pal_gpio_output_mode(drv_ctx.config->gpio.reset, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.reset);
            break;
        }

        err = sid_pal_gpio_set_direction(drv_ctx.config->gpio.reset, SID_PAL_GPIO_DIRECTION_OUTPUT);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.reset);
            break;
        }

        /* Wait for reset to take effect */
        sid_pal_scheduler_delay_ms(TESEO_GNSS_RESET_HOLD_TIME_MS);
        /*----------------------------------------------------------------------------*/

        /* Configure Teseo wakeup pin -------------------------------------------------*/
        err = _prv_release_teseo_wakeup();
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to de-assert Teseo wakeup line. error %d", (int32_t)err);
            break;
        }

        err = sid_pal_gpio_output_mode(drv_ctx.config->gpio.wakeup, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.wakeup);
            break;
        }

        err = sid_pal_gpio_set_direction(drv_ctx.config->gpio.wakeup, SID_PAL_GPIO_DIRECTION_OUTPUT);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.wakeup);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_gpio_deinit(void)
{
    sid_error_t err;

    do
    {
        /* Set Teseo reset pin to Hi-Z -----------------------------------------------*/
        err = sid_pal_gpio_input_mode(drv_ctx.config->gpio.reset, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.reset);
            break;
        }

        err = sid_pal_gpio_set_direction(drv_ctx.config->gpio.reset, SID_PAL_GPIO_DIRECTION_INPUT);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.reset);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Set Teseo wakeup pin to Hi-Z  ---------------------------------------------*/
        err = sid_pal_gpio_input_mode(drv_ctx.config->gpio.wakeup, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.wakeup);
            break;
        }

        err = sid_pal_gpio_set_direction(drv_ctx.config->gpio.wakeup, SID_PAL_GPIO_DIRECTION_INPUT);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)err, drv_ctx.config->gpio.wakeup);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_uart_init(void)
{
    sid_error_t err;

    do
    {
        /* Create extended UART interface */
        err = sid_pal_uart_client_create_ext(&drv_ctx.uart_ifc, drv_ctx.config->uart_cfg, &teseo_uart_client_params);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Ensure the inital UART baud rate is aligned with the desired/expected speed */
        uint32_t target_baud_rate;
        if ((drv_ctx.init_done != FALSE) && (drv_ctx.cdb_cache.validity.nmea_port_baudrate != FALSE))
        {
            (void)_prv_uart_teseo_param_to_baudrate(drv_ctx.cdb_cache.nmea_port_baudrate.br_cfg_val, &target_baud_rate);
        }
        else
        {
            target_baud_rate = drv_ctx.config->uart_cfg->baud_rate;
        }

        err = _prv_uart_change_baud_rate(target_baud_rate);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _prv_uart_change_baud_rate() */
            break;
        }

        /* Start continuous Rx */
        err = (*drv_ctx.uart_ifc)->start_rx(drv_ctx.uart_ifc);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_uart_change_baud_rate(const uint32_t new_baud_rate)
{
    sid_error_t err;

    do
    {
        uint32_t current_baud_rate;

        err = (*drv_ctx.uart_ifc)->get_current_baud_rate(drv_ctx.uart_ifc, &current_baud_rate);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Check if baud rate change is actually required */
        if (new_baud_rate == current_baud_rate)
        {
            err = SID_ERROR_NONE;
            break;
        }

        /* Suspend all communication before baud rate change */
        err = (*drv_ctx.uart_ifc)->suspend_uart(drv_ctx.uart_ifc);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Apply new baud rate */
        err = (*drv_ctx.uart_ifc)->set_baud_rate(drv_ctx.uart_ifc, new_baud_rate);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Resume UART operation */
        err = (*drv_ctx.uart_ifc)->resume_uart(drv_ctx.uart_ifc);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Restart continuous Rx */
        err = (*drv_ctx.uart_ifc)->start_rx(drv_ctx.uart_ifc);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_uart_try_autodetect_baud_rate(void)
{
    sid_error_t err;

    do
    {
        char cmd[] = "$PSTMGETUCODE";

        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(teseo_supported_baud_rates); i++)
        {
            SID_PAL_LOG_INFO("Trying to communicate with Teseo at %ubps...", teseo_supported_baud_rates[i].baud_rate);

            /* Change UART baud rate */
            err = _prv_uart_change_baud_rate(teseo_supported_baud_rates[i].baud_rate);
            if (err != SID_ERROR_NONE)
            {
                /* Logs provided by _prv_uart_change_baud_rate() */
                break;
            }

            /* Invalidate data in the storage */
            _prv_invalidate_gnss_data_storage(PSTMGETUCODE);

            /* Send command to Teseo */
            err = teseo_gnss_send_command(cmd);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to send Teseo Get Unique Code command. Error %d", (int32_t)err);
                break;
            }

            /* Wait for response */
            err = _prv_wait_teseo_data_availability(PSTMGETUCODE, TESEO_GNSS_PSTMGETUCODE_CMD_TIMEOUT_MS);
            if (SID_ERROR_NONE == err)
            {
                /* Valid TEseo response detected, current baud rate is a valid one */
                SID_PAL_LOG_INFO("Auto-detected Teseo baud rate is %u", teseo_supported_baud_rates[i].baud_rate);
                break;
            }
            else
            {
                if (SID_ERROR_TIMEOUT == err)
                {
                    /* Timeout error is an expected one if baud rate is wrong */
                    continue;
                }
                else
                {
                    /* Any other other error indicates some serious issues, can't proceed */
                    SID_PAL_LOG_ERROR("Failed to read Teseo Unique Code. Error %d", (int32_t)err);
                    break;
                }
            }
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_uart_baudrate_to_teseo_param(const uint32_t baud_rate, teseo_nmea_port_baudrate_t * const out_cfg_val)
{
    sid_error_t err;

    do
    {
        if (NULL == out_cfg_val)
        {
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        err = SID_ERROR_NOSUPPORT;

        for (uint32_t i = 0u; SID_STM32_UTIL_ARRAY_SIZE(teseo_supported_baud_rates); i++)
        {
            if (baud_rate == teseo_supported_baud_rates[i].baud_rate)
            {
                *out_cfg_val = teseo_supported_baud_rates[i].cfg_val;
                err          = SID_ERROR_NONE;
                break;
            }
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_uart_teseo_param_to_baudrate(const teseo_nmea_port_baudrate_t cfg_val, uint32_t * const out_baud_rate)
{
    sid_error_t err;

    do
    {
        if (NULL == out_baud_rate)
        {
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        err = SID_ERROR_NOSUPPORT;

        for (uint32_t i = 0u; SID_STM32_UTIL_ARRAY_SIZE(teseo_supported_baud_rates); i++)
        {
            if (cfg_val == teseo_supported_baud_rates[i].cfg_val)
            {
                *out_baud_rate = teseo_supported_baud_rates[i].baud_rate;
                err            = SID_ERROR_NONE;
                break;
            }
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_uart_tx_done(void * user_ctx)
{
    (void)user_ctx;

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_uart_rx_done(void * user_ctx)
{
    sid_error_t err;
    GNSSParser_Status_t parser_status;

    (void)user_ctx;

    do
    {
        uint8_t * rx_data;
        size_t rx_data_size;

        /* Pop message from the UART driver */
        err = (*drv_ctx.uart_ifc)->get_frame(drv_ctx.uart_ifc, &rx_data, &rx_data_size);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Check if the message is command mirroring by Teseo */
        if (SID_STM32_UTIL_fast_memcmp(drv_ctx.last_command, rx_data, rx_data_size <= sizeof(drv_ctx.last_command) ? rx_data_size : sizeof(drv_ctx.last_command)) == 0)
        {
            /* This is a command mirror message for the most recent command, no parsing needed */
            (void)osSemaphoreRelease(drv_ctx.cmd_completion_lock);
            err = SID_ERROR_NONE;
            break;
        }

        /* Do a basic sanity check */
        parser_status = GNSS_PARSER_CheckSanity(rx_data, rx_data_size);
        if (parser_status != GNSS_PARSER_OK)
        {
            SID_PAL_LOG_WARNING("Received a corrupted message from Teseo (checksum mismatch)");
            err = SID_ERROR_NONE; /* Report no error to the UART driver - this is app-level protocol issue, not the UART layer problem */
            break;
        }

        /* Indicate Teseo is not in Standby mode any longer if any valid NMEA message is received */
        drv_ctx.in_standby = FALSE;

        /* Parse the message */
        for (uint32_t msg_type = 0u; msg_type < NMEA_MSGS_NUM; msg_type++)
        {
            sid_pal_enter_critical_region();
            parser_status = GNSS_PARSER_ParseMsg(&drv_ctx.gnss_data, (eNMEAMsg)msg_type, rx_data);
            sid_pal_exit_critical_region();

            if (parser_status != GNSS_PARSER_OK)
            {
                /* Unsuccessful, try to parse as a different type of message */
                continue;
            }
            else
            {
                /* Release the associated semaphore to indicate data availability */
                (void)osSemaphoreRelease(drv_ctx.gnss_data_locks[msg_type]);

                /* Parsed successfully, do message type-specific post-processing */
                switch ((eNMEAMsg)msg_type)
                {
                    case GPTXT:
                        if ((drv_ctx.cdb_cache.validity.app_cfg1 != FALSE) && (drv_ctx.cdb_cache.app_cfg1.CONFIG_TXT_HEADER_EN != FALSE)
                          && (drv_ctx.cdb_cache.validity.startup_msg != FALSE) && (strcmp(drv_ctx.cdb_cache.startup_msg, drv_ctx.gnss_data.gptxt_data.log_msg) == 0))
                        {
                            /* Capture message is a startup message */
                            _prv_on_gnss_engine_restarted();
                        }
                        else
                        {
                            /* Print out Teseo module log message as is */
                            SID_PAL_LOG_INFO(TESEO_GNSS_GPTXT_LOG_PREFIX "%s", drv_ctx.gnss_data.gptxt_data.log_msg);
                        }
                        break;

                    case GPGGA:
                    case GPRMC:
                    case GLL:
#if TESEO_GNSS_CFG_WAIT_FOR_ACURACY_DATA
                    case GPGST:
#endif /* TESEO_GNSS_CFG_WAIT_FOR_ACURACY_DATA */
                        sid_pal_enter_critical_region();

                        /* Check for Fix Obtained / Fix Lost events */
                        if ((FALSE == drv_ctx.gnss_fix_obtained) && (_prv_are_all_positioning_messages_valid() != FALSE) && (_prv_are_all_positioning_timestamps_in_sync() != FALSE))
                        {
                            _prv_on_gnss_fix_obtained();
                        }
                        else if ((drv_ctx.gnss_fix_obtained != FALSE) && (_prv_are_all_positioning_messages_valid() == FALSE))
                        {
                            _prv_on_gnss_fix_lost();
                        }
                        else
                        {
                            /* No actions required */
                        }

                        /* Check for Position Update event */
                        if ((drv_ctx.gnss_fix_obtained != FALSE) && (_prv_are_all_positioning_timestamps_in_sync() != FALSE))
                        {
                            _prv_on_gnss_position_update();
                        }

                        sid_pal_exit_critical_region();
                        break;

                    case  PSTMGPSSUSPEND:
                        if (GNSS_OP_OK == drv_ctx.gnss_data.result)
                        {
                            /* Artificially trigger GNSS fix loss event since GNSS engine is stopped and won't provide any updates */
                            _prv_trigger_gnss_fix_loss();
                        }
                        break;

                    case PSTMFORCESTANDBY:
                        if (GNSS_OP_OK == drv_ctx.gnss_data.result)
                        {
                            /* Indicate Standby state is active and UART communication is ceased */
                            drv_ctx.in_standby = TRUE;

                            /* Artificially trigger GNSS fix loss event since Teseo is now in the Standby state and won't provide any updates */
                            _prv_trigger_gnss_fix_loss();
                        }
                        break;

                    default:
                        break;
                }

                /* Message parsed, loop can be terminated */
                break;
            }
        }

        /* Printout unknown messages if CDB cache is synchronized, otherwise parsing may fail due to mismatch between the driver and actual configuration */
        if ((parser_status != GNSS_PARSER_OK) && (drv_ctx.cdb_cache.validity.raw != 0u))
        {
            /* Unknown/unsupported message detected - print out message type only */
            char * print_limit_ptr = (char *)&rx_data[0];
            while ((*print_limit_ptr != '*') && (*print_limit_ptr != ',') && (*print_limit_ptr != '\r') && (*print_limit_ptr != '\n') && (*print_limit_ptr != '\0'))
            {
                print_limit_ptr++;
            }
            const char delim_symbol = *print_limit_ptr;
            *print_limit_ptr = '\0';
            SID_PAL_LOG_WARNING("Failed to parse Teseo message - unknown type (\"%s\")", rx_data);
            *print_limit_ptr = delim_symbol;
        }

        /* Mirror Teseo message to the log console if requested */
        if (drv_ctx.echo_to_logs != FALSE)
        {
            rx_data[rx_data_size - 2u] = '\0'; /* Skip \r\n for logging */
            teseo_gnss_print((char *)rx_data, SID_PAL_LOG_SEVERITY_INFO);
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static void _prv_invalidate_gnss_data_storage(const eNMEAMsg data_message_type)
{
    SID_PAL_ASSERT(drv_ctx.gnss_data_locks[data_message_type] != NULL);

    sid_pal_enter_critical_region();

    while (osSemaphoreGetCount(drv_ctx.gnss_data_locks[data_message_type]) != 0u)
    {
        (void)osSemaphoreAcquire(drv_ctx.gnss_data_locks[data_message_type], 0u);
    }

    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

static inline uint32_t _prv_is_teseo_data_available(const eNMEAMsg data_message_type)
{
    uint32_t data_available = FALSE;

    SID_PAL_ASSERT(drv_ctx.gnss_data_locks[data_message_type] != NULL);

    if (osSemaphoreGetCount(drv_ctx.gnss_data_locks[data_message_type]) > 0u)
    {
        data_available = TRUE;
    }

    return data_available;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_wait_teseo_data_availability(const eNMEAMsg data_message_type, const uint32_t timeout_ms)
{
    sid_error_t err;

    SID_PAL_ASSERT(drv_ctx.gnss_data_locks[data_message_type] != NULL);

    do
    {
        uint32_t timeout_os_ticks = (TESEO_GNSS_TIMEOUT_INFINITE == timeout_ms) ? osWaitForever : TESEO_GNSS_MS_TO_OS_TICKS(timeout_ms);

        osStatus_t os_status = osSemaphoreAcquire(drv_ctx.gnss_data_locks[data_message_type], timeout_os_ticks);

        switch (os_status)
        {
            case osOK:
                /* Release the semaphore after the wait is finished successfully as an indication of data validity */
                (void)osSemaphoreRelease(drv_ctx.gnss_data_locks[data_message_type]);
                err = SID_ERROR_NONE;
                break;

            case osErrorParameter: /* This error is reported from IRQ context if timeout is non-zero since waiting in IRQ is not an option */
            case osErrorTimeout:
                err = SID_ERROR_TIMEOUT;
                break;

            default:
                err = SID_ERROR_GENERIC;
                break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_request_cdb_param(const teseo_cdb_param_id_t param_id, const teseo_cdb_block_id_t config_block)
{
    sid_error_t err;

    do
    {
        char cmd[20];

        const uint32_t bytes_written = (uint32_t)TESEO_GNSS_SNPRINTF(cmd, sizeof(cmd), "$PSTMGETPAR,%u%u", (uint32_t)config_block, (uint32_t)param_id);
        if (bytes_written >= (sizeof(cmd) - 1u))
        {
            SID_PAL_LOG_ERROR("Unable to request CDB parameter %u - command buffer overflow", param_id);
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Invalidate data in the storage */
        _prv_invalidate_gnss_data_storage(PSTMSETPAR);

        /* Send command to Teseo */
        err = teseo_gnss_send_command(cmd);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to request Teseo CDB parameter %u. Error %d", (uint32_t)param_id, (int32_t)err);
            break;
        }

        /* Wait for response */
        err = _prv_wait_teseo_data_availability(PSTMSETPAR, TESEO_GNSS_DEFAULT_CMD_TIMEOUT_MS);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to retrieve Teseo CDB parameter %u. Error %d", (uint32_t)param_id, (int32_t)err);
            break;
        }

        /* Check if Teseo replied with good result */
        if (drv_ctx.gnss_data.pstmsetpar_data.result != GNSS_OP_OK)
        {
            SID_PAL_LOG_ERROR("Teseo CDB parameter %u read rejected. Make sure parameter ID and CDB block is valid", (uint32_t)param_id);
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_set_cdb_int_param(const teseo_cdb_param_id_t param_id, const uint32_t value)
{
    sid_error_t err;

    do
    {
        char cmd[32];

        const uint32_t bytes_written = (uint32_t)TESEO_GNSS_SNPRINTF(cmd, sizeof(cmd), "$PSTMSETPAR,%u%u,%X,", TESEO_CDB_CURRENT_CONFIG, (uint32_t)param_id, value); /* Config Block Id is ignored by $PSTMSETPAR command, parameter is always written to RAM */
        if (bytes_written >= (sizeof(cmd) - 1u))
        {
            SID_PAL_LOG_ERROR("Unable to set CDB parameter %u - command buffer overflow", param_id);
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Invalidate data in the storage */
        _prv_invalidate_gnss_data_storage(PSTMSETPAR);

        /* Send command to Teseo */
        err = teseo_gnss_send_command(cmd);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to send value for Teseo CDB parameter %u. Error %d", (uint32_t)param_id, (int32_t)err);
            break;
        }

        /* Wait for response */
        err = _prv_wait_teseo_data_availability(PSTMSETPAR, TESEO_GNSS_DEFAULT_CMD_TIMEOUT_MS);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to acknowledge Teseo CDB parameter %u update. Error %d", (uint32_t)param_id, (int32_t)err);
            break;
        }

        /* Check if Teseo replied with good result */
        if (drv_ctx.gnss_data.pstmsetpar_data.result != GNSS_OP_OK)
        {
            SID_PAL_LOG_ERROR("Teseo CDB parameter %u write rejected. Make sure parameter ID and parameter value is valid", (uint32_t)param_id);
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static inline void _prv_invalidate_cdb_cache(void)
{
    sid_pal_enter_critical_region();
    drv_ctx.cdb_cache.validity.raw = 0u;
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_load_cdb_cache(teseo_cdb_cache_t * const cache_storage)
{
    sid_error_t err;

    SID_PAL_ASSERT(cache_storage != NULL);

    do
    {
        teseo_cdb_cache_t local_storage;

        /* Teseo's UART baud rate: CDB 102 */
        err = teseo_gnss_request_nmea_port_baudrate_config(&local_storage.nmea_port_baudrate);
        if (err != SID_ERROR_NONE)
        {
            break;
        }
        local_storage.validity.nmea_port_baudrate = TRUE;

        /* Teseo's GPS App config: CDB 200 and 227 */
        err = teseo_gnss_request_app_config(&local_storage.app_cfg1, &local_storage.app_cfg2);
        if (err != SID_ERROR_NONE)
        {
            break;
        }
        local_storage.validity.app_cfg1 = TRUE;
        local_storage.validity.app_cfg2 = TRUE;

        /* UART message list configuration: CDB 201 and 228 */
        err = teseo_gnss_request_message_config(&local_storage.msg_list_low, &local_storage.msg_list_high);
        if (err != SID_ERROR_NONE)
        {
            break;
        }
        local_storage.validity.msg_list_low = TRUE;
        local_storage.validity.msg_list_high = TRUE;

        /* Startup message text */
        err = teseo_gnss_request_startup_message_text(local_storage.startup_msg, sizeof(local_storage.startup_msg));
        if (err != SID_ERROR_NONE)
        {
            break;
        }
        local_storage.validity.startup_msg = TRUE;

        sid_pal_enter_critical_region();
        SID_STM32_UTIL_fast_memcpy(cache_storage, &local_storage, sizeof(*cache_storage));
        sid_pal_exit_critical_region();
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _prv_are_all_positioning_timestamps_in_sync(void)
{
    uint32_t in_sync;
    const UTC_Info_t * reference_timestamp = NULL;

    /* WARNING: this method should be called within a critical section. It's the responsibility of the caller to ensure that */

    do
    {
        /* Ensure CDB cache validity to ensure further checks are valid */
        if ((FALSE == drv_ctx.cdb_cache.validity.msg_list_low) || (FALSE == drv_ctx.cdb_cache.validity.msg_list_high))
        {
            in_sync = FALSE;
            break;
        }

        /* Check GPGGA validity only if this message is enabled */
        if ((drv_ctx.cdb_cache.msg_list_low.GPGGA_EN != FALSE) && (_prv_is_teseo_data_available(GPGGA) != FALSE))
        {
            /* Set reference timestamp pointer */
            SID_PAL_ASSERT(NULL == reference_timestamp);
            reference_timestamp = &drv_ctx.gnss_data.gpgga_data.utc;
        }

        /* Check GPRMC validity only if this message is enabled */
        if ((drv_ctx.cdb_cache.msg_list_low.GPRMC_EN != FALSE) && (_prv_is_teseo_data_available(GPRMC) != FALSE))
        {
            /* Set reference timestamp pointer if it is not set yet */
            if (NULL == reference_timestamp)
            {
                reference_timestamp = &drv_ctx.gnss_data.gprmc_data.utc;
            }
            else
            {
                /* Compare GPRMC timestamp to the reference one */
                if (SID_STM32_UTIL_fast_memcmp(&drv_ctx.gnss_data.gprmc_data.utc, reference_timestamp, sizeof(*reference_timestamp)) != 0u)
                {
                    /* Timestamp mismatch - messages belong to different reporting cycles */
                    in_sync = FALSE;
                    break;
                }
            }
        }

        /* Check GPGLL validity only if this message is enabled */
        if ((drv_ctx.cdb_cache.msg_list_low.GPGLL_EN != FALSE) && (_prv_is_teseo_data_available(GLL) != FALSE))
        {
            /* Set reference timestamp pointer if it is not set yet */
            if (NULL == reference_timestamp)
            {
                reference_timestamp = &drv_ctx.gnss_data.gll_data.utc;
            }
            else
            {
                /* Compare GPGLL timestamp to the reference one */
                if (SID_STM32_UTIL_fast_memcmp(&drv_ctx.gnss_data.gll_data.utc, reference_timestamp, sizeof(*reference_timestamp)) != 0u)
                {
                    /* Timestamp mismatch - messages belong to different reporting cycles */
                    in_sync = FALSE;
                    break;
                }
            }
        }

#if TESEO_GNSS_CFG_WAIT_FOR_ACURACY_DATA
        /* Check GPGST validity only if this message is enabled */
        if ((drv_ctx.cdb_cache.msg_list_low.GPGST_EN != FALSE) && (_prv_is_teseo_data_available(GPGST) != FALSE))
        {
            /* Set reference timestamp pointer if it is not set yet */
            if (NULL == reference_timestamp)
            {
                reference_timestamp = &drv_ctx.gnss_data.gpgst_data.utc;
            }
            else
            {
                /* Compare GPGST timestamp to the reference one */
                if (SID_STM32_UTIL_fast_memcmp(&drv_ctx.gnss_data.gpgst_data.utc, reference_timestamp, sizeof(*reference_timestamp)) != 0u)
                {
                    /* Timestamp mismatch - messages belong to different reporting cycles */
                    in_sync = FALSE;
                    break;
                }
            }
        }
#endif /* TESEO_GNSS_CFG_WAIT_FOR_ACURACY_DATA */

        /* Done */
        if (reference_timestamp != NULL) /* At least one relevant message is received */
        {
            in_sync = TRUE;
        }
        else
        {
            in_sync = FALSE;
        }
    } while (0);

    return in_sync;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _prv_are_all_positioning_messages_valid(void)
{
    uint32_t messages_valid;

    /* WARNING: this method should be called within a critical section. It's the responsibility of the caller to ensure that */

    do
    {
        /* Ensure CDB cache validity to ensure further checks are valid */
        if ((FALSE == drv_ctx.cdb_cache.validity.msg_list_low) || (FALSE == drv_ctx.cdb_cache.validity.msg_list_high))
        {
            messages_valid = FALSE;
            break;
        }

        /* At least one message with position data should be enabled */
        if ((FALSE == drv_ctx.cdb_cache.msg_list_low.GPGGA_EN) && (FALSE == drv_ctx.cdb_cache.msg_list_low.GPRMC_EN) && (FALSE == drv_ctx.cdb_cache.msg_list_low.GPGLL_EN))
        {
            messages_valid = FALSE;
            break;
        }

        /* Check GPGGA validity only if this message is enabled */
        if (drv_ctx.cdb_cache.msg_list_low.GPGGA_EN != FALSE)
        {
            if ((_prv_is_teseo_data_available(GPGGA) == FALSE) || (INVALID == drv_ctx.gnss_data.gpgga_data.valid))
            {
                /* No valid GPGGA received yet */
                messages_valid = FALSE;
                break;
            }
        }

        /* Check GPRMC validity only if this message is enabled */
        if (drv_ctx.cdb_cache.msg_list_low.GPRMC_EN != FALSE)
        {
            if ((_prv_is_teseo_data_available(GPRMC) == FALSE) || ('V' == drv_ctx.gnss_data.gprmc_data.status))
            {
                /* No valid GPRMC received yet */
                messages_valid = FALSE;
                break;
            }
        }

        /* Check GPGLL validity only if this message is enabled */
        if (drv_ctx.cdb_cache.msg_list_low.GPGLL_EN != FALSE)
        {
            if ((_prv_is_teseo_data_available(GLL) == FALSE) || (INVALID == drv_ctx.gnss_data.gll_data.valid))
            {
                /* No valid GPGLL received yet */
                messages_valid = FALSE;
                break;
            }
        }

#if TESEO_GNSS_CFG_WAIT_FOR_ACURACY_DATA
        /* Check GPGST validity only if this message is enabled */
        if (drv_ctx.cdb_cache.msg_list_low.GPGST_EN != FALSE)
        {
            if ((_prv_is_teseo_data_available(GPGST) == FALSE) || (0.f == drv_ctx.gnss_data.gpgst_data.EHPE) || (0.f == drv_ctx.gnss_data.gpgst_data.alt_err_dev))
            {
                /* No valid GPGST received yet */
                messages_valid = FALSE;
                break;
            }
        }
#endif /* TESEO_GNSS_CFG_WAIT_FOR_ACURACY_DATA */

        /* Done */
        messages_valid = TRUE;
    } while (0);

    return messages_valid;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _prv_invalidate_gnss_fix(void)
{
    sid_pal_enter_critical_region();
    _prv_invalidate_gnss_data_storage(GPGGA);
    _prv_invalidate_gnss_data_storage(GPRMC);
    _prv_invalidate_gnss_data_storage(GLL);
    _prv_invalidate_gnss_data_storage(GPGST);
    drv_ctx.gnss_fix_obtained = FALSE;
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _prv_trigger_gnss_fix_loss(void)
{
    /* Invalidate GNSS fix if it was available */
    sid_pal_enter_critical_region();
    if (drv_ctx.gnss_fix_obtained != FALSE)
    {
        _prv_invalidate_gnss_fix();
        _prv_on_gnss_fix_lost();
    }
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _prv_on_gnss_fix_obtained(void)
{
    SID_PAL_LOG_INFO(TESEO_GNSS_GPTXT_LOG_PREFIX "GNSS fix obtained");

    /* WARNING: this method should be called within a critical section. It's the responsibility of the caller to ensure that */

    /* Lock GNSS Fix Lost semaphore and release GNSS Fix Obtained semaphore */
    while (osSemaphoreGetCount(drv_ctx.gnss_fix_lost_sem) != 0u)
    {
        (void)osSemaphoreAcquire(drv_ctx.gnss_fix_lost_sem, 0u);
    }
    (void)osSemaphoreRelease(drv_ctx.gnss_fix_obtained_sem);
    drv_ctx.gnss_fix_obtained = TRUE;

    if (drv_ctx.user_callbacks.on_gnss_fix_obtained != NULL)
    {
        drv_ctx.user_callbacks.on_gnss_fix_obtained(drv_ctx.user_callbacks.user_arg);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _prv_on_gnss_fix_lost(void)
{
    SID_PAL_LOG_INFO(TESEO_GNSS_GPTXT_LOG_PREFIX "GNSS fix lost");

    /* WARNING: this method should be called within a critical section. It's the responsibility of the caller to ensure that */

    /* Lock GNSS Fix Obtained semaphore and release GNSS Fix Lost semaphore */
    while (osSemaphoreGetCount(drv_ctx.gnss_fix_obtained_sem) != 0u)
    {
        (void)osSemaphoreAcquire(drv_ctx.gnss_fix_obtained_sem, 0u);
    }
    (void)osSemaphoreRelease(drv_ctx.gnss_fix_lost_sem);
    drv_ctx.gnss_fix_obtained = FALSE;

    if (drv_ctx.user_callbacks.on_gnss_fix_lost != NULL)
    {
        drv_ctx.user_callbacks.on_gnss_fix_lost(drv_ctx.user_callbacks.user_arg);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _prv_on_gnss_position_update(void)
{
    /* WARNING: this method should be called within a critical section. It's the responsibility of the caller to ensure that */

    /* Compute latitude value - NMEA message contains data in ddmm.(m) format (dd - degrees, mm - minutes) - we need to recalculate the minutes into fractional degrees */
    drv_ctx.latest_position.latitude = _prv_nmea_degree_minutes_to_degrees(drv_ctx.gnss_data.gpgga_data.xyz.lat);
    if ('S' == drv_ctx.gnss_data.gpgga_data.xyz.ns)
    {
        /* Adjust sign for south */
        drv_ctx.latest_position.latitude = -drv_ctx.latest_position.latitude;
    }

    /* Compute longitude value - similarly, we need to recalculate the minutes into fractional degrees */
    drv_ctx.latest_position.longitude = _prv_nmea_degree_minutes_to_degrees(drv_ctx.gnss_data.gpgga_data.xyz.lon);
    if ('W' == drv_ctx.gnss_data.gpgga_data.xyz.ew)
    {
        /* Adjust sign for west */
        drv_ctx.latest_position.longitude = -drv_ctx.latest_position.longitude;
    }

    /* Extract horizontal accuracy data */
    if (drv_ctx.gnss_data.gpgst_data.EHPE != 0.f)
    {
        drv_ctx.latest_position.horizontal_accuracy = drv_ctx.gnss_data.gpgst_data.EHPE;
    }
    else
    {
        drv_ctx.latest_position.horizontal_accuracy = NAN;
    }

    /* Compute elevation over WGS84 and extract accuracy data */
    drv_ctx.latest_position.elevation = drv_ctx.gnss_data.gpgga_data.xyz.alt + drv_ctx.gnss_data.gpgga_data.geoid.height;
    if (drv_ctx.gnss_data.gpgst_data.alt_err_dev != 0.f)
    {
        drv_ctx.latest_position.vertical_accuracy = drv_ctx.gnss_data.gpgst_data.alt_err_dev;
    }
    else
    {
        drv_ctx.latest_position.vertical_accuracy = NAN;
    }

    /* Populate timestamp information */
    drv_ctx.latest_position.utc_time.tm_year  = (drv_ctx.gnss_data.gprmc_data.date % 100) + 100; /* tm_year is years since 1900 */
    drv_ctx.latest_position.utc_time.tm_mon   = ((drv_ctx.gnss_data.gprmc_data.date / 100) % 100) - 1; /* Months since January [0–11] */
    drv_ctx.latest_position.utc_time.tm_mday  = (drv_ctx.gnss_data.gprmc_data.date / 10000);
    drv_ctx.latest_position.utc_time.tm_hour  = drv_ctx.gnss_data.gpgga_data.utc.hh;
    drv_ctx.latest_position.utc_time.tm_min   = drv_ctx.gnss_data.gpgga_data.utc.mm;
    drv_ctx.latest_position.utc_time.tm_sec   = drv_ctx.gnss_data.gpgga_data.utc.ss;
    drv_ctx.latest_position.utc_time.tm_isdst = 0;

    /* Compute POSIX time for the timestamp */
    drv_ctx.latest_position.posix_time = _prv_timegm_like(&drv_ctx.latest_position.utc_time);

    if (drv_ctx.user_callbacks.on_gnss_position_update != NULL)
    {
        drv_ctx.user_callbacks.on_gnss_position_update(&drv_ctx.latest_position, drv_ctx.user_callbacks.user_arg);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _prv_on_gnss_engine_restarted(void)
{
    SID_PAL_LOG_INFO(TESEO_GNSS_GPTXT_LOG_PREFIX "GNSS engine (re)start completed");

    (void)osSemaphoreRelease(drv_ctx.gnss_engine_restart_sem);

    if (drv_ctx.user_callbacks.on_gnss_engine_restarted != NULL)
    {
        drv_ctx.user_callbacks.on_gnss_engine_restarted(drv_ctx.user_callbacks.user_arg);
    }
}

/* Global function definitions -----------------------------------------------*/

teseo_drv_ctx_t * teseo_gnss_prv_get_drv_ctx(void)
{
    return &drv_ctx;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_reset(const teseo_gnss_reset_type_t reset_type)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        switch (reset_type)
        {
            case TESEO_RESET_SYS_HARDWARE:
                SID_PAL_LOG_INFO("Performing Teseo hard reset");
                err = _prv_sys_hw_reset();
                break;

            case TESEO_RESET_SYS_SOFTWAREWARE:
                SID_PAL_LOG_INFO("Performing Teseo soft reset");
                err = teseo_gnss_send_command("$PSTMSRR");
                break;

            case TESEO_RESET_GNSS_ENGINE:
                SID_PAL_LOG_INFO("Performing Teseo GNSS engine reset");
                err = teseo_gnss_send_command("$PSTMGPSRESET");
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to reset Teseo. Error %d", (int32_t)err);
            break;
        }

        if (TESEO_RESET_GNSS_ENGINE == reset_type)
        {
            /* There's no response provided for GNSS engine-only reset, but we can check for the loss of the GNSS fix as an indication of GNSS engine reset */
            osStatus_t os_status = osOK;

            if (drv_ctx.gnss_fix_obtained != FALSE)
            {
                os_status = osSemaphoreAcquire(drv_ctx.gnss_fix_lost_sem, TESEO_GNSS_MS_TO_OS_TICKS(TESEO_GNSS_ENGINE_RESET_CMD_TIMEOUT_MS));
            }

            if (osOK == os_status) /* Either there was no GNSS fix or the fix was lost - in both cases GNSS engine reset is considered as done */
            {
                SID_PAL_LOG_INFO("Teseo GNSS engine reset completed");
                err = SID_ERROR_NONE;
                break;
            }
            else if ((osErrorTimeout == os_status) || (osErrorParameter == os_status)) /* osErrorParameter is reported from IRQ context if timeout is non-zero since waiting in IRQ is not an option */
            {
                SID_PAL_LOG_ERROR("Failed to reset Teseo GNSS engine - timeout on waiting for GNSS fix loss");
                err = SID_ERROR_TIMEOUT;
                break;
            }
            else
            {
                SID_PAL_LOG_ERROR("Failed to reset Teseo GNSS engine. Error code 0x%08X", (uint32_t)os_status);
                err = SID_ERROR_GENERIC;
                break;
            }

            /* GNSS engine-only reset terminates from here */
        }

        /* Reset GNSS fix information right after the reset */
        _prv_invalidate_gnss_fix();

        /* Invalidate CDB cache since configuration parameters are reloaded upon Teseo reset (not required after GNSS engine-only reset) */
        _prv_invalidate_cdb_cache();

        /* Indicate we are waiting for Teseo to come online, this may take a few seconds */
        SID_PAL_LOG_INFO("Waiting for Teseo to start after reset...");

        /* Capture the first UART message */
        // TODO: implement more robust method to detect the actual reset, e.g., wait for any data to arrive, not for a specific message
        _prv_invalidate_gnss_data_storage(GPTXT);
        err = _prv_wait_teseo_data_availability(GPTXT, TESEO_GNSS_BOOT_TIMEOUT_MS);
        if (err != SID_ERROR_NONE)
        {
            if (SID_ERROR_TIMEOUT == err)
            {
                SID_PAL_LOG_WARNING("No response from Teseo unit after reset. Trying baud rate auto-detection");
                err = _prv_uart_try_autodetect_baud_rate();
                if (err != SID_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Teseo unit not detected after reset. Check physical connection");
                }
            }
            else
            {
                SID_PAL_LOG_ERROR("Failed to monitor activity on Teseo UART. Error %d", (int32_t)err);
            }
            break;
        }

        /* We've received first message from Teseo */
        SID_PAL_LOG_INFO("Activity detected on Teseo UART");

        /* Notify the GNSS engine was restarted - have to trigger _prv_on_gnss_engine_restarted explicitly from here due to CDB cache invalidation */
        _prv_on_gnss_engine_restarted();

        // FIXME: reload CDB cache from here?
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_engine_restart(const teseo_gnss_start_type_t start_type, const uint32_t flags)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        switch (start_type)
        {
            case TESEO_COLD_START:
                {
                    if ((flags & ~TESEO_GNSS_START_FLAGS_ALL) != 0u)
                    {
                        err = SID_ERROR_INVALID_ARGS;
                        break;
                    }

                    SID_PAL_LOG_INFO("Performing Teseo cold start");
                    char cold_start_cmd[] = "$PSTMCOLD,0x_";
                    const uint32_t flags_pos_offset = 2u; /* Offset from the end of cold_start_cmd to place the flags value */
                    (void)TESEO_GNSS_SNPRINTF(&cold_start_cmd[sizeof(cold_start_cmd) - flags_pos_offset], flags_pos_offset, "%1X", flags);
                    err = teseo_gnss_send_command(cold_start_cmd);
                }
                break;

            case TESEO_WARM_START:
                SID_PAL_LOG_INFO("Performing Teseo warm start");
                err = teseo_gnss_send_command("$PSTMWARM");
                break;

            case TESEO_HOT_START:
                SID_PAL_LOG_INFO("Performing Teseo hot start");
                err = teseo_gnss_send_command("$PSTMHOT");
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to restart Teseo GNSS engine. Error %d", (int32_t)err);
            break;
        }

        /* Lock GNSS Engine Restart semaphore */
        while (osSemaphoreGetCount(drv_ctx.gnss_engine_restart_sem) != 0u)
        {
            (void)osSemaphoreAcquire(drv_ctx.gnss_engine_restart_sem, 0u);
        }

        /* Reset GNSS fix information right after the (re)start */
        _prv_invalidate_gnss_fix();

        /* Wait for restart completion if we are not in an IRQ context or a critical section */
        if (SID_STM32_UTIL_IS_IRQ() == FALSE)
        {
            osStatus_t os_status = osSemaphoreAcquire(drv_ctx.gnss_engine_restart_sem, TESEO_GNSS_MS_TO_OS_TICKS(TESEO_GNSS_BOOT_TIMEOUT_MS));
            switch (os_status)
            {
                case osOK:
                    err = SID_ERROR_NONE;
                    break;

                case osErrorTimeout:
                    err = SID_ERROR_TIMEOUT;
                    break;

                default:
                    SID_PAL_LOG_ERROR("Failed to wait for Teseo GNSS engine (re)start, error code 0x%08X", (uint32_t)os_status);
                    err = SID_ERROR_GENERIC;
                    break;
            }
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_restore_factory_defaults(void)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Invalidate data in the storage */
        _prv_invalidate_gnss_data_storage(PSTMRESTOREPAR);

        /* Send command to Teseo */
        err = teseo_gnss_send_command("$PSTMRESTOREPAR");
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to send Teseo configuration restore command. Error %d", (int32_t)err);
            break;
        }

        /* Wait for response */
        err = _prv_wait_teseo_data_availability(PSTMRESTOREPAR, TESEO_GNSS_SAVEPAR_CMD_TIMEOUT_MS);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to receive Teseo configuration restore ack. Error %d", (int32_t)err);
            break;
        }

        /* Perform hard-reset here to ensure all configuration parameters are applied */
        err = teseo_gnss_reset(TESEO_RESET_SYS_HARDWARE);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Reload CDB cache after configuration update */
        err = _prv_load_cdb_cache(&drv_ctx.cdb_cache);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_enable_log_mirroring(void)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        drv_ctx.echo_to_logs = TRUE;
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_disable_log_mirroring(void)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        drv_ctx.echo_to_logs = FALSE;
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_get_log_mirroring_status(uint8_t * const is_enabled)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        if (NULL == is_enabled)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        *is_enabled = drv_ctx.echo_to_logs;
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_send_command(const char * const cmd)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        if (drv_ctx.in_standby != FALSE)
        {
            /* Can't accept NMEA commands in Standby state because Teseo's UART is deactivated */
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        if (NULL == cmd)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        const uint32_t cmd_str_len = strlen(cmd);
        if (0u == cmd_str_len)
        {
            if (NULL == cmd)
            {
                err = SID_ERROR_INVALID_ARGS;
                break;
            }
        }

        // FIXME: use mutex to lock Teseo access

        // FIXME: ensure 20ms minimum delay between commands is respected

        /**
         * @note Even though the UART driver is capable of automatically adding '$' as a start marker and "\r\n" as the end marker, we still need to handle this
         *       here instead of offloading to the UART driver. The reason is that this driver checks the command echo responses from Teseo to monitor if Teseo
         *       has accepted the respective command, so we need to have a properly formed NMEA string in the driver context that will match Teseo echo message.
         *       If preformatting of the NMEA strings will be offloaded to the Tx portion of the UART driver, we will still need some logic to properly compare
         *       loosely-formatted cmd strings to the NMEA-formatted Teseo echo messages, so there's no performance advantage. Instead, just enforce strict NMEA
         *       formatting of the cmd string right here, keeping the echo cross-check logic simple and clear.
         */

        /* Check if the command will fit into the buffer */
        const uint32_t cmd_prefix_present = (cmd[0] == '$') ? TRUE : FALSE;
        const uint32_t cmd_line_ending_ok = strcmp(TESEO_GNSS_NMEA_LINE_ENDING, &cmd[cmd_str_len - sizeof(TESEO_GNSS_NMEA_LINE_ENDING) + 1]) == 0 ? TRUE : FALSE;
        const char * cmd_end_ptr = (cmd_line_ending_ok) != FALSE ? &cmd[cmd_str_len - sizeof(TESEO_GNSS_NMEA_LINE_ENDING)] : &cmd[cmd_str_len - 1];
        while (((*cmd_end_ptr == '\r') || (*cmd_end_ptr == '\n')) && (cmd_end_ptr > (cmd + TESEO_GNSS_NMEA_CHECKSUM_CHARS)))
        {
            cmd_end_ptr--;
        }
        const uint32_t cmd_crc_present = ('*' == *(cmd_end_ptr - TESEO_GNSS_NMEA_CHECKSUM_CHARS + 1u)) ? TRUE : FALSE;
        const uint32_t cmd_data_len = (uint32_t)(void *)cmd_end_ptr - (uint32_t)(void *)cmd + 1u;

        const uint32_t sendout_data_len = (cmd_prefix_present == FALSE ? sizeof('$') : 0u) + cmd_data_len + (cmd_crc_present == FALSE ? TESEO_GNSS_NMEA_CHECKSUM_CHARS : 0u) + (sizeof(TESEO_GNSS_NMEA_LINE_ENDING) - 1u);
        if (sendout_data_len > sizeof(drv_ctx.last_command))
        {
            /* Command is too long, cannot send it */
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Wait for the previous command to complete */
        err = teseo_gnss_wait_command_completion(TESEO_GNSS_DEFAULT_CMD_TIMEOUT_MS);
        if (SID_ERROR_TIMEOUT == err)
        {
            /* Print a warning, but still proceed with the current command */
            char * print_limit_ptr = (char *)&drv_ctx.last_command[sizeof(drv_ctx.last_command) - 1u];
            while (('\0' == *print_limit_ptr) || ('\r' == *print_limit_ptr) || ('\n' == *print_limit_ptr))
            {
                print_limit_ptr--;
            }
            print_limit_ptr++;
            *print_limit_ptr = '\0'; /* Skip \r\n for logging */
            SID_PAL_LOG_WARNING("Teseo command execution stuck (no ack received). Command: %s", drv_ctx.last_command);
        }
        else if ((err != SID_ERROR_NONE) && (err != SID_ERROR_INVALID_STATE))
        {
            /* Fatal error, terminate */
            SID_PAL_LOG_ERROR("Failed to execute Teseo command %s due to issues with preceding command", cmd);
            break;
        }
        else
        {
            /* Proceed normally */
        }

        sid_pal_enter_critical_region();

        /* Ensure command completion lock is ready to monitor the current command */
        SID_PAL_ASSERT(drv_ctx.cmd_completion_lock != NULL);
        while (osSemaphoreGetCount(drv_ctx.cmd_completion_lock) != 0u)
        {
            (void)osSemaphoreAcquire(drv_ctx.cmd_completion_lock, 0u);
        }

        char * buffer_store_ptr = (char *)drv_ctx.last_command;
        if (FALSE == cmd_prefix_present)
        {
            *buffer_store_ptr = '$';
            buffer_store_ptr++;
        }

        if ((cmd_line_ending_ok != FALSE) && (cmd_crc_present != FALSE))
        {
            /* Copy the command as is into the buffer */
            SID_STM32_UTIL_fast_memcpy(buffer_store_ptr, cmd, cmd_str_len);
            buffer_store_ptr += cmd_str_len;
        }
        else
        {
            /* Copy the valid part of the command - all the data without line ending */
            SID_STM32_UTIL_fast_memcpy(buffer_store_ptr, cmd, cmd_data_len);
            buffer_store_ptr += cmd_data_len;

            if (FALSE == cmd_crc_present)
            {
                /* Calculate the checksum */
                uint8_t checksum = 0u;
                for (const uint8_t * check_ptr = &drv_ctx.last_command[1]; check_ptr < (const uint8_t *)buffer_store_ptr; check_ptr++)
                {
                    checksum = (checksum ^ *check_ptr);
                }

                /* Store checksum into the buffer */
                (void)TESEO_GNSS_SNPRINTF(buffer_store_ptr, (TESEO_GNSS_NMEA_CHECKSUM_CHARS + 1u /* add one byte for '\0' put by snprintf */), "*%02X", checksum);
                buffer_store_ptr += TESEO_GNSS_NMEA_CHECKSUM_CHARS;
            }

            /* Set correct line ending */
            SID_STM32_UTIL_fast_memcpy(buffer_store_ptr, TESEO_GNSS_NMEA_LINE_ENDING, sizeof(TESEO_GNSS_NMEA_LINE_ENDING) - 1u); /* Use memcpy() since command buffer is not a null-terminated string, there might be no space left for '\0' produced by snprintf() */
            buffer_store_ptr += (sizeof(TESEO_GNSS_NMEA_LINE_ENDING) - 1u);
        }

        /* Clear the unused bytes in the command buffer to avoid confusion */
        SID_STM32_UTIL_fast_memset(buffer_store_ptr, 0u, sizeof(drv_ctx.last_command) - sendout_data_len);

        err = (*drv_ctx.uart_ifc)->send(drv_ctx.uart_ifc, drv_ctx.last_command, sendout_data_len);
        sid_pal_exit_critical_region();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_wait_command_completion(const uint32_t timeout_ms)
{
    sid_error_t err;
    osStatus_t os_status;

    SID_PAL_ASSERT(drv_ctx.cmd_completion_lock != NULL);

    do
    {
        /* If command echoing is enabled, wait for the previous command completion */
        sid_pal_enter_critical_region();
        const uint32_t cmd_echoing_enabled = ((drv_ctx.cdb_cache.validity.app_cfg2 != FALSE) && (drv_ctx.cdb_cache.app_cfg2.NMEA_COMMAND_ECHO_ENABLE != FALSE)) ? TRUE : FALSE;
        sid_pal_exit_critical_region();

        if (cmd_echoing_enabled != FALSE)
        {
            os_status = osSemaphoreAcquire(drv_ctx.cmd_completion_lock, TESEO_GNSS_MS_TO_OS_TICKS(timeout_ms));
            switch (os_status)
            {
                case osOK:
                    err = SID_ERROR_NONE;
                    break;

                case osErrorTimeout:
                    err = SID_ERROR_TIMEOUT;
                    break;

                default:
                    SID_PAL_LOG_ERROR("Failed to wait for preceding Teseo command completion, error code 0x%08X", (uint32_t)os_status);
                    err = SID_ERROR_GENERIC;
                    break;
            }
        }
        else
        {
            /* Can't wait for Teseo command ack because command echoing is disabled or its state is unknown */
            err = SID_ERROR_INVALID_STATE;
        }

    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_request_version_info(const teseo_gnss_version_info_type_t version_type)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate inputs */
        switch (version_type)
        {
            case TESEO_VER_TYPE_GNSS_LIB:
            case TESEO_VER_TYPE_OS20:
            case TESEO_VER_TYPE_SDK_APP:
            case TESEO_VER_TYPE_FW:
            case TESEO_VER_TYPE_HW:
            case TESEO_VER_TYPE_UCODE:
            case TESEO_VER_TYPE_ALL_SUPPORTED:
                err = SID_ERROR_NONE;
                break;

            case TESEO_VER_TYPE_SW_CFG_ID:
            case TESEO_VER_TYPE_PRODUCT_ID:
            case TESEO_VER_TYPE_CFG_DATA_BLOCK:
            case TESEO_VER_TYPE_ALL_STRINGS:
                err = SID_ERROR_NOSUPPORT;
                break;

            default:
                err = SID_ERROR_INVALID_ARGS;
                break;
        }
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        if ((version_type != TESEO_VER_TYPE_ALL_SUPPORTED) && (version_type != TESEO_VER_TYPE_UCODE))
        {
            char cmd[20];

            /* Build version request command */
            const uint32_t bytes_written = TESEO_GNSS_SNPRINTF(cmd, sizeof(cmd), "$PSTMGETSWVER,%u", (uint32_t)version_type);
            if (bytes_written >= (sizeof(cmd) - 1u))
            {
                SID_PAL_LOG_ERROR("Unable to request Teseo version info - command buffer overflow");
                err = SID_ERROR_BUFFER_OVERFLOW;
                break;
            }

            /* Some Teseo variants may do unsolicited version reports on boot, causing conflicts with this explicit version request. To address that, version request will be retried several times before reporting an error */
            for (uint32_t i = 0u; i < TESEO_GNSS_SWVER_REQUEST_ATTEMPTS_MAX; i++)
            {
                /* Invalidate data in the storage */
                _prv_invalidate_gnss_data_storage(PSTMVER);

                /* Send command to Teseo */
                err = teseo_gnss_send_command(cmd);
                if (err != SID_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Failed to request Teseo version info. Error %d", (int32_t)err);
                    break;
                }

                /* Wait for response */
                err = _prv_wait_teseo_data_availability(PSTMVER, TESEO_GNSS_DEFAULT_CMD_TIMEOUT_MS);
                if (err != SID_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Failed to retrieve Teseo version information (type: %u). Error %d", (uint32_t)version_type, (int32_t)err);
                    break;
                }

                /* Process received data */
                switch (version_type)
                {
                    case TESEO_VER_TYPE_FW:
                        err = _prv_parse_bin_image_version_string((char *)drv_ctx.gnss_data.pstmver_data.pstmver_string);
                        break;

                    case TESEO_VER_TYPE_GNSS_LIB:
                        err = _prv_parse_gnss_lib_version_string((char *)drv_ctx.gnss_data.pstmver_data.pstmver_string);
                        break;

                    case TESEO_VER_TYPE_OS20:
                        err = _prv_parse_os20_lib_version_string((char *)drv_ctx.gnss_data.pstmver_data.pstmver_string);
                        break;

                    case TESEO_VER_TYPE_SDK_APP:
                        err = _prv_parse_gps_app_version_string((char *)drv_ctx.gnss_data.pstmver_data.pstmver_string);
                        break;

                    case TESEO_VER_TYPE_HW:
                        err = _prv_parse_hw_version_string((char *)drv_ctx.gnss_data.pstmver_data.pstmver_string);
                        break;

                    default:
                        /* No parsing required */
                        err = SID_ERROR_NONE;
                        break;
                }

                if (SID_ERROR_NONE == err)
                {
                    /* Jump out of the version request loop if the version information was processed successfully */
                    break;
                }
                else if ((TESEO_GNSS_SWVER_REQUEST_ATTEMPTS_MAX - 1u) == i)
                {
                    /* This is the last attempt and it failed */
                    SID_PAL_LOG_ERROR("Failed to parse Teseo version information (type: %u). Error %d", (uint32_t)version_type, (int32_t)err);
                }
                else
                {
                    /* Continue with read attempts */
                }
            }
        }
        else if (TESEO_VER_TYPE_UCODE == version_type)
        {
            char cmd[] = "$PSTMGETUCODE";

            /* Invalidate data in the storage */
            _prv_invalidate_gnss_data_storage(PSTMGETUCODE);

            /* Send command to Teseo */
            err = teseo_gnss_send_command(cmd);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to send Teseo Get Unique Code command. Error %d", (int32_t)err);
                break;
            }

            /* Wait for response */
            err = _prv_wait_teseo_data_availability(PSTMGETUCODE, TESEO_GNSS_PSTMGETUCODE_CMD_TIMEOUT_MS);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to read Teseo Unique Code. Error %d", (int32_t)err);
                break;
            }

            /* Copy as many bytes as fits into storage */
            uint32_t bytes_to_copy = strlen((char *)drv_ctx.gnss_data.pstmver_data.pstmver_string);
            if (bytes_to_copy >= sizeof(drv_ctx.version_info.ucode))
            {
                bytes_to_copy = sizeof(drv_ctx.version_info.ucode) - 1u;
            }
            sid_pal_enter_critical_region();
            SID_STM32_UTIL_fast_memcpy(drv_ctx.version_info.ucode, drv_ctx.gnss_data.pstmver_data.pstmver_string, bytes_to_copy);
            drv_ctx.version_info.ucode[bytes_to_copy] = '\0';
            sid_pal_exit_critical_region();
        }
        else /* if (TESEO_VER_TYPE_ALL_SUPPORTED == version_type) */
        {
            /* Sequentially request every supported version string */
            const teseo_gnss_version_info_type_t supported_types[] = {
                TESEO_VER_TYPE_HW , TESEO_VER_TYPE_FW, TESEO_VER_TYPE_GNSS_LIB, TESEO_VER_TYPE_OS20, TESEO_VER_TYPE_SDK_APP, TESEO_VER_TYPE_UCODE,
            };

            for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(supported_types); i++)
            {
                err = teseo_gnss_request_version_info(supported_types[i]);
                if (err != SID_ERROR_NONE)
                {
                    break;
                }
            }
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_get_version_info(teseo_gnss_version_info_t * const version_info_storage)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate the inputs*/
        if (NULL == version_info_storage)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        sid_pal_enter_critical_region();
        SID_STM32_UTIL_fast_memcpy(version_info_storage, &drv_ctx.version_info, sizeof(*version_info_storage));
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_get_ucode(char * const buffer, const uint32_t buffer_size)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate the inputs*/
        if (NULL == buffer)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (buffer_size <= sizeof(drv_ctx.version_info.ucode))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        sid_pal_enter_critical_region();
        SID_STM32_UTIL_fast_memcpy(buffer, &drv_ctx.version_info.ucode, sizeof(drv_ctx.version_info.ucode));
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_request_nmea_port_baudrate_config(teseo_cdb_102_nmea_port_baudrate_t * const nmea_port_baudrate)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate the inputs*/
        if (NULL == nmea_port_baudrate)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        err = _prv_request_cdb_param(TESEO_CDB_PARAM_NMEA_PORT_BAUDRATE, TESEO_CDB_CURRENT_CONFIG);

        /* Update parameter value and validity in a critical section to avoid race conditions */
        sid_pal_enter_critical_region();
        if ((TESEO_CDB_PARAM_NMEA_PORT_BAUDRATE == drv_ctx.gnss_data.pstmsetpar_data.param_id) && (SID_ERROR_NONE == err))
        {
            nmea_port_baudrate->raw = (uint32_t)drv_ctx.gnss_data.pstmsetpar_data.value.u64;
        }
        else
        {
            if (SID_ERROR_NONE == err)
            {
                err = SID_ERROR_INVALID_RESPONSE;
            }
        }
        sid_pal_exit_critical_region();

        /* Terminate on error */
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_send_nmea_port_baudrate_config(const teseo_cdb_102_nmea_port_baudrate_t nmea_port_baudrate)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        err = _prv_set_cdb_int_param(TESEO_CDB_PARAM_NMEA_PORT_BAUDRATE, nmea_port_baudrate.raw);
        if (err != SID_ERROR_NONE)
        {
            /* Logs are provided by _prv_set_cdb_int_param() */
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_request_app_config(teseo_cdb_200_application_onoff_t * const app_cfg1, teseo_cdb_227_application_onoff_2_t * const app_cfg2)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate the inputs*/
        if ((NULL == app_cfg1) && (NULL == app_cfg2))
        {
            /* At least one configuration storage should be valid */
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Read Application On/Off 2 CDB first since it affects communication flow - we need to align with the actual config ASAP */
        if (app_cfg2 != NULL)
        {
            err = _prv_request_cdb_param(TESEO_CDB_PARAM_APPLICATION_ONOFF_2, TESEO_CDB_CURRENT_CONFIG);

            /* Update parameter value and validity in a critical section to avoid race conditions */
            sid_pal_enter_critical_region();
            if ((TESEO_CDB_PARAM_APPLICATION_ONOFF_2 == drv_ctx.gnss_data.pstmsetpar_data.param_id) && (SID_ERROR_NONE == err))
            {
                app_cfg2->raw = (uint32_t)drv_ctx.gnss_data.pstmsetpar_data.value.u64;
            }
            else
            {
                if (SID_ERROR_NONE == err)
                {
                    err = SID_ERROR_INVALID_RESPONSE;
                }
            }
            sid_pal_exit_critical_region();

            /* Terminate on error */
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        if (app_cfg1 != NULL)
        {
            err = _prv_request_cdb_param(TESEO_CDB_PARAM_APPLICATION_ONOFF, TESEO_CDB_CURRENT_CONFIG);

            /* Update parameter value and validity in a critical section to avoid race conditions */
            sid_pal_enter_critical_region();
            if ((TESEO_CDB_PARAM_APPLICATION_ONOFF == drv_ctx.gnss_data.pstmsetpar_data.param_id) && (SID_ERROR_NONE == err))
            {
                app_cfg1->raw = (uint32_t)drv_ctx.gnss_data.pstmsetpar_data.value.u64;
            }
            else
            {
                if (SID_ERROR_NONE == err)
                {
                    err = SID_ERROR_INVALID_RESPONSE;
                }
            }
            sid_pal_exit_critical_region();

            /* Terminate on error */
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_send_app_config(const teseo_cdb_200_application_onoff_t app_cfg1, const teseo_cdb_227_application_onoff_2_t app_cfg2)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        err = _prv_set_cdb_int_param(TESEO_CDB_PARAM_APPLICATION_ONOFF, app_cfg1.raw);
        if (err != SID_ERROR_NONE)
        {
            /* Logs are provided by _prv_set_cdb_int_param() */
            break;
        }

        err = _prv_set_cdb_int_param(TESEO_CDB_PARAM_APPLICATION_ONOFF_2, app_cfg2.raw);
        if (err != SID_ERROR_NONE)
        {
            /* Logs are provided by _prv_set_cdb_int_param() */
            break;
        }

    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_request_message_config(teseo_cdb_201_uart_msg_list_low_t * const msg_list_low, teseo_cdb_228_uart_msg_list_high_t * const msg_list_high)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate the inputs*/
        if ((NULL == msg_list_low) && (NULL == msg_list_high))
        {
            /* At least one configuration storage should be valid */
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (msg_list_low != NULL)
        {
            err = _prv_request_cdb_param(TESEO_CDB_PARAM_UART_MSG_LIST_LOW, TESEO_CDB_CURRENT_CONFIG);

            /* Update parameter value and validity in a critical section to avoid race conditions */
            sid_pal_enter_critical_region();
            if ((TESEO_CDB_PARAM_UART_MSG_LIST_LOW == drv_ctx.gnss_data.pstmsetpar_data.param_id) && (SID_ERROR_NONE == err))
            {
                msg_list_low->raw = (uint32_t)drv_ctx.gnss_data.pstmsetpar_data.value.u64;
            }
            else
            {
                if (SID_ERROR_NONE == err)
                {
                    err = SID_ERROR_INVALID_RESPONSE;
                }
            }
            sid_pal_exit_critical_region();

            /* Terminate on error */
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        if (msg_list_high != NULL)
        {
            err = _prv_request_cdb_param(TESEO_CDB_PARAM_UART_MSG_LIST_HIGH, TESEO_CDB_CURRENT_CONFIG);

            /* Update parameter value and validity in a critical section to avoid race conditions */
            sid_pal_enter_critical_region();
            if ((TESEO_CDB_PARAM_UART_MSG_LIST_HIGH == drv_ctx.gnss_data.pstmsetpar_data.param_id) && (SID_ERROR_NONE == err))
            {
                msg_list_high->raw = (uint32_t)drv_ctx.gnss_data.pstmsetpar_data.value.u64;
            }
            else
            {
                if (SID_ERROR_NONE == err)
                {
                    err = SID_ERROR_INVALID_RESPONSE;
                }
            }
            sid_pal_exit_critical_region();

            /* Terminate on error */
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_send_message_config(const teseo_cdb_201_uart_msg_list_low_t msg_list_low, const teseo_cdb_228_uart_msg_list_high_t msg_list_high, const uint8_t message_rate)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        char cmd[40];

        const uint32_t bytes_written = TESEO_GNSS_SNPRINTF(cmd, sizeof(cmd), "$PSTMCFGMSGL,0,%u,%X,%X", message_rate, msg_list_low.raw, msg_list_high.raw);
        if (bytes_written >= (sizeof(cmd) - 1u))
        {
            SID_PAL_LOG_ERROR("Unable to send Teseo message list config - command buffer overflow");
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Invalidate data in the storage */
        _prv_invalidate_gnss_data_storage(PSTMSGL);

        /* Send command to Teseo */
        err = teseo_gnss_send_command(cmd);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to send Teseo message list config. Error %d", (int32_t)err);
            break;
        }

        /* Wait for response */
        err = _prv_wait_teseo_data_availability(PSTMSGL, TESEO_GNSS_DEFAULT_CMD_TIMEOUT_MS);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to apply Teseo message list config. Error %d", (int32_t)err);
            break;
        }

        /* Check if Teseo replied with good result */
        if (drv_ctx.gnss_data.result != GNSS_OP_OK)
        {
            SID_PAL_LOG_ERROR("Teseo rejected message list config. Make sure supplied config is valid");
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* CDB cache is not updated here because configuration becomes active only after the teseo_gnss_apply_configuration() call */

        /* Done */
        err = SID_ERROR_NONE;

    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_request_startup_message_text(char * const msg, const uint32_t max_size)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate the inputs*/
        if ((NULL == msg) || (max_size < 2u))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        err = _prv_request_cdb_param(TESEO_CDB_PARAM_STARTUP_MESSAGE, TESEO_CDB_CURRENT_CONFIG);

        /* Update parameter value and validity in a critical section to avoid race conditions */
        sid_pal_enter_critical_region();
        if ((TESEO_CDB_PARAM_STARTUP_MESSAGE == drv_ctx.gnss_data.pstmsetpar_data.param_id) && (SID_ERROR_NONE == err))
        {
            uint32_t copy_length = strlen(drv_ctx.gnss_data.pstmsetpar_data.value.str) + 1u;
            if (copy_length > max_size)
            {
                copy_length = max_size;
            }
            SID_STM32_UTIL_fast_memcpy(msg, drv_ctx.gnss_data.pstmsetpar_data.value.str, copy_length);
            drv_ctx.gnss_data.pstmsetpar_data.value.str[copy_length] = '\0';
        }
        else
        {
            if (SID_ERROR_NONE == err)
            {
                err = SID_ERROR_INVALID_RESPONSE;
            }
        }
        sid_pal_exit_critical_region();

        /* Terminate on error */
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_apply_configuration(void)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Check if Teseo's current config differs from the local cache */
        teseo_cdb_cache_t current_cfg;
        err = _prv_load_cdb_cache(&current_cfg);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        sid_pal_enter_critical_region();
        const uint32_t config_differs = SID_STM32_UTIL_fast_memcmp(&current_cfg, &drv_ctx.cdb_cache, sizeof(current_cfg)) == 0u ? FALSE : TRUE;
        sid_pal_exit_critical_region();

        if (FALSE == config_differs)
        {
            /* Teseo won't reply either with $SPSTMSAVEPAROK or $PSTMSAVEPARERROR for this case, we have to exit from here */
            SID_PAL_LOG_INFO("No differences detected in Teseo's current config and driver cache. CDB NVM store skipped");
            err = SID_ERROR_NONE;
            break;
        }

        /* Proceed with NVM storage */
        char cmd[] = "$PSTMSAVEPAR";

        /* Invalidate data in the storage */
        _prv_invalidate_gnss_data_storage(PSTMSAVEPAR);

        /* Send command to Teseo */
        err = teseo_gnss_send_command(cmd);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to send Teseo CDB NVM store command. Error %d", (int32_t)err);
            break;
        }

        /* Wait for response */
        err = _prv_wait_teseo_data_availability(PSTMSAVEPAR, TESEO_GNSS_SAVEPAR_CMD_TIMEOUT_MS);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to acknowledge Teseo CDB storage to NVM. Error %d", (int32_t)err);
            break;
        }

        /* Check if Teseo replied with good result */
        if (drv_ctx.gnss_data.result != GNSS_OP_OK)
        {
            SID_PAL_LOG_ERROR("Teseo CDB NVM Store command rejected");
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Adjust UART baud rate to stay aligned with Teseo configuration */
        uint32_t target_baud_rate;
        if (current_cfg.validity.nmea_port_baudrate != FALSE)
        {
            (void)_prv_uart_teseo_param_to_baudrate(current_cfg.nmea_port_baudrate.br_cfg_val, &target_baud_rate);
        }
        else
        {
            /* Fall back to the default selection */
            target_baud_rate = drv_ctx.config->uart_cfg->baud_rate;
        }

        err = _prv_uart_change_baud_rate(target_baud_rate);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _prv_uart_change_baud_rate() */
            break;
        }

        /* Perform a reset here to ensure all configuration parameters are applied. Hardware reset is required since UART baud rate may have changed */
        err = teseo_gnss_reset(TESEO_RESET_SYS_HARDWARE);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Reload CDB cache after configuration update */
        err = _prv_load_cdb_cache(&drv_ctx.cdb_cache);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_is_gnss_fix_available(uint8_t * const available)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate the inputs*/
        if (NULL == available)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        *available = drv_ctx.gnss_fix_obtained;
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_get_position(teseo_gnss_position_data_t * const postion_storage)
{
    sid_error_t err;

    sid_pal_enter_critical_region();

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate the inputs*/
        if (NULL == postion_storage)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (FALSE == drv_ctx.gnss_fix_obtained)
        {
            /* Position data is not available */
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        SID_STM32_UTIL_fast_memcpy(postion_storage, &drv_ctx.latest_position, sizeof(drv_ctx.latest_position));

        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_suspend_gnss_engine(void)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        char cmd[] = "PSTMGPSSUSPEND"; // TODO: this command is not available in LIV4F

        /* Invalidate data in the storage */
        _prv_invalidate_gnss_data_storage(PSTMGPSSUSPEND);

        /* Send command to Teseo */
        err = teseo_gnss_send_command(cmd);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to send Teseo Suspend GNSS Engine command. Error %d", (int32_t)err);
            break;
        }

        /* Wait for response */
        err = _prv_wait_teseo_data_availability(PSTMGPSSUSPEND, TESEO_GNSS_GPSSUSPEND_CMD_TIMEOUT_MS);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to acknowledge Teseo Suspend GNSS Engine. Error %d", (int32_t)err);
            break;
        }

        /* Check if Teseo replied with good result */
        if (drv_ctx.gnss_data.result != GNSS_OP_OK)
        {
            SID_PAL_LOG_ERROR("Teseo Suspend GNSS Engine command rejected");
            err = SID_ERROR_PARAM_OUT_OF_RANGE;
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_standby(const uint32_t duration_s)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        if (drv_ctx.in_standby != FALSE)
        {
            /* In standby mode already, no actions required */
            err = SID_ERROR_NONE;
            break;
        }

        /**
         * Suspend GNSS engine first so that it stops producing NMEA messages - this step is needed because $PSTMFORCESTANDBY response may not be transmitted by
         * LIV3 module before it enters Standby mode due to its Tx message queue being overwhelmed by pending messages, leading to a false-positive $PSTMFORCESTANDBY
         * command timeout error report.
         */
        char suspend_cmd[] = "PSTMGPSSUSPEND"; // TODO: this command is not available in LIV4F
        char standby_cmd[30];

        const uint32_t bytes_written = TESEO_GNSS_SNPRINTF(standby_cmd, sizeof(standby_cmd), "$PSTMFORCESTANDBY,%u", duration_s);
        if (bytes_written >= (sizeof(standby_cmd) - 1u))
        {
            SID_PAL_LOG_ERROR("Unable to send Teseo Force Standby - command buffer overflow");
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Invalidate data in the storage */
        _prv_invalidate_gnss_data_storage(PSTMGPSSUSPEND);
        _prv_invalidate_gnss_data_storage(PSTMFORCESTANDBY);

        /* Send Suspend command to Teseo */
        err = teseo_gnss_send_command(suspend_cmd);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to send Teseo Suspend GNSS Engine command. Error %d", (int32_t)err);
            break;
        }

        /* Send Force Standby command to Teseo right after the Suspend command */
        err = teseo_gnss_send_command(standby_cmd);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to send Teseo Force Standby command. Error %d", (int32_t)err);
            break;
        }

        /* Wait for Suspend command response */
        err = _prv_wait_teseo_data_availability(PSTMGPSSUSPEND, TESEO_GNSS_GPSSUSPEND_CMD_TIMEOUT_MS);
        if (err != SID_ERROR_NONE)
        {
            if (err != SID_ERROR_TIMEOUT)
            {
                SID_PAL_LOG_ERROR("Failed to acknowledge Teseo Suspend GNSS Engine. Error %d", (int32_t)err);
                break;
            }
            else
            {
                /* Still proceed since resposne to the Force Stadnby command still may arrive. THe timeout may be caused just by slow response of Teseo under high CPU load */
                SID_PAL_LOG_WARNING("Failed to acknowledge Teseo Suspend GNSS Engine. Error %d", (int32_t)err);
            }
        }

        /* Wait for Force Standby command response */
        err = _prv_wait_teseo_data_availability(PSTMFORCESTANDBY, TESEO_GNSS_FORCESTANDBY_CMD_TIMEOUT_MS);
        if ((err != SID_ERROR_NONE) && (err != SID_ERROR_TIMEOUT))
        {
            SID_PAL_LOG_ERROR("Failed to acknowledge Teseo Force Standby command. Error %d", (int32_t)err);
            break;
        }

        /* Check if Teseo replied with good result */
        if ((SID_ERROR_NONE == err) && (drv_ctx.gnss_data.result != GNSS_OP_OK))
        {
            // TODO: cancel Rx restart timer from here
            SID_PAL_LOG_ERROR("Teseo Force Standby command rejected");
            err = SID_ERROR_PARAM_OUT_OF_RANGE;
            break;
        }
        else
        {
            /**
             * Timeout on waiting for $PSTMFORCESTANDBYOK response - that can happen since sometimes Teseo's UART communication is cut before this message is delivered.
             * Contrary to that, if something is wrong with the $PSTMFORCESTANDBY request, the $PSTMFORCESTANDBYERROR response is delivered almost instantl. Based on that,
             * assume Teseo entered standby state if no response arrived at all
             */

            /* Invalidate GNSS fix if it was available - still need to have this call here to protect from $PSTMFORCESTANDBYOK response not arriving */
            _prv_trigger_gnss_fix_loss();
        }

        /* Teseo is in standby mode from now on */
        SID_PAL_LOG_INFO(TESEO_GNSS_GPTXT_LOG_PREFIX "Entered Software Standby mode on request");
        drv_ctx.in_standby = TRUE; /* Even if the below code fails, Teseo is in standby and its UART is non-operational */
        __COMPILER_BARRIER();      /* Ensure drv_ctx.in_standby is stored right now and it won't be re-ordered by the compiler */

        /* Calculated the intended time for keeping the UART deactivated */
        const uint32_t uart_sleep_time_ms = (duration_s * SID_TIME_MSEC_PER_SEC) > TESEO_GNSS_UART_LOOKAHEAD_WAKEUP_MS ? ((duration_s * SID_TIME_MSEC_PER_SEC) - TESEO_GNSS_UART_LOOKAHEAD_WAKEUP_MS) : 0u;

        /* Suspend UART peripheral since Teseo's UART is deactivated in Standby state - this will stop the continuous Rx and disable UART operation, allowing the MCU to enter deeper low-power modes */
        if ((TESEO_GNSS_STANDBY_DURATION_INFINITE == duration_s) || (uart_sleep_time_ms > 0u))
        {
            err = (*drv_ctx.uart_ifc)->suspend_uart(drv_ctx.uart_ifc);
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        /* Schedule UART re-activation */
        if (duration_s != TESEO_GNSS_STANDBY_DURATION_INFINITE)
        {
            struct sid_timespec wakeup_ts;

            /* Compute the wakeup timestamp */
            err = sid_pal_uptime_now(&wakeup_ts);
            if (err != SID_ERROR_NONE)
            {
                break;
            }
            sid_add_ms_to_timespec(&wakeup_ts, uart_sleep_time_ms);

            /* Start the timer for resuming UART communication later */
            (void)sid_pal_timer_cancel(&drv_ctx.standby_wakeup_timer);
            err = sid_pal_timer_arm(&drv_ctx.standby_wakeup_timer, SID_PAL_TIMER_PRIO_CLASS_PRECISE, &wakeup_ts, NULL);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to start Teseo UART wakeup. Error %d", (int32_t)err);
                break;
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_wakeup(void)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Ensure the UART wakeup timer is stopped */
        (void)sid_pal_timer_cancel(&drv_ctx.standby_wakeup_timer);

        /* Re-enable UART peripheral */
        err = (*drv_ctx.uart_ifc)->resume_uart(drv_ctx.uart_ifc);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Restart continuous UART Rx */
        err = (*drv_ctx.uart_ifc)->start_rx(drv_ctx.uart_ifc);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Toggle wake-up line */
        err = _prv_sys_wakeup();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        SID_PAL_LOG_INFO(TESEO_GNSS_GPTXT_LOG_PREFIX "Wake-up on request");
        drv_ctx.in_standby = FALSE;
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_init(const teseo_gnss_device_config_t * const config, teseo_gnss_event_callbacks_t * const event_callbacks)
{
    sid_error_t                err;
    GNSSParser_Status_t        parser_status;
    teseo_nmea_port_baudrate_t uart_baudrate_cfg_val;

    do
    {
        if (drv_ctx.init_done != FALSE)
        {
            err = SID_ERROR_ALREADY_INITIALIZED;
            break;
        }

        /* Validate inputs */
        if ((NULL == config) || ((NULL == config->uart_cfg)))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        err = _prv_uart_baudrate_to_teseo_param(config->uart_cfg->baud_rate, &uart_baudrate_cfg_val);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Teseo does not support UART baud rate of %u", config->uart_cfg->baud_rate);
        }

        /* Store configuration */
        drv_ctx.config = config;
        if (event_callbacks != NULL)
        {
            SID_STM32_UTIL_fast_memcpy(&drv_ctx.user_callbacks, event_callbacks, sizeof(drv_ctx.user_callbacks));
        }
        else
        {
            SID_STM32_UTIL_fast_memset(&drv_ctx.user_callbacks, 0u, sizeof(drv_ctx.user_callbacks));
        }

        /* Initialize GPIO */
        err = _prv_gpio_init();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Create semaphores to manage Teseo data updates */
        const osSemaphoreAttr_t default_teseo_sem_attr = {0};
        err = SID_ERROR_NONE;
        for (uint32_t msg_type = 0u; msg_type < NMEA_MSGS_NUM; msg_type++)
        {
            drv_ctx.gnss_data_locks[msg_type] = osSemaphoreNew(1u, 0u, &default_teseo_sem_attr);
            if (NULL == drv_ctx.gnss_data_locks[msg_type])
            {
                err = SID_ERROR_OOM;
                break;
            }
        }
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to create GNSS message semaphores");
            break;
        }

        /* Create command completion semaphore */
        drv_ctx.cmd_completion_lock = osSemaphoreNew(1u, 1u, &default_teseo_sem_attr);
        if (NULL == drv_ctx.cmd_completion_lock)
        {
            SID_PAL_LOG_ERROR("Failed to create GNSS command completion semaphore");
            err = SID_ERROR_OOM;
            break;
        }

        /* Create GNSS fix semaphores */
        drv_ctx.gnss_fix_obtained_sem = osSemaphoreNew(1u, 0u, &default_teseo_sem_attr);
        if (NULL == drv_ctx.gnss_fix_obtained_sem)
        {
            SID_PAL_LOG_ERROR("Failed to create GNSS Fix Obtained event semaphore");
            err = SID_ERROR_OOM;
            break;
        }
        drv_ctx.gnss_fix_lost_sem = osSemaphoreNew(1u, 0u, &default_teseo_sem_attr);
        if (NULL == drv_ctx.gnss_fix_lost_sem)
        {
            SID_PAL_LOG_ERROR("Failed to create GNSS Fix Lost event semaphore");
            err = SID_ERROR_OOM;
            break;
        }

        /* GNSS engine restart semaphore */
        drv_ctx.gnss_engine_restart_sem = osSemaphoreNew(1u, 0u, &default_teseo_sem_attr);
        if (NULL == drv_ctx.gnss_engine_restart_sem)
        {
            SID_PAL_LOG_ERROR("Failed to create GNSS Engine Restart event semaphore");
            err = SID_ERROR_OOM;
            break;
        }

        /* Initialize GNSS message parser */
        parser_status = GNSS_PARSER_Init(&drv_ctx.gnss_data);
        if (parser_status != GNSS_PARSER_OK)
        {
            SID_PAL_LOG_ERROR("Failed to initialize GNSS message parser");
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Initialize UART */
        err = _prv_uart_init();
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to initialize UART for Teseo. Error %d", (int32_t)err);
            break;
        }

        /* Create timer to periodically check received data for valid frames */
        err = sid_pal_timer_init(&drv_ctx.standby_wakeup_timer, _prv_wakeup_timer_event_handler, NULL);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to create Teseo wakeup timer. Error %d");
            break;
        }

        /* Set initialization flag to allow usage of public API calls */
        drv_ctx.init_done = TRUE;

        /* Reset Teseo module using nRESET pin */
        err = teseo_gnss_reset(TESEO_RESET_SYS_HARDWARE);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Request version information */
        SID_PAL_LOG_INFO("Retrieving Teseo version information...");
        err = teseo_gnss_request_version_info(TESEO_VER_TYPE_ALL_SUPPORTED);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Print out version information */
        SID_PAL_LOG_INFO("Detected Teseo %s. FW: %u.%u.%u.%u, GNSS Lib: %u.%u.%u.%u, OS20 Lib: %u.%u.%u, GPS App: %u.%u.%u.%u",
            drv_ctx.version_info.dev_name,
            drv_ctx.version_info.bin_image.major, drv_ctx.version_info.bin_image.minor, drv_ctx.version_info.bin_image.patch, drv_ctx.version_info.bin_image.build,
            drv_ctx.version_info.gnss_lib.major, drv_ctx.version_info.gnss_lib.minor, drv_ctx.version_info.gnss_lib.patch, drv_ctx.version_info.gnss_lib.build,
            drv_ctx.version_info.os20_lib.major, drv_ctx.version_info.os20_lib.minor, drv_ctx.version_info.os20_lib.patch,
            drv_ctx.version_info.gps_app.major, drv_ctx.version_info.gps_app.minor, drv_ctx.version_info.gps_app.patch, drv_ctx.version_info.gps_app.build
        );
        SID_PAL_LOG_INFO("Teseo unique code: %s", drv_ctx.version_info.ucode);

        /* Download CDB parameters so that this driver will be aligned with the actual configuration of Teseo */
        SID_PAL_LOG_INFO("Retrieving Teseo configuration info...");
        err = _prv_load_cdb_cache(&drv_ctx.cdb_cache);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Apply desired configuration */
        uint32_t config_update_required = FALSE;

        /* Run in a critical section to guarantee CDB cache integrity */
        sid_pal_enter_critical_region();

        /* Ensure stored UART baud rate corresponds to the desired one  --------------*/
        teseo_cdb_102_nmea_port_baudrate_t nmea_port_baudrate = {
            .raw = drv_ctx.cdb_cache.nmea_port_baudrate.raw,
        };

        nmea_port_baudrate.br_cfg_val = uart_baudrate_cfg_val;

        if (nmea_port_baudrate.raw != drv_ctx.cdb_cache.nmea_port_baudrate.raw)
        {
            /* Desired config differs from what is currently applied in Teseo - update is required */
            uint32_t actual_baudrate;
            (void)_prv_uart_teseo_param_to_baudrate(drv_ctx.cdb_cache.nmea_port_baudrate.br_cfg_val, &actual_baudrate);
            SID_PAL_LOG_WARNING("Teseo UART is configured to run at %ubps instead of %ubps", actual_baudrate, config->uart_cfg->baud_rate);
            config_update_required = TRUE;
        }
        /*----------------------------------------------------------------------------*/

        /* Adjust message list -------------------------------------------------------*/
        teseo_cdb_201_uart_msg_list_low_t  msg_list_low = {
            .raw = drv_ctx.cdb_cache.msg_list_low.raw,
        };
        teseo_cdb_228_uart_msg_list_high_t msg_list_high = {
            .raw = drv_ctx.cdb_cache.msg_list_high.raw,
        };

        /* Absolutely required messages */
        msg_list_low.GPGGA_EN = TRUE; /* The main message used for positioning information */
        msg_list_low.GPGST_EN = TRUE; /* Required to calculate positioning data accuracy */
        msg_list_low.GPRMC_EN = TRUE; /* Used to extract date information */
        msg_list_low.GPVTG_EN = TRUE; /* Used for speed and heading data */

        /* Unused/redundant messages that can be safely disabled to reduce UART traffic */
        // TODO: make message list user-configurable for the optional messages
        msg_list_low.GPGLL_EN = FALSE;
        msg_list_low.GPGSA_EN = FALSE;
        msg_list_low.GPGSV_EN = FALSE;

        /* The rest of the messages are kept as is, their configuration is preserved */

        if ((msg_list_low.raw != drv_ctx.cdb_cache.msg_list_low.raw) || (msg_list_high.raw != drv_ctx.cdb_cache.msg_list_high.raw))
        {
            /* Desired config differs from what is currently applied in Teseo - update is required */
            config_update_required = TRUE;
        }
        /*----------------------------------------------------------------------------*/

        /* Adjust Teseo's GPS app config ---------------------------------------------*/
        teseo_cdb_200_application_onoff_t   app_cfg1 = {
            .raw = drv_ctx.cdb_cache.app_cfg1.raw,
        };
        teseo_cdb_227_application_onoff_2_t app_cfg2 = {
            .raw = drv_ctx.cdb_cache.app_cfg2.raw,
        };

        /* Disable command echoing to save on UART traffic. When enabled, teseo_gnss_send_command() will wait for the echo to arrive before executing new command */
        app_cfg2.NMEA_COMMAND_ECHO_ENABLE = FALSE;

        /* Adjust constellations to use from user-supplied config */
        app_cfg1.GPS_ENABLE               = config->constellations_to_use.use_gps;
        app_cfg1.GPS_USE_ENABLE           = config->constellations_to_use.use_gps;

        app_cfg2.GALILEO_ENABLE           = config->constellations_to_use.use_galileo;
        app_cfg2.GALILEO_USAGE_ENABLE     = config->constellations_to_use.use_galileo;

        app_cfg2.BEIDOU_ENABLE            = config->constellations_to_use.use_beidou;
        app_cfg2.BEIDOU_USAGE_ENABLE      = config->constellations_to_use.use_beidou;

        app_cfg1.QZSS_ENABLE              = config->constellations_to_use.use_qzss;
        app_cfg1.QZSS_USE_ENABLE          = config->constellations_to_use.use_qzss;

        app_cfg1.GLONASS_ENABLE           = config->constellations_to_use.use_glonass;
        app_cfg1.GLONASS_USE_ENABLE       = config->constellations_to_use.use_glonass;

        /* Misc configuration settings */
        app_cfg1.CONFIG_TXT_HEADER_EN     = TRUE;

        /* Check if application config has changed */
        if ((app_cfg1.raw != drv_ctx.cdb_cache.app_cfg1.raw) || (app_cfg2.raw != drv_ctx.cdb_cache.app_cfg2.raw))
        {
            /* Desired config differs from what is currently applied in Teseo - update is required */
            config_update_required = TRUE;
        }
        /*----------------------------------------------------------------------------*/

        /* Done with critical stuff, further actions don't required the critical section */
        sid_pal_exit_critical_region();

        /* Apply config changes if needed */
        if (config_update_required != FALSE)
        {
#if TESEO_GNSS_CFG_ENFORCE_COMPATIBLE_CONFIG
            SID_PAL_LOG_INFO("Teseo configuration is not aligned with the driver. Updating Teseo's configuration...");

            /* Send out UART baud rate config */
            err = teseo_gnss_send_nmea_port_baudrate_config(nmea_port_baudrate);
            if (err != SID_ERROR_NONE)
            {
                /* Logs are provided by teseo_gnss_send_message_config() */
                break;
            }

            /* Send out desired message config */
            err = teseo_gnss_send_message_config(msg_list_low, msg_list_high, 1u);
            if (err != SID_ERROR_NONE)
            {
                /* Logs are provided by teseo_gnss_send_message_config() */
                break;
            }

            err = teseo_gnss_send_app_config(app_cfg1, app_cfg2);
            if (err != SID_ERROR_NONE)
            {
                /* Logs are provided by teseo_gnss_send_app_config() */
                break;
            }

            /* Apply the new configuration - this will activate the new config and write it to Teseo's NVM */
            err = teseo_gnss_apply_configuration();
            if (err != SID_ERROR_NONE)
            {
                /* Logs are provided by teseo_gnss_apply_configuration() */
                break;
            }
#else
            SID_PAL_LOG_WARNING("Teseo configuration is not aligned with the driver's expectations. Fix the config to avoid issues");
#endif /* TESEO_GNSS_CFG_ENFORCE_COMPATIBLE_CONFIG */
        }
        else
        {
            SID_PAL_LOG_INFO("Teseo configuration is aligned with the driver. No changes required");
        }

        /* Done */
        SID_PAL_LOG_INFO("Teseo initialization completed");
        err = SID_ERROR_NONE;
    } while (0);

    if ((err != SID_ERROR_NONE) && (err != SID_ERROR_ALREADY_INITIALIZED))
    {
        /* Release all resources if initialization has failed */
        drv_ctx.init_done = TRUE; /* Temporarily override the initialization state to allow the teseo_gnss_deinit() to release any allocated resources */
        (void)teseo_gnss_deinit();
    }

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t teseo_gnss_deinit(void)
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_ctx.init_done)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* De-initialize wakeup timer */
        err = sid_pal_timer_deinit(&drv_ctx.standby_wakeup_timer);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to de-initialize Teseo wakeup timer. Error %d", (int32_t)err);
            /* Proceed with de-initialization anyway */
        }

        /* De-initialize UART */
        if ((drv_ctx.uart_ifc != NULL) && ((*drv_ctx.uart_ifc)->destroy != NULL))
        {
            (*drv_ctx.uart_ifc)->destroy(drv_ctx.uart_ifc);
        }

        /* De-initialize GPIO */
        err = _prv_gpio_deinit();
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to de-initialize Teseo GPIO. Error %d", (int32_t)err);
            /* Proceed with de-initialization anyway */
        }

        /* Delete data synchronization semaphores */
        for (uint32_t msg_type = 0u; msg_type < NMEA_MSGS_NUM; msg_type++)
        {
            if (drv_ctx.gnss_data_locks[msg_type] != NULL)
            {
                (void)osSemaphoreDelete(drv_ctx.gnss_data_locks[msg_type]);
            }
        }

        /* Delete command completion semaphore */
        if (drv_ctx.cmd_completion_lock != NULL)
        {
            (void)osSemaphoreDelete(drv_ctx.cmd_completion_lock);
        }

        /* Delete GNSS fix semaphores */
        if (drv_ctx.gnss_fix_obtained_sem != NULL)
        {
            (void)osSemaphoreDelete(drv_ctx.gnss_fix_obtained_sem);
        }
        if (drv_ctx.gnss_fix_lost_sem != NULL)
        {
            (void)osSemaphoreDelete(drv_ctx.gnss_fix_lost_sem);
        }

        /* Delete GNSS engine restart semaphore */
        if (drv_ctx.gnss_engine_restart_sem != NULL)
        {
            (void)osSemaphoreDelete(drv_ctx.gnss_engine_restart_sem);
        }

        /* Clear context */
        SID_STM32_UTIL_fast_memset(&drv_ctx, 0u, sizeof(drv_ctx));

        /* Done */
        SID_PAL_LOG_INFO("Teseo de-initialization completed");
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}
