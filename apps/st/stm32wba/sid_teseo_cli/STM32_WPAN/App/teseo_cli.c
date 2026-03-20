/**
  ******************************************************************************
  * @file    teseo_cli.c
  * @brief   CLI for driving Teseo GNSS receivers
  *
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

#include <stdlib.h>
#include <string.h>

/* Sidewalk DUT interfaces */
#include <sid_asd_cli.h>

/* Sidewalk interfaces */
#include <sid_hal_reset_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_gpio_ifc.h>

/* Teseo interfaces */
#include "teseo_gnss.h"

#include "app_teseo_config.h"

/* Private defines -----------------------------------------------------------*/

#ifdef SYNTAX_ERR
#undef SYNTAX_ERR
#endif
#define SYNTAX_ERR                                    "Syntax err \r\n"

/*----------------------------------------------------------------------------*/

#define SID_STM32_CLI_REBOOT_H                        "Reboot the host MCU (not the Teseo unit)"

#define SID_STM32_CLI_REBOOT_CMD                      "reboot"

/*----------------------------------------------------------------------------*/

#define SID_STM32_TESEO_CLI_INIT_H                    "Initialize UART bus and GPIO to access Teseo module"
#define SID_STM32_TESEO_CLI_DEINIT_H                  "Release Teseo module and related hardware resources"
#define SID_STM32_TESEO_CLI_RESET_H                   "Reset Teseo. Optionally use \"-hard\", \"-soft\", \"-gnss\" parameters to select reset type. Hardware reset is performed by default"
#define SID_STM32_TESEO_CLI_VERSION_H                 "Prints out version information of the Teseo module"
#define SID_STM32_TESEO_CLI_ECHO_H                    "Enable/disable echoing of Teseo messages to Sidewalk log console. Usage: echo [on|off]"
#define SID_STM32_TESEO_CLI_GET_POSITION_H            "Get the most recent GNSS position data"
#define SID_STM32_TESEO_CLI_SEND_COMMAND_H            "Send a raw command string to Teseo unit. Format: "SID_STM32_TESEO_CLI_SEND_COMMAND_CMD" \"$command,param1,param2\". Use with caution and don't forget to enclose command string in quotes"
#define SID_STM32_TESEO_CLI_RESTORE_DEFAULTS_H        "Restores factory default configuration of the Teseo unit via $PSTMRESTOREPAR command"
#define SID_STM32_TESEO_CLI_STANDBY_H                 "Force Teseo into Standby mode"
#define SID_STM32_TESEO_CLI_WAKEUP_H                  "Wakeup Teseo from Standby mode"

#define SID_STM32_TESEO_CLI_ROOT                      "teseo"

#define SID_STM32_TESEO_CLI_INIT_CMD                  SID_STM32_TESEO_CLI_ROOT" init"
#define SID_STM32_TESEO_CLI_DEINIT_CMD                SID_STM32_TESEO_CLI_ROOT" deinit"
#define SID_STM32_TESEO_CLI_RESET_CMD                 SID_STM32_TESEO_CLI_ROOT" reset"
#define SID_STM32_TESEO_CLI_VERSION_CMD               SID_STM32_TESEO_CLI_ROOT" version"
#define SID_STM32_TESEO_CLI_ECHO_CMD                  SID_STM32_TESEO_CLI_ROOT" echo"
#define SID_STM32_TESEO_CLI_GET_POSITION_CMD          SID_STM32_TESEO_CLI_ROOT" get_position"
#define SID_STM32_TESEO_CLI_SEND_COMMAND_CMD          SID_STM32_TESEO_CLI_ROOT" cmd"
#define SID_STM32_TESEO_CLI_RESTORE_DEFAULTS_CMD      SID_STM32_TESEO_CLI_ROOT" restore_factory_defaults"
#define SID_STM32_TESEO_CLI_STANDBY_CMD               SID_STM32_TESEO_CLI_ROOT" standby"
#define SID_STM32_TESEO_CLI_WAKEUP_CMD                SID_STM32_TESEO_CLI_ROOT" wakeup"

/*----------------------------------------------------------------------------*/

#define SID_STM32_TESEO_CLI_BASE_16                   (16u)
#define SID_STM32_TESEO_CLI_BASE_10                   (10u)
#define SID_STM32_TESEO_CLI_CAPITAL_START             (64u)
#define SID_STM32_TESEO_CLI_CAPITAL_END               (71u)
#define SID_STM32_TESEO_CLI_CAPITAL_SHIFT             (7u)
#define SID_STM32_TESEO_CLI_LOWER_START               (96u)
#define SID_STM32_TESEO_CLI_LOWER_END                 (103u)
#define SID_STM32_TESEO_CLI_LOWER_SHIFT               (39u)
#define SID_STM32_TESEO_CLI_NUMBER_START              (47u)
#define SID_STM32_TESEO_CLI_NUMBER_END                (58u)

/* Private macro -------------------------------------------------------------*/

#define SID_STM32_TESEO_CLI_STRINGIFY(__S__)                    #__S__
#define SID_STM32_TESEO_CLI_ASSERT_RESULT(__FUNC__, __ERROR__)  if (sid_err != SID_ERROR_NONE) \
                                                                { \
                                                                    CLI_LOG_ERROR(SID_STM32_TESEO_CLI_STRINGIFY(__FUNC__) " failed with error %d", (int32_t)(__ERROR__)); \
                                                                    break; \
                                                                }

#define CLI_LOG_OUTCOME(__ERR__)                                if (SID_ERROR_NONE == (__ERR__)) \
                                                                { \
                                                                    CLI_LOG_INFO("CMD: ERR: %d", (int32_t)(__ERR__)); \
                                                                } \
                                                                else \
                                                                { \
                                                                    CLI_LOG_ERROR("CMD: ERR: %d", (int32_t)(__ERR__)); \
                                                                }

/* Private function prototypes -----------------------------------------------*/

static inline uint32_t     teseo_cli_utils_str_to_uint(const char * const str, const uint32_t base);
static inline int32_t      teseo_cli_utils_str_to_int(const char * const str, const uint8_t base);
static        int32_t      teseo_cli_utils_parse_input_num(const char * const buf);

static        void         on_gnss_fix_obtained(void * user_arg);
static        void         on_gnss_fix_lost(void * user_arg);

static        ace_status_t teseo_cli_init_cmd(int32_t argc, const char **argv);
static        ace_status_t teseo_cli_deinit_cmd(int32_t argc, const char **argv);
static        ace_status_t teseo_cli_reset_cmd(int32_t argc, const char **argv);
static        ace_status_t teseo_cli_version_cmd(int32_t argc, const char **argv);
static        ace_status_t teseo_cli_echo_cmd(int32_t argc, const char **argv);
static        ace_status_t teseo_cli_get_position_cmd(int32_t argc, const char **argv);
static        ace_status_t teseo_cli_send_command_cmd(int32_t argc, const char **argv);
static        ace_status_t teseo_cli_restore_defaults_cmd(int32_t argc, const char **argv);
static        ace_status_t teseo_cli_standby_cmd(int32_t argc, const char **argv);
static        ace_status_t teseo_cli_wakeup_cmd(int32_t argc, const char **argv);

static        ace_status_t teseo_cli_mcu_reset_cmd(int32_t argc, const char **argv);

/* Private constants ---------------------------------------------------------*/

SID_CLI_REGISTER_COMMAND(m_sub_teseo)
{
    SID_CLI_DEFINE_COMMAND(                    init,      NULL, SID_STM32_TESEO_CLI_INIT_H,             teseo_cli_init_cmd),
    SID_CLI_DEFINE_COMMAND(                  deinit,      NULL, SID_STM32_TESEO_CLI_DEINIT_H,           teseo_cli_deinit_cmd),
    SID_CLI_DEFINE_COMMAND(                   reset,      NULL, SID_STM32_TESEO_CLI_RESET_H,            teseo_cli_reset_cmd),
    SID_CLI_DEFINE_COMMAND(                 version,      NULL, SID_STM32_TESEO_CLI_VERSION_H,          teseo_cli_version_cmd),
    SID_CLI_DEFINE_COMMAND(                    echo,      NULL, SID_STM32_TESEO_CLI_ECHO_H,             teseo_cli_echo_cmd),
    SID_CLI_DEFINE_COMMAND(            get_position,      NULL, SID_STM32_TESEO_CLI_GET_POSITION_H,     teseo_cli_get_position_cmd),
    SID_CLI_DEFINE_COMMAND(                     cmd,      NULL, SID_STM32_TESEO_CLI_SEND_COMMAND_H,     teseo_cli_send_command_cmd),
    SID_CLI_DEFINE_COMMAND(restore_factory_defaults,      NULL, SID_STM32_TESEO_CLI_RESTORE_DEFAULTS_H, teseo_cli_restore_defaults_cmd),
    SID_CLI_DEFINE_COMMAND(                 standby,      NULL, SID_STM32_TESEO_CLI_STANDBY_H,          teseo_cli_standby_cmd),
    SID_CLI_DEFINE_COMMAND(                  wakeup,      NULL, SID_STM32_TESEO_CLI_WAKEUP_H,           teseo_cli_wakeup_cmd),
    SID_CLI_SUBCMD_SET_END,
};

SID_CLI_REGISTER_COMMAND(m_teseo_commands)
{
    SID_CLI_DEFINE_COMMAND(                  reboot,      NULL, SID_STM32_CLI_REBOOT_H,                 teseo_cli_mcu_reset_cmd),
    SID_CLI_DEFINE_SUB_COMMAND_SET(           teseo,      NULL, "Teseo control commands",               m_sub_teseo),
    SID_CLI_SUBCMD_SET_END,
};

/* Private function definitions ----------------------------------------------*/

static inline uint32_t teseo_cli_utils_str_to_uint(const char * const str, const uint32_t base)
{
    uint32_t uint_val;

    do
    {
        char stoi_char;
        uint32_t flag;

        if (NULL == str)
        {
            uint_val = 0;
            break;
        }

        uint_val = 0u;
        const char * str_pos = str;
        while ((*str_pos != '\0') && (*str_pos != '\n') && (*str_pos != '\r') && (*str_pos != ' '))
        {
            stoi_char = *str_pos;
            if (stoi_char != '.') /* skip over decimal point to convert floats to ints */
            {
                uint_val = uint_val * base;   /* mult by base */
                flag = FALSE;

                if ((SID_STM32_TESEO_CLI_BASE_16 == base) && (stoi_char > SID_STM32_TESEO_CLI_CAPITAL_START) && (stoi_char < SID_STM32_TESEO_CLI_CAPITAL_END))
                {
                    stoi_char -= SID_STM32_TESEO_CLI_CAPITAL_SHIFT;
                    flag = TRUE;
                }

                if ((FALSE == flag) && (SID_STM32_TESEO_CLI_BASE_16 == base) && (stoi_char > SID_STM32_TESEO_CLI_LOWER_START) && (stoi_char < SID_STM32_TESEO_CLI_LOWER_END))
                {
                    stoi_char -= SID_STM32_TESEO_CLI_LOWER_SHIFT;
                    flag = TRUE;
                }

                if ((FALSE == flag) && (stoi_char > SID_STM32_TESEO_CLI_NUMBER_START) && (stoi_char < SID_STM32_TESEO_CLI_NUMBER_END))
                {
                    flag = TRUE;
                }

                if (flag != FALSE)
                {
                    uint_val = uint_val + (uint32_t)stoi_char;
                    uint_val = uint_val - (uint32_t)'0';
                }
                else
                {
                    /* Parsing error */
                    uint_val = 0u;
                    break;
                }
            }
            str_pos++;
        }
    } while (0);

    return uint_val;
}

/*----------------------------------------------------------------------------*/

static inline int32_t teseo_cli_utils_str_to_int(const char * const str, const uint8_t base)
{
    int32_t int_val;

    do
    {
        if (NULL == str)
        {
            int_val = 0;
            break;
        }

        if (*str == '-')
        {
            int_val = -teseo_cli_utils_str_to_uint(&str[1], base);
        }
        else
        {
            int_val = teseo_cli_utils_str_to_uint(str, base);
        }
    } while (0);

    return int_val;
}

/*----------------------------------------------------------------------------*/

static int32_t teseo_cli_utils_parse_input_num(const char * const buf)
{
    int32_t int_val;
    uint32_t hex_format;
    uint32_t idx = 0u;

    if ((buf[0] == '0') && ((buf[1] == 'x') || (buf[1] == 'X')))
    {
        idx = 2u;
        hex_format = TRUE;
    }
    else if ((buf[0] == 'x') || (buf[0] == 'X'))
    {
        idx = 1u;
        hex_format = TRUE;
    }
    else
    {
        hex_format = FALSE;
    }

    if (hex_format != FALSE)
    {
        int_val = teseo_cli_utils_str_to_int(&(buf[idx]), SID_STM32_TESEO_CLI_BASE_16);
    }
    else
    {
        int_val = teseo_cli_utils_str_to_int(&(buf[idx]), SID_STM32_TESEO_CLI_BASE_10);
    }

    return int_val;
}

/*----------------------------------------------------------------------------*/

static void on_gnss_fix_obtained(void * user_arg)
{
    (void)user_arg;
    CLI_LOG_INFO("GNSS fix obtained");
}

/*----------------------------------------------------------------------------*/

static void on_gnss_fix_lost(void * user_arg)
{
    (void)user_arg;
    CLI_LOG_INFO("GNSS fix lost");
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_init_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_INIT_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        teseo_gnss_event_callbacks_t teseo_callbacks = {
            .on_gnss_fix_obtained = on_gnss_fix_obtained,
            .on_gnss_fix_lost = on_gnss_fix_lost,
        };

        /* Initialize Teseo driver */
        sid_err = teseo_gnss_init(get_teseo_cfg(), &teseo_callbacks);
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_init, sid_err);

        /* Forward Teseo output to Sidewalk log console */
        sid_err = teseo_gnss_enable_log_mirroring();
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_enable_log_mirroring, sid_err);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_deinit_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_DEINIT_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Initialize Teseo driver */
        sid_err = teseo_gnss_deinit();
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_deinit, sid_err);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_reset_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        teseo_gnss_reset_type_t reset_type;

        if (argc > 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_RESET_CMD " takes at most one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if ((0 == argc) || (strcmp(argv[0], "-h") == 0) || (strcmp(argv[0], "--hard") == 0))
        {
            /* Perform system hard reset */
            reset_type = TESEO_RESET_SYS_HARDWARE;
        }
        else if ((strcmp(argv[0], "-s") == 0) || (strcmp(argv[0], "--soft") == 0))
        {
            /* Perform system soft reset */
            reset_type = TESEO_RESET_SYS_SOFTWAREWARE;
        }
        else if ((strcmp(argv[0], "-g") == 0) || (strcmp(argv[0], "--gnss") == 0))
        {
            /* Perform GNSS engine-only reset */
            reset_type = TESEO_RESET_GNSS_ENGINE;
        }
        else
        {
            CLI_LOG_ERROR("\"%s\" is not a recognized reset type. Valid values are: --hard (-h), --soft (-s), and --gnss (-g)", argv[0]);
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Perform the selected Teseo reset */
        sid_err = teseo_gnss_reset(reset_type);
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_reset, sid_err);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_version_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_VERSION_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Retrieve version data */
        teseo_gnss_version_info_t version_info;
        sid_err = teseo_gnss_get_version_info(&version_info);
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_get_version_info, sid_err);

        /* Print out version information */
        CLI_LOG_INFO("Teseo version info: HW: %s, FW: %u.%u.%u.%u, GNSS Lib: %u.%u.%u.%u, OS20 Lib: %u.%u.%u, GPS App: %u.%u.%u.%u",
            version_info.dev_name,
            version_info.bin_image.major, version_info.bin_image.minor, version_info.bin_image.patch, version_info.bin_image.build,
            version_info.gnss_lib.major,  version_info.gnss_lib.minor,  version_info.gnss_lib.patch,  version_info.gnss_lib.build,
            version_info.os20_lib.major,  version_info.os20_lib.minor,  version_info.os20_lib.patch,
            version_info.gps_app.major,   version_info.gps_app.minor,   version_info.gps_app.patch,   version_info.gps_app.build
        );
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_echo_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc > 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_ECHO_CMD " takes at most one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (0 == argc)
        {
            uint8_t is_enabled;

            sid_err = teseo_gnss_get_log_mirroring_status(&is_enabled);
            SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_get_log_mirroring_status, sid_err);
            if (FALSE == is_enabled)
            {
                CLI_LOG_INFO("Teseo echoing is disabled");
            }
            else
            {
                CLI_LOG_INFO("Teseo echoing is enabled");
            }
        }
        else if (strcmp(argv[0], "off") == 0)
        {
            /* Disable message mirroring */
            sid_err = teseo_gnss_disable_log_mirroring();
            SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_disable_log_mirroring, sid_err);
        }
        else if (strcmp(argv[0], "on") == 0)
        {
            /* Forward Teseo output to Sidewalk log console */
            sid_err = teseo_gnss_enable_log_mirroring();
            SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_enable_log_mirroring, sid_err);
        }
        else
        {
            CLI_LOG_ERROR("\"%s\" is not a recognized value. Valid options are: on, off", argv[0]);
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_get_position_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_GET_POSITION_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Retrieve position data */
        teseo_gnss_position_data_t position;
        sid_err = teseo_gnss_get_position(&position);
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_deinit, sid_err);

        CLI_LOG_INFO("Latitude: %d.%07u, Longitude: %d.%07u, Elevation: %d.%02u",
            (int32_t)position.latitude,  abs((int32_t)(fmodf(position.latitude, 1.f) * 1e7f)), /* Have to do this because logging API uses tiny_vsnprintf and does not support %f format */
            (int32_t)position.longitude, abs((int32_t)(fmodf(position.longitude, 1.f) * 1e7f)),
            (int32_t)position.elevation, abs((int32_t)(fmodf(position.elevation, 1.f) * 1e2f))
        );
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_send_command_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_SEND_COMMAND_CMD " takes exactly one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Forward the command to Teseo */
        sid_err = teseo_gnss_send_command(argv[0]);
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_send_command, sid_err);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_restore_defaults_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_RESTORE_DEFAULTS_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        sid_err = teseo_gnss_restore_factory_defaults();
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_restore_factory_defaults, sid_err);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_standby_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        uint32_t standby_duration_seconds;

        if (argc > 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_STANDBY_CMD " takes up to one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (argc == 0)
        {
            standby_duration_seconds = 0u;
        }
        else
        {
            standby_duration_seconds = teseo_cli_utils_parse_input_num(argv[0]);
        }

        sid_err = teseo_gnss_standby(standby_duration_seconds);
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_standby, sid_err);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_wakeup_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_TESEO_CLI_WAKEUP_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        sid_err = teseo_gnss_wakeup();
        SID_STM32_TESEO_CLI_ASSERT_RESULT(teseo_gnss_wakeup, sid_err);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t teseo_cli_mcu_reset_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_CLI_REBOOT_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        CLI_LOG_WARNING("Resetting host MCU on request...");
        CLI_LOG_FLUSH();

        sid_err = sid_hal_reset(SID_HAL_RESET_NORMAL);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/* Global function definitions -----------------------------------------------*/

sid_error_t teseo_cli_init(void)
{
    return SID_CLI_REGISTER_SUB_COMMAND_SET(m_teseo_commands);
}
