/**
  ******************************************************************************
  * @file    lr11xx_sid_pal_gnss_wifi.c
  * @brief   Implementation of sid_pal_wifi and sid_pal_gnss layers for LR11xx
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

#include <assert.h>
#include <stdbool.h>

#include "halo_lr11xx_radio.h"
#include "lr11xx_radio_config.h"
#include "lr11xx_radio_ext_ifc.h"

/* Semtech LoRa Basics Modem middleware */
#include <smtc_modem_ext_api.h>
#include <smtc_modem_geolocation_api.h>
#include <smtc_modem_ext_ral_bsp.h>
#include <smtc_modem_utilities.h>

/* Sidewalk interfaces */
#include <sid_location.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_timer_ifc.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>

#if SID_SDK_CONFIG_ENABLE_GNSS
#  include <sid_pal_gnss_ifc.h>
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

#if SID_SDK_CONFIG_ENABLE_WIFI
#  include <sid_pal_wifi_ifc.h>
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

/* Utilities and helpers */
#include "stm32_rtos.h"
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#ifndef LR11XX_SID_PAL_LOCATION_EXTRA_LOGGING
/* Set LR11XX_SID_PAL_LOCATION_EXTRA_LOGGING to 1 to enable extended logs */
#  define LR11XX_SID_PAL_LOCATION_EXTRA_LOGGING         (0)
#endif

#define LR11XX_SID_PAL_LOCATION_GNSS_NG_TLV_TAG         ((uint8_t)(0x53u))
#define LR11XX_SID_PAL_LOCATION_GNSS_NG_LAST_ENTRY_FLAG ((uint8_t)(1u << 7))

/* Private macros ------------------------------------------------------------*/

#if LR11XX_SID_PAL_LOCATION_EXTRA_LOGGING
#  define LR11XX_SID_PAL_LOCATION_LOG_ERROR(...)        SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define LR11XX_SID_PAL_LOCATION_LOG_WARNING(...)      SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define LR11XX_SID_PAL_LOCATION_LOG_INFO(...)         SID_PAL_LOG_INFO(__VA_ARGS__)
#  define LR11XX_SID_PAL_LOCATION_LOG_DEBUG(...)        SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define LR11XX_SID_PAL_LOCATION_LOG_TRACE(...)        SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define LR11XX_SID_PAL_LOCATION_LOG_ERROR(...)        ((void)0u)
#  define LR11XX_SID_PAL_LOCATION_LOG_WARNING(...)      ((void)0u)
#  define LR11XX_SID_PAL_LOCATION_LOG_INFO(...)         ((void)0u)
#  define LR11XX_SID_PAL_LOCATION_LOG_DEBUG(...)        ((void)0u)
#  define LR11XX_SID_PAL_LOCATION_LOG_TRACE(...)        ((void)0u)
#endif /* LR11XX_SID_PAL_LOCATION_EXTRA_LOGGING */

/* Private types -------------------------------------------------------------*/

/**
 * @brief Context of the location features of LR11xx. Holds internal states and data
 */
typedef struct {
    /* Note: the layout of this structure is crafted to minimize RAM footprint. This results in excessive amount of preprocessor directives */
#if SID_SDK_CONFIG_ENABLE_GNSS
    uint8_t                                gnss_init_done;
    uint8_t                                gnss_scan_scheduled;
    uint8_t                                gnss_scan_retry_counter;
    uint8_t                                almanac_demod_started;
    smtc_modem_gnss_event_data_scan_done_t gnss_scan_done_data;
    struct sid_pal_gnss_config             gnss_sid_config;
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

#if SID_SDK_CONFIG_ENABLE_WIFI
    uint8_t                                wifi_init_done;
    uint8_t                                wifi_scan_scheduled;
    uint8_t                                wifi_scan_retry_counter;
    smtc_modem_wifi_event_data_scan_done_t wifi_scan_done_data;
    struct sid_pal_wifi_config             wifi_sid_config;
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

    bool                                   lbm_radio_comm_suspended;
    smtc_modem_return_code_t               lbm_last_error;
} lr11xx_location_ctx_t;

#if SID_SDK_CONFIG_ENABLE_GNSS
typedef __PACKED_STRUCT {
    uint8_t tag;
    uint8_t length;
} lr11xx_location_gnss_tlv_header_t;

typedef __PACKED_STRUCT {
    lr11xx_location_gnss_tlv_header_t tlv_header;
    uint8_t                           token;
} lr11xx_location_gnss_scan_header_t;
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

/* Private variables ---------------------------------------------------------*/

static lr11xx_location_ctx_t drv_location_ctx = { 0 };

/* Private constants ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static sid_error_t smtc_lbm_schedule_modem_engine_run(const uint32_t delay_ms, const enum sid_location_method_type source);
static sid_error_t smtc_lbm_process_events(const enum sid_location_method_type source);
static void        smtc_lbm_event_callback(void);
static sid_error_t sid_pal_location_common_init(void);
static sid_error_t sid_pal_location_common_deinit(void);

/* Private function definitions ----------------------------------------------*/

static sid_error_t smtc_lbm_schedule_modem_engine_run(const uint32_t delay_ms, const enum sid_location_method_type source)
{
    sid_error_t err;

    do
    {
#if SID_SDK_CONFIG_ENABLE_WIFI
        if ((source & SID_LOCATION_METHOD_WIFI) != 0u)
        {
            if ((FALSE == drv_location_ctx.wifi_init_done) || (NULL == drv_location_ctx.wifi_sid_config.on_wifi_event))
            {
                err = SID_ERROR_UNINITIALIZED;
                /* Do not terminate from here, SID_LOCATION_METHOD_GNSS may be still be set and handled by the GNSS portion if enabled */
            }
            else
            {
                drv_location_ctx.wifi_sid_config.on_wifi_event(drv_location_ctx.wifi_sid_config.ctx, SID_PAL_WIFI_INTERNAL, delay_ms);
                err = SID_ERROR_NONE;
                break;
            }
        }
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

#if SID_SDK_CONFIG_ENABLE_GNSS
        if ((source & SID_LOCATION_METHOD_GNSS) != 0u)
        {
            if ((FALSE == drv_location_ctx.gnss_init_done) || (NULL == drv_location_ctx.gnss_sid_config.on_gnss_event))
            {
                err = SID_ERROR_UNINITIALIZED;
                break;
            }
            else
            {
                drv_location_ctx.gnss_sid_config.on_gnss_event(drv_location_ctx.gnss_sid_config.ctx, SID_PAL_GNSS_INTERNAL, delay_ms);
                err = SID_ERROR_NONE;
                break;
            }
        }
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

        /* Report no-support if source was not handled by either GNSS or WiFi branches */
        err = SID_ERROR_NOSUPPORT;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t smtc_lbm_process_events(const enum sid_location_method_type source)
{
    sid_error_t err;
    uint32_t    lbm_next_event_sleep_time_ms;

    /* Process LoRa Basics Modem event(s) */
    do
    {
        lbm_next_event_sleep_time_ms = smtc_modem_run_engine();
    } while ((0u == lbm_next_event_sleep_time_ms) || (smtc_modem_is_irq_flag_pending() != false));

    /* Schedule next invocation of LBM event processing */
    err = smtc_lbm_schedule_modem_engine_run(lbm_next_event_sleep_time_ms, source);

    return err;
}

/*----------------------------------------------------------------------------*/

static void smtc_lbm_event_callback(void)
{
    smtc_modem_return_code_t                                    lbm_err;
    smtc_modem_event_t                                          lbm_current_event;
    uint8_t                                                     lbm_pending_event_count;
#if SID_SDK_CONFIG_ENABLE_GNSS
    smtc_modem_almanac_demodulation_event_data_almanac_update_t almanac_update_data;
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */
    const halo_drv_semtech_ctx_t * const                        drv_ctx = lr11xx_get_drv_ctx();

    /* Retrieve LBM events until all of them are processed */
    do
    {
        /* Read modem event */
        lbm_err = smtc_modem_get_event(&lbm_current_event, &lbm_pending_event_count);
        if (lbm_err != SMTC_MODEM_RC_OK)
        {
            SID_PAL_LOG_ERROR("LBM: Failed to fetch active event. Error %u", (uint32_t)lbm_err);
            break;
        }

        switch (lbm_current_event.event_type)
        {
            case SMTC_MODEM_EVENT_RESET:
                LR11XX_SID_PAL_LOCATION_LOG_DEBUG("LBM: Event received: RESET");

                do
                {
#if SID_SDK_CONFIG_ENABLE_GNSS
                    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

                    if (NULL == drv_ctx->config)
                    {
                        SID_PAL_LOG_ERROR("Failed to initialize LBM, LR11xx configuration is missing");
                        lbm_err = SMTC_MODEM_RC_NOT_INIT;
                        break;
                    }

                    /* Configure almanac demodulation service */
                    lbm_err = smtc_modem_almanac_demodulation_set_constellations(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, drv_ctx->config->location.gnss.active_constellations);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to configure constellations for almanac demodulation. Error %u", (uint32_t)lbm_err);
                        break;
                    }

                    /* Configure GNSS scan */
                    lbm_err = smtc_modem_gnss_send_mode(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, SMTC_MODEM_SEND_MODE_BYPASS);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to configure GNSS scan result send mode. Error %u", (uint32_t)lbm_err);
                        break;
                    }

                    lbm_err = smtc_modem_gnss_set_constellations(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, drv_ctx->config->location.gnss.active_constellations);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to configure constellations for GNSS scan. Error %u", (uint32_t)lbm_err);
                        break;
                    }
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

#if SID_SDK_CONFIG_ENABLE_WIFI
                    /* Configure Wi-Fi scan */
                    lbm_err = smtc_modem_wifi_send_mode(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, SMTC_MODEM_SEND_MODE_BYPASS);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to configure WiFi scan result send mode. Error %u", (uint32_t)lbm_err);
                        break;
                    }
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

                    /* Put the radio into sleep mode - can't use sid_pal_radio_sleep() here due to LBM bridge being active */
                    lbm_err = smtc_modem_ext_sleep(0u);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to put radio to sleep via LBM. Error %u", (uint32_t)lbm_err);
                        break;
                    }

                    /* Done */
                    lbm_err = SMTC_MODEM_RC_OK;
                } while (0);

                /* Store LBM initialization error */
                drv_location_ctx.lbm_last_error = lbm_err;
                break;

#if SID_SDK_CONFIG_ENABLE_GNSS
            case SMTC_MODEM_EVENT_GNSS_SCAN_DONE:
                /* Nothing to do here */
                LR11XX_SID_PAL_LOCATION_LOG_DEBUG("LBM: Event received: GNSS_SCAN_DONE");
                break;

            case SMTC_MODEM_EVENT_GNSS_TERMINATED:
                LR11XX_SID_PAL_LOCATION_LOG_DEBUG("LBM: Event received: GNSS_TERMINATED");
                lbm_err = smtc_modem_gnss_get_event_data_scan_done(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, &drv_location_ctx.gnss_scan_done_data);
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    SID_PAL_LOG_ERROR("LBM: Failed to get GNSS scan results from LBM. Error %u", (uint32_t)lbm_err);
                    /* Don't terminate here, we may need to relaunch GNSS scan */
                }
                else
                {
                    SID_PAL_LOG_INFO("LBM: GNSS scan finished, number of valid NAV messages: %u ", drv_location_ctx.gnss_scan_done_data.nb_scans_valid);
                    if (drv_location_ctx.gnss_scan_done_data.indoor_detected != false)
                    {
                        SID_PAL_LOG_WARNING("LBM: the device is possibly located indoors, GNSS signals are obstructed");
                    }
                }

                if ((lbm_err != SMTC_MODEM_RC_OK) || (false == drv_location_ctx.gnss_scan_done_data.is_valid))
                {
                    /* Unsatisfactory outcomes, let's relaunch the scan */
                    if (drv_location_ctx.gnss_scan_retry_counter < drv_ctx->config->location.gnss.retry_limit)
                    {
                        drv_location_ctx.gnss_scan_retry_counter++;

                        lbm_err = smtc_modem_gnss_scan(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, drv_ctx->config->location.gnss.scan_mode, drv_ctx->config->location.gnss.retry_interval_s);
                        if (lbm_err != SMTC_MODEM_RC_OK)
                        {
                            SID_PAL_LOG_ERROR("LBM: Failed to restart GNSS scan. Error %u", (uint32_t)lbm_err);
                            /* Let the flow proceed and notify Sidewalk stack as this situation is non-recoverable from here */
                        }
                        else
                        {
                            SID_PAL_LOG_INFO("LBM: Unsatisfactory GNSS scan results. Retry attempt #%u (out of %u) will start in %u seconds",
                                             drv_location_ctx.gnss_scan_retry_counter,
                                             drv_ctx->config->location.gnss.retry_limit,
                                             drv_ctx->config->location.gnss.retry_interval_s);

                            /* Jump out here and don't notify Sidewalk stack, we are waiting for the new GNSS scan results */
                            break;
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_ERROR("LBM: GNSS scan finished unsuccessfully, retry limit reached");
                        lbm_err = SMTC_MODEM_RC_FAIL;
                    }
                }

                /* Reset the retry counter */
                drv_location_ctx.gnss_scan_retry_counter = 0u;
                /* Invalidate GNSS scan data if scan was not completed successfully */
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    drv_location_ctx.gnss_scan_done_data.is_valid = false;
                    drv_location_ctx.gnss_scan_done_data.nb_scans_valid = 0u;
                }
                /* Indicate GNSS scan is over */
                drv_location_ctx.gnss_scan_scheduled = FALSE;
                /* Notify Sidewalk stack about GNSS results availability */
                if (drv_location_ctx.gnss_sid_config.on_gnss_event != NULL)
                {
                    drv_location_ctx.gnss_sid_config.on_gnss_event(drv_location_ctx.gnss_sid_config.ctx, SID_PAL_GNSS_SCAN_COMPLETE, 0u);
                }
                break;

            case SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE:
                LR11XX_SID_PAL_LOCATION_LOG_DEBUG("LBM: Event received: GNSS_ALMANAC_DEMOD_UPDATE");
                lbm_err = smtc_modem_almanac_demodulation_get_event_data_almanac_update(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, &almanac_update_data);
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    SID_PAL_LOG_ERROR("LBM: Failed to get almanac demodulation results from LBM. Error %u", (uint32_t)lbm_err);
                    break;
                }

                if (almanac_update_data.status_gps != SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN)
                {
                    SID_PAL_LOG_INFO("LBM: GPS almanac update progress: %u%%", almanac_update_data.update_progress_gps);
                }
                if (almanac_update_data.status_beidou != SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN)
                {
                    SID_PAL_LOG_INFO("LBM: BeiDou almanac update progress: %u%%", almanac_update_data.update_progress_beidou);
                }
                break;
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

#if SID_SDK_CONFIG_ENABLE_WIFI
            case SMTC_MODEM_EVENT_WIFI_SCAN_DONE:
                /* Nothing to do here */
                LR11XX_SID_PAL_LOCATION_LOG_DEBUG("LBM: Event received: WIFI_SCAN_DONE");
                break;

            case SMTC_MODEM_EVENT_WIFI_TERMINATED:
                LR11XX_SID_PAL_LOCATION_LOG_DEBUG("LBM: Event received: WIFI_TERMINATED");
                lbm_err = smtc_modem_wifi_get_event_data_scan_done(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, &drv_location_ctx.wifi_scan_done_data);
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    SID_PAL_LOG_ERROR("LBM: Failed to get WiFi scan results from LBM. Error %u", (uint32_t)lbm_err);
                    /* Don't terminate here, we may need to relaunch WiFi scan */
                }
                else
                {
#  if SID_SDK_CONFIG_ENABLE_GNSS
                    if ((0u == drv_location_ctx.wifi_scan_done_data.nbr_results) && (0u == drv_location_ctx.wifi_scan_done_data.scan_duration_ms))
                    {
                        /* Most probably WiFi task was preempted by GNSS or almanac tasks. Don't increment retry counter, just relaunch the WiFi scan ASAP */
                        lbm_err = smtc_modem_wifi_scan(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, 0u);
                        if (lbm_err != SMTC_MODEM_RC_OK)
                        {
                            SID_PAL_LOG_ERROR("LBM: Failed to restart WiFi scan. Error %u", (uint32_t)lbm_err);
                            /* Let the flow proceed and notify SIdewalk stack as this situation is non-recoverable from here */
                        }
                        else
                        {
                            /* Wait for the new scan iteration to complete */
                            SID_PAL_LOG_INFO("LBM: WiFi scan rescheduled due to preemption");
                            break;
                        }
                    }
#  endif /* SID_SDK_CONFIG_ENABLE_GNSS */
                    SID_PAL_LOG_INFO("LBM: WiFi scan finished, discovered %u BSSIDs", drv_location_ctx.wifi_scan_done_data.nbr_results);
                }

                if ((lbm_err != SMTC_MODEM_RC_OK) || (drv_location_ctx.wifi_scan_done_data.nbr_results < drv_ctx->config->location.wifi.min_results))
                {
                    /* Unsatisfactory outcomes, let's relaunch the scan */
                    if (drv_location_ctx.wifi_scan_retry_counter < drv_ctx->config->location.wifi.retry_limit)
                    {
                        drv_location_ctx.wifi_scan_retry_counter++;

                        lbm_err = smtc_modem_wifi_scan(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, drv_ctx->config->location.wifi.retry_interval_s);
                        if (lbm_err != SMTC_MODEM_RC_OK)
                        {
                            SID_PAL_LOG_ERROR("LBM: Failed to restart WiFi scan. Error %u", (uint32_t)lbm_err);
                            /* Let the flow proceed and notify SIdewalk stack as this situation is non-recoverable from here */
                        }
                        else
                        {
                            SID_PAL_LOG_INFO("LBM: Unsatisfactory WiFi scan results. Retry attempt #%u (out of %u) will start in %u seconds",
                                             drv_location_ctx.wifi_scan_retry_counter,
                                             drv_ctx->config->location.wifi.retry_limit,
                                             drv_ctx->config->location.wifi.retry_interval_s);
                            /* Jump out here and don't notify Sidewalk stack, we are waiting for the new WiFi scan results */
                            break;
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_ERROR("LBM: WiFi scan finished unsuccessfully, retry limit reached");
                        lbm_err = SMTC_MODEM_RC_FAIL;
                    }
                }

                /* Reset the retry counter */
                drv_location_ctx.wifi_scan_retry_counter = 0u;
                /* Invalidate WiFi scan data if scan was not completed successfully */
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    drv_location_ctx.wifi_scan_done_data.nbr_results = 0u;
                }
                /* Indicate WiFi scan is over */
                drv_location_ctx.wifi_scan_scheduled = FALSE;
                /* Notify Sidewalk stack about WiFi results availability */
                if (drv_location_ctx.wifi_sid_config.on_wifi_event != NULL)
                {
                    drv_location_ctx.wifi_sid_config.on_wifi_event(drv_location_ctx.wifi_sid_config.ctx, SID_PAL_WIFI_SCAN_COMPLETE, 0u);
                }
                break;
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

            default:
                SID_PAL_LOG_ERROR("LBM: Unknown LBM event type (%u)", lbm_current_event.event_type);
                break;
        }
    } while (lbm_pending_event_count != 0u);
}

/*----------------------------------------------------------------------------*/

static sid_error_t sid_pal_location_common_init(void)
{
    sid_error_t err;
    int32_t     radio_err;

    do
    {
        if ((FALSE) /* Using dummy condition to enable compile-time selection of GNSS and WiFi support */
#if SID_SDK_CONFIG_ENABLE_GNSS
            || (drv_location_ctx.gnss_init_done != FALSE)
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */
#if SID_SDK_CONFIG_ENABLE_WIFI
            || (drv_location_ctx.wifi_init_done != FALSE)
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */
        )
        {
            /* LBM is initialized already, skip the initialization */
            err = SID_ERROR_NONE;
            break;
        }

        /* Check if the radio is initialized */
        const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
        if (FALSE == drv_ctx->init_done)
        {
            /* Radio driver shall be initialized properly before initializing the location functionality */
            SID_PAL_LOG_ERROR("Can't initialize LR11xx location. Radio driver is not initialized yet");
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Activate LoRa Basics Modem bridge in exclusive mode. This is required since LBM will interact with LR11xx during initialization and Sidewalk shall not interfere this */
        radio_err = sid_pal_radio_lr11xx_start_lbm_bridge_mode(RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE);
        if (radio_err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to enable LBM bridge exclusive mode in Sidewalk radio driver. Error %d", radio_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Initialize RALF layer to link it with Sidewalk radio driver */
        const ralf_t * const board_ralf = smtc_modem_ext_ral_bsp_init();
        if (NULL == board_ralf)
        {
            SID_PAL_LOG_ERROR("LBM BSP initialization failed");
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        /* Initialize LBM library */
        smtc_modem_init(&smtc_lbm_event_callback, false /* Do not reset the LR11xx - required to keep Sidewalk modem config alive */);

        /**
         * smtc_modem_init() will eventually call smtc_lbm_event_callback() with SMTC_MODEM_EVENT_RESET event report. As of current state of
         * LBM library, the smtc_lbm_event_callback() is invoked in the context of this function. Should LBM be ever modified to invoke
         * smtc_lbm_event_callback() asynchronously at initialization, this driver shall be modified to properly wait for initialization (e.g.,
         * lock on a Semaphore that is released during SMTC_MODEM_EVENT_RESET event handling in smtc_lbm_event_callback)
         */

        /* Check if LBM was initialized successfully */
        if (drv_location_ctx.lbm_last_error != SMTC_MODEM_RC_OK)
        {
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Indicate LBM is active and may schedule radio event */
        drv_location_ctx.lbm_radio_comm_suspended = false;

        /* Switch off the LBM bridge to allow sub-GHz Sidewalk links to run */
        radio_err = sid_pal_radio_lr11xx_stop_lbm_bridge_mode();
        if (radio_err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to stop LBM bridge mode in Sidewalk radio driver. Error %d", radio_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        SID_PAL_LOG_INFO("LR11xx LBM initialized");
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t sid_pal_location_common_deinit(void)
{
    sid_error_t              err;
    int32_t                  radio_err;
    smtc_modem_return_code_t lbm_err;

    do
    {
        if ((TRUE) /* Using dummy condition to enable compile-time selection of GNSS and WiFi support */
#if SID_SDK_CONFIG_ENABLE_GNSS
            && (FALSE == drv_location_ctx.gnss_init_done)
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */
#if SID_SDK_CONFIG_ENABLE_WIFI
            && (FALSE == drv_location_ctx.wifi_init_done)
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */
        )
        {
            /* LBM is de-initialized already, skip the de-initialization */
            err = SID_ERROR_NONE;
            break;
        }

        /* Ensure the radio is not de-initialized already */
        const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
        if (drv_ctx->init_done != FALSE)
        {
            do
            {
                /**
                 * Note: You need to call smtc_modem_suspend_radio_communications(true) if LBM runs some persistent tasks that never terminate
                 *       (e.g. almanac demodulation). If your application does not initiate any such tasks in LBM and all the other LBM tasks (e.g.
                 *       WiFi scan, GNSS scan, etc.) are finished by this point you may safely skip smtc_modem_suspend_radio_communications(true)
                 *       call here as well as smtc_modem_suspend_radio_communications(false) to resume LBM scheduler at a later point
                 */
                lbm_err = smtc_modem_suspend_radio_communications(true);
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    /* LBM operations may not be suspended immediately (e.g. if the radio runs a scan), give it some cooldown time and retry */
                    sid_pal_scheduler_delay_ms(250u);
                }
            } while (lbm_err != SMTC_MODEM_RC_OK);

            /* Indicate LBM is suspended and won't trigger any radio communication */
            drv_location_ctx.lbm_radio_comm_suspended = true;

            /* Switch off LBM bridge if required */
            const radio_lr11xx_lbm_bridge_state_t lmb_bridge_state = sid_pal_radio_lr11xx_get_lbm_bridge_mode();
            if (lmb_bridge_state != RADIO_LR11XX_LBM_BRIDGE_STATE_INACTIVE)
            {
                radio_err = sid_pal_radio_lr11xx_stop_lbm_bridge_mode();
                if (radio_err != RADIO_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Failed to stop LBM bridge mode in Sidewalk radio driver. Error %d", radio_err);
                    err = SID_ERROR_IO_ERROR;
                    break;
                }
            }
        }
        else
        {
            /* Radio driver is not functional any longer, so we can't (and actually don't need to) operate LBM bridge any longer as it is deactivated as well during radio de-init */
            drv_location_ctx.lbm_radio_comm_suspended = true;
        }

        /* Done */
        SID_PAL_LOG_INFO("LR11xx LBM de-initialized");
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/* Global function definitions -----------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_WIFI
sid_error_t sid_pal_wifi_init(struct sid_pal_wifi_config * config)
{
    sid_error_t err;

    do
    {
        if (drv_location_ctx.wifi_init_done != FALSE)
        {
            SID_PAL_LOG_WARNING("LR11xx WiFi location features are initialized already");
            err = SID_ERROR_NONE;
            break;
        }

        err = sid_pal_location_common_init();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Store the location stack config provided by Sidewalk stack (do not confuse with LR11xx config of location features) */
        drv_location_ctx.wifi_sid_config = *config; /* Can't store pointer because config is allocated on the caller's stack */

        /* Done */
        SID_PAL_LOG_INFO("LR11xx WiFi location features initialized");
        drv_location_ctx.wifi_init_done = TRUE;
    } while (0);

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_WIFI
sid_error_t sid_pal_wifi_deinit()
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_location_ctx.wifi_init_done)
        {
            SID_PAL_LOG_WARNING("LR11xx WiFi location features are not initialized, de-init skipped");
            err = SID_ERROR_NONE;
            break;
        }

        /* Finished with the WiFi-specific de-initialization, but we may still need to bring down LBM in general */
        SID_PAL_LOG_INFO("LR11xx WiFi location features de-initialized");

#if SID_SDK_CONFIG_ENABLE_GNSS
        /* Do not de-initialize the LBM if GNSS functionality is still in use */
        if (FALSE == drv_location_ctx.gnss_init_done)
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */
        {
            err = sid_pal_location_common_deinit();
        }
#if SID_SDK_CONFIG_ENABLE_GNSS
        else
        {
            err = SID_ERROR_NONE;
        }
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

        /* Clean up WiFi location context */
        SID_STM32_UTIL_fast_memset(&drv_location_ctx.wifi_sid_config, NULL, sizeof(drv_location_ctx.wifi_sid_config));

        /* Clear init flag here to allow sid_pal_location_common_deinit() to complete properly */
        drv_location_ctx.wifi_init_done = FALSE;

        /* Done. err is assigned by sid_pal_location_common_deinit() call */
    } while (0);

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_WIFI
sid_error_t sid_pal_wifi_process_event(uint8_t event_id)
{
    sid_error_t err;

    err = smtc_lbm_process_events(SID_LOCATION_METHOD_WIFI);

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_WIFI
sid_error_t sid_pal_wifi_schedule_scan(uint32_t scan_delay_s)
{
    sid_error_t              err;
    smtc_modem_return_code_t lbm_err;

    do
    {
        if (FALSE == drv_location_ctx.wifi_init_done)
        {
            SID_PAL_LOG_ERROR("LR11xx WiFi is not initialized");
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Ensure GNSS scan is not scheduled already */
        if (drv_location_ctx.wifi_scan_scheduled != FALSE)
        {
            SID_PAL_LOG_ERROR("LR11xx WiFi scan is scheduled already");
            err = SID_ERROR_IN_PROGRESS;
            break;
        }

#if !LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT || !LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING
        /* If Sidewalk-LBM concurrency is not enabled, the Sidewalk LBM Bridge has to be switched on in the LBM exclusive mode */
        const radio_lr11xx_lbm_bridge_state_t lmb_bridge_state = sid_pal_radio_lr11xx_get_lbm_bridge_mode();
        if (lmb_bridge_state != RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE)
        {
            SID_PAL_LOG_ERROR("Failed to start LR11xx WiFi scan. LBM bridge is not activated in exclusive mode");
            err = SID_ERROR_INVALID_STATE;
            break;
        }
#endif /* !LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT || !LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING */

        /* Initiate WiFi scanning */
        /**
         * Note: While the capabilities of LBM's Radio Planner allow to manage concurrent radio tasks (GNSS scan, almanac demodulation, WiFi scan),
         *       on practice it may be more energy-efficient to run such tasks one by one to avoid additional wait times caused by
         *       radio task rescheduling caused by LBM's radio planning managing the priorities
         */
        lbm_err = smtc_modem_wifi_scan(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, scan_delay_s);
        if (lbm_err != SMTC_MODEM_RC_OK)
        {
            SID_PAL_LOG_ERROR("Failed to start LR11xx WiFi scan. Error %u", (uint32_t)lbm_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Indicate GNSS scan is now running */
        drv_location_ctx.wifi_scan_scheduled = TRUE;

        /* Since LBM may not had any active tasks we need to force an engine run to process the above call and schedule the scan */
        smtc_lbm_schedule_modem_engine_run(0u, SID_LOCATION_METHOD_WIFI);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_WIFI
sid_error_t sid_pal_wifi_cancel_scan()
{
    sid_error_t                          err;
    smtc_modem_return_code_t             lbm_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    sid_pal_enter_critical_region();

    do
    {
        /* Ensure the driver is initialized properly */
        if ((FALSE == drv_ctx->init_done) || (NULL == drv_ctx->config) || (FALSE == drv_location_ctx.wifi_init_done))
        {
            SID_PAL_LOG_ERROR("LR11xx WiFi is not initialized");
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Ensure WiFi scan is actually scheduled */
        if (FALSE == drv_location_ctx.wifi_scan_scheduled)
        {
            SID_PAL_LOG_ERROR("LR11xx WiFi scan is not scheduled");
            err = SID_ERROR_NONE;
            break;
        }

        lbm_err = smtc_modem_wifi_scan_cancel(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID);
        if (lbm_err != SMTC_MODEM_RC_OK)
        {
            if (SMTC_MODEM_RC_BUSY == lbm_err)
            {
                /* The scan sequency is actively running in LR11xx and cannot be canceled now */
                err = SID_ERROR_BUSY;
            }
            else
            {
                /* Something really unexpected happened */
                SID_PAL_LOG_ERROR("Failed to cancel LR11xx WiFi scan. LBM error %u", lbm_err);
                err = SID_ERROR_GENERIC;
            }
            break;
        }

        /* Done */
        drv_location_ctx.wifi_scan_scheduled = FALSE;
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_WIFI
sid_error_t sid_pal_wifi_get_scan_payload(struct sid_pal_wifi_payload * wifi_scan_result)
{
    sid_error_t                          err;
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();


    /* Run in a critical section to avoid sudden changes to drv_location_ctx.wifi_scan_done_data */
    sid_pal_enter_critical_region();

    do
    {
        /* Ensure the driver is initialized properly */
        if ((FALSE == drv_ctx->init_done) || (NULL == drv_ctx->config) || (FALSE == drv_location_ctx.wifi_init_done))
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate inputs */
        if (NULL == wifi_scan_result)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Ensure we have enough BSSIDs discovered */
        if (drv_location_ctx.wifi_scan_done_data.nbr_results < drv_ctx->config->location.wifi.min_results)
        {
            err = SID_ERROR_INSUFFICIENT_RESULTS;
            break;
        }

        /* Reset the buffer */
        SID_STM32_UTIL_fast_memset(wifi_scan_result, 0u, sizeof(*wifi_scan_result));

        /* Static checks to ensure Sidewalk and LBM definitions are compatible */
        static_assert(SID_STM32_UTIL_ARRAY_SIZE(wifi_scan_result->results[0].mac) == SID_STM32_UTIL_ARRAY_SIZE(drv_location_ctx.wifi_scan_done_data.results[0].mac_address));

        /* Copy scan results */
        const uint32_t results_to_copy = MIN(drv_location_ctx.wifi_scan_done_data.nbr_results, MIN(SID_WIFI_MAX_RESULTS, SID_STM32_UTIL_ARRAY_SIZE(drv_location_ctx.wifi_scan_done_data.results)));
        // TODO: If the number of available scan results exceeds SID_WIFI_MAX_RESULTS, consider sorting the results by RSSI and copying the ones with the highest RSSI
        for (uint32_t i = 0u; i < results_to_copy; i++)
        {
            wifi_scan_result->results[i].rssi = drv_location_ctx.wifi_scan_done_data.results[i].rssi;
            SID_STM32_UTIL_fast_memcpy(wifi_scan_result->results[i].mac, drv_location_ctx.wifi_scan_done_data.results[i].mac_address, SID_STM32_UTIL_ARRAY_SIZE(wifi_scan_result->results[i].mac));
        }

        wifi_scan_result->nbr_results = results_to_copy;
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_GNSS
sid_error_t sid_pal_gnss_init(struct sid_pal_gnss_config * config)
{
    sid_error_t err;

    do
    {
        if (drv_location_ctx.gnss_init_done != FALSE)
        {
            SID_PAL_LOG_WARNING("LR11xx GNSS location features are initialized already");
            err = SID_ERROR_NONE;
            break;
        }

        err = sid_pal_location_common_init();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Store the location stack config provided by Sidewalk stack (do not confuse with LR11xx config of location features) */
        drv_location_ctx.gnss_sid_config = *config; /* Can't store pointer because config is allocated on the caller's stack */

        /* Done */
        SID_PAL_LOG_INFO("LR11xx GNSS location features initialized");
        drv_location_ctx.gnss_init_done = TRUE;
    } while (0);

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_GNSS
sid_error_t sid_pal_gnss_deinit()
{
    sid_error_t err;

    do
    {
        if (FALSE == drv_location_ctx.gnss_init_done)
        {
            SID_PAL_LOG_WARNING("LR11xx GNSS location features are not initialized, de-init skipped");
            err = SID_ERROR_NONE;
            break;
        }

        /* Clean up GNSS location context */
        SID_STM32_UTIL_fast_memset(&drv_location_ctx.gnss_sid_config, NULL, sizeof (drv_location_ctx.gnss_sid_config));

        /* Finished with the GNSS-specific de-initialization, but we may still need to bring down LBM in general */
        SID_PAL_LOG_INFO("LR11xx GNSS location features de-initialized");

#if SID_SDK_CONFIG_ENABLE_WIFI
        /* Do not de-initialize the LBM if WiFi functionality is still in use */
        if (FALSE == drv_location_ctx.wifi_init_done)
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */
        {
            err = sid_pal_location_common_deinit();
        }
#if SID_SDK_CONFIG_ENABLE_WIFI
        else
        {
            err = SID_ERROR_NONE;
        }
#endif /* SID_SDK_CONFIG_ENABLE_WIFI */
        /* Clear init flag here to allow sid_pal_location_common_deinit() to complete properly */
        drv_location_ctx.gnss_init_done = FALSE;
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_GNSS
sid_error_t sid_pal_gnss_process_event(uint8_t event_id)
{
    sid_error_t err;

    err = smtc_lbm_process_events(SID_LOCATION_METHOD_GNSS);

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_GNSS
sid_error_t sid_pal_gnss_schedule_scan(uint32_t scan_delay_s)
{
    sid_error_t                          err;
    smtc_modem_return_code_t             lbm_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    do
    {
        /* Ensure the driver is initialized properly */
        if ((FALSE == drv_ctx->init_done) || (NULL == drv_ctx->config) || (FALSE == drv_location_ctx.gnss_init_done))
        {
            SID_PAL_LOG_ERROR("LR11xx GNSS is not initialized");
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Ensure GNSS scan is not scheduled already */
        if (drv_location_ctx.gnss_scan_scheduled != FALSE)
        {
            SID_PAL_LOG_ERROR("LR11xx GNSS scan is scheduled already");
            err = SID_ERROR_IN_PROGRESS;
            break;
        }

#if !LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT || !LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING
        /* If Sidewalk-LBM concurrency is not enabled, the Sidewalk LBM Bridge has to be switched on in the LBM exclusive mode */
        const radio_lr11xx_lbm_bridge_state_t lmb_bridge_state = sid_pal_radio_lr11xx_get_lbm_bridge_mode();
        if (lmb_bridge_state != RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE)
        {
            SID_PAL_LOG_ERROR("Failed to start LR11xx GNSS scan. LBM bridge is not activated in exclusive mode");
            err = SID_ERROR_INVALID_STATE;
            break;
        }
#endif /* !LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT || !LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */

        /* Initiate GNSS scanning */
        /**
         * Note: While the capabilities of LBM's Radio Planner allow to manage concurrent radio tasks (GNSS scan, almanac demodulation, WiFi scan),
         *       on practice it may be more energy-efficient to run such tasks one by one to avoid additional wait times caused by
         *       radio task rescheduling caused by LBM's radio planning managing the priorities
         */
        lbm_err = smtc_modem_gnss_scan(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID, drv_ctx->config->location.gnss.scan_mode, scan_delay_s);
        if (lbm_err != SMTC_MODEM_RC_OK)
        {
            SID_PAL_LOG_ERROR("Failed to start LR11xx GNSS scan. Error %u", (uint32_t)lbm_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Indicate GNSS scan is now running */
        drv_location_ctx.gnss_scan_scheduled = TRUE;

        /* Since LBM may not had any active tasks we need to force an engine run to process the above call and schedule the scan */
        smtc_lbm_schedule_modem_engine_run(0u, SID_LOCATION_METHOD_GNSS);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_GNSS
sid_error_t sid_pal_gnss_cancel_scan()
{
    sid_error_t                          err;
    smtc_modem_return_code_t             lbm_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    sid_pal_enter_critical_region();

    do
    {
        /* Ensure the driver is initialized properly */
        if ((FALSE == drv_ctx->init_done) || (NULL == drv_ctx->config) || (FALSE == drv_location_ctx.gnss_init_done))
        {
            SID_PAL_LOG_ERROR("LR11xx GNSS is not initialized");
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Ensure GNSS scan is actually scheduled */
        if (FALSE == drv_location_ctx.gnss_scan_scheduled)
        {
            SID_PAL_LOG_ERROR("LR11xx GNSS scan is not scheduled");
            err = SID_ERROR_NONE;
            break;
        }

        lbm_err = smtc_modem_gnss_scan_cancel(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID);
        if (lbm_err != SMTC_MODEM_RC_OK)
        {
            if (SMTC_MODEM_RC_BUSY == lbm_err)
            {
                /* The scan sequency is actively running in LR11xx and cannot be canceled now */
                err = SID_ERROR_BUSY;
            }
            else
            {
                /* Something really unexpected happened */
                SID_PAL_LOG_ERROR("Failed to cancel LR11xx GNSS scan. LBM error %u", lbm_err);
                err = SID_ERROR_GENERIC;
            }
            break;
        }

        /* Done */
        drv_location_ctx.gnss_scan_scheduled = FALSE;
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_GNSS
sid_error_t sid_pal_gnss_get_scan_payload(struct sid_pal_gnss_payload * gnss_scan_group)
{
    sid_error_t                          err;
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();


    /* Run in a critical section to avoid sudden changes to drv_location_ctx.wifi_scan_done_data */
    sid_pal_enter_critical_region();

    do
    {
        /* Ensure the driver is initialized properly */
        if ((FALSE == drv_ctx->init_done) || (NULL == drv_ctx->config) || (FALSE == drv_location_ctx.gnss_init_done))
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Validate inputs */
        if (NULL == gnss_scan_group)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Reset the buffer */
        SID_STM32_UTIL_fast_memset(gnss_scan_group, 0u, sizeof(*gnss_scan_group));

        /* Ensure GNSS scan data is valid and contains enough NAV3 messages */
        if (false == drv_location_ctx.gnss_scan_done_data.is_valid)
        {
            err = SID_ERROR_INSUFFICIENT_RESULTS;
            break;
        }

        /* Fill in GNSS scan results payload */
        err = SID_ERROR_NONE;
        for (uint32_t i = 0u; i < drv_location_ctx.gnss_scan_done_data.nb_scans_valid; i++)
        {
            /* Compute utility values */
            const uint32_t scan_src_idx = (uint32_t)drv_location_ctx.gnss_scan_done_data.nb_scans_valid - (i + 1u);
            const uint32_t scan_entry_size = sizeof(lr11xx_location_gnss_scan_header_t) + drv_location_ctx.gnss_scan_done_data.scans[scan_src_idx].nav_size;
            const uint32_t expected_full_size = (uint32_t)gnss_scan_group->size + scan_entry_size;

            LR11XX_SID_PAL_LOCATION_LOG_DEBUG("Processing GNSS scan record %u of %u, nb_svs: %u, nav_size: %u",
                (i + 1u),
                drv_location_ctx.gnss_scan_done_data.nb_scans_valid,
                drv_location_ctx.gnss_scan_done_data.scans[scan_src_idx].nb_svs,
                drv_location_ctx.gnss_scan_done_data.scans[scan_src_idx].nav_size);

            /* Protect from working buffer overflow. This mainly targets use cases when  a single GNSS scan record is larger than Sidewalk's internal buffer */
            if (expected_full_size > GNSS_MAX_PAYLOAD_SIZE)
            {
                SID_PAL_LOG_ERROR("LR11xx GNSS payload is too large and cannot be processed (%uB vs %uB limit)", expected_full_size, GNSS_MAX_PAYLOAD_SIZE);
                SID_STM32_UTIL_fast_memset(gnss_scan_group, 0u, sizeof(*gnss_scan_group));
                err = SID_ERROR_BUFFER_OVERFLOW;
                break;
            }

            /* Check if this is the last record we are going to put into the uplink payload */
            bool is_last_record;
            if (0u == scan_src_idx)
            {
                /* That's truly the last one */
                is_last_record = true;
            }
            else
            {
                /* This is not the last scan record available, but let's check if the next record will fit into the payload buffer */
                const uint32_t expected_full_size_with_next_record = expected_full_size + sizeof(lr11xx_location_gnss_scan_header_t) + drv_location_ctx.gnss_scan_done_data.scans[scan_src_idx - 1u].nav_size;
                is_last_record = expected_full_size_with_next_record > GNSS_MAX_PAYLOAD_SIZE;
                if (is_last_record != false)
                {
                    SID_PAL_LOG_WARNING("Some LR11xx GNSS scan records do not fit into the buffer and will be skipped");
                }
            }

            /* Populate scan result TLV header */
            lr11xx_location_gnss_scan_header_t * const record_header = (lr11xx_location_gnss_scan_header_t *)(void *)&gnss_scan_group->payload_data[gnss_scan_group->size];
            record_header->tlv_header.tag    = LR11XX_SID_PAL_LOCATION_GNSS_NG_TLV_TAG;
            record_header->tlv_header.length = scan_entry_size - sizeof(record_header->tlv_header); /* Length in TLV header shall not account for Tag and Length fields as these are always present */
            record_header->token             = (false == is_last_record) ? drv_location_ctx.gnss_scan_done_data.token : (drv_location_ctx.gnss_scan_done_data.token | LR11XX_SID_PAL_LOCATION_GNSS_NG_LAST_ENTRY_FLAG);
            gnss_scan_group->size           += sizeof(*record_header);

            /* Copy NAV3 payload */
            const uint32_t nav3_payload_size = drv_location_ctx.gnss_scan_done_data.scans[scan_src_idx].nav_size;
            SID_STM32_UTIL_fast_memcpy(&gnss_scan_group->payload_data[gnss_scan_group->size], drv_location_ctx.gnss_scan_done_data.scans[scan_src_idx].nav, nav3_payload_size);
            SID_PAL_ASSERT(((uint32_t)gnss_scan_group->size + nav3_payload_size) <= GNSS_MAX_PAYLOAD_SIZE);
            gnss_scan_group->size += (uint8_t)nav3_payload_size;
        }
        if (err != SID_ERROR_NONE)
        {
            /* Propagate error from the for() loop */
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_GNSS
sid_error_t sid_pal_gnss_alm_demod_start()
{
    sid_error_t                          err;
    smtc_modem_return_code_t             lbm_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    do
    {
        /* Ensure the driver is initialized properly */
        if ((FALSE == drv_ctx->init_done) || (NULL == drv_ctx->config) || (FALSE == drv_location_ctx.gnss_init_done))
        {
            SID_PAL_LOG_ERROR("LR11xx GNSS is not initialized");
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Ensure almanac decoding is not running already */
        if (drv_location_ctx.almanac_demod_started != FALSE)
        {
            SID_PAL_LOG_ERROR("LR11xx almanac demodulation is running already");
            err = SID_ERROR_IN_PROGRESS;
            break;
        }

        /* Initiate almanac demodulation */
        lbm_err = smtc_modem_almanac_demodulation_start(LR11XX_RADIO_CFG_SID_BRIDGE_LBM_STACK_ID);
        if (lbm_err != SMTC_MODEM_RC_OK)
        {
            SID_PAL_LOG_ERROR("Failed to start LR11xx GNSS almanac demodulation. Error %u", (uint32_t)lbm_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Indicate demodulation is now running */
        drv_location_ctx.almanac_demod_started = TRUE;

        /* Since LBM may not had any active tasks we need to force an engine run to process the above call and schedule the scan */
        smtc_lbm_schedule_modem_engine_run(0u, SID_LOCATION_METHOD_GNSS);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* SID_SDK_CONFIG_ENABLE_GNSS */

/*----------------------------------------------------------------------------*/

void smtc_modem_hal_user_lbm_irq(void)
{
    sid_error_t err;

    /* Schedule LBM event processing ASAP */
    err = smtc_lbm_schedule_modem_engine_run(0u, (SID_LOCATION_METHOD_WIFI | SID_LOCATION_METHOD_GNSS));

    /* Report an error if any abnormal behavior is detected */
    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("LBM: error when handling IRQ indication. Error %d", (int32_t)err);
    }
}
