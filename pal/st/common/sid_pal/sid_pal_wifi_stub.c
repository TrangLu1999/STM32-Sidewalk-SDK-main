/**
  ******************************************************************************
  * @file    sid_pal_wifi_stub.c
  * @brief   Stubs of sid_pal_wifi layer to support builds with no WiFi hardware
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

#include <sid_pal_wifi_ifc.h>
#include <cmsis_compiler.h>

/* Global function definitions -----------------------------------------------*/

__WEAK sid_error_t sid_pal_wifi_init(struct sid_pal_wifi_config * config)
{
    (void)config;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_wifi_process_event(uint8_t event_id)
{
    (void)event_id;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_wifi_schedule_scan(uint32_t scan_delay_s)
{
    (void)scan_delay_s;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_wifi_cancel_scan(void)
{
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_wifi_get_scan_payload(struct sid_pal_wifi_payload * wifi_scan_result)
{
    (void)wifi_scan_result;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_wifi_deinit()
{
    return SID_ERROR_NOSUPPORT;
}
