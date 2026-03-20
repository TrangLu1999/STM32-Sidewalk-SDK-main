/**
  ******************************************************************************
  * @file    sid_pal_gnss_stub.c
  * @brief   Stubs of sid_pal_gnss layer to support builds with no GNSS hardware
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

#include <sid_pal_gnss_ifc.h>
#include <cmsis_compiler.h>

/* Global function definitions -----------------------------------------------*/

__WEAK sid_error_t sid_pal_gnss_init(struct sid_pal_gnss_config * config)
{
    (void)config;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_gnss_process_event(uint8_t event_id)
{
    (void)event_id;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_gnss_schedule_scan(uint32_t scan_delay_s)
{
    (void)scan_delay_s;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_gnss_cancel_scan(void)
{
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_gnss_get_scan_payload(struct sid_pal_gnss_payload * gnss_scan_group)
{
    (void)gnss_scan_group;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_gnss_alm_demod_start(void)
{
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK sid_error_t sid_pal_gnss_deinit(void)
{
    return SID_ERROR_NOSUPPORT;
}
