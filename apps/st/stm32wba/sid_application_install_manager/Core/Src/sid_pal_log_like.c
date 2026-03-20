/**
  ******************************************************************************
  * @file    sid_pal_log_like.c
  * @brief   Lightweight logging interface for AIM compatible with sid_pal_log
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

#include "sid_pal_log_like.h"

#include <stm32_adv_trace.h>
#include <timer_if.h>

/* Private defines -----------------------------------------------------------*/

#ifndef SID_PAL_LOG_MSG_LENGTH_MAX
#  error "Maximum length of the log message is not defined. Please set SID_PAL_LOG_MSG_LENGTH_MAX compile definition"
#endif

/* Global function definitions -----------------------------------------------*/

void __sid_pal_log_like(uint8_t severity, const char * const fmt, ...)
{
    va_list args;
    uint32_t timestamp_ms, s;
    uint16_t ms;

    /* Get current timestamp from RTC */
    s = TIMER_IF_GetTime(&ms);
    timestamp_ms = (s * 1000u) + (uint32_t)ms;

    /* Print message timestamp and severity */
    switch (severity)
    {
        case SID_PAL_LOG_SEVERITY_ERROR:
            UTIL_ADV_TRACE_FSend("[%u]"LOG_ERROR_COLOR"["LOG_SEVERITY_NAME_ERROR"]"COLOR_RESET": ", timestamp_ms);
            break;

        case SID_PAL_LOG_SEVERITY_WARNING:
            UTIL_ADV_TRACE_FSend("[%u]"LOG_WARNING_COLOR"["LOG_SEVERITY_NAME_WARNING"]"COLOR_RESET": ", timestamp_ms);
            break;

        case SID_PAL_LOG_SEVERITY_INFO:
            UTIL_ADV_TRACE_FSend("[%u]"LOG_INFO_COLOR"["LOG_SEVERITY_NAME_INFO"]"COLOR_RESET": ", timestamp_ms);
            break;

        case SID_PAL_LOG_SEVERITY_DEBUG:
            UTIL_ADV_TRACE_FSend("[%u]"LOG_DEBUG_COLOR"["LOG_SEVERITY_NAME_DEBUG"]"COLOR_RESET": ", timestamp_ms);
            break;

        default:
            /* Ignore unknown/disabled message types */
            return;
    }

    /* Print the message itself */
    char msg_buf[SID_PAL_LOG_MSG_LENGTH_MAX];

    va_start(args, fmt);
    (void)UTIL_ADV_TRACE_VSNPRINTF(msg_buf, sizeof(msg_buf), fmt, args);
    UTIL_ADV_TRACE_FSend("%s", msg_buf); /* This is faster than just UTIL_ADV_TRACE_FSend(msg_buf) because the underlying vsnprintf() won't have to parse the entire msg_buf but rather it will just copy it into output buffer as is */
    va_end(args);

    /* Print line ending */
    UTIL_ADV_TRACE_FSend(SID_PAL_LOG_LINE_ENDING);
}
