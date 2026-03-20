/**
  ******************************************************************************
  * @file    gnss_lib_config.c
  * @brief   Configure how the libGNSS accesses the GNSS module
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

#include "gnss_lib_config.h"
#include "teseo_gnss.h"
#include "teseo_gnss_private.h"

/* Sidewalk interfaces */
#include <sid_pal_delay_ifc.h>
#include <sid_pal_log_ifc.h>

#define TESEO_GNSS_LOG_SID_PREFIX                                    "\e[1;36m" "[GNSS] " "\e[0m"

/* Global function definitions -----------------------------------------------*/

int32_t GNSS_Wrapper_Send(uint8_t * buffer, uint16_t length)
{
    sid_error_t err;

    (void)length;
    err = teseo_gnss_send_command((const char *)buffer);

    return (int32_t)err;
}

/*----------------------------------------------------------------------------*/

int32_t GNSS_Wrapper_Reset(void)
{
    int32_t status;

    status = (int32_t)teseo_gnss_reset(TESEO_RESET_SYS_HARDWARE);
    return status;
}

/*----------------------------------------------------------------------------*/

void GNSS_Wrapper_Delay(uint32_t Delay)
{
#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
    sid_pal_scheduler_delay_ms(Delay);
#else
    sid_pal_delay_us(Delay * 1000u);
#endif /* SID_PAL_ENABLE_SCHEDULER_DELAY */
}

/*----------------------------------------------------------------------------*/

int GNSS_PRINT(char * pBuffer)
{
    return teseo_gnss_print(pBuffer, SID_PAL_LOG_SEVERITY_INFO);
}

/*----------------------------------------------------------------------------*/

int teseo_gnss_print(const char * const pBuffer, const sid_pal_log_severity_t severity)
{
    switch (severity)
    {
        case SID_PAL_LOG_SEVERITY_ERROR:
            SID_PAL_LOG_ERROR(TESEO_GNSS_LOG_SID_PREFIX "%s", pBuffer);
            break;

        case SID_PAL_LOG_SEVERITY_WARNING:
            SID_PAL_LOG_WARNING(TESEO_GNSS_LOG_SID_PREFIX "%s", pBuffer);
            break;

        case SID_PAL_LOG_SEVERITY_INFO:
            SID_PAL_LOG_INFO(TESEO_GNSS_LOG_SID_PREFIX "%s", pBuffer);
            break;

        case SID_PAL_LOG_SEVERITY_DEBUG:
            SID_PAL_LOG_DEBUG(TESEO_GNSS_LOG_SID_PREFIX "%s", pBuffer);
            break;

        default:
            break;
    }

    return 0;
}

