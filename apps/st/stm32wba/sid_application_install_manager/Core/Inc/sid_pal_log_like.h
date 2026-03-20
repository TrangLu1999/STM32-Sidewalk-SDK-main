/**
  ******************************************************************************
  * @file    sid_pal_log_like.h
  * @brief   Logging definitions that provide an API similar to the sid_pal_log
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

 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SID_PAL_LOG_LIKE_H_
#define __SID_PAL_LOG_LIKE_H_

/* Includes ------------------------------------------------------------------*/

/* Platform definitions */
#include <stm32wbaxx.h>

/* STM32 Utilities */
#include "app_conf.h"
#include "stm32_adv_trace.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

#define REGULAR_COLOR_BLACK             "\e[0;30m"
#define REGULAR_COLOR_RED               "\e[0;31m"
#define REGULAR_COLOR_GREEN             "\e[0;32m"
#define REGULAR_COLOR_YELLOW            "\e[0;33m"
#define REGULAR_COLOR_BLUE              "\e[0;34m"
#define REGULAR_COLOR_MAGENTA           "\e[0;35m"
#define REGULAR_COLOR_CYAN              "\e[0;36m"
#define REGULAR_COLOR_WHITE             "\e[0;37m"

#define BOLD_COLOR_BLACK                "\e[1;30m"
#define BOLD_COLOR_RED                  "\e[1;31m"
#define BOLD_COLOR_GREEN                "\e[1;32m"
#define BOLD_COLOR_YELLOW               "\e[1;33m"
#define BOLD_COLOR_BLUE                 "\e[1;34m"
#define BOLD_COLOR_MAGENTA              "\e[1;35m"
#define BOLD_COLOR_CYAN                 "\e[1;36m"
#define BOLD_COLOR_WHITE                "\e[1;37m"

#define UNDERLINE_COLOR_BLACK           "\e[4;30m"
#define UNDERLINE_COLOR_RED             "\e[4;31m"
#define UNDERLINE_COLOR_GREEN           "\e[4;32m"
#define UNDERLINE_COLOR_YELLOW          "\e[4;33m"
#define UNDERLINE_COLOR_BLUE            "\e[4;34m"
#define UNDERLINE_COLOR_MAGENTA         "\e[4;35m"
#define UNDERLINE_COLOR_CYAN            "\e[4;36m"
#define UNDERLINE_COLOR_WHITE           "\e[4;37m"

#define COLOR_RESET                     "\e[0m"


#define LOG_ERROR_COLOR                 BOLD_COLOR_RED
#define LOG_WARNING_COLOR               BOLD_COLOR_YELLOW
#define LOG_INFO_COLOR                  BOLD_COLOR_GREEN
#define LOG_DEBUG_COLOR                 BOLD_COLOR_BLUE
#define LOG_UNDEFINED_COLOR             BOLD_COLOR_WHITE


#define LOG_SEVERITY_NAME_ERROR         "ERROR"
#define LOG_SEVERITY_NAME_WARNING       "WARNING"
#define LOG_SEVERITY_NAME_INFO          "INFO"
#define LOG_SEVERITY_NAME_DEBUG         "DEBUG"
#define LOG_SEVERITY_NAME_UNDEFINED     "UNDEFINED"


#define SID_PAL_LOG_SEVERITY_ERROR      (VLEVEL_ALWAYS)
#define SID_PAL_LOG_SEVERITY_WARNING    (VLEVEL_L)
#define SID_PAL_LOG_SEVERITY_INFO       (VLEVEL_M)
#define SID_PAL_LOG_SEVERITY_DEBUG      (VLEVEL_H)

#ifndef SID_PAL_LOG_LEVEL
#  ifdef DEBUG
#    define SID_PAL_LOG_LEVEL SID_PAL_LOG_SEVERITY_DEBUG
#  else
#    define SID_PAL_LOG_LEVEL SID_PAL_LOG_SEVERITY_INFO
#  endif
#endif

/* Exported macros -----------------------------------------------------------*/

#if CFG_LOG_SUPPORTED
void __sid_pal_log_like(uint8_t severity, const char * const fmt, ...);
#define __SID_PAL_LOG_LIKE_PRINT_FUNC(sev_, fmt_, ...)   __sid_pal_log_like(sev_, fmt_, ##__VA_ARGS__)
#else
#  define __SID_PAL_LOG_LIKE_PRINT_FUNC(sev_, fmt_, ...) ((void)0u)
#endif /* CFG_LOG_SUPPORTED */

#define SID_PAL_LOG_LINE_ENDING                          "\n"

#if SID_PAL_LOG_LEVEL >= SID_PAL_LOG_SEVERITY_ERROR
#  define SID_PAL_LOG_ERROR(msg, ...)                    __SID_PAL_LOG_LIKE_PRINT_FUNC(SID_PAL_LOG_SEVERITY_ERROR, msg, ##__VA_ARGS__)
#else
#  define SID_PAL_LOG_ERROR(...)                         ((void)0u)
#endif

#if SID_PAL_LOG_LEVEL >= SID_PAL_LOG_SEVERITY_WARNING
#  define SID_PAL_LOG_WARNING(msg, ...)                  __SID_PAL_LOG_LIKE_PRINT_FUNC(SID_PAL_LOG_SEVERITY_WARNING, msg, ##__VA_ARGS__)
#else
#  define SID_PAL_LOG_WARNING(...)                       ((void)0u)
#endif

#if SID_PAL_LOG_LEVEL >= SID_PAL_LOG_SEVERITY_INFO
#  define SID_PAL_LOG_INFO(msg, ...)                     __SID_PAL_LOG_LIKE_PRINT_FUNC(SID_PAL_LOG_SEVERITY_INFO, msg, ##__VA_ARGS__)
#else
#  define SID_PAL_LOG_INFO(...)                          ((void)0u)
#endif

#if SID_PAL_LOG_LEVEL >= SID_PAL_LOG_SEVERITY_DEBUG
#  define SID_PAL_LOG_DEBUG(msg, ...)                    __SID_PAL_LOG_LIKE_PRINT_FUNC(SID_PAL_LOG_SEVERITY_DEBUG, msg, ##__VA_ARGS__)
#else
#  define SID_PAL_LOG_DEBUG(...)                         ((void)0u)
#endif

#define SID_PAL_LOG_TRACE(...)                           SID_PAL_LOG_INFO("%s:%i %s() TRACE --", __FILENAME__, __LINE__, __FUNCTION__)

#if SID_PAL_LOG_LEVEL >= SID_PAL_LOG_SEVERITY_DEBUG
#  define SID_PAL_LOG_FLUSH()                            do \
                                                         { \
                                                             __NOP(); \
                                                         } while (UTIL_ADV_TRACE_IsBufferEmpty() == 0u)
#else
#  define SID_PAL_LOG_FLUSH()                            ((void)0u)
#endif

/* Exported functions prototypes ---------------------------------------------*/

#if CFG_LOG_SUPPORTED
void __sid_pal_log_like(uint8_t severity, const char * const fmt, ...);
#endif /* CFG_LOG_SUPPORTED */

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_LOG_LIKE_H_ */
