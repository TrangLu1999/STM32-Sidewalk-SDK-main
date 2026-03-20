/**
  ******************************************************************************
  * @file    common_memory_symbols.h
  * @brief   Glue code to use linker-defined symbols in C code
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __TARGET_COMMON_MEMORY_SYMBOLS_H_
#define __TARGET_COMMON_MEMORY_SYMBOLS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

extern uint32_t __FLASH_region_start__;
extern uint32_t __FLASH_region_size__;
extern uint32_t __FLASH_region_end__;
#define APP_FLASH_START                             ((uint32_t)&__FLASH_region_start__)
#define APP_FLASH_SIZE                              ((uint32_t)&__FLASH_region_size__)
#define APP_FLASH_END                               ((uint32_t)&__FLASH_region_end__)


extern uint32_t __MFG_DATA_region_start__;
extern uint32_t __MFG_DATA_region_size__;
extern uint32_t __MFG_DATA_region_end__;
#define MANUFACTURE_FLASH_START                     ((uint32_t)&__MFG_DATA_region_start__)
#define MANUFACTURE_FLASH_SIZE                      ((uint32_t)&__MFG_DATA_region_size__)
#define MANUFACTURE_FLASH_END                       ((uint32_t)&__MFG_DATA_region_end__)


extern uint32_t __LITTLE_FS_region_start__;
extern uint32_t __LITTLE_FS_region_size__;
extern uint32_t __LITTLE_FS_region_end__;
#define APP_CONFIG_FLASH_START                      ((uint32_t)&__LITTLE_FS_region_start__)
#define APP_CONFIG_FLASH_SIZE                       ((uint32_t)&__LITTLE_FS_region_size__)
#define APP_CONFIG_FLASH_END                        ((uint32_t)&__LITTLE_FS_region_end__)


extern uint32_t __SNVMA_ARBTR_region_start__;
extern uint32_t __SNVMA_ARBTR_region_size__;
extern uint32_t __SNVMA_ARBTR_region_end__;
#define APP_CONFIG_SNVMA_FLASH_START                ((uint32_t)&__SNVMA_ARBTR_region_start__)
#define APP_CONFIG_SNVMA_FLASH_SIZE                 ((uint32_t)&__SNVMA_ARBTR_region_size__)
#define APP_CONFIG_SNVMA_FLASH_END                  ((uint32_t)&__SNVMA_ARBTR_region_end__)


extern uint32_t __start_STACK;
extern uint32_t __end_STACK;
extern uint32_t __start_STACK_GUARD;
extern uint32_t __end_STACK_GUARD;
#define APP_CONFIG_USER_STACK_START                 ((uint32_t)&__start_STACK)
#define APP_CONFIG_USER_STACK_END                   ((uint32_t)&__end_STACK)
#define APP_CONFIG_USER_STACK_GUARD_START           ((uint32_t)&__start_STACK_GUARD)
#define APP_CONFIG_USER_STACK_GUARD_END             ((uint32_t)&__end_STACK_GUARD)


extern uint32_t __start_HEAP;
extern uint32_t __end_HEAP;
#define APP_CONFIG_USER_HEAP_START                  ((uint32_t)&__start_HEAP)
#define APP_CONFIG_USER_HEAP_END                    ((uint32_t)&__end_HEAP)


#ifdef AIM_OTA_SUPPORT
extern uint32_t __start_ota_status_area;
extern uint32_t __end_ota_status_area;
#  define APP_CONFIG_AIM_OTA_STATUS_AREA_START      ((uint32_t)&__start_ota_status_area)
#  define APP_CONFIG_AIM_OTA_STATUS_AREA_END        ((uint32_t)&__end_ota_status_area)
#  define APP_CONFIG_AIM_OTA_STATUS_AREA_SIZE_PAGES ((APP_CONFIG_AIM_OTA_STATUS_AREA_END - APP_CONFIG_AIM_OTA_STATUS_AREA_START) / FLASH_PAGE_SIZE)


extern uint32_t __start_app_active_area;
extern uint32_t __end_app_active_area;
extern uint32_t __start_active_app_vtor;
extern uint32_t __active_app_estack;
extern uint32_t __active_app_entry;
extern uint32_t __start_aim_active_slot_header;
extern uint32_t __offset_aim_active_slot_header;
#  define APP_CONFIG_AIM_ACTIVE_SLOT_START          ((uint32_t)&__start_app_active_area)
#  define APP_CONFIG_AIM_ACTIVE_SLOT_END            ((uint32_t)&__end_app_active_area)
#  define APP_CONFIG_AIM_ACTIVE_SLOT_VTOR_START     ((uint32_t)&__start_active_app_vtor)
#  define APP_CONFIG_AIM_ACTIVE_SLOT_ESTACK         ((uint32_t)&__active_app_estack)
#  define APP_CONFIG_AIM_ACTIVE_SLOT_ENTRY_POINT    ((uint32_t)&__active_app_entry)
#  define APP_CONFIG_AIM_ACTIVE_SLOT_HEADER_START   ((uint32_t)&__start_aim_active_slot_header)
#  define APP_CONFIG_AIM_ACTIVE_SLOT_HEADER_OFFSET  ((uint32_t)&__offset_aim_active_slot_header)
#  define APP_CONFIG_AIM_ACTIVE_SLOT_SIZE           (APP_CONFIG_AIM_ACTIVE_SLOT_END - APP_CONFIG_AIM_ACTIVE_SLOT_START)
#  define APP_CONFIG_AIM_ACTIVE_SLOT_SIZE_PAGES     (APP_CONFIG_AIM_ACTIVE_SLOT_SIZE / FLASH_PAGE_SIZE)


extern uint32_t __start_app_rollback_area;
extern uint32_t __start_aim_rollback_slot_header;
extern uint32_t __end_app_rollback_area;
#  define APP_CONFIG_AIM_ROLLBACK_SLOT_START        ((uint32_t)&__start_app_rollback_area)
#  define APP_CONFIG_AIM_ROLLBACK_SLOT_END          ((uint32_t)&__end_app_rollback_area)
#  define APP_CONFIG_AIM_ROLLBACK_SLOT_HEADER_START ((uint32_t)&__start_aim_rollback_slot_header)
#  define APP_CONFIG_AIM_ROLLBACK_SLOT_SIZE         (APP_CONFIG_AIM_ROLLBACK_SLOT_END - APP_CONFIG_AIM_ROLLBACK_SLOT_START)
#  define APP_CONFIG_AIM_ROLLBACK_SLOT_SIZE_PAGES   (APP_CONFIG_AIM_ROLLBACK_SLOT_SIZE / FLASH_PAGE_SIZE)


extern uint32_t __start_app_staging_area;
extern uint32_t __end_app_staging_area;
extern uint32_t __start_aim_staging_slot_header;
#  define APP_CONFIG_AIM_STAGING_SLOT_START         ((uint32_t)&__start_app_staging_area)
#  define APP_CONFIG_AIM_STAGING_SLOT_END           ((uint32_t)&__end_app_staging_area)
#  define APP_CONFIG_AIM_STAGING_SLOT_HEADER_START  ((uint32_t)&__start_aim_staging_slot_header)
#  define APP_CONFIG_AIM_STAGING_SLOT_SIZE          (APP_CONFIG_AIM_STAGING_SLOT_END - APP_CONFIG_AIM_STAGING_SLOT_START)
#  define APP_CONFIG_AIM_STAGING_SLOT_SIZE_PAGES    (APP_CONFIG_AIM_STAGING_SLOT_SIZE / FLASH_PAGE_SIZE)


extern uint32_t __total_flash_memory_size;
#  define APP_CONFIG_TOTAL_FLASH_SIZE               ((uint32_t)&__total_flash_memory_size)
#endif /* AIM_OTA_SUPPORT */

#ifdef __cplusplus
}
#endif

#endif /* __TARGET_COMMON_MEMORY_SYMBOLS_H_ */
