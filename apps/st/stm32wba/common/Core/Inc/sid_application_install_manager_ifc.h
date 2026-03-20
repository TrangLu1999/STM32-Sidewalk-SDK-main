/**
  ******************************************************************************
  * @file    sid_application_install_manager_ifc.h
  * @brief   Definitions to interface the applications with the AIM
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

#ifndef __SID_APPLICATION_INSTALL_MANAGER_IFC_H_
#define __SID_APPLICATION_INSTALL_MANAGER_IFC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "utilities_conf.h"

#include <cmsis_compiler.h>
#include <common_memory_symbols.h>
#include <sid_stm32_common_utils.h>

#include <stm32wbaxx.h>

/* Exported constants --------------------------------------------------------*/

#define SID_AIM_IFC_IMAGE_VALIDITY_MAGIC_WORD              (0x94448A29u)

#define SID_AIM_IFC_VALIDITY_MARKER_MAGIC_WORD_INITIALIZER { 0x7AC3F91Eu, 0xD45E8B20u, 0x31B7C4DAu, 0xE9A0625Fu, }

/* Define mapping of OTA messages in SRAM */
#define SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD0_ADDR           ((uint32_t *)(void *)(SRAM1_BASE + 0u))
#define SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD1_ADDR           ((uint32_t *)(void *)(SRAM1_BASE + sizeof(uint32_t)))

#define SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD0                ((uint32_t)0x6DA3F94Cu)
#define SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD1                ((uint32_t)0x925C06B3u)

#define SID_AIM_IFC_FLASH_WORD_SIZE                        (16u) /* in bytes */

/* Exported macros -----------------------------------------------------------*/

#define SID_AIM_IFC_DECLARE_OTA_METADATA()      /* OTA footer for the current app image (not the OTA update one) */ \
                                                UTIL_PLACE_IN_SECTION(".ota_footer") \
                                                const sid_aim_ota_footer_t app_ota_footer = { \
                                                    .magic_word = SID_AIM_IFC_IMAGE_VALIDITY_MAGIC_WORD, \
                                                }; \
                                                \
                                                /* OTA app validity marker - this is set by the active app as the last step of the OTA to indicate to the bootloader that the app is fully operational */ \
                                                UTIL_PLACE_IN_SECTION(".ota_app_validity_marker") \
                                                SID_STM32_ALIGN_16BYTES(sid_aim_validity_marker_t app_ota_validity_marker); \
                                                \
                                                /* OTA header for the current app image (not the OTA update one) */ \
                                                UTIL_PLACE_IN_SECTION(".ota_header") \
                                                const sid_aim_ota_header_t app_ota_header = { \
                                                    .ota_footer_location      = &app_ota_footer, \
                                                    .validity_marker_location = &app_ota_validity_marker, \
                                                    .app_version_major        = SID_APP_PROJECT_MAJOR_VERSION, \
                                                    .app_version_minor        = SID_APP_PROJECT_MINOR_VERSION, \
                                                    .app_version_patch        = SID_APP_PROJECT_PATCH_VERSION, \
                                                };

/* Exported types ------------------------------------------------------------*/

typedef struct {
    uint32_t validity_magic_word[SID_AIM_IFC_FLASH_WORD_SIZE / sizeof(uint32_t)];
} sid_aim_validity_marker_t;

typedef __PACKED_STRUCT {
    const uint8_t  signature[64]; /*!< Ed25519 signature of the OTA image */
    const uint32_t magic_word;    /*!< Image integrity marker */
    const uint32_t checksum;      /*!< CRC of the OTA image, from the start up to the magic word above */
} sid_aim_ota_footer_t;

typedef __PACKED_STRUCT {
    const sid_aim_ota_footer_t *      ota_footer_location; /*!< Pointer to the sid_aim_ota_footer_t location in the end of the OTA image */
    const sid_aim_validity_marker_t * validity_marker_location; /*!< Pointer to the validity marker location. This marker shall be set with the special magic word by the app to indicate that OTA update is confirmed to be good */
    const uint8_t                     app_version_major;
    const uint8_t                     app_version_minor;
    const uint16_t                    app_version_patch;
} sid_aim_ota_header_t;


typedef enum {
    AIMAVS_CONFIRMATION_INVALID       = 0u, /*!< Special value to indicate that the confirmation state is not known */
    AIMAVS_CONFIRMATION_PENDING       = 1u,
    AIMAVS_CONFIRMED                  = 2u,
    AIMAVS_REJECTED                   = 3u,
} sid_aim_app_validity_state_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Sets an indication to AIM that the application self-confirms its own validity.
 * This method should typically be called on the first boot after firmware update once
 * the app considers itself fully operational. As a rule of thumb, a good confirmation
 * point as the first successful connection to the network as this confirms the wireless
 * stack is fully operational and further updates can be received in the future.
 *
 * @note This is a non-blocking call. The actual flash write operation may be delayed (
 * e.g., if BLE or Sidewalk stack expect a time-sensitive radio event shortly).
 *
 * @note This method is safe to call multiple times, even when the confirmation marker is
 * written already tio the flash location.
 *
 * @return SID_ERROR_NONE on success. See @ref sid_error_t
 */
int32_t sid_aim_ifc_confirm_app_is_verified(void);

/**
 * @brief Sets an indication to AIM that the application self-confirmation has failed and
 * a rollback is requested. This method can be used to notify the AIM that something is
 * not ok with the new app even though it passed CRC and digital signature checks (e.g.,
 * BLE or Sidewalk is dysfunctional, no network connection can be established)
 *
 * @note This method is non-blocking and behaves similar to
 * sid_aim_ifc_confirm_app_is_verified(). The only difference is that MCU reset is requested
 * automaticaly on completion to initiate the rollback.
 *
 * @attention This call is a point of no return. Once initiated, the app cannot be marked
 * as good by sid_aim_ifc_confirm_app_is_verified() call any longer until the rollback is
 * completed
 *
 * @return SID_ERROR_NONE on success. See @ref sid_error_t
 */
int32_t sid_aim_ifc_set_app_verification_failed(void);

/**
 * @brief Read current self-confirmation state from the flash
 *
 * @return Current state. See @ref sid_aim_app_validity_state_t
 */
sid_aim_app_validity_state_t sid_aim_ifc_get_app_verification_state(void);

void sid_aim_ifc_boot_in_dfu_mode(void);

int32_t sid_aim_ifc_init(void);

int32_t sid_aim_ifc_deinit(void);

int32_t sid_aim_ifc_store_ota_data_chunk(void * data, const uint32_t offset, const size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __SID_APPLICATION_INSTALL_MANAGER_IFC_H_ */
