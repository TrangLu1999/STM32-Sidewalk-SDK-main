/**
  ******************************************************************************
  * @file    mfg_store.c
  * @brief   sid_pal_mfg_store library implementation for STM32WBAxx MCUs
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

/* Includes ------------------------------------------------------------------*/

#include <assert.h>
#include <stdint.h>

#include "mfg_store_offsets.h"

/* Sidewalk interfaces */
#include <sid_error.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_mfg_store_ifc.h>
#include <sid_endian.h>

/* Platform interfaces */
#include <stm32wbaxx_hal.h>
#include <flash_driver.h>

/* Utilities */
#include <cmsis_compiler.h>
#include <sid_stm32_common_utils.h>

/* Use private Sidewalk SDK config for builds from SDK source code */
#if defined(USE_SID_PRECOMPILED_LIBS) && (USE_SID_PRECOMPILED_LIBS == 0)
#  include <sid_sdk_internal_config.h>
#endif

/* Private defines -----------------------------------------------------------*/
/* Manufacturing store write capability is not enabled by default. It
 * is currently required for internal diagnostic apps and for SWAT
 */
#if (defined (HALO_ENABLE_DIAGNOSTICS) && HALO_ENABLE_DIAGNOSTICS) || defined(SWAT_DEVICE_TYPE)
#  define ENABLE_MFG_STORE_WRITE
#endif

#ifdef ENABLE_MFG_STORE_WRITE
#  warning "You are building Sidewalk MFG Storage code with flash write capabilities. Normally this functionality shall be excluded from builds for security reasons. Ignore this warning if write capability is enabled intentionally"
#endif /* ENABLE_MFG_STORE_WRITE */

#if !defined(ENABLE_MFG_STORE_OVERWRITE_READ)
#  define ENABLE_MFG_STORE_OVERWRITE_READ           (0u)
#endif /* ENABLE_MFG_STORE_OVERWRITE_READ */

#if ENABLE_MFG_STORE_OVERWRITE_READ
#  warning "You are building Sidewalk MFG Storage code with read operations override enabled. Normally this functionality shall be excluded from builds for security reasons. Ignore this warning if write capability is enabled intentionally"
#endif /* ENABLE_MFG_STORE_OVERWRITE_READ */

#define MFG_VERSION_1_VAL                           (0x01000000u)
#define MFG_VERSION_2_VAL                           (0x02u)

#define ENCODED_DEV_ID_SIZE_5_BYTES_MASK            (0xA0u)
#define DEV_ID_MSB_MASK                             (0x1Fu)

#define MFG_STORE_TLV_HEADER_SIZE                   (4u)
#define MFG_STORE_TLV_TAG_EMPTY                     (0xFFFFu)
#define MFG_STORE_TLV_LENGTH_EMPTY                  (0xFFFFu)

#define MFG_STORE_DEVID_V1_SIZE                     (8u)

#define MFG_STORE_PAGE_ERASE_RETRY_LIMIT            (5u) /*!< If a page erase operation fails during manufacturing provisioning process, it will be retried up to this amount of times before reporting a permanent failure */

#if defined(USE_SID_PRECOMPILED_LIBS) && (USE_SID_PRECOMPILED_LIBS == 0) && defined(SID_MFG_STORE_SUPPORT_FIXED_OFFSETS) && (SID_MFG_STORE_SUPPORT_FIXED_OFFSETS != SID_MFG_STORE_SUPPORT_FIXED_OFFSETS_DEFAULT)
#  warning "The specified SID_MFG_STORE_SUPPORT_FIXED_OFFSETS value differs from the default one for the builds from static libs. Ignore this warning if this is intentional, otherwise make sure SID_MFG_STORE_SUPPORT_FIXED_OFFSETS_DEFAULT and SID_MFG_STORE_SUPPORT_FIXED_OFFSETS in sid_sdk_internal_config.h are aligned"
#endif

#ifndef SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
#  define SID_MFG_STORE_SUPPORT_FIXED_OFFSETS       (0)
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */

#define MFG_STORE_LOG_PREFIX                        "MFG storage: "

/* Private typedef -----------------------------------------------------------*/

typedef struct {
    uint16_t tag;
    uint16_t length;
    size_t   offset; /*!< TLV offset from mfg_store_region.addr_start in bytes */
} mfg_store_tlv_info_t;

#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
typedef struct {
    sid_pal_mfg_store_value_t value;
    uint16_t                  size;
    uint32_t                  offset;
} mfg_store_record_info_t;
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */

/* Private constants ---------------------------------------------------------*/

#if !defined(SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED) || (SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED == 0)
static const uint8_t mfg_magic_word[] = {'S', 'I', 'D', '0'};
#endif /* !SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

/* External functions prototypes ---------------------------------------------*/

void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/

#if ENABLE_MFG_STORE_OVERWRITE_READ
static        bool     default_overwrite_read(uint16_t value, uint8_t * buffer, uint16_t length);
#endif /* ENABLE_MFG_STORE_OVERWRITE_READ */

static        uint32_t default_app_value_to_offset(int value);
static        bool     sid_pal_mfg_store_search_for_tag(const uint16_t tag, mfg_store_tlv_info_t * const tlv_info);
static        bool     sid_pal_mfg_store_check_record_exists(const uint16_t tag, const uint32_t mfg_storage_version);
static        bool     sid_pal_mfg_store_validate(void);

#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
static inline void *   calculate_validated_addr(const uint32_t offset, const uintptr_t start_address, const uintptr_t end_address);
static inline bool     is_valid_value_offset(const uint32_t offset);
static        void *   value_to_address(const sid_pal_mfg_store_value_t value, const uintptr_t start_address, const uintptr_t end_address);
static        uint16_t value_to_size(const sid_pal_mfg_store_value_t value);
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */

#ifdef ENABLE_MFG_STORE_WRITE
static        bool     write_to_flash(const void * const dest_address, void * const src_address, const size_t length);
#endif

/* Private variables ---------------------------------------------------------*/

static bool                       mfg_store_init_done  = false;
static sid_pal_mfg_store_region_t stm_mfg_store_region = {0};

/* Private constants ---------------------------------------------------------*/

#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
static const mfg_store_record_info_t sid_pal_mfg_store_app_value_to_offset_table[] = {
    {SID_PAL_MFG_STORE_VERSION,                      SID_PAL_MFG_STORE_VERSION_SIZE,                      SID_PAL_MFG_STORE_OFFSET_VERSION},
    {SID_PAL_MFG_STORE_DEVID,                        SID_PAL_MFG_STORE_DEVID_SIZE,                        SID_PAL_MFG_STORE_OFFSET_DEVID},
    {SID_PAL_MFG_STORE_SERIAL_NUM,                   SID_PAL_MFG_STORE_SERIAL_NUM_SIZE,                   SID_PAL_MFG_STORE_OFFSET_SERIAL_NUM},
    {SID_PAL_MFG_STORE_SMSN,                         SID_PAL_MFG_STORE_SMSN_SIZE,                         SID_PAL_MFG_STORE_OFFSET_SMSN},
    {SID_PAL_MFG_STORE_APID,                         SID_PAL_MFG_STORE_APID_SIZE,                         SID_PAL_MFG_STORE_OFFSET_APID},
    {SID_PAL_MFG_STORE_APP_PUB_ED25519,              SID_PAL_MFG_STORE_APP_PUB_ED25519_SIZE,              SID_PAL_MFG_STORE_OFFSET_APP_PUB_ED25519},
    {SID_PAL_MFG_STORE_DEVICE_PRIV_ED25519,          SID_PAL_MFG_STORE_DEVICE_PRIV_ED25519_SIZE,          SID_PAL_MFG_STORE_OFFSET_DEVICE_PRIV_ED25519},
    {SID_PAL_MFG_STORE_DEVICE_PUB_ED25519,           SID_PAL_MFG_STORE_DEVICE_PUB_ED25519_SIZE,           SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_ED25519},
    {SID_PAL_MFG_STORE_DEVICE_PUB_ED25519_SIGNATURE, SID_PAL_MFG_STORE_DEVICE_PUB_ED25519_SIGNATURE_SIZE, SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_DEVICE_PRIV_P256R1,           SID_PAL_MFG_STORE_DEVICE_PRIV_P256R1_SIZE,           SID_PAL_MFG_STORE_OFFSET_DEVICE_PRIV_P256R1},
    {SID_PAL_MFG_STORE_DEVICE_PUB_P256R1,            SID_PAL_MFG_STORE_DEVICE_PUB_P256R1_SIZE,            SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_P256R1},
    {SID_PAL_MFG_STORE_DEVICE_PUB_P256R1_SIGNATURE,  SID_PAL_MFG_STORE_DEVICE_PUB_P256R1_SIGNATURE_SIZE,  SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_DAK_PUB_ED25519,              SID_PAL_MFG_STORE_DAK_PUB_ED25519_SIZE,              SID_PAL_MFG_STORE_OFFSET_DAK_PUB_ED25519},
    {SID_PAL_MFG_STORE_DAK_PUB_ED25519_SIGNATURE,    SID_PAL_MFG_STORE_DAK_PUB_ED25519_SIGNATURE_SIZE,    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_DAK_ED25519_SERIAL,           SID_PAL_MFG_STORE_DAK_ED25519_SERIAL_SIZE,           SID_PAL_MFG_STORE_OFFSET_DAK_ED25519_SERIAL},
    {SID_PAL_MFG_STORE_DAK_PUB_P256R1,               SID_PAL_MFG_STORE_DAK_PUB_P256R1_SIZE,               SID_PAL_MFG_STORE_OFFSET_DAK_PUB_P256R1},
    {SID_PAL_MFG_STORE_DAK_PUB_P256R1_SIGNATURE,     SID_PAL_MFG_STORE_DAK_PUB_P256R1_SIGNATURE_SIZE,     SID_PAL_MFG_STORE_OFFSET_DAK_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_DAK_P256R1_SERIAL,            SID_PAL_MFG_STORE_DAK_P256R1_SERIAL_SIZE,            SID_PAL_MFG_STORE_OFFSET_DAK_P256R1_SERIAL},
    {SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519,          SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519_SIZE,          SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_ED25519},
    {SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519_SIGNATURE,SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519_SIGNATURE_SIZE,SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_PRODUCT_ED25519_SERIAL,       SID_PAL_MFG_STORE_PRODUCT_ED25519_SERIAL_SIZE,       SID_PAL_MFG_STORE_OFFSET_PRODUCT_ED25519_SERIAL},
    {SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1,           SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1_SIZE,           SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_P256R1},
    {SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1_SIGNATURE, SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1_SIGNATURE_SIZE, SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_PRODUCT_P256R1_SERIAL,        SID_PAL_MFG_STORE_PRODUCT_P256R1_SERIAL_SIZE,        SID_PAL_MFG_STORE_OFFSET_PRODUCT_P256R1_SERIAL},
    {SID_PAL_MFG_STORE_MAN_PUB_ED25519,              SID_PAL_MFG_STORE_MAN_PUB_ED25519_SIZE,              SID_PAL_MFG_STORE_OFFSET_MAN_PUB_ED25519},
    {SID_PAL_MFG_STORE_MAN_PUB_ED25519_SIGNATURE,    SID_PAL_MFG_STORE_MAN_PUB_ED25519_SIGNATURE_SIZE,    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_MAN_ED25519_SERIAL,           SID_PAL_MFG_STORE_MAN_ED25519_SERIAL_SIZE,           SID_PAL_MFG_STORE_OFFSET_MAN_ED25519_SERIAL},
    {SID_PAL_MFG_STORE_MAN_PUB_P256R1,               SID_PAL_MFG_STORE_MAN_PUB_P256R1_SIZE,               SID_PAL_MFG_STORE_OFFSET_MAN_PUB_P256R1},
    {SID_PAL_MFG_STORE_MAN_PUB_P256R1_SIGNATURE,     SID_PAL_MFG_STORE_MAN_PUB_P256R1_SIGNATURE_SIZE,     SID_PAL_MFG_STORE_OFFSET_MAN_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_MAN_P256R1_SERIAL,            SID_PAL_MFG_STORE_MAN_P256R1_SERIAL_SIZE,            SID_PAL_MFG_STORE_OFFSET_MAN_P256R1_SERIAL},
    {SID_PAL_MFG_STORE_SW_PUB_ED25519,               SID_PAL_MFG_STORE_SW_PUB_ED25519_SIZE,               SID_PAL_MFG_STORE_OFFSET_SW_PUB_ED25519},
    {SID_PAL_MFG_STORE_SW_PUB_ED25519_SIGNATURE,     SID_PAL_MFG_STORE_SW_PUB_ED25519_SIGNATURE_SIZE,     SID_PAL_MFG_STORE_OFFSET_SW_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_SW_ED25519_SERIAL,            SID_PAL_MFG_STORE_SW_ED25519_SERIAL_SIZE,            SID_PAL_MFG_STORE_OFFSET_SW_ED25519_SERIAL},
    {SID_PAL_MFG_STORE_SW_PUB_P256R1,                SID_PAL_MFG_STORE_SW_PUB_P256R1_SIZE,                SID_PAL_MFG_STORE_OFFSET_SW_PUB_P256R1},
    {SID_PAL_MFG_STORE_SW_PUB_P256R1_SIGNATURE,      SID_PAL_MFG_STORE_SW_PUB_P256R1_SIGNATURE_SIZE,      SID_PAL_MFG_STORE_OFFSET_SW_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_SW_P256R1_SERIAL,             SID_PAL_MFG_STORE_SW_P256R1_SERIAL_SIZE,             SID_PAL_MFG_STORE_OFFSET_SW_P256R1_SERIAL},
    {SID_PAL_MFG_STORE_AMZN_PUB_ED25519,             SID_PAL_MFG_STORE_AMZN_PUB_ED25519_SIZE,             SID_PAL_MFG_STORE_OFFSET_AMZN_PUB_ED25519},
    {SID_PAL_MFG_STORE_AMZN_PUB_P256R1,              SID_PAL_MFG_STORE_AMZN_PUB_P256R1_SIZE,              SID_PAL_MFG_STORE_OFFSET_AMZN_PUB_P256R1},
};
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */

#if SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED
static const uint8_t product_apid[]          = {0x76u, 0x43u, 0x74u, 0x32u};
static const uint8_t app_server_public_key[] = {0xB2u, 0x40u, 0xBFu, 0x98u, 0xC6u, 0x5Cu, 0xDFu, 0x84u,
                                                0xBFu, 0x2Au, 0xA1u, 0xACu, 0x29u, 0x11u, 0x14u, 0x1Fu,
                                                0xB4u, 0x80u, 0x7Cu, 0xBCu, 0xB6u, 0x6Eu, 0xCFu, 0x09u,
                                                0x1Cu, 0x20u, 0x04u, 0xB3u, 0x37u, 0xB4u, 0x06u, 0x47u
                                               };
#endif /* SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

/* Private function definitions ----------------------------------------------*/

#if ENABLE_MFG_STORE_OVERWRITE_READ
SID_STM32_SPEED_OPTIMIZED static bool default_overwrite_read(uint16_t value, uint8_t * buffer, uint16_t length)
{
    SID_PAL_LOG_WARNING(MFG_STORE_LOG_PREFIX "no support for overwrite_read");
    return false;
}
#endif /* ENABLE_MFG_STORE_OVERWRITE_READ */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static uint32_t default_app_value_to_offset(int value)
{
    SID_PAL_LOG_WARNING(MFG_STORE_LOG_PREFIX "no support for app_value_to_offset");
    return SID_PAL_MFG_STORE_INVALID_OFFSET;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static bool sid_pal_mfg_store_search_for_tag(const uint16_t tag, mfg_store_tlv_info_t * const tlv_info)
{
    const uint8_t * address   = (uint8_t *)(void *)MFG_ALIGN_TO_WORD_BOUNDARY(stm_mfg_store_region.addr_start /* Starting right after the MFG storage header */
                                    + (SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE)
                                    + SID_PAL_MFG_STORE_VERSION_SIZE);
    bool            tag_found = false;

    /* Ensure TLV header and at least one byte of data fit into flash word size, otherwise the below free space check becomes invalid */
    static_assert(MFG_WORD_SIZE > MFG_STORE_TLV_HEADER_SIZE);
    while ((false == tag_found) && ((uintptr_t)(address + MFG_WORD_SIZE) <= stm_mfg_store_region.addr_end)) /* while there's a space for at least one record */
    {
        /* Parse TLV Tag ID and TLV Data Length */
        const uint16_t current_tag  = address[0] << 8 | address[1]; /* Stored in Big Endian on flash */
        const uint16_t value_length = address[2] << 8 | address[3]; /* Stored in Big Endian on flash */

        if (tag == current_tag)
        {
            /* Desired tag found */
            tlv_info->tag    = tag;
            tlv_info->length = value_length;
            tlv_info->offset = (size_t)(address - stm_mfg_store_region.addr_start);
            tag_found = true;
        }
        else
        {
            if (MFG_STORE_TLV_TAG_EMPTY == current_tag)
            {
                /* Last TLV record was processed */
                break;
            }
            /*
             * Go to the next TLV.
             * Since data is written to flash with data aligned to flash word size, we must take this
             * into account if the data length is not a multiple of flash word size.
             */
            address += MFG_ALIGN_TO_WORD_BOUNDARY(MFG_STORE_TLV_HEADER_SIZE + value_length);
        }
    }

    return tag_found;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static bool sid_pal_mfg_store_check_record_exists(const uint16_t tag, const uint32_t mfg_storage_version)
{
    bool record_exists = false;

    switch (mfg_storage_version)
    {
        case SID_PAL_MFG_STORE_TLV_VERSION:
            {
                mfg_store_tlv_info_t tlv_info;

                if (sid_pal_mfg_store_search_for_tag(tag, &tlv_info) != false)
                {
                    record_exists = true;
                }
            }
            break;

        case SID_PAL_MFG_STORE_FIXED_OFFSETS_VERSION:
            {
#  if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
                void * const record_address = value_to_address(tag, stm_mfg_store_region.addr_start, stm_mfg_store_region.addr_end);
                if (record_address != NULL)
                {
                    record_exists = true;
                }
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */
            }
            break;

        default:
            SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "can't check if tag 0x%04X exists - unknown MFG storage format (%u)", tag, mfg_storage_version);
            break;
    }

    return record_exists;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static bool sid_pal_mfg_store_validate(void)
{
    bool mfg_data_valid;

    do
    {
        mfg_data_valid = true;

#if !defined(SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED) || (SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED == 0)
        /* Check if the magic word is in place */
        const void * const magic_word_address = (void *)(stm_mfg_store_region.addr_start + (SID_PAL_MFG_STORE_OFFSET_MAGIC * MFG_WORD_SIZE));

        if (SID_STM32_UTIL_fast_memcmp(magic_word_address, mfg_magic_word, sizeof(mfg_magic_word)) != 0u)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "magic word not found");
            mfg_data_valid = false;
            break;
        }
#endif /* !SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

        /* Check if MFG storage format version is present and known */
        static_assert(SID_PAL_MFG_STORE_VERSION_SIZE == sizeof(uint32_t));
        const uint32_t * const version_address = (uint32_t *)(void *)(stm_mfg_store_region.addr_start + (SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE));
        const uint32_t stored_version = sid_ntohl(*version_address);
        if ((stored_version != SID_PAL_MFG_STORE_TLV_VERSION)
#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
         && (stored_version != SID_PAL_MFG_STORE_FIXED_OFFSETS_VERSION)
#endif /*SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */
        )
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "format version is invalid (0x%08X)", stored_version);
            mfg_data_valid = false;
            break;
        }

        /* Iterate over entries for TLV format to ensure all TLV headers are valid */
        if (SID_PAL_MFG_STORE_TLV_VERSION == stored_version)
        {
            const uint8_t * address   = (uint8_t *)(void *)MFG_ALIGN_TO_WORD_BOUNDARY(stm_mfg_store_region.addr_start /* Starting right after the MFG storage header */
                                            + (SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE)
                                            + SID_PAL_MFG_STORE_VERSION_SIZE);

            /* Ensure TLV header and at least one byte of data fit into flash word size, otherwise the below free space check becomes invalid */
            static_assert(MFG_WORD_SIZE > MFG_STORE_TLV_HEADER_SIZE);
            while ((uintptr_t)address < stm_mfg_store_region.addr_end)
            {
                /* Parse TLV Tag ID and TLV Data Length */
                const uint16_t current_tag  = address[0] << 8 | address[1]; /* Stored in Big Endian on flash */
                const uint16_t value_length = address[2] << 8 | address[3]; /* Stored in Big Endian on flash */

                if ((MFG_STORE_TLV_TAG_EMPTY == current_tag) && (MFG_STORE_TLV_LENGTH_EMPTY == value_length))
                {
                    /* End of TLV records reached */
                    break;
                }

                /* Ensure the current tag looks like a valid Tag value */
                if ((SID_PAL_MFG_STORE_VERSION == current_tag) || (current_tag > SID_PAL_MFG_STORE_VALUE_MAX))
                {
                    SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "tag ID 0x%04X at offset 0x%08X is not a valid tag ID", current_tag, address);
                    mfg_data_valid = false;
                    break;
                }

                /* Check the value length is plausible */
                if ((0u == value_length) || ((uintptr_t)(address + MFG_ALIGN_TO_WORD_BOUNDARY(MFG_STORE_TLV_HEADER_SIZE + value_length)) > stm_mfg_store_region.addr_end))
                {
                    SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "length of the record with tag ID 0x%04X at offset 0x%08X is not valid", current_tag, address);
                    mfg_data_valid = false;
                    break;
                }

                /* Move to the next record */
                address += MFG_ALIGN_TO_WORD_BOUNDARY(MFG_STORE_TLV_HEADER_SIZE + value_length);
            }
            if (false == mfg_data_valid)
            {
                break;
            }
        }

        /* Check MFG storage contains SMSN */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_SMSN, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "SMSN record not found");
            mfg_data_valid = false;
            break;
        }

#if !defined(SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED) || (SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED == 0)
        /* Check MFG storage contains App ID */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_APID, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "AppID record not found");
            mfg_data_valid = false;
            break;
        }
#endif /* !SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

        /* Check MFG storage contains device private Ed25519 key */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DEVICE_PRIV_ED25519, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device private Ed25519 key record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device public Ed25519 key */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DEVICE_PUB_ED25519, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device public Ed25519 key record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device public Ed25519 key signature */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DEVICE_PUB_ED25519_SIGNATURE, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device public Ed25519 key signature record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device private P256r1 key */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DEVICE_PRIV_P256R1, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device private P256r1 key record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device public P256r1 key */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DEVICE_PUB_P256R1, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device public P256r1 key record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device public P256r1 key signature */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DEVICE_PUB_P256R1_SIGNATURE, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device public P256r1 key signature record not found");
            mfg_data_valid = false;
            break;
        }

#if !defined(SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED) || (SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED == 0)
        /* Check MFG storage contains device attestation Ed25519 key serial */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DAK_ED25519_SERIAL, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device attestation Ed25519 key serial record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device public Ed25519 key */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DAK_PUB_ED25519, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device attestation public Ed25519 key record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device attestation public Ed25519 key signature */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DAK_PUB_ED25519_SIGNATURE, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device attestation public Ed25519 key signature record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device attestation P256r1 key serial */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DAK_P256R1_SERIAL, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device attestation P256r1 key serial record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device attestation public P256r1 key */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DAK_PUB_P256R1, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device attestation public P256r1 key record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains device attestation public P256r1 key signature */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_DAK_PUB_P256R1_SIGNATURE, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "device attestation public P256r1 key signature record not found");
            mfg_data_valid = false;
            break;
        }
#endif /* !SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

        /* Check MFG storage contains product Ed25519 key serial */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_PRODUCT_ED25519_SERIAL, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "product Ed25519 key serial record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains product public Ed25519 key */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "product public Ed25519 key record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains product public Ed25519 key signature */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519_SIGNATURE, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "product public Ed25519 key signature record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains product P256r1 key serial */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_PRODUCT_P256R1_SERIAL, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "product P256r1 key serial record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains product public P256r1 key */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "product public P256r1 key record not found");
            mfg_data_valid = false;
            break;
        }

        /* Check MFG storage contains product public P256r1 key signature */
        if (sid_pal_mfg_store_check_record_exists(SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1_SIGNATURE, stored_version) == false)
        {
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "product public P256r1 key signature record not found");
            mfg_data_valid = false;
            break;
        }

        SID_PAL_LOG_INFO(MFG_STORE_LOG_PREFIX "validation passed");
    } while (0);

    return mfg_data_valid;
}

/*----------------------------------------------------------------------------*/

#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
SID_STM32_SPEED_OPTIMIZED static inline void * calculate_validated_addr(const uint32_t offset, const uintptr_t start_address, const uintptr_t end_address)
{
    void * addr;

    if ((start_address + offset) >= end_address)
    {
        SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "offset past manufacturing store end: %u", offset);
        addr = NULL;
    }
    else
    {
        addr = (void*)(start_address + offset);
    }

    return addr;
}
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */

/*----------------------------------------------------------------------------*/

#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
SID_STM32_SPEED_OPTIMIZED static inline bool is_valid_value_offset(const uint32_t offset)
{
    return offset != SID_PAL_MFG_STORE_INVALID_OFFSET;
}
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */

/*----------------------------------------------------------------------------*/

#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
SID_STM32_SPEED_OPTIMIZED static void * value_to_address(const sid_pal_mfg_store_value_t value, const uintptr_t start_address, const uintptr_t end_address)
{
    void *   record_address;
    uint32_t offset = SID_PAL_MFG_STORE_INVALID_OFFSET;

    do
    {
        if (value >= SID_PAL_MFG_STORE_VALUE_MAX)
        {
            /* Invalid value ID */
            record_address = NULL;
            break;
        }

        if (value >= SID_PAL_MFG_STORE_CORE_VALUE_MAX)
        {
            /* This is not a core value. Search for this value among those provided by the application. */
            SID_PAL_ASSERT(stm_mfg_store_region.app_value_to_offset != NULL);
            offset = stm_mfg_store_region.app_value_to_offset(value);
        }
        else
        {
            /* Standard Sidewalk values */
            for (size_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(sid_pal_mfg_store_app_value_to_offset_table); i++)
            {
                if (sid_pal_mfg_store_app_value_to_offset_table[i].value == value)
                {
                    offset = sid_pal_mfg_store_app_value_to_offset_table[i].offset;
                    break;
                }
            }
        }

        /* Validate discovered offset and compute record address */
        if (is_valid_value_offset(offset) != false)
        {
            record_address = calculate_validated_addr(offset, start_address, end_address);
        }
        else
        {
            record_address = NULL;
            SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "no MFG store offset defined for tag 0x%04X", value);
        }
    } while (0);

    return record_address;
}
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */

/*----------------------------------------------------------------------------*/

#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
SID_STM32_SPEED_OPTIMIZED static uint16_t value_to_size(const sid_pal_mfg_store_value_t value)
{
    const size_t table_count = SID_STM32_UTIL_ARRAY_SIZE(sid_pal_mfg_store_app_value_to_offset_table);

    for (size_t i = 0u; i < table_count; i++)
    {
        if (value == sid_pal_mfg_store_app_value_to_offset_table[i].value)
        {
            return is_valid_value_offset(sid_pal_mfg_store_app_value_to_offset_table[i].offset) ?
                sid_pal_mfg_store_app_value_to_offset_table[i].size : 0u;
        }
    }

    /* NOTE: Getting size for App values >= SID_PAL_MFG_STORE_CORE_VALUE_MAX is not supported */
    return 0u;
}
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */

/*----------------------------------------------------------------------------*/

#ifdef ENABLE_MFG_STORE_WRITE
static bool write_to_flash(const void * const dest_address, void * const src_address, const size_t length)
{
    bool                success;
    FD_FlashOp_Status_t fd_err = FD_FLASHOP_FAILURE;

    if ((length % MFG_WORD_SIZE) != 0u)
    {
        SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "invalid data length for flash write. It can only be written in multiples of %u bytes. Requested length: %u bytes", MFG_WORD_SIZE, length);
        return false;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "failed to unlock flash for writing");
        return false;
    }

    /* Clear flash error flags before proceeding */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    uint32_t write_address   = (uint32_t)dest_address;
    uint32_t read_address    = (uint32_t)src_address;
    size_t   bytes_processed = 0u;

    /* Write data in quad-word chunks */
    while (bytes_processed < length)
    {
        SID_PAL_ASSERT(write_address % MFG_WORD_SIZE == 0u);
        SID_PAL_ASSERT(read_address % sizeof(uint32_t) == 0u);

        fd_err = FD_WriteData(write_address, read_address);
        if (fd_err != FD_FLASHOP_SUCCESS)
        {
            SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "flash write error at offset 0x%08X", write_address);
            break;
        }

        /* Move to the next quad-word */
        write_address   += MFG_WORD_SIZE;
        read_address    += MFG_WORD_SIZE;
        bytes_processed += MFG_WORD_SIZE;
    }

    /* Always lock the flash after writing regardless of the outcome */
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        SID_PAL_LOG_WARNING(MFG_STORE_LOG_PREFIX "failed to lock flash after writing");
    }

    success = (FD_FLASHOP_SUCCESS == fd_err) ? true : false;

    return success;
}
#endif

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_pal_mfg_store_init(sid_pal_mfg_store_region_t mfg_store_region)
{
    if (mfg_store_init_done != false)
    {
        return;
    }

    stm_mfg_store_region = mfg_store_region;

    if (NULL == stm_mfg_store_region.app_value_to_offset)
    {
        stm_mfg_store_region.app_value_to_offset = default_app_value_to_offset;
    }

#if ENABLE_MFG_STORE_OVERWRITE_READ
    if (NULL == stm_mfg_store_region.overwrite_read)
    {
        stm_mfg_store_region.overwrite_read = default_overwrite_read;
    }
#endif /* ENABLE_MFG_STORE_OVERWRITE_READ */

    if (sid_pal_mfg_store_validate() == false)
    {
#ifdef ENABLE_MFG_STORE_WRITE
        /* Empty storage is expected when storage write functionality is enabled (e.g., during manufacturing process). And if MFG storage is actually corrupted, it can be overwritten */
        if (sid_pal_mfg_store_is_empty() == false)
        {
            SID_PAL_LOG_WARNING(MFG_STORE_LOG_PREFIX "validation failed. Sidewalk link will remain inoperable until valid MFG data is written");
        }
        else
        {
            SID_PAL_LOG_INFO(MFG_STORE_LOG_PREFIX "storage area is empty");
        }
#else
        /* For a regular application empty or corrupted MFG data is an unrecoverable failure */
        SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "validation failed. Cannot proceed due to invalid manufacturing data");
        Error_Handler();
#endif /* ENABLE_MFG_STORE_WRITE */
    }

    /* Ensure mfg_store_init_done is set only after all the above actions are completed */
    __COMPILER_BARRIER();
    mfg_store_init_done = true;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_pal_mfg_store_deinit(void)
{
    if (false == mfg_store_init_done)
    {
        return;
    }

    SID_STM32_UTIL_fast_memset(&stm_mfg_store_region, 0u, sizeof(sid_pal_mfg_store_region_t));
    mfg_store_init_done = false;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_mfg_store_write(uint16_t value, const uint8_t * buffer, uint16_t length)
{
#ifdef ENABLE_MFG_STORE_WRITE
    sid_error_t err;
    SID_STM32_ALIGN_4BYTES(uint8_t wr_array[SID_PAL_MFG_STORE_MAX_FLASH_WRITE_LEN]); /* Flash controller on STM32WBAxx requires the source data to be aligned to a 32-bit boundary */
    bool ret;

    do
    {
        if ((0u == length)
         || (MFG_STORE_TLV_TAG_EMPTY == value)
         || (NULL == buffer))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (false == mfg_store_init_done)
        {
            SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "cannot write storage, MFG is not initialized yet");
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* The SID_PAL_MFG_STORE_VERSION is the first entry to be stored and it should be placed at fixed offset to ensure MFG header can be read by any version of MFG PAL */
        if (SID_PAL_MFG_STORE_VERSION == value)
        {
            if (sid_pal_mfg_store_is_empty() == false)
            {
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "MFG must be erased before writing version");
                err = SID_ERROR_INVALID_STATE;
                break;
            }

#if !defined(SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED) || (SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED == 0)
            const void * const magic_word_address     = (void *)(stm_mfg_store_region.addr_start + (SID_PAL_MFG_STORE_OFFSET_MAGIC * MFG_WORD_SIZE));
            const size_t      magic_word_write_length = MFG_ALIGN_TO_WORD_BOUNDARY(sizeof(mfg_magic_word));

            SID_STM32_UTIL_fast_memset(wr_array, 0x00u, magic_word_write_length);
            SID_STM32_UTIL_fast_memcpy(wr_array, mfg_magic_word, sizeof(mfg_magic_word));

            ret = write_to_flash(magic_word_address, wr_array, magic_word_write_length);
            if (false == ret)
            {
                err = SID_ERROR_STORAGE_WRITE_FAIL;
                break;
            }
#endif /* !SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

            const void * const version_address = (void *)(stm_mfg_store_region.addr_start + (SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE));
            const size_t       write_length    = MFG_ALIGN_TO_WORD_BOUNDARY(length);
            if (write_length != length)
            {
                /* Pre-fill the buffer with padding if data alignment will take place */
                SID_STM32_UTIL_fast_memset(wr_array, 0x00u, write_length);
            }
            SID_STM32_UTIL_fast_memcpy(wr_array, buffer, length);
            ret = write_to_flash(version_address, wr_array, write_length);

            if (false == ret)
            {
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "failed to write MFG header")
                err = SID_ERROR_STORAGE_WRITE_FAIL;
            }
            else
            {
                err = SID_ERROR_NONE;
            }

            /* At this point we are done with writing MFG header, terminating from here */
            break;
        }

        /* Process regular MFG storage entries */
        if (sid_pal_mfg_store_get_version() == SID_PAL_MFG_STORE_TLV_VERSION)
        {
            mfg_store_tlv_info_t tlv_info;

            if (sid_pal_mfg_store_search_for_tag(value, &tlv_info) != false)
            {
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "the tag value %u already exists. We can't write duplicate", value);
                err = SID_ERROR_ALREADY_EXISTS;
                break;
            }

            /* Search for the end of data */
            if (sid_pal_mfg_store_search_for_tag(MFG_STORE_TLV_TAG_EMPTY, &tlv_info) == false)
            {
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "can't write tag %u. MFG storage is full", value);
                err = SID_ERROR_STORAGE_FULL;
                break;
            }

            /* The length should be a multiple of the program unit */
            const size_t full_length = MFG_ALIGN_TO_WORD_BOUNDARY(length + MFG_STORE_TLV_HEADER_SIZE);
            uintptr_t    address     = stm_mfg_store_region.addr_start + tlv_info.offset;

            /* Check the remaining storage size */
            if ((address + full_length) > stm_mfg_store_region.addr_end)
            {
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "not enough space to store record %u (required: %u, available: %u)", value, full_length, (stm_mfg_store_region.addr_end - address));
                err = SID_ERROR_STORAGE_FULL;
                break;
            }

            /* Clean up flash write buffer */
            SID_STM32_UTIL_fast_memset(wr_array, 0xFFu, sizeof(wr_array));

            /* Construct TLV header */
            wr_array[0] = (uint8_t)(value >> 8); /* Tag ID in Big Endian */
            wr_array[1] = (uint8_t)value;
            wr_array[2] = (uint8_t)(length >> 8); /* Value length in Big Endian */
            wr_array[3] = (uint8_t)(length);

            /* Add as many data as fits into the remaining part of the write buffer */
            const size_t firstr_copy_length = full_length > sizeof(wr_array) ? (sizeof(wr_array) - MFG_STORE_TLV_HEADER_SIZE) : (full_length - MFG_STORE_TLV_HEADER_SIZE);
            SID_STM32_UTIL_fast_memcpy(&wr_array[MFG_STORE_TLV_HEADER_SIZE], buffer, firstr_copy_length);

            /* Write the first chunk containing TLV header */
            size_t    write_length    = firstr_copy_length + MFG_STORE_TLV_HEADER_SIZE;
            uintptr_t write_address   = address;
            size_t    bytes_processed = 0u;

            ret = write_to_flash((void*)write_address, wr_array, write_length);

            /* Move to the next chunk */
            bytes_processed += write_length;
            write_address   += write_length;

            /* Write any remaining chunks until entire tag is processed */
            while ((bytes_processed < full_length) && (ret != false))
            {
                write_length = (full_length - bytes_processed) > sizeof(wr_array) ? sizeof(wr_array) : (full_length - bytes_processed);
                SID_STM32_UTIL_fast_memset(wr_array, 0xFFu, sizeof(wr_array));
                SID_STM32_UTIL_fast_memcpy(wr_array, &buffer[bytes_processed - MFG_STORE_TLV_HEADER_SIZE], write_length);
                ret = write_to_flash((void*)write_address, wr_array, write_length);

                /* Move to the next chunk */
                bytes_processed += write_length;
                write_address += write_length;
            }

            if (false == ret)
            {
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "failed to write data at offset 0x%08X for tag %u", (write_address - write_length), value);
                err = SID_ERROR_STORAGE_WRITE_FAIL;
            }
            else
            {
                err = SID_ERROR_NONE;
            }
        }
        else
        {
#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
            if (length > sizeof(wr_array))
            {
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "supplied value length for tag %u exceeds write buffer size (%u vs %u)", value, length, sizeof(wr_array));
                err = SID_ERROR_BUFFER_OVERFLOW;
                break;
            }

            const void * const value_address = value_to_address(value, stm_mfg_store_region.addr_start, stm_mfg_store_region.addr_end);
            if (value_address == NULL)
            {
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "unable to determine MFG value offset for writing. Value ID: %u", value);
                err = SID_ERROR_PARAM_OUT_OF_RANGE;
                break;
            }

            const size_t write_length = MFG_ALIGN_TO_WORD_BOUNDARY(length);
            if (write_length != length)
            {
                /* Pre-fill the buffer with padding if data alignment will take place */
                SID_STM32_UTIL_fast_memset(wr_array, 0xFFu, write_length);
            }
            SID_STM32_UTIL_fast_memcpy(wr_array, buffer, length);
            ret = write_to_flash(value_address, wr_array, write_length);

            if (false == ret)
            {
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "failed to write data at offset 0x%08X for tag %u", value_address, value);
                err = SID_ERROR_STORAGE_WRITE_FAIL;
            }
            else
            {
                err = SID_ERROR_NONE;
            }
#  else
            err = SID_ERROR_NOSUPPORT;
#  endif
        }
    } while (0);

    if (SID_ERROR_NONE == err)
    {
        SID_PAL_LOG_INFO(MFG_STORE_LOG_PREFIX "successfully stored record %u", value);
    }

    return (int32_t)err;
#else
    return (int32_t)SID_ERROR_NOSUPPORT;
#endif
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_pal_mfg_store_read(uint16_t value, uint8_t * buffer, uint16_t length)
{
    bool record_found = false;

    do
    {
        if (false == mfg_store_init_done)
        {
            SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "cannot read storage, MFG is not initialized yet");
            break;
        }

#if ENABLE_MFG_STORE_OVERWRITE_READ
        if ((stm_mfg_store_region.overwrite_read != NULL) && (stm_mfg_store_region.overwrite_read(value, buffer, length) != false))
        {
            /* Read override was applied, skip reading out the actual value */
            break;
        }
#endif /* ENABLE_MFG_STORE_OVERWRITE_READ */

        const uint32_t mfg_storage_version = sid_pal_mfg_store_get_version();

        switch (mfg_storage_version)
        {
            case SID_PAL_MFG_STORE_TLV_VERSION:
                {
                    /* The SID_PAL_MFG_STORE_VERSION we should read as fixed offset */
                    if (SID_PAL_MFG_STORE_VERSION == value)
                    {
                        const void * const version_address = (void *)(stm_mfg_store_region.addr_start + SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE);

                        SID_STM32_UTIL_fast_memcpy(buffer, version_address, length);
                        break;
                    }

                    /* Perform regular TLV search and read */
                    mfg_store_tlv_info_t tlv_info;
                    if (sid_pal_mfg_store_search_for_tag(value, &tlv_info) == false)
                    {
                        SID_PAL_LOG_WARNING(MFG_STORE_LOG_PREFIX "read failed, tag 0x%04X not found in MFG storage", value);
                        break;
                    }

                    /* Copy record data */
                    const size_t copy_length = tlv_info.length >= length ? length : tlv_info.length; /* Limit readout length to a single record */
                    if (copy_length != length)
                    {
                        SID_PAL_LOG_WARNING(MFG_STORE_LOG_PREFIX "invalid read length requested while reading tag 0x%04X. Record length: %u, requested length: %u", value, copy_length, length);
                    }
                    SID_STM32_UTIL_fast_memcpy(buffer,
                                               (void *)(stm_mfg_store_region.addr_start + tlv_info.offset + MFG_STORE_TLV_HEADER_SIZE),
                                               copy_length);
                    record_found = true;
                }
                break;

            case SID_PAL_MFG_STORE_FIXED_OFFSETS_VERSION:
                {
#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
                    const void * const value_address = value_to_address(value, stm_mfg_store_region.addr_start, stm_mfg_store_region.addr_end);

                    if (value_address != NULL)
                    {
                        SID_STM32_UTIL_fast_memcpy(buffer, value_address, length);
                        record_found = true;
                    }
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */
                }
                break;

            default:
                SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "can't read tag 0x%04X - unknown MFG storage format (%u)", value, mfg_storage_version);
                break;
        }
    } while (0);

    if (false == record_found)
    {
        /* Fill-in the buffer with safe values if reading failed */
        SID_STM32_UTIL_fast_memset(buffer, 0xFFu, length);
        SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "can't read tag 0x%04X - not found in MFG storage", value);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint16_t sid_pal_mfg_store_get_length_for_value(uint16_t value)
{
    uint16_t length = 0u;
    if (sid_pal_mfg_store_get_version() == SID_PAL_MFG_STORE_TLV_VERSION)
    {
        mfg_store_tlv_info_t tlv_info;
        if (sid_pal_mfg_store_search_for_tag(value, &tlv_info))
        {
            length = tlv_info.length;
        }
    }
    else
    {
#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
        length = value_to_size(value);
#else
        SID_PAL_ASSERT(false);
#endif
    }
    return length;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_mfg_store_erase(void)
{
#ifdef ENABLE_MFG_STORE_WRITE
    sid_error_t err;
    FD_FlashOp_Status_t fd_err;

    do
    {
        const uint32_t mfg_size   = stm_mfg_store_region.addr_end - stm_mfg_store_region.addr_start;
        const uint32_t start_page = (stm_mfg_store_region.addr_start - FLASH_BASE_NS) / FLASH_PAGE_SIZE; /* Round down to the nearest page start */
        const uint32_t num_pages  = (mfg_size + (FLASH_PAGE_SIZE - 1u)) / FLASH_PAGE_SIZE; /* Round up to the nearest whole number of pages */

        if (false == mfg_store_init_done)
        {
            SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "cannot erase storage, MFG is not initialized yet");
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        if ((mfg_size % FLASH_PAGE_SIZE != 0u) || ((stm_mfg_store_region.addr_start & (FLASH_PAGE_SIZE - 1u)) != 0u))
        {
            /**
             * NOTE: Erase is only supported on a per-page basis. If the user configures the manufacturing
             * store so that it partially uses a page, that ENTIRE page will be erased here.
             */
            SID_PAL_LOG_WARNING(MFG_STORE_LOG_PREFIX "erasing entire contents of all pages which contain MFG data");
        }

        SID_PAL_LOG_INFO(MFG_STORE_LOG_PREFIX "erasing flash. Starting page: %u, pages to erase: %u", start_page, num_pages);

        /* Unlock the flash to allow erase operations */
        if (HAL_FLASH_Unlock() != HAL_OK)
        {
            SID_PAL_LOG_ERROR(MFG_STORE_LOG_PREFIX "failed to unlock flash for erasing");
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Erase manufacturing storage area */
        for (uint32_t i = 0u; i < num_pages; i++)
        {
            uint32_t erase_attempt_counter = 0u;

            /* Flash driver erases only one page at a time */
            SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "erasing flash page %u...", start_page + i);

            /* Erasing page with retries */
            do
            {
                /* Clear flash error flags before proceeding */
                __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

                /* try to erase the page */
                fd_err = FD_EraseSectors(start_page + i);

                if (fd_err != FD_FLASHOP_SUCCESS)
                {
                    erase_attempt_counter++;
                    SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "failed to erase flash page %u. Attempt #%u", start_page + i, erase_attempt_counter);
                }
            } while ((fd_err != FD_FLASHOP_SUCCESS) && (erase_attempt_counter < MFG_STORE_PAGE_ERASE_RETRY_LIMIT));

            /* CHeck for permanent failure */
            if (fd_err != FD_FLASHOP_SUCCESS)
            {
                SID_PAL_LOG_DEBUG(MFG_STORE_LOG_PREFIX "failed to erase flash page %u. Retry limit reached", start_page + i);
                err = SID_ERROR_STORAGE_ERASE_FAIL;
                break;
            }
        }

        /* Lock the flash after completion. This is a required step even if erase failed */
        if (HAL_FLASH_Lock() != HAL_OK)
        {
            SID_PAL_LOG_WARNING(MFG_STORE_LOG_PREFIX "failed to lock flash after erasing");
            err = SID_ERROR_IO_ERROR;
            break;
        }

        if (FD_FLASHOP_SUCCESS == fd_err)
        {
            SID_PAL_LOG_INFO(MFG_STORE_LOG_PREFIX "storage erased successfully");
            err = SID_ERROR_NONE;
        }
    } while (0);

    return (int32_t)err;
#else
    return SID_ERROR_NOSUPPORT;
#endif
}

/*----------------------------------------------------------------------------*/

#ifdef ENABLE_MFG_STORE_WRITE
SID_STM32_SPEED_OPTIMIZED bool sid_pal_mfg_store_is_empty(void)
{
    const uint32_t EMPTY_WORD = 0xFFFFFFFFu;

    SID_PAL_ASSERT((stm_mfg_store_region.addr_start % sizeof(EMPTY_WORD)) == 0u);
    SID_PAL_ASSERT((stm_mfg_store_region.addr_end % sizeof(EMPTY_WORD)) == 0u);

    for (uint32_t addr = stm_mfg_store_region.addr_start; addr < stm_mfg_store_region.addr_end; addr += sizeof(EMPTY_WORD))
    {
        const uint32_t * const p = (const uint32_t * const)addr;
        if (*p != EMPTY_WORD)
        {
            return false;
        }
    }

    return true;
}
#endif /* ENABLE_MFG_STORE_WRITE */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED bool sid_pal_mfg_store_is_tlv_support(void)
{
    return true;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_mfg_store_get_version(void)
{
    static_assert(SID_PAL_MFG_STORE_VERSION_SIZE == sizeof(uint32_t));
    const uint32_t * const version_address = (uint32_t *)(void *)(stm_mfg_store_region.addr_start + SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE);

    /* Assuming that we keep this behavior for both 1P & 3P */
    const uint32_t version = sid_ntohl(*version_address);

    return version;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED bool sid_pal_mfg_store_dev_id_get(uint8_t dev_id[SID_PAL_MFG_STORE_DEVID_SIZE])
{
    bool error_code = false;
    uint8_t buffer[] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};

    static_assert(sizeof(buffer) == SID_PAL_MFG_STORE_DEVID_SIZE, "dev ID buffer wrong size");

    sid_pal_mfg_store_read(SID_PAL_MFG_STORE_DEVID,
                            buffer, SID_PAL_MFG_STORE_DEVID_SIZE);

    static const uint8_t UNSET_DEV_ID[] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
    static_assert(sizeof(UNSET_DEV_ID) == SID_PAL_MFG_STORE_DEVID_SIZE, "Unset dev ID wrong size");
    if (SID_STM32_UTIL_fast_memcmp(buffer, UNSET_DEV_ID, SID_PAL_MFG_STORE_DEVID_SIZE) == 0)
    {
        const uint32_t udn = LL_FLASH_GetUDN(); /* The 32-bit unique device number is a sequential number, different for each individual device. */

        buffer[0] = 0xBFu;
        buffer[1] = 0xFFu;
        buffer[2] = (uint8_t)((udn >> 16) & 0xFFu);
        buffer[3] = (uint8_t)((udn >> 8) & 0xFFu);
        buffer[4] = (uint8_t)(udn & 0xFFu);
    }
    else
    {
        const uint32_t version = sid_pal_mfg_store_get_version();
        if ((MFG_VERSION_1_VAL == version) || (0x1u == version))
        {
            /**
             * Correct dev_id for mfg version 1
             * For devices with mfg version 1, the device Id is stored as two words
             * in network endian format.
             * To read the device Id two words at SID_PAL_MFG_STORE_DEVID has to be
             * read and each word needs to be changed to host endian format.
             */
            uint8_t dev_id_buffer[MFG_STORE_DEVID_V1_SIZE];
            uint32_t val = 0u;
            sid_pal_mfg_store_read(SID_PAL_MFG_STORE_DEVID, dev_id_buffer, MFG_STORE_DEVID_V1_SIZE);
            SID_STM32_UTIL_fast_memcpy(&val, &dev_id_buffer[0], sizeof(val));
            val = sid_ntohl(val);
            SID_STM32_UTIL_fast_memcpy(&dev_id_buffer[0], &val, sizeof(val));
            SID_STM32_UTIL_fast_memcpy(&val, &dev_id_buffer[sizeof(val)], sizeof(val));
            val = sid_ntohl(val);
            SID_STM32_UTIL_fast_memcpy(&dev_id_buffer[sizeof(val)], &val, sizeof(val));
            /* Encode the size in the first 3 bits in MSB of the devId */
            dev_id_buffer[0] = (dev_id_buffer[0] & DEV_ID_MSB_MASK) | ENCODED_DEV_ID_SIZE_5_BYTES_MASK;
            SID_STM32_UTIL_fast_memcpy(buffer, dev_id_buffer, SID_PAL_MFG_STORE_DEVID_SIZE);
        }
        error_code = true;
    }

    SID_STM32_UTIL_fast_memcpy(dev_id, buffer, SID_PAL_MFG_STORE_DEVID_SIZE);
    return error_code;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED bool sid_pal_mfg_store_serial_num_get(uint8_t serial_num[SID_PAL_MFG_STORE_SERIAL_NUM_SIZE])
{
    uint32_t buffer[((SID_PAL_MFG_STORE_SERIAL_NUM_SIZE + (MFG_WORD_SIZE - 1u)) / MFG_WORD_SIZE) * ((MFG_WORD_SIZE + (sizeof(uint32_t) - 1u)) / sizeof(uint32_t))];

    sid_pal_mfg_store_read(SID_PAL_MFG_STORE_SERIAL_NUM,
                            (uint8_t*)buffer, SID_PAL_MFG_STORE_SERIAL_NUM_SIZE);

    static const uint8_t UNSET_SERIAL_NUM[] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
                                               0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
    static_assert(sizeof(UNSET_SERIAL_NUM) == SID_PAL_MFG_STORE_SERIAL_NUM_SIZE, "Unset serial num wrong size");
    if(SID_STM32_UTIL_fast_memcmp(buffer, UNSET_SERIAL_NUM, SID_PAL_MFG_STORE_SERIAL_NUM_SIZE) == 0u)
    {
        return false;
    }

    const uint32_t version = sid_pal_mfg_store_get_version();

    if ((MFG_VERSION_1_VAL == version) || (0x1u == version))
    {
        for (size_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(buffer); ++i)
        {
            buffer[i] = sid_ntohl(buffer[i]);
        }
    }

    SID_STM32_UTIL_fast_memcpy(serial_num, buffer, SID_PAL_MFG_STORE_SERIAL_NUM_SIZE);
    return true;
}

/*----------------------------------------------------------------------------*/

#if SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED
SID_STM32_SPEED_OPTIMIZED void sid_pal_mfg_store_apid_get(uint8_t apid[SID_PAL_MFG_STORE_APID_SIZE])
{
    SID_STM32_UTIL_fast_memcpy(apid, product_apid, sizeof(product_apid));

}
#endif /* SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

/*----------------------------------------------------------------------------*/

#if SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED
SID_STM32_SPEED_OPTIMIZED void sid_pal_mfg_store_app_pub_key_get(uint8_t app_pub[SID_PAL_MFG_STORE_APP_PUB_ED25519_SIZE])
{
    SID_STM32_UTIL_fast_memcpy(app_pub, app_server_public_key, sizeof(app_server_public_key));
}
#endif /* SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */
