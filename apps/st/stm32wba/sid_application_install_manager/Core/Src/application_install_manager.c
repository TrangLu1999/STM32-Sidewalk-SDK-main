/**
  ******************************************************************************
  * @file    application_install_manager.c
  * @brief   Application Install Manager for Sidewalk applications
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

#include "main.h"
#include "application_install_manager.h"
#include "application_verification_key.h"
#include SID_APP_VERSION_HEADER

/* AIM-Application Interfaces */
#include <sid_application_install_manager_ifc.h>

/* Platform-specific Headers */
#include <cmsis_compiler.h>
#include <stm32wbaxx_hal.h>
#include <stm32wbaxx_ll_icache.h>

/* Crypto library */
#include <cmox_crypto.h>

/* Utilities */
#include "sid_pal_log_like.h"
#include <sid_stm32_common_utils.h>
#include <stm32_mcu_info.h>
#include <timer_if.h>

/* Private defines -----------------------------------------------------------*/

/* PLL1 configuration to be used by AIM */
#define AIM_PLL1_CFG_CLOCK_SOURCE           (LL_RCC_PLL1SOURCE_HSI)
#define AIM_PLL1_CFG_PLLM_DIVIDER           (3u)
#define AIM_PLL1_CFG_PLLN_MULTIPLIER        (36u)
#define AIM_PLL1_CFG_PLLR_DIVIDER           (2u)
#define AIM_PLL1_CFG_TARGET_PLLR_FREQUENCY  (96000000u)

/* The list of reboot reasons */
#define REBOOT_ON_FW_APP                    (0x00)
#define REBOOT_ON_APP_INSTALL_MNGR          (0x01)

#define ECC_ERROR_ADDRESS_INVALID           (0u)         /*!< A value indicating there's no active ECC uncorrectable error detected at any location */

#define AIM_CMOX_WORKING_BUF_SIZE           (1u * 1024u) /*!< Size (in bytes) of the working buffer for CMOX library */

#define AIM_FLASH_LINE_SIZE                 (16u)        /*!< Flash can be written only in the multiplies of 16 bytes on STM32WBA platform */

#define AIM_BLOCK_BACKUP_RETRY_LIMIT        (5u)         /*!< Retry limit for the block (flash page) backup operation. If the backup of a block fails for this amount of times in a row, the failure is considered permanent */
#define AIM_BLOCK_UPDATE_RETRY_LIMIT        (5u)         /*!< Retry limit for the block (flash page) update operation. If the update of a block fails for this amount of times in a row, the failure is considered permanent */

/* App confirmation config */
#define AIM_APP_CONFIRMATION_ATTEMPT_LIMIT  (5u)         /*!< Maximum number of boots for the user app to set validity marker. AIM increments the counter on every boot of the app. If the limit is reached, a rollback is enforced */
#define AIM_ATTEMPT_STATUS_VAL_PER_LINE     (2u)         /*!< The number of counts a flash line can hold. This is hard-coded to 2 on STM32WBA because every flash line is ECC-protected and can be modified only twice (1 - write any value; 2 - write all 0s) */
#define AIM_ATTEMPT_STATUS_BLOCK_SIZE       ((AIM_APP_CONFIRMATION_ATTEMPT_LIMIT + (AIM_ATTEMPT_STATUS_VAL_PER_LINE - 1u)) / AIM_ATTEMPT_STATUS_VAL_PER_LINE) /*!< The amount of flash lines required to keep up to AIM_APP_CONFIRMATION_ATTEMPT_LIMIT boot records */

#define AIM_ACTIVE_IMAGE_BLOCK_NUM_OFFSET   (16u)
#define AIM_ACTIVE_IMAGE_BLOCK_NUM_MASK     (0xFFFFu << AIM_ACTIVE_IMAGE_BLOCK_NUM_OFFSET)
#define AIM_INSTALL_IMAGE_BLOCK_NUM_OFFSET  (0u)
#define AIM_INSTALL_IMAGE_BLOCK_NUM_MASK    (0xFFFFu << AIM_INSTALL_IMAGE_BLOCK_NUM_OFFSET)

extern uint32_t __AIM_FLASH_region_start__;
extern uint32_t __AIM_FLASH_region_size__;
extern uint32_t __AIM_FLASH_region_end__;
#define AIM_FLASH_START                     ((uint32_t)&__AIM_FLASH_region_start__)
#define AIM_FLASH_SIZE                      ((uint32_t)&__AIM_FLASH_region_size__)
#define AIM_FLASH_END                       ((uint32_t)&__AIM_FLASH_region_end__)

/* Private macros ------------------------------------------------------------*/

#ifndef MAX
#  define MAX(a, b)                         (((a) > (b)) ? (a) : (b))
#endif /* MAX */

/* Private typedef -----------------------------------------------------------*/

typedef __PACKED_UNION {
    __PACKED_STRUCT {
        uint8_t build;
        uint8_t patch;
        uint8_t minor;
        uint8_t major;
    };
    uint32_t raw;
} hal_version_info_t;

/**
 * @brief Key values to select a boot mode
 *
 * @note Using high-distance values to eliminate chances of an accidental key change by a bit flip
 */
typedef enum {
    AIMBM_START_ACTIVE_APP            = 0xBA5ECA11u,
    AIMBM_INSTALL_UPDATE              = 0xC0FF1A7Eu,
    AIMBM_ROLL_BACK                   = 0xABAD1DEAu,
} aim_boot_mode_t;

typedef enum {
    AIMFWVS_UNKNOWN                   = 0u,
    AIMFWVS_FOOTER_ADDRESS_INVALID    = 1u,
    AIMFWVS_MAGIC_WORD_INVALID        = 2u,
    AIMFWVS_CRC_MISMATCH              = 3u,
    AIMFWVS_VERIFICATION_KEY_INVALID  = 4u,
    AIMFWVS_SIGNATURE_MISMATCH        = 5u,
    AIMFWVS_UNCORRECTABLE_ECC         = 6u,
    AIMFWVS_VALID                     = 0xCAu,
} aim_fw_validity_state_t;

typedef enum {
    AIMFWIT_UPDATE                    = 0x600DDA7Au,
    AIMFWIT_ROLL_BACK                 = 0xFEE1DEADu,
} aim_fw_image_type_t;

typedef enum {
    AIMFAA_INSTALL_METADATA           = 0xB105F00Du,
    AIMFAA_ACTIVE_APP_SLOT            = 0xC0DEC0DEu,
    AIMFAA_STAGING_SLOT               = 0xC0FFEE00u,
    AIMFAA_BACKUP_SLOT                = 0x5AFEB007u,
} aim_flash_access_area_t;

typedef enum {
    AIMIS_INVALID                     = 0u, /*!< Special value to indicate that the update state is not known */
    AIMIS_IDLE                        = 1u,
    AIMIS_UPDATE_ONGOING              = 2u,
    AIMIS_ROLLBACK_ONGOING            = 3u,
    AIMIS_UPDATE_CONFIRMATION_PENDING = 4u,
    AIMIS_FINISHED                    = 5u,
    AIMIS_ROLLBACK_REQUIRED           = 6u,
} aim_install_state_t;

typedef enum {
    AIMIBS_INVALID                    = 0u, /*!< Special value to indicate that the update state is not known */
    AIMIBS_NOT_PROCESSED              = 1u,
    AIMIBS_BACKUP_COMPLETED           = 2u,
    AIMIBS_PROCESSED                  = 3u,
} aim_image_block_state_t;

typedef uint32_t aim_flash_line_t[AIM_FLASH_LINE_SIZE / sizeof(uint32_t)];

typedef struct {
    aim_flash_line_t status_word_0;
    aim_flash_line_t status_word_1;
} aim_flash_install_state_storage_t;

typedef struct {
    aim_flash_install_state_storage_t install_state;
    aim_flash_line_t                  confirmation_boot_counter[AIM_ATTEMPT_STATUS_BLOCK_SIZE];
    aim_flash_line_t                  block_states[128];
} aim_flash_install_metadata_storage_t;

typedef enum {
    AIM_ERROR_NONE                      =  0u,
    AIM_ERROR_GENERIC                   =  1u,
    AIM_ERROR_INVALID_ARGS              =  2u,
    AIM_ERROR_INVALID_STATE             =  3u,
    AIM_ERROR_INVALID_STATE_TRANSITION  =  4u,
    AIM_ERROR_UNEXPECTED_DATA           =  5u,
    AIM_ERROR_UNKNOWN_SLOT_ID           =  6u,
    AIM_ERROR_UNKNOWN_IMAGE_TYPE        =  7u,
    AIM_ERROR_BLOCK_INDEX_OUT_OF_BOUNDS =  8u,
    AIM_ERROR_FLASH_PAGE_ERASE_FAIL     =  9u,
    AIM_ERROR_FLASH_WRITE_FAIL          = 10u,
    AIM_ERROR_FLASH_UNCORRECTABLE_ECC   = 11u,
    AIM_ERROR_FLASH_DATA_MISMATCH       = 12u,
    AIM_ERROR_FLASH_DATA_IDENTICAL      = 13u,
    AIM_ERROR_FLASH_NO_FREE_SPACE       = 14u,
} aim_error_t;

/* Imported variables --------------------------------------------------------*/

extern CRC_HandleTypeDef hcrc;

/* Global variables ----------------------------------------------------------*/

volatile uint32_t app_flash_ecc_error_address;
volatile uint32_t aim_ota_status_ecc_error_address;

/* Private variables ---------------------------------------------------------*/

static uint32_t rcc_reset_flags;

/* CMOX crypto resources */
static cmox_ecc_handle_t cmox_ecc_ctx;
SID_STM32_ALIGN_4BYTES(static uint8_t cmox_working_buffer[AIM_CMOX_WORKING_BUF_SIZE]);

/* Private constants ---------------------------------------------------------*/

UTIL_PLACE_IN_SECTION(".ota_status_area")
SID_STM32_ALIGN_16BYTES(static aim_flash_install_metadata_storage_t install_metadata);

static const uint32_t         aim_state_update_ongoing_magic_word[3]   = { 0x3AC5F27Bu, 0xC6B19D4Eu, 0x9E47A2D1u, };
static const uint32_t         aim_state_rollback_ongoing_magic_word[3] = { 0x5B8C7034u, 0xE1D39AF2u, 0x7C4E1B89u, };
static const aim_flash_line_t aim_state_finished_magic_word            = { 0xA92DF65Cu, 0xD0473BA1u, 0x26FA9CDEu, 0x8BF01467u, };

static const aim_flash_line_t block_state_backup_completed_magic_word  = { 0x74B9D53Eu, 0x1CE6A4F9u, 0xE87B29C4u, 0x5DA03E97u, };

static const aim_flash_line_t conf_boot_counter_inc_magic_word         = { 0x3E7A91C4u, 0xA5D2F9B8u, 0x7C14E603u, 0xD8B43FA1u, };

static const aim_flash_line_t aim_app_validity_magic_word              = SID_AIM_IFC_VALIDITY_MARKER_MAGIC_WORD_INITIALIZER;

/* Private function prototypes -----------------------------------------------*/

static void                    switch_to_pll_clock(void);
static void                    switch_to_hsi_clock(void);
static void                    invalidate_icache(void);
static uint32_t                compute_crc32(const uint8_t * const data, const uint32_t length);
static aim_fw_validity_state_t check_fw_image_validity(const aim_boot_mode_t fw_selection, uint32_t * const out_image_blocks_count);
static uint32_t                is_flash_line_all_1s(const aim_flash_line_t line);
static uint32_t                is_flash_line_all_0s(const aim_flash_line_t line);
static aim_error_t             is_flash_block_empty(const uint32_t block_idx, const aim_flash_access_area_t flash_area_id, uint32_t * const out_is_empty);
static aim_error_t             erase_flash_pages(const uint32_t start_block_idx, const uint32_t num_pages_to_erase, const aim_flash_access_area_t flash_area_id);
static aim_error_t             read_confirmation_boot_counter(uint32_t * const out_boot_counter);
static aim_error_t             increment_confirmation_boot_counter(void);
static aim_error_t             read_confirmation_state(sid_aim_app_validity_state_t * const out_validity_confirmed);
static aim_error_t             read_image_block_state(const uint32_t block_idx, aim_image_block_state_t * const out_state);
static aim_error_t             estimate_image_block_state(const uint32_t block_idx, const aim_fw_image_type_t image_type, aim_image_block_state_t * const out_state);
static aim_error_t             estimate_image_block_state_from_contents(const uint32_t block_idx, aim_image_block_state_t * const out_state);
static aim_error_t             store_image_block_state(const uint32_t block_idx, const aim_image_block_state_t new_state);
static aim_error_t             get_processed_image_blocks_count(const aim_fw_image_type_t image_type, uint32_t * const out_processed_blocks);
static aim_error_t             copy_image_block(const uint32_t block_idx, const aim_flash_access_area_t dst_flash_area_id, const aim_flash_access_area_t src_flash_area_id, const uint32_t skip_copy_offset, const uint32_t skip_copy_size);
static aim_error_t             compare_image_block(const uint32_t block_idx, const aim_flash_access_area_t flash_area_a, const aim_flash_access_area_t flash_area_b, const uint32_t skip_compare_offset, const uint32_t skip_compare_size);
static aim_error_t             get_first_mismatch_line_in_block(const uint32_t block_idx, const aim_flash_access_area_t flash_area_a, const aim_flash_access_area_t flash_area_b, uint32_t * const out_line_offset);
static aim_error_t             read_install_state(aim_install_state_t * const out_state, uint32_t * const out_active_image_size_pages, uint32_t * const install_image_size_pages);
static aim_error_t             store_install_state(const aim_install_state_t state, const uint32_t active_image_size_pages, const uint32_t install_image_size_pages);
static aim_error_t             install_image(const aim_fw_image_type_t image_type, const uint32_t active_image_size_pages, const uint32_t install_image_size_pages);
static aim_fw_validity_state_t check_and_install_update_image(const uint32_t active_app_blocks_to_backup);
static aim_fw_validity_state_t check_and_install_rollback_image(void);
static void                    enforce_rollback(void);
static void                    jump_selection_on_power_up(void);
static void                    user_app_pre_entry_actions(void);

/* Private function definitions ----------------------------------------------*/

static void switch_to_pll_clock(void)
{
    /* Ensure PLL is disabled and voltage regulator is properly configured for high-frequency operation */
    assert_param(LL_RCC_PLL1_IsReady() == 0u);
    assert_param(LL_PWR_GetRegulVoltageScaling() == LL_PWR_REGU_VOLTAGE_SCALE1);

    /* Configure PLL parameters */
    LL_RCC_PLL1_ConfigDomain_PLL1R(AIM_PLL1_CFG_CLOCK_SOURCE, AIM_PLL1_CFG_PLLM_DIVIDER, AIM_PLL1_CFG_PLLN_MULTIPLIER, AIM_PLL1_CFG_PLLR_DIVIDER);

    /* Start PLL and wait for it to lock */
    LL_RCC_PLL1_Enable();
    while (LL_RCC_PLL1_IsReady() == 0u)
    {
        __NOP();
    }

    /* Enable PLL output to SysClock */
    LL_RCC_PLL1_EnableDomain_PLL1R();

    /* Adjust flash wait states */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
    while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_3)
    {
        __NOP();
    }

    /* Adjust AHB and APB prescalers to keep up with the PLL clock frequency */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHB5Divider(LL_RCC_AHB5_DIVIDER_1);
    LL_RCC_SetAHB5Prescaler(LL_RCC_AHB5_DIV_3);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetAPB7Prescaler(LL_RCC_APB7_DIV_1);

    /* Initiate switch to PLL clock */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1R);

    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1R)
    {
        __NOP();
    }

    /* Set RAM wait states to 0 after switching to 96MHz PLL clock */
    RAMCFG_SRAM1->CR = RAMCFG_WAITSTATE_0;
    RAMCFG_SRAM2->CR = RAMCFG_WAITSTATE_0;

    /* Store new SysClock frequency */
    LL_SetSystemCoreClock(AIM_PLL1_CFG_TARGET_PLLR_FREQUENCY);

#if (CFG_LOG_SUPPORTED)
    /* Re-apply UART config to restore the original baud rate after APB clock change */
    if (LOG_UART_HANDLER.gState != HAL_UART_STATE_RESET)
    {
        UART_SetConfig(&LOG_UART_HANDLER);
    }
#endif /* CFG_LOG_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

static void switch_to_hsi_clock(void)
{
    /* Ensure HSI is ok. Normally this shall not be an issue since PLL is configured with HSI as source clock */
    assert_param(LL_RCC_HSI_IsReady() != 0u);

    /* Set RAM wait states to 1 to support 16MHz SysCLock */
    RAMCFG_SRAM1->CR = RAMCFG_WAITSTATE_1;
    RAMCFG_SRAM2->CR = RAMCFG_WAITSTATE_1;

    /* Initiate switch to HSI clock */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
    {
        __NOP();
    }

    /* Disable PLL output to SysClock */
    LL_RCC_PLL1_DisableDomain_PLL1R();

     /* Stop PLL */
    LL_RCC_PLL1_Disable();
    while (LL_RCC_PLL1_IsReady() != 0u)
    {
        __NOP();
    }

    /* Adjust flash wait states */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
    while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
    {
        __NOP();
    }

    /* Adjust AHB and APB prescalers to match 16MHz SysClock */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHB5Divider(LL_RCC_AHB5_DIVIDER_1);
    LL_RCC_SetAHB5Prescaler(LL_RCC_AHB5_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetAPB7Prescaler(LL_RCC_APB7_DIV_1);

    /* Store new SysClock frequency */
    LL_SetSystemCoreClock(HSI_VALUE);

#if (CFG_LOG_SUPPORTED)
    /* Re-apply UART config to restore the original baud rate after APB clock change */
    if (LOG_UART_HANDLER.gState != HAL_UART_STATE_RESET)
    {
        UART_SetConfig(&LOG_UART_HANDLER);
    }
#endif /* CFG_LOG_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

static void invalidate_icache(void)
{
    /* Check if no ongoing operation */
    if (LL_ICACHE_IsActiveFlag_BUSY() == 0u)
    {
        /* Launch cache invalidation */
        LL_ICACHE_Invalidate();
    }

    /* Check if ongoing invalidation operation */
    if (LL_ICACHE_IsActiveFlag_BUSY() != 0u)
    {
        /* Wait for end of cache invalidation */
        while (LL_ICACHE_IsActiveFlag_BSYEND() == 0u)
        {
            __NOP();
        }
    }

    /* Clear BSYENDF */
    LL_ICACHE_ClearFlag_BSYEND();
}

/*----------------------------------------------------------------------------*/

static uint32_t compute_crc32(const uint8_t * const data, const uint32_t length)
{
    uint32_t crc;

    assert_param(length % sizeof(uint32_t) == 0u);

    /* Ensure the correct CRC config is applied since CMOX library may alter it */
    (void)HAL_CRC_Init(&hcrc);

    /* Compute IEEE 802.3 (Ethernet) CRC32 value */
    crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)(void *)data, length / sizeof(uint32_t));
    crc = ~crc; /* Equivalent to the final crc ^ 0xFFFFFFFFu */

    return crc;
}

/*----------------------------------------------------------------------------*/

static aim_fw_validity_state_t check_fw_image_validity(const aim_boot_mode_t fw_selection, uint32_t * const out_image_blocks_count)
{
    aim_fw_validity_state_t status = AIMFWVS_UNKNOWN;
    uint32_t slot_start_address;
    uint32_t slot_end_address;
    uint32_t fw_image_footer_offset;
    const sid_aim_ota_header_t * fw_image_header;
    const sid_aim_ota_footer_t * fw_image_footer;
    const char * fw_image_name;

    do
    {
        switch (fw_selection)
        {
            case AIMBM_INSTALL_UPDATE:
                slot_start_address     = APP_CONFIG_AIM_STAGING_SLOT_START;
                slot_end_address       = APP_CONFIG_AIM_STAGING_SLOT_END;
                fw_image_header        = (const sid_aim_ota_header_t *)(void *)APP_CONFIG_AIM_STAGING_SLOT_HEADER_START;
                fw_image_footer_offset = APP_CONFIG_AIM_STAGING_SLOT_START - APP_CONFIG_AIM_ACTIVE_SLOT_START;
                fw_image_name          = "update";
                break;

            case AIMBM_ROLL_BACK:
                slot_start_address     = APP_CONFIG_AIM_ROLLBACK_SLOT_START;
                slot_end_address       = APP_CONFIG_AIM_ROLLBACK_SLOT_END;
                fw_image_header        = (const sid_aim_ota_header_t *)(void *)APP_CONFIG_AIM_ROLLBACK_SLOT_HEADER_START;
                fw_image_footer_offset = APP_CONFIG_AIM_ROLLBACK_SLOT_START - APP_CONFIG_AIM_ACTIVE_SLOT_START;
                fw_image_name          = "rollback";
                break;

            case AIMBM_START_ACTIVE_APP:
            default:
                slot_start_address     = APP_CONFIG_AIM_ACTIVE_SLOT_START;
                slot_end_address       = APP_CONFIG_AIM_ACTIVE_SLOT_END;
                fw_image_header        = (const sid_aim_ota_header_t *)(void *)APP_CONFIG_AIM_ACTIVE_SLOT_HEADER_START;
                fw_image_footer_offset = 0u;
                fw_image_name          = "active";
                break;
        }

        SID_PAL_LOG_INFO("Verifying %s FW image...", fw_image_name);

        /* Check if the image footer address stored in the image header belongs to the current slot */
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB(); /* Reset ECC error detection mechanism */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds" /* GCC 13 struggles with analyzing pointer arithmetic involving _PACKED_STRUCT correctly */
        if ((((uint32_t)(void *)fw_image_header->ota_footer_location + fw_image_footer_offset) < slot_start_address)
         || (((uint32_t)(void *)fw_image_header->ota_footer_location + fw_image_footer_offset) > slot_end_address)
         || (((uint32_t)(void *)fw_image_header->ota_footer_location + fw_image_footer_offset + sizeof(*fw_image_footer)) > slot_end_address))
#pragma GCC diagnostic pop
        {
            /* Check for uncorrectable ECC errors first as invalid footer address can be just a consequence */
            if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
            {
                SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
                status = AIMFWVS_UNCORRECTABLE_ECC;
                break;
            }

            /* Report footer address error */
            SID_PAL_LOG_ERROR("Invalid image footer address in the header: 0x%08X", fw_image_header->ota_footer_location);
            status = AIMFWVS_FOOTER_ADDRESS_INVALID;
            break;
        }
        /* Footer address is ok and can be used */
        fw_image_footer = (const sid_aim_ota_footer_t *)(void *)((uint32_t)(void *)fw_image_header->ota_footer_location + fw_image_footer_offset);
        SID_PAL_LOG_INFO("Image footer is located at 0x%08X", fw_image_footer);

        /* Check the magic word in the footer */
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB(); /* Reset ECC error detection mechanism */
        if (fw_image_footer->magic_word != SID_AIM_IFC_IMAGE_VALIDITY_MAGIC_WORD)
        {
            /* Check for uncorrectable ECC errors first as invalid magic word can be just a consequence */
            if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
            {
                SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
                status = AIMFWVS_UNCORRECTABLE_ECC;
                break;
            }

            /* Report the magic word is invalid */
            SID_PAL_LOG_ERROR("Incorrect magic word in the image footer: 0x%08X", fw_image_footer->magic_word);
            status = AIMFWVS_MAGIC_WORD_INVALID;
            break;
        }
        /* Magic word is ok, can proceed with the CRC check */
        SID_PAL_LOG_INFO("Detected a valid magic word in the image footer");

        /* CRC check */
        SID_PAL_LOG_INFO("Performing image CRC check...");
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB(); /* Reset ECC error detection mechanism */
        const uint32_t fw_image_crc_covered_area_start_addr = slot_start_address;
        const uint32_t fw_image_crc_covered_area_end_addr   = (uint32_t)(void *)&fw_image_footer->checksum;
        const uint32_t fw_image_crc_covered_area_size_bytes = fw_image_crc_covered_area_end_addr - fw_image_crc_covered_area_start_addr;

        /* Protect from systematic SW failures */
        assert_param(fw_image_crc_covered_area_start_addr % sizeof(uint32_t) == 0u);
        assert_param(fw_image_crc_covered_area_end_addr % sizeof(uint32_t) == 0u);
        assert_param(fw_image_crc_covered_area_size_bytes % sizeof(uint32_t) == 0u);

        /* Compute CRC */
        const uint32_t computed_crc = compute_crc32((uint8_t *)(void *)fw_image_crc_covered_area_start_addr, fw_image_crc_covered_area_size_bytes);

        /* Check for uncorrectable ECC errors first as CRC mismatch can be just a consequence */
        if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
            status = AIMFWVS_UNCORRECTABLE_ECC;
            break;
        }

        /* Check the CRC results */
        if (computed_crc != fw_image_footer->checksum)
        {

            /* This is purely a CRC problem */
            SID_PAL_LOG_ERROR("Image CRC mismatch");
            status = AIMFWVS_CRC_MISMATCH;
            break;
        }
        /* CRC is ok */
        SID_PAL_LOG_INFO("CRC checked passed");

        /* Signature check */
        SID_PAL_LOG_INFO("Performing image signature check...");
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB(); /* Reset ECC error detection mechanism */
        const uint32_t fw_image_eddsa_covered_area_start_addr = slot_start_address;
        const uint32_t fw_image_eddsa_covered_area_end_addr   = (uint32_t)(void *)fw_image_footer;
        const uint32_t fw_image_eddsa_covered_area_size_bytes = fw_image_eddsa_covered_area_end_addr - fw_image_eddsa_covered_area_start_addr;

        /* Protect from systematic SW failures */
        assert_param(fw_image_eddsa_covered_area_start_addr % sizeof(uint32_t) == 0u);
        assert_param(fw_image_eddsa_covered_area_end_addr % sizeof(uint32_t) == 0u);
        assert_param(fw_image_eddsa_covered_area_size_bytes % sizeof(uint32_t) == 0u);
        assert_param(sizeof(fw_image_footer->signature) == CMOX_ECC_ED25519_SIG_LEN);

        uint8_t            fw_digest[CMOX_SHA512_SIZE];
        size_t             produced_fw_digest_len;
        cmox_hash_retval_t cmox_hash_retval;

        /**
         * Compute firmware digest since cmox_eddsa_verify() implementation expects the entire message to fit into the working buffer in RAM
         * which is not an option for a firmware image. Using SHA-512 algorithm here since Ed25519 is also based on SHA-512, meaning the SHA
         * functions are already present in the AIM code and using SHA-512 here won't cause any increase in the flash footprint of the AIM.
         */
        cmox_hash_retval = cmox_hash_compute(CMOX_SHA512_ALGO,
                                             (uint8_t *)(void *)fw_image_eddsa_covered_area_start_addr, fw_image_eddsa_covered_area_size_bytes,
                                             fw_digest, sizeof(fw_digest), &produced_fw_digest_len);

        /* Check for uncorrectable ECC errors first as signature mismatch can be just a consequence */
        if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            /**
             * The chances of getting a new ECC during the signature check are pretty low since the entire FW image, including the signature itself,
             * was accessed during the CRC check. However, these chances are non-zero and it's important to keep checking for ECC detections to keep
             * the root cause reporting correct.
             */
            SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
            status = AIMFWVS_UNCORRECTABLE_ECC;
            break;
        }
        if (cmox_hash_retval != CMOX_HASH_SUCCESS)
        {
            /* Report signature mismatch */
            SID_PAL_LOG_ERROR("Failed to compute FW image digest. CMOX error 0x%08X", cmox_hash_retval);
            status = AIMFWVS_CRC_MISMATCH;
            break;
        }

        /* Load the verification key */
        const ed25519_pub_key_t * app_verification_key_ptr = aim_load_application_verification_key();
        if (NULL == app_verification_key_ptr)
        {
            SID_PAL_LOG_ERROR("Unable to load verification key");
            status = AIMFWVS_VERIFICATION_KEY_INVALID;
            break;
        }
        for (uint32_t i = 0u; i < sizeof(*app_verification_key_ptr); i++)
        {
            if (((*app_verification_key_ptr)[i] != 0x00u) && ((*app_verification_key_ptr)[i] != 0xFFu))
            {
                break;
            }
        }

        /* Run EdDSA signature check for the computed firmware image digest */
        cmox_ecc_retval_t cmox_ecc_retval;
        uint32_t          cmox_fault_check; /* Redundant check for successful signature match. This value shall be ignored if cmox_ecc_retval != CMOX_ECC_AUTH_SUCCESS */

        cmox_ecc_construct(&cmox_ecc_ctx, CMOX_ECC256_MATH_FUNCS, cmox_working_buffer, sizeof(cmox_working_buffer));
        cmox_ecc_retval = cmox_eddsa_verify(&cmox_ecc_ctx,                                                  /* CMOX ECC context */
                                            CMOX_ECC_CURVE_ED25519,                                         /* ED25519 ECC curve selected */
                                            *app_verification_key_ptr, sizeof(*app_verification_key_ptr),   /* Public key for verification */
                                            fw_digest, produced_fw_digest_len,                              /* SHA-512 digest of the firmware image */
                                            fw_image_footer->signature, sizeof(fw_image_footer->signature), /* Signature value stored in the OTA footer of the image */
                                            &cmox_fault_check);                                             /* Additional CMOX fault check value, used for higher resilience */
        cmox_ecc_cleanup(&cmox_ecc_ctx);

        /* Release the verification key */
        aim_release_application_verification_key();

        /* Check for uncorrectable ECC errors first as signature mismatch can be just a consequence */
        if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            /**
             * The chances of getting a new ECC during the signature check are pretty low since the entire FW image, including the signature itself,
             * was accessed during the CRC check. However, these chances are non-zero and it's important to keep checking for ECC detections to keep
             * the root cause reporting correct.
             */
            SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
            status = AIMFWVS_UNCORRECTABLE_ECC;
            break;
        }
        if ((cmox_ecc_retval != CMOX_ECC_AUTH_SUCCESS) || (cmox_fault_check != CMOX_ECC_AUTH_SUCCESS))
        {
            if (CMOX_ECC_AUTH_FAIL == cmox_ecc_retval)
            {
                /* Report signature mismatch */
                SID_PAL_LOG_ERROR("Image signature mismatch");
            }
            else
            {
                /* It's some sort of an internal CMOX error or failure, but not directly a signature mismatch */
                SID_PAL_LOG_ERROR("Failed to verify signature. CMOX error 0x%08X", cmox_ecc_retval);
            }
            status = AIMFWVS_SIGNATURE_MISMATCH;
            break;
        }
        /* Signature is ok */
        SID_PAL_LOG_INFO("Signature checked passed");

        /* All checks passed */
        status = AIMFWVS_VALID;

        /* Report back image size in the number of pages */
        if (out_image_blocks_count != NULL)
        {
            *out_image_blocks_count = ((uint32_t)(void *)fw_image_footer + sizeof(*fw_image_footer) - slot_start_address + (FLASH_PAGE_SIZE - 1u)) / FLASH_PAGE_SIZE;
        }
    } while (0);

    if (status != AIMFWVS_VALID)
    {
        SID_PAL_LOG_ERROR("The %s FW image is corrupted and cannot be used", fw_image_name);
        if (out_image_blocks_count != NULL)
        {
            *out_image_blocks_count = 0u;
        }
    }
    else
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds" /* GCC 13 struggles with analyzing the usage of pointers to a _PACKED_STRUCT correctly */
        SID_PAL_LOG_INFO("Discovered a valid %s FW image, version: %u.%u.%u", fw_image_name, fw_image_header->app_version_major, fw_image_header->app_version_minor, fw_image_header->app_version_patch);
#pragma GCC diagnostic pop
    }

    return status;
}

/*----------------------------------------------------------------------------*/

static uint32_t is_flash_line_all_1s(const aim_flash_line_t line)
{
    uint32_t is_all_1s = TRUE;

    for (uint32_t i = 0u; i < (AIM_FLASH_LINE_SIZE / sizeof(uint32_t)); i++)
    {
        if (line[i] != UINT32_MAX)
        {
            is_all_1s = FALSE;
            break;
        }
    }

    return is_all_1s;
}

/*----------------------------------------------------------------------------*/

static uint32_t is_flash_line_all_0s(const aim_flash_line_t line)
{
    uint32_t is_all_0s = TRUE;

    for (uint32_t i = 0u; i < (AIM_FLASH_LINE_SIZE / sizeof(uint32_t)); i++)
    {
        if (line[i] != 0u)
        {
            is_all_0s = FALSE;
            break;
        }
    }

    return is_all_0s;
}

/*----------------------------------------------------------------------------*/

static aim_error_t is_flash_block_empty(const uint32_t block_idx, const aim_flash_access_area_t flash_area_id, uint32_t * const out_is_empty)
{
    aim_error_t err;

    assert_param(out_is_empty != NULL);

    do
    {
        if (block_idx > (SID_STM32_UTIL_ARRAY_SIZE(install_metadata.block_states) - 1u))
        {
            err = AIM_ERROR_BLOCK_INDEX_OUT_OF_BOUNDS;
            break;
        }

        const uint32_t physical_start_addr = (block_idx * FLASH_PAGE_SIZE) + (
                                                 (AIMFAA_ACTIVE_APP_SLOT == flash_area_id) ? APP_CONFIG_AIM_ACTIVE_SLOT_START :
                                                 (AIMFAA_STAGING_SLOT    == flash_area_id) ? APP_CONFIG_AIM_STAGING_SLOT_START :
                                                 (AIMFAA_BACKUP_SLOT     == flash_area_id) ? APP_CONFIG_AIM_ROLLBACK_SLOT_START : 0u
                                             );

        if (0u == physical_start_addr)
        {
            err = AIM_ERROR_INVALID_ARGS;
            break;
        }

        /* Set the initial state */
        err           = AIM_ERROR_NONE;
        *out_is_empty = TRUE;

        /* Reset ECC error detection mechanism */
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB();

        for (uint32_t read_addr = physical_start_addr; read_addr < (physical_start_addr + FLASH_PAGE_SIZE); read_addr += sizeof(aim_flash_line_t))
        {
            const aim_flash_line_t * current_line = (const aim_flash_line_t *)(void *)read_addr;

            if (is_flash_line_all_1s(*current_line) == FALSE)
            {
                /* Current flash line is written with something, the page is not empty */
                *out_is_empty = FALSE;
                break;
            }
        }

        /* Check for uncorrectable ECC errors first as non-empty line state can be just a consequence */
        if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
            err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t erase_flash_pages(const uint32_t start_block_idx, const uint32_t num_pages_to_erase, const aim_flash_access_area_t flash_area_id)
{
    aim_error_t err;

    do
    {
        uint32_t flash_area_start_page_idx;
        uint32_t flash_area_end_page_idx;

        if (0u == num_pages_to_erase)
        {
            err = AIM_ERROR_INVALID_ARGS;
            break;
        }

        err = AIM_ERROR_NONE;

        switch (flash_area_id)
        {
            case AIMFAA_INSTALL_METADATA:
                flash_area_start_page_idx = (APP_CONFIG_AIM_OTA_STATUS_AREA_START - FLASH_BASE) / FLASH_PAGE_SIZE; /* Use math floor */
                flash_area_end_page_idx   = ((APP_CONFIG_AIM_OTA_STATUS_AREA_END - FLASH_BASE) + (FLASH_PAGE_SIZE - 1u)) / FLASH_PAGE_SIZE; /* Use math ceiling */
                break;

            case AIMFAA_ACTIVE_APP_SLOT:
                flash_area_start_page_idx = (APP_CONFIG_AIM_ACTIVE_SLOT_START - FLASH_BASE) / FLASH_PAGE_SIZE; /* Use math floor */
                flash_area_end_page_idx   = ((APP_CONFIG_AIM_ACTIVE_SLOT_END - FLASH_BASE) + (FLASH_PAGE_SIZE - 1u)) / FLASH_PAGE_SIZE; /* Use math ceiling */
                break;

            case AIMFAA_STAGING_SLOT:
                flash_area_start_page_idx = (APP_CONFIG_AIM_STAGING_SLOT_START - FLASH_BASE) / FLASH_PAGE_SIZE; /* Use math floor */
                flash_area_end_page_idx   = ((APP_CONFIG_AIM_STAGING_SLOT_END - FLASH_BASE) + (FLASH_PAGE_SIZE - 1u)) / FLASH_PAGE_SIZE; /* Use math ceiling */
                break;

            case AIMFAA_BACKUP_SLOT:
                flash_area_start_page_idx = (APP_CONFIG_AIM_ROLLBACK_SLOT_START - FLASH_BASE) / FLASH_PAGE_SIZE; /* Use math floor */
                flash_area_end_page_idx   = ((APP_CONFIG_AIM_ROLLBACK_SLOT_END - FLASH_BASE) + (FLASH_PAGE_SIZE - 1u)) / FLASH_PAGE_SIZE; /* Use math ceiling */
                break;

            default:
                flash_area_start_page_idx = UINT32_MAX;
                flash_area_end_page_idx   = UINT32_MAX;
                err                       = AIM_ERROR_UNKNOWN_SLOT_ID;
                break;
        }

        if (err != AIM_ERROR_NONE)
        {
            break;
        }

        /* Ensure the erase operation is within the boundaries */
        if ((flash_area_start_page_idx + start_block_idx + num_pages_to_erase) > flash_area_end_page_idx)
        {
            err = AIM_ERROR_BLOCK_INDEX_OUT_OF_BOUNDS;
            break;
        }

#if defined(FLASH_DBANK_SUPPORT)
        if (((flash_area_start_page_idx + start_block_idx) < FLASH_PAGE_NB) && ((flash_area_start_page_idx + start_block_idx + num_pages_to_erase) > FLASH_PAGE_NB))
        {
            /* Flash area spans across two banks and cannot be erased in a single pass */
            FLASH_EraseInitTypeDef p_erase_init_a = {
                .TypeErase = FLASH_TYPEERASE_PAGES,
                .NbPages   = FLASH_PAGE_NB - (flash_area_start_page_idx + start_block_idx),
                .Page      = (uint32_t)((flash_area_start_page_idx + start_block_idx) & (FLASH_PAGE_NB - 1u)),
            };
            FLASH_EraseInitTypeDef p_erase_init_b = {
                .TypeErase = FLASH_TYPEERASE_PAGES,
                .NbPages   = num_pages_to_erase - p_erase_init_a.NbPages,
                .Page      = 0u,
            };
            uint32_t          page_error_a;
            uint32_t          page_error_b;
            HAL_StatusTypeDef status_a;
            HAL_StatusTypeDef status_b;

            /* Take bank swap state into account */
            if ((FLASH_PAGE_NB & (flash_area_start_page_idx + start_block_idx)) ^ (1u & READ_BIT (FLASH->OPTR, FLASH_OPTR_SWAP_BANK_Msk)))
            {
                p_erase_init_a.Banks = FLASH_BANK_2;
                p_erase_init_b.Banks = FLASH_BANK_1;
            }
            else
            {
                p_erase_init_a.Banks = FLASH_BANK_1;
                p_erase_init_b.Banks = FLASH_BANK_2;
            }

            /* Clear all Flash flags before erase operation */
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

            /* Erase the specified pages */
            HAL_FLASH_Unlock();
            status_a = HAL_FLASHEx_Erase(&p_erase_init_a, &page_error_a);
            status_b = HAL_FLASHEx_Erase(&p_erase_init_b, &page_error_b);
            HAL_FLASH_Lock();

            if ((status_a != HAL_OK) || (page_error_a != UINT32_MAX))
            {
                SID_PAL_LOG_ERROR("Failed to erase pages %u-%u, bank: %u, status: 0x%08X, page err: 0x%08X",
                                  p_erase_init_a.Page,
                                  (FLASH_BANK_1 == p_erase_init_a.Banks) ? 1u: 2u,
                                  (p_erase_init_a.NbPages - 1u),
                                  (uint32_t)status_a, page_error_a);
                err = AIM_ERROR_FLASH_PAGE_ERASE_FAIL;
            }

            if ((status_b != HAL_OK) || (page_error_b != UINT32_MAX))
            {
                SID_PAL_LOG_ERROR("Failed to erase pages %u-%u, bank: %u, status: 0x%08X, page err: 0x%08X",
                                  p_erase_init_b.Page,
                                  (FLASH_BANK_1 == p_erase_init_b.Banks) ? 1u: 2u,
                                  (p_erase_init_b.NbPages - 1u),
                                  (uint32_t)status_b, page_error_b);
                err = AIM_ERROR_FLASH_PAGE_ERASE_FAIL;
            }
        }
        else
#endif /* FLASH_DBANK_SUPPORT */
        {
            /* Configure erase operation */
            FLASH_EraseInitTypeDef p_erase_init = {
                .TypeErase = FLASH_TYPEERASE_PAGES,
                .NbPages   = num_pages_to_erase,
                .Page      = (uint32_t)((flash_area_start_page_idx + start_block_idx) & (FLASH_PAGE_NB - 1u)),
            };
            uint32_t          page_error;
            HAL_StatusTypeDef status;

#if defined(FLASH_DBANK_SUPPORT)
            /* Take bank swap state into account */
            if ((FLASH_PAGE_NB & (flash_area_start_page_idx + start_block_idx)) ^ (1u & READ_BIT (FLASH->OPTR, FLASH_OPTR_SWAP_BANK_Msk)))
            {
                p_erase_init.Banks = FLASH_BANK_2;
            }
            else
            {
                p_erase_init.Banks = FLASH_BANK_1;
            }
#endif /* FLASH_DBANK_SUPPORT */

            /* Clear all Flash flags before erase operation */
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

            /* Erase the specified pages */
            HAL_FLASH_Unlock();
            status = HAL_FLASHEx_Erase(&p_erase_init, &page_error);
            HAL_FLASH_Lock();

            if ((status != HAL_OK) || (page_error != UINT32_MAX))
            {
#if defined(FLASH_DBANK_SUPPORT)
                SID_PAL_LOG_ERROR("Failed to erase pages %u-%u, bank: %u, status: 0x%08X, page err: 0x%08X",
                                  p_erase_init.Page,
                                  (FLASH_BANK_1 == p_erase_init.Banks) ? 1u: 2u,
                                  (p_erase_init.NbPages - 1u),
                                  (uint32_t)status, page_error);
#else
                SID_PAL_LOG_ERROR("Failed to erase pages %u-%u, status: 0x%08X, page err: 0x%08X",
                                  p_erase_init.Page,
                                  (p_erase_init.NbPages - 1u),
                                  (uint32_t)status, page_error);
#endif /* FLASH_DBANK_SUPPORT */

                err = AIM_ERROR_FLASH_PAGE_ERASE_FAIL;
            }
        }

        /* Invalidate ICACHE to ensure flash erase is actually committed */
        invalidate_icache();
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t read_confirmation_boot_counter(uint32_t * const out_boot_counter)
{
    aim_error_t err        = AIM_ERROR_NONE;
    uint32_t    boot_count = 0u;

    assert_param(out_boot_counter != NULL);

    do
    {
        /* Reset ECC error detection mechanism */
        aim_ota_status_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB();

        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(install_metadata.confirmation_boot_counter); i++)
        {
            if (is_flash_line_all_1s(install_metadata.confirmation_boot_counter[i]) != FALSE)
            {
                /* Empty cell, terminate */
                break;
            }

            if (is_flash_line_all_0s(install_metadata.confirmation_boot_counter[i]) != FALSE)
            {
                boot_count += 2u;
            }
            else if (SID_STM32_UTIL_fast_memcmp(install_metadata.confirmation_boot_counter[i], conf_boot_counter_inc_magic_word, sizeof(conf_boot_counter_inc_magic_word)) == 0u)
            {
                boot_count += 1u;
            }
            else
            {
                err = AIM_ERROR_UNEXPECTED_DATA;
                break;
            }
        }

        if (aim_ota_status_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            /* Override AIM_ERROR_UNEXPECTED_DATA with flash ECC detection */
            err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
        }
    } while (0);

    if (err != AIM_ERROR_NONE)
    {
        boot_count = UINT32_MAX;
    }

    *out_boot_counter = boot_count;

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t increment_confirmation_boot_counter(void)
{
    aim_error_t err = AIM_ERROR_NONE;

    do
    {
        uint32_t         line_address = 0u;
        aim_flash_line_t line_data;


        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(install_metadata.confirmation_boot_counter); i++)
        {
            if (is_flash_line_all_1s(install_metadata.confirmation_boot_counter[i]) != FALSE)
            {
                /* Empty line - fill with the magic word */
                line_address = (uint32_t)(void *)&install_metadata.confirmation_boot_counter[i];
                SID_STM32_UTIL_fast_memcpy(line_data, conf_boot_counter_inc_magic_word, sizeof(line_data));
                break;
            }
            else if (SID_STM32_UTIL_fast_memcmp(install_metadata.confirmation_boot_counter[i], conf_boot_counter_inc_magic_word, sizeof(conf_boot_counter_inc_magic_word)) == 0u)
            {
                /* Line is filled with the magic word - set to all 0s */
                line_address = (uint32_t)(void *)install_metadata.confirmation_boot_counter[i];
                SID_STM32_UTIL_fast_memset(line_data, 0u, sizeof(line_data));
                break;
            }
            else
            {
                /* Continue the search for the free space */
            }
        }

        if (0u == line_address)
        {
            err = AIM_ERROR_FLASH_NO_FREE_SPACE;
            break;
        }

        /* Write down the new state to the flash */
        HAL_StatusTypeDef status = HAL_ERROR;

        /* Clear all Flash flags before write operation */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

        HAL_FLASH_Unlock();
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, line_address, (uint32_t)(void *)&line_data);
        HAL_FLASH_Lock();

        /* Invalidate ICACHE to ensure flash erase is actually committed */
        invalidate_icache();

        if (status != HAL_OK)
        {
            err = AIM_ERROR_FLASH_WRITE_FAIL;
            break;
        }

        /* Done */
        err = AIM_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t read_confirmation_state(sid_aim_app_validity_state_t * const out_validity_confirmed)
{
    aim_error_t err;

    assert_param(out_validity_confirmed != NULL);

    do
    {
        const sid_aim_ota_header_t *      fw_image_header;
        const sid_aim_ota_footer_t *      fw_image_footer;
        const sid_aim_validity_marker_t * fw_validity_marker;

        fw_image_header = (const sid_aim_ota_header_t *)(void *)APP_CONFIG_AIM_ACTIVE_SLOT_HEADER_START;

        /* Check if the image footer address stored in the image header belongs to the current slot */
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB(); /* Reset ECC error detection mechanism */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds" /* GCC 13 struggles with analyzing pointer arithmetic involving _PACKED_STRUCT correctly */
        if (((uint32_t)(void *)fw_image_header->ota_footer_location < APP_CONFIG_AIM_ACTIVE_SLOT_START)
         || ((uint32_t)(void *)fw_image_header->ota_footer_location > APP_CONFIG_AIM_ACTIVE_SLOT_END)
         || (((uint32_t)(void *)fw_image_header->ota_footer_location + sizeof(*fw_image_footer)) > APP_CONFIG_AIM_ACTIVE_SLOT_END))
#pragma GCC diagnostic pop
        {
            /* Check for uncorrectable ECC errors first as invalid footer address can be just a consequence */
            if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
            {
                SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
                err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
                break;
            }

            /* The footer address is invalid */
            err = AIM_ERROR_UNEXPECTED_DATA;
            break;
        }
        /* Footer address is ok and can be used */
        fw_image_footer = (const sid_aim_ota_footer_t *)(void *)fw_image_header->ota_footer_location;

        /* Check the magic word in the footer */
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB(); /* Reset ECC error detection mechanism */
        if (fw_image_footer->magic_word != SID_AIM_IFC_IMAGE_VALIDITY_MAGIC_WORD)
        {
            /* Check for uncorrectable ECC errors first as invalid magic word can be just a consequence */
            if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
            {
                SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
                err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
                break;
            }

            /* The magic word is invalid */
            err = AIM_ERROR_UNEXPECTED_DATA;
            break;
        }
        /* Magic word is ok, can proceed with the validation marker */

        /* Check if the image validation marker address stored in the image header belongs to the current slot */
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB(); /* Reset ECC error detection mechanism */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds" /* GCC 13 struggles with analyzing pointer arithmetic involving _PACKED_STRUCT correctly */
        if (((uint32_t)(void *)fw_image_header->validity_marker_location < APP_CONFIG_AIM_ACTIVE_SLOT_START)
         || ((uint32_t)(void *)fw_image_header->validity_marker_location > APP_CONFIG_AIM_ACTIVE_SLOT_END)
         || (((uint32_t)(void *)fw_image_header->validity_marker_location + sizeof(*fw_validity_marker)) > APP_CONFIG_AIM_ACTIVE_SLOT_END))
#pragma GCC diagnostic pop
        {
            /* Check for uncorrectable ECC errors first as invalid footer address can be just a consequence */
            if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
            {
                SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
                err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
                break;
            }

            /* The validity marker address is invalid */
            err = AIM_ERROR_UNEXPECTED_DATA;
            break;
        }
        /* Validity marker address is ok and can be used */
        fw_validity_marker = fw_image_header->validity_marker_location;

        /* Compare the validity marker with the expected magic word */
        if (SID_STM32_UTIL_fast_memcmp(fw_validity_marker->validity_magic_word, aim_app_validity_magic_word, sizeof(*fw_validity_marker)) == 0u)
        {
            *out_validity_confirmed = AIMAVS_CONFIRMED;
        }
        else if (is_flash_line_all_1s(fw_validity_marker->validity_magic_word) != FALSE)
        {
            *out_validity_confirmed = AIMAVS_CONFIRMATION_PENDING;
        }
        else if (is_flash_line_all_0s(fw_validity_marker->validity_magic_word) != FALSE)
        {
            *out_validity_confirmed = AIMAVS_REJECTED;
        }
        else
        {
            *out_validity_confirmed = AIMAVS_CONFIRMATION_INVALID;
        }

        /* Done */
        err = AIM_ERROR_NONE;
    } while (0);

    if (err != AIM_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to read app self-confirmation status. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t read_image_block_state(const uint32_t block_idx, aim_image_block_state_t * const out_state)
{
    aim_error_t err;

    assert_param(out_state != NULL);

    do
    {
        if (block_idx > (SID_STM32_UTIL_ARRAY_SIZE(install_metadata.block_states) - 1u))
        {
            *out_state = AIMIBS_INVALID;
            err = AIM_ERROR_BLOCK_INDEX_OUT_OF_BOUNDS;
            break;
        }

        /* Reset ECC error detection mechanism */
        aim_ota_status_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB();

        if (is_flash_line_all_1s(install_metadata.block_states[block_idx]) != FALSE)
        {
            *out_state = AIMIBS_NOT_PROCESSED;
        }
        else if (is_flash_line_all_0s(install_metadata.block_states[block_idx]) != FALSE)
        {
            *out_state = AIMIBS_PROCESSED;
        }
        else if (SID_STM32_UTIL_fast_memcmp(install_metadata.block_states[block_idx], block_state_backup_completed_magic_word, sizeof(install_metadata.block_states[block_idx])) == 0u)
        {
            *out_state = AIMIBS_BACKUP_COMPLETED;
        }
        else
        {
            /* Normally this shall not happen */
            *out_state = AIMIBS_INVALID;
            err = AIM_ERROR_UNEXPECTED_DATA;
        }

        /* Check for uncorrectable ECC errors */
        if (aim_ota_status_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", aim_ota_status_ecc_error_address);
            *out_state = AIMIBS_INVALID;
            err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
        }
        else
        {
            err = AIM_ERROR_NONE;
        }
    } while (0);

    if (err != AIM_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to read the state of block #%u. Error: %u", block_idx, (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t estimate_image_block_state(const uint32_t block_idx, const aim_fw_image_type_t image_type, aim_image_block_state_t * const out_state)
{
    aim_error_t             err;
    aim_image_block_state_t estimated_state = AIMIBS_INVALID;

    assert_param(out_state != NULL);

    /* This is a fallback method to estimate the state of the current block if it cannot be reliably read from the flash (e.g., uncorrectable ECC is reported) */

    do
    {
        err = read_image_block_state(block_idx, out_state);
        if (AIM_ERROR_NONE == err)
        {
            /* The state can be read from the flash reliably and there's no need to estimate it */
            break;
        }
        else if ((err != AIM_ERROR_FLASH_UNCORRECTABLE_ECC) && (err != AIM_ERROR_UNEXPECTED_DATA))
        {
            /* The error is not related to the unreliable flash reading, terminate */
            break;
        }
        else
        {
            /* Proceed with the state estimation */
        }

        if (0u == block_idx)
        {
            /* Special case - the very first block */
            aim_image_block_state_t next_block_state;

            /* Read the state of the next block */
            (void)read_image_block_state((block_idx + 1u), &next_block_state);

            /* Try to estimate the state based on the state of the adjacent block */
            if (next_block_state > AIMIBS_NOT_PROCESSED)
            {
                /* The next block departed the AIMIBS_NOT_PROCESSED state - the current block must be in the AIMIBS_PROCESSED state then */
                estimated_state = AIMIBS_PROCESSED;
                err = AIM_ERROR_NONE;
                break;
            }
            else if (AIMIBS_NOT_PROCESSED == next_block_state)
            {
                /* The next block is not touched yet. The current block state is ambiguous and can be anything */
                if (AIMFWIT_ROLL_BACK == image_type)
                {
                    /* For the rollback mode it is safe to report AIMIBS_NOT_PROCESSED state since backup pages are always preserved and can be copied into the current block */
                    estimated_state = AIMIBS_NOT_PROCESSED;
                    err = AIM_ERROR_NONE;
                    break;
                }

                /* Handling the update mode - try to estimate the state based on the contents of the active, backup, and update blocks */
                err = estimate_image_block_state_from_contents(block_idx, out_state);
                break;
            }
            else
            {
                /* Unfortunately the state of the current block cannot be reliably estimated nor fail-safe substitution value can be provided */
                estimated_state = AIMIBS_INVALID;
                err = AIM_ERROR_INVALID_STATE;
                break;
            }
        }
        else if ((SID_STM32_UTIL_ARRAY_SIZE(install_metadata.block_states) - 1u) == block_idx)
        {
            /* Special case - the very last block */
            aim_image_block_state_t prev_block_state;

            /* Read the state of the previous block */
            (void)read_image_block_state((block_idx - 1u), &prev_block_state);

            /* Try to estimate the state based on the state of the adjacent block */
            if ((prev_block_state != AIMIBS_INVALID) && (prev_block_state != AIMIBS_PROCESSED))
            {
                /* The previous block is not fully processed yet - the current block must be in the AIMIBS_NOT_PROCESSED state */
                estimated_state = AIMIBS_NOT_PROCESSED;
                err = AIM_ERROR_NONE;
                break;
            }
            else if (AIMIBS_PROCESSED == prev_block_state)
            {
                /* The previous block is processed, the current block state is ambiguous and can be anything */
                if (AIMFWIT_ROLL_BACK == image_type)
                {
                    /* For the rollback mode it is safe to report AIMIBS_NOT_PROCESSED state since backup pages are always preserved and can be copied into the current block */
                    estimated_state = AIMIBS_NOT_PROCESSED;
                    err = AIM_ERROR_NONE;
                    break;
                }

                /* Handling the update mode - try to estimate the state based on the contents of the active, backup, and update blocks */
                err = estimate_image_block_state_from_contents(block_idx, out_state);
                break;
            }
            else
            {
                /* Unfortunately the state of the current block cannot be reliably estimated nor fail-safe substitution value can be provided */
                estimated_state = AIMIBS_INVALID;
                err = AIM_ERROR_INVALID_STATE;
                break;
            }
        }
        else
        {
            aim_image_block_state_t prev_block_state;
            aim_image_block_state_t next_block_state;

            /* Read the state of the previous block */
            (void)read_image_block_state((block_idx - 1u), &prev_block_state);

            /* Read the state of the next block */
            (void)read_image_block_state((block_idx + 1u), &next_block_state);

            /* Try to estimate the state based on the states of the adjacent blocks */
            if ((prev_block_state == next_block_state) && (next_block_state != AIMIBS_INVALID) && (next_block_state != AIMIBS_BACKUP_COMPLETED))
            {
                /* Both previous and next blocks have an identical valid state - the current block should be in the same state too. Note: AIMIBS_BACKUP_COMPLETED is expected to be set for a single block by design. Multiple blocks with AIMIBS_BACKUP_COMPLETED is an invalid state */
                estimated_state = next_block_state;
                err = AIM_ERROR_NONE;
                break;
            }
            else if ((AIMIBS_PROCESSED == prev_block_state) && (next_block_state > AIMIBS_NOT_PROCESSED))
            {
                /* The previous block is fully processed and the next block departed the AIMIBS_NOT_PROCESSED state - the current block must be in the AIMIBS_PROCESSED state too */
                estimated_state = AIMIBS_PROCESSED;
                err = AIM_ERROR_NONE;
                break;
            }
            else if ((AIMIBS_PROCESSED == prev_block_state) && (AIMIBS_NOT_PROCESSED == next_block_state))
            {
                /* The previous block is processed, the next block is not touched yet. The current block state is ambiguous and can be anything */
                if (AIMFWIT_ROLL_BACK == image_type)
                {
                    /* For the rollback mode it is safe to report AIMIBS_NOT_PROCESSED state since backup pages are always preserved and can be copied into the current block */
                    estimated_state = AIMIBS_NOT_PROCESSED;
                    err = AIM_ERROR_NONE;
                    break;
                }

                /* Handling the update mode - try to estimate the state based on the contents of the active, backup, and update blocks */
                err = estimate_image_block_state_from_contents(block_idx, out_state);
                break;
            }
            else
            {
                /* Unfortunately the state of the current block cannot be reliably estimated nor fail-safe substitution value can be provided */
                estimated_state = AIMIBS_INVALID;
                err = AIM_ERROR_INVALID_STATE;
                break;
            }
        }
    } while (0);

    if (AIM_ERROR_NONE == err)
    {
        if (estimated_state != AIMIBS_INVALID)
        {
            SID_PAL_LOG_WARNING("Estimated block #%u state is %u", block_idx, (uint32_t)estimated_state);
            *out_state = estimated_state;
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Unable to estimate the state of block #%u. Error: %u", block_idx, (uint32_t)err);
        *out_state = AIMIBS_INVALID;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t estimate_image_block_state_from_contents(const uint32_t block_idx, aim_image_block_state_t * const out_state)
{
    aim_error_t err;

    assert_param(out_state != NULL);

    do
    {
        aim_error_t compare_backup_err = compare_image_block(block_idx, AIMFAA_ACTIVE_APP_SLOT, AIMFAA_BACKUP_SLOT, 0u, 0u);
        aim_error_t compare_update_err = compare_image_block(block_idx, AIMFAA_ACTIVE_APP_SLOT, AIMFAA_STAGING_SLOT, 0u, 0u);

        /* Estimate the state based on the comparison of the content in active, rollback, and staging slots. This estimation relies on the fact that the next block is not processed yet, so the staging slot still keeps the original update data for the current block */
        if ((AIM_ERROR_NONE == compare_backup_err) && (AIM_ERROR_FLASH_DATA_MISMATCH == compare_update_err))
        {
            /* Current block contents matches the corresponding backup block context, but not the update block contents - assume the backup phase is done */
            *out_state = AIMIBS_BACKUP_COMPLETED;
            err = AIM_ERROR_NONE;
            break;
        }
        else if ((AIM_ERROR_FLASH_DATA_MISMATCH == compare_backup_err) && (AIM_ERROR_NONE == compare_update_err))
        {
            /* Current block contents matches the corresponding update block context, but not the backup block contents - assume the update phase is done */
            *out_state = AIMIBS_PROCESSED;
            err = AIM_ERROR_NONE;
            break;
        }
        else if ((AIM_ERROR_NONE == compare_backup_err) && (AIM_ERROR_NONE == compare_update_err))
        {
            /* Corner case when the block content is identical in the current (backup) image and the update image - report AIMIBS_PROCESSED since there's no point in copying the same data */
            *out_state = AIMIBS_PROCESSED;
            err = AIM_ERROR_NONE;
            break;
        }
        else
        {
            uint32_t is_active_block_erased;
            uint32_t is_backup_block_erased;

            /* Block contents does not match neither backup block nor the update block, suggesting either backup or update operation was interrupted mid-flight */

            /* Check if current block and corresponding backup block are erased */
            err = is_flash_block_empty(block_idx, AIMFAA_ACTIVE_APP_SLOT, &is_active_block_erased);
            if (err != AIM_ERROR_NONE)
            {
                /* Cannot estimate current block state */
                *out_state = AIMIBS_INVALID;
                break;
            }

            err = is_flash_block_empty(block_idx, AIMFAA_BACKUP_SLOT, &is_backup_block_erased);
            if (err != AIM_ERROR_NONE)
            {
                /* Cannot estimate current block state */
                *out_state = AIMIBS_INVALID;
                break;
            }

            if ((is_active_block_erased != FALSE) && (FALSE == is_backup_block_erased))
            {
                /* Backup block holds some data while the active block is empty - assume the backup phase was completed */
                *out_state = AIMIBS_BACKUP_COMPLETED;
                err = AIM_ERROR_NONE;
                break;
            }
            else if ((FALSE == is_active_block_erased) && (is_backup_block_erased != FALSE))
            {
                /**
                 * Backup block is erased, while active block holds some data - either backup was skipped (e.g., firmware update length is greater than
                 * the current firmware and backup of an empty active block was skipped and now the active block keeps a partial update) or the backup
                 * was about to start, but failed to do so. Assume AIMIBS_NOT_PROCESSED state, in worst case we will backup some garbage that will be
                 * cleaned post-install.
                 */

                *out_state = AIMIBS_NOT_PROCESSED;
                err = AIM_ERROR_NONE;
                break;
            }
            else
            {
                /* Proceed with further checks */
            }

            /**
             * This is the worst-case scenario when the active block contains partial data, meaning either backup or update phase was interrupted and
             * block status is corrupted. This is extremely unfortunate, but still may happen. The best we can do is to compare if the partial content
             * in the active block looks more like a backup block or an update block.
             */

            /* Detect the location where the contents start to differ for both active-backup and active-update block pairs */
            uint32_t backup_block_identical_bytes;
            uint32_t install_block_identical_bytes;

            err = get_first_mismatch_line_in_block(block_idx, AIMFAA_ACTIVE_APP_SLOT, AIMFAA_BACKUP_SLOT, &backup_block_identical_bytes);
            if (err != AIM_ERROR_NONE)
            {
                /* Cannot estimate current block state */
                *out_state = AIMIBS_INVALID;
                break;
            }

            err = get_first_mismatch_line_in_block(block_idx, AIMFAA_ACTIVE_APP_SLOT, AIMFAA_STAGING_SLOT, &install_block_identical_bytes);
            if (err != AIM_ERROR_NONE)
            {
                /* Cannot estimate current block state */
                *out_state = AIMIBS_INVALID;
                break;
            }

            if (install_block_identical_bytes >= backup_block_identical_bytes)
            {
                /* The contents in the active block looks more like the install block - assume this is an interrupted update phase and the backup was completed earlier */
                *out_state = AIMIBS_BACKUP_COMPLETED;
                err = AIM_ERROR_NONE;
                break;
            }
            else
            {
                /* The contents in the active block looks more like the backup block - assume this is an interrupted backup phase and the update was never started */
                *out_state = AIMIBS_NOT_PROCESSED;
                err = AIM_ERROR_NONE;
                break;
            }
        }

        /* Unable to reliably estimate the state if this point is reached */
        *out_state = AIMIBS_INVALID;
        err = AIM_ERROR_INVALID_STATE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t store_image_block_state(const uint32_t block_idx, const aim_image_block_state_t new_state)
{
    aim_error_t err;

    do
    {
        aim_flash_line_t new_status_word;
        aim_image_block_state_t current_state;

        if (block_idx > (SID_STM32_UTIL_ARRAY_SIZE(install_metadata.block_states) - 1u))
        {
            err = AIM_ERROR_BLOCK_INDEX_OUT_OF_BOUNDS;
            break;
        }

        err = read_image_block_state(block_idx, &current_state);
        if (err != AIM_ERROR_NONE)
        {
            if (((AIM_ERROR_FLASH_UNCORRECTABLE_ECC == err) || (AIM_ERROR_UNEXPECTED_DATA == err)) && (AIMIBS_PROCESSED == new_state))
            {
                /* This is a permissible recovery step because a flash line can be written with all 0s from any state. Assume AIMIBS_BACKUP_COMPLETED state to proceed */
                current_state = AIMIBS_BACKUP_COMPLETED;
            }
            else
            {
                /* This is unrecoverable situation - current state is corrupted and writing anything other than all 0s will be rejected by the flash controller anyway */
                break;
            }
        }

        switch (current_state)
        {
            case  AIMIBS_NOT_PROCESSED:
                if (new_state != AIMIBS_BACKUP_COMPLETED)
                {
                    err = AIM_ERROR_INVALID_STATE_TRANSITION;
                    break;
                }

                SID_STM32_UTIL_fast_memcpy(&new_status_word, block_state_backup_completed_magic_word, sizeof(new_status_word));
                err = AIM_ERROR_NONE;
                break;

            case AIMIBS_BACKUP_COMPLETED:
                if (new_state != AIMIBS_PROCESSED)
                {
                    err = AIM_ERROR_INVALID_STATE_TRANSITION;
                    break;
                }

                SID_STM32_UTIL_fast_memset(&new_status_word, 0u, sizeof(new_status_word));
                err = AIM_ERROR_NONE;
                break;

            case AIMIBS_PROCESSED:
            default:
                err = AIM_ERROR_INVALID_STATE_TRANSITION;
                break;
        }

        if (err != AIM_ERROR_NONE)
        {
            break;
        }

        /* Write down the new state to the flash */
        HAL_StatusTypeDef status = HAL_ERROR;

        /* Clear all Flash flags before write operation */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

        HAL_FLASH_Unlock();
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, (uint32_t)(void *)&install_metadata.block_states[block_idx], (uint32_t)(void *)&new_status_word);
        HAL_FLASH_Lock();

        /* Invalidate ICACHE to ensure flash erase is actually committed */
        invalidate_icache();

        if (status != HAL_OK)
        {
            err = AIM_ERROR_FLASH_WRITE_FAIL;
            break;
        }

        /* Done */
        err = AIM_ERROR_NONE;
    } while (0);

    if (err != AIM_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set block #%u state to %u. Error %u", block_idx, (uint32_t)new_state,(uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t get_processed_image_blocks_count(const aim_fw_image_type_t image_type, uint32_t * const out_processed_blocks)
{
    aim_error_t err;
    uint32_t    processed_blocks;

    assert_param(out_processed_blocks != NULL);

    do
    {
        /* Set initial state */
        processed_blocks = SID_STM32_UTIL_ARRAY_SIZE(install_metadata.block_states);
        err              = AIM_ERROR_NONE;

        /* Search for the last fully processed block in the status metadata */
        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(install_metadata.block_states); i++)
        {
            aim_image_block_state_t block_state;

            /* Use heuristic block state read routine to handle inconsistent block state reports (e.g., due to power loss during block state update) */
            err = estimate_image_block_state(i, image_type, &block_state);
            if (err != AIM_ERROR_NONE)
            {
                break;
            }

            if (block_state != AIMIBS_PROCESSED)
            {
                /* Found the first block that is not fully processed during the update */
                processed_blocks = i;
                break;
            }
        }

        /* Check for uncorrectable ECC errors */
        if (aim_ota_status_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", aim_ota_status_ecc_error_address);
            processed_blocks = UINT32_MAX;
            err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
            break;
        }

        /* Done */
    } while (0);

    /* Provide the number of processed blocks */
    *out_processed_blocks = processed_blocks;

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t copy_image_block(const uint32_t block_idx, const aim_flash_access_area_t dst_flash_area_id, const aim_flash_access_area_t src_flash_area_id, const uint32_t skip_copy_offset, const uint32_t skip_copy_size)
{
    aim_error_t err;

    /* Ensure skip area offset and size are aligned to the flash line size */
    assert_param((skip_copy_offset & (sizeof(aim_flash_line_t) - 1u)) == 0u);
    assert_param((skip_copy_size & (sizeof(aim_flash_line_t) - 1u)) == 0u);
    assert_param((skip_copy_offset + skip_copy_size) <= FLASH_PAGE_SIZE);

    do
    {
        if (src_flash_area_id == dst_flash_area_id)
        {
            /* Source and destination cannot be the same */
            err = AIM_ERROR_INVALID_ARGS;
            break;
        }

        if (block_idx > (SID_STM32_UTIL_ARRAY_SIZE(install_metadata.block_states) - 1u))
        {
            err = AIM_ERROR_BLOCK_INDEX_OUT_OF_BOUNDS;
            break;
        }

        const uint32_t physical_dst_start_addr = (block_idx * FLASH_PAGE_SIZE) + (
                                                     (AIMFAA_ACTIVE_APP_SLOT == dst_flash_area_id) ? APP_CONFIG_AIM_ACTIVE_SLOT_START :
                                                     (AIMFAA_STAGING_SLOT    == dst_flash_area_id) ? APP_CONFIG_AIM_STAGING_SLOT_START :
                                                     (AIMFAA_BACKUP_SLOT     == dst_flash_area_id) ? APP_CONFIG_AIM_ROLLBACK_SLOT_START : 0u
                                                 );

        const uint32_t physical_src_start_addr = (block_idx * FLASH_PAGE_SIZE) + (
                                                     (AIMFAA_ACTIVE_APP_SLOT == src_flash_area_id) ? APP_CONFIG_AIM_ACTIVE_SLOT_START :
                                                     (AIMFAA_STAGING_SLOT    == src_flash_area_id) ? APP_CONFIG_AIM_STAGING_SLOT_START :
                                                     (AIMFAA_BACKUP_SLOT     == src_flash_area_id) ? APP_CONFIG_AIM_ROLLBACK_SLOT_START : 0u
                                                 );

        if ((0u == physical_dst_start_addr) || (0u == physical_src_start_addr))
        {
            err = AIM_ERROR_INVALID_ARGS;
            break;
        }

        /* Assume no write error by default */
        err = AIM_ERROR_NONE;

        /* Copy page contents in 128-byte bursts */
        const uint32_t default_write_step = 8u * sizeof(aim_flash_line_t); /* Size of 128-byte (8x16) burst write */
        for (uint32_t offset = 0u; offset < FLASH_PAGE_SIZE; offset += default_write_step)
        {
            HAL_StatusTypeDef status = HAL_ERROR;

            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); /* Clear all Flash flags before write operation*/

            const uint32_t write_addr = physical_dst_start_addr + offset;
            const uint32_t src_addr   = physical_src_start_addr + offset;

            HAL_FLASH_Unlock();
            if (((0u == skip_copy_size) || ((write_addr + default_write_step) <= (physical_dst_start_addr + skip_copy_offset)) || (write_addr >= (physical_dst_start_addr + skip_copy_offset + skip_copy_size)))
              && ((write_addr & (default_write_step - 1u)) == 0u))
            {
                /* No skip area defined or the write operation is out of the skip area - use 8x16-byte burst write mode */
                status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BURST, write_addr, src_addr);
            }
            else
            {
                /* Writing near or inside the skip area */
                if (((write_addr + sizeof(aim_flash_line_t)) <= (physical_dst_start_addr + skip_copy_offset)) || (write_addr >= (physical_dst_start_addr + skip_copy_offset + skip_copy_size)))
                {
                    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, write_addr, src_addr);

                    /* Adjust offset since write position is incremented only by 16 bytes instead of 8x16 bytes */
                    offset -= (default_write_step - sizeof(aim_flash_line_t));
                }
                else
                {
                    /* Skip the designated area in one step */
                    offset += skip_copy_size - default_write_step;
                    status = HAL_OK;
                }
            }
            HAL_FLASH_Lock();

            if (status != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Flash write error 0x%02X, dst addr: 0x%08X, src addr: 0x%08X", status, write_addr, src_addr);
                err = AIM_ERROR_FLASH_WRITE_FAIL;
                break;
            }
        }

        if (err != AIM_ERROR_NONE)
        {
            break;
        }

        /* Invalidate ICACHE to ensure flash erase is actually committed */
        invalidate_icache();
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t compare_image_block(const uint32_t block_idx, const aim_flash_access_area_t flash_area_a, const aim_flash_access_area_t flash_area_b, const uint32_t skip_compare_offset, const uint32_t skip_compare_size)
{
    aim_error_t err;

    assert_param((skip_compare_offset + skip_compare_size) <= FLASH_PAGE_SIZE);

    do
    {
        if (flash_area_a == flash_area_b)
        {
            /* There's no point in comparing the block to itself */
            err = AIM_ERROR_INVALID_ARGS;
            break;
        }

        if (block_idx > (SID_STM32_UTIL_ARRAY_SIZE(install_metadata.block_states) - 1u))
        {
            err = AIM_ERROR_BLOCK_INDEX_OUT_OF_BOUNDS;
            break;
        }

        const uint32_t physical_start_add_a = (block_idx * FLASH_PAGE_SIZE) + (
                                                  (AIMFAA_ACTIVE_APP_SLOT == flash_area_a) ? APP_CONFIG_AIM_ACTIVE_SLOT_START :
                                                  (AIMFAA_STAGING_SLOT    == flash_area_a) ? APP_CONFIG_AIM_STAGING_SLOT_START :
                                                  (AIMFAA_BACKUP_SLOT     == flash_area_a) ? APP_CONFIG_AIM_ROLLBACK_SLOT_START : 0u
                                              );

        const uint32_t physical_start_add_b = (block_idx * FLASH_PAGE_SIZE) + (
                                                  (AIMFAA_ACTIVE_APP_SLOT == flash_area_b) ? APP_CONFIG_AIM_ACTIVE_SLOT_START :
                                                  (AIMFAA_STAGING_SLOT    == flash_area_b) ? APP_CONFIG_AIM_STAGING_SLOT_START :
                                                  (AIMFAA_BACKUP_SLOT     == flash_area_b) ? APP_CONFIG_AIM_ROLLBACK_SLOT_START : 0u
                                              );

        /* Reset ECC error detection mechanism */
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB();

        uint32_t diff;

        if (0u == skip_compare_size)
        {
            diff = SID_STM32_UTIL_fast_memcmp((void *)physical_start_add_a, (void *)physical_start_add_b, FLASH_PAGE_SIZE);
        }
        else
        {
            /* Check the portion that precedes the skipped area */
            diff = SID_STM32_UTIL_fast_memcmp((void *)physical_start_add_a, (void *)physical_start_add_b, skip_compare_offset);
            if (FLASH_PAGE_SIZE - (skip_compare_offset + skip_compare_size) > 0u)
            {
                /* Check the portion that follows the skipped area (if present) */
                diff |= SID_STM32_UTIL_fast_memcmp((void *)(physical_start_add_a + skip_compare_offset + skip_compare_size), (void *)(physical_start_add_b + skip_compare_offset + skip_compare_size), (FLASH_PAGE_SIZE - (skip_compare_offset + skip_compare_size)));
            }
        }

        if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
            err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
            break;
        }

        /* Done */
        err = (0u == diff) ? AIM_ERROR_NONE : AIM_ERROR_FLASH_DATA_MISMATCH;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t get_first_mismatch_line_in_block(const uint32_t block_idx, const aim_flash_access_area_t flash_area_a, const aim_flash_access_area_t flash_area_b, uint32_t * const out_line_offset)
{
    aim_error_t err;

    do
    {
        if (flash_area_a == flash_area_b)
        {
            /* There's no point in comparing the block to itself */
            err = AIM_ERROR_INVALID_ARGS;
            break;
        }

        if (block_idx > (SID_STM32_UTIL_ARRAY_SIZE(install_metadata.block_states) - 1u))
        {
            err = AIM_ERROR_BLOCK_INDEX_OUT_OF_BOUNDS;
            break;
        }

        *out_line_offset = UINT32_MAX;

        const uint32_t physical_start_add_a = (block_idx * FLASH_PAGE_SIZE) + (
                                                  (AIMFAA_ACTIVE_APP_SLOT == flash_area_a) ? APP_CONFIG_AIM_ACTIVE_SLOT_START :
                                                  (AIMFAA_STAGING_SLOT    == flash_area_a) ? APP_CONFIG_AIM_STAGING_SLOT_START :
                                                  (AIMFAA_BACKUP_SLOT     == flash_area_a) ? APP_CONFIG_AIM_ROLLBACK_SLOT_START : 0u
                                              );

        const uint32_t physical_start_add_b = (block_idx * FLASH_PAGE_SIZE) + (
                                                  (AIMFAA_ACTIVE_APP_SLOT == flash_area_b) ? APP_CONFIG_AIM_ACTIVE_SLOT_START :
                                                  (AIMFAA_STAGING_SLOT    == flash_area_b) ? APP_CONFIG_AIM_STAGING_SLOT_START :
                                                  (AIMFAA_BACKUP_SLOT     == flash_area_b) ? APP_CONFIG_AIM_ROLLBACK_SLOT_START : 0u
                                              );

        /* Reset ECC error detection mechanism */
        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB();

        for (uint32_t check_offset = 0u; check_offset < FLASH_PAGE_SIZE; check_offset += sizeof(aim_flash_line_t))
        {
            const aim_flash_line_t * line_a_ptr = (const aim_flash_line_t *)(void *)(physical_start_add_a + check_offset);
            const aim_flash_line_t * line_b_ptr = (const aim_flash_line_t *)(void *)(physical_start_add_b + check_offset);

            if (SID_STM32_UTIL_fast_memcmp((void *)(*line_a_ptr), (void *)(*line_b_ptr), sizeof(aim_flash_line_t)) != 0u)
            {
                *out_line_offset = check_offset;
            }
        }

        if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
            *out_line_offset = UINT32_MAX;
            err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
            break;
        }

        /* Done */
        err = (UINT32_MAX == *out_line_offset) ? AIM_ERROR_FLASH_DATA_IDENTICAL : AIM_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t read_install_state(aim_install_state_t * const out_state, uint32_t * const out_active_image_size_pages, uint32_t * const install_image_size_pages)
{
    aim_error_t err;

    assert_param(out_state != NULL);

    do
    {
        /* Set initial values */
        if (out_active_image_size_pages != NULL)
        {
            *out_active_image_size_pages = 0u;
        }

        if (install_image_size_pages != NULL)
        {
            *install_image_size_pages = 0u;
        }

        aim_ota_status_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB(); /* Reset ECC error detection mechanism */

        if ((is_flash_line_all_1s(install_metadata.install_state.status_word_0) != FALSE) && (is_flash_line_all_1s(install_metadata.install_state.status_word_1) != FALSE))
        {
            *out_state = AIMIS_IDLE;
            err = AIM_ERROR_NONE;
        }
        else if ((SID_STM32_UTIL_fast_memcmp(install_metadata.install_state.status_word_0, aim_state_update_ongoing_magic_word, sizeof(aim_state_update_ongoing_magic_word)) == 0u) && (is_flash_line_all_1s(install_metadata.install_state.status_word_1) != FALSE))
        {
            *out_state = AIMIS_UPDATE_ONGOING;

            /* Decode image sizes (in flash pages) from status words */
            if (out_active_image_size_pages != NULL)
            {
                *out_active_image_size_pages = (install_metadata.install_state.status_word_0[SID_STM32_UTIL_ARRAY_SIZE(install_metadata.install_state.status_word_0) - 1u] & AIM_ACTIVE_IMAGE_BLOCK_NUM_MASK) >> AIM_ACTIVE_IMAGE_BLOCK_NUM_OFFSET;
            }

            if (install_image_size_pages != NULL)
            {
                *install_image_size_pages = (install_metadata.install_state.status_word_0[SID_STM32_UTIL_ARRAY_SIZE(install_metadata.install_state.status_word_0) - 1u] & AIM_INSTALL_IMAGE_BLOCK_NUM_MASK) >> AIM_INSTALL_IMAGE_BLOCK_NUM_OFFSET;
            }

            err = AIM_ERROR_NONE;
        }
        else if ((SID_STM32_UTIL_fast_memcmp(install_metadata.install_state.status_word_0, aim_state_rollback_ongoing_magic_word, sizeof(aim_state_rollback_ongoing_magic_word)) == 0u) && (is_flash_line_all_1s(install_metadata.install_state.status_word_1) != FALSE))
        {
            *out_state = AIMIS_ROLLBACK_ONGOING;

            /* Decode image sizes (in flash pages) from status words */
            if (out_active_image_size_pages != NULL)
            {
                *out_active_image_size_pages = (install_metadata.install_state.status_word_0[SID_STM32_UTIL_ARRAY_SIZE(install_metadata.install_state.status_word_0) - 1u] & AIM_ACTIVE_IMAGE_BLOCK_NUM_MASK) >> AIM_ACTIVE_IMAGE_BLOCK_NUM_OFFSET;
            }

            if (install_image_size_pages != NULL)
            {
                *install_image_size_pages = (install_metadata.install_state.status_word_0[SID_STM32_UTIL_ARRAY_SIZE(install_metadata.install_state.status_word_0) - 1u] & AIM_INSTALL_IMAGE_BLOCK_NUM_MASK) >> AIM_INSTALL_IMAGE_BLOCK_NUM_OFFSET;
            }

            err = AIM_ERROR_NONE;
        }
        else if ((is_flash_line_all_0s(install_metadata.install_state.status_word_0) != FALSE) && (is_flash_line_all_1s(install_metadata.install_state.status_word_1) != FALSE))
        {
            *out_state = AIMIS_UPDATE_CONFIRMATION_PENDING;
            err = AIM_ERROR_NONE;
        }
        else if ((is_flash_line_all_0s(install_metadata.install_state.status_word_0) != FALSE) && (SID_STM32_UTIL_fast_memcmp(install_metadata.install_state.status_word_1, aim_state_finished_magic_word, sizeof(aim_state_finished_magic_word)) == 0u))
        {
            *out_state = AIMIS_FINISHED;
            err = AIM_ERROR_NONE;
        }
        else if ((is_flash_line_all_0s(install_metadata.install_state.status_word_0) != FALSE) && (is_flash_line_all_0s(install_metadata.install_state.status_word_1) != FALSE))
        {
            *out_state = AIMIS_ROLLBACK_REQUIRED;
            err = AIM_ERROR_NONE;
        }
        else
        {
            *out_state = AIMIS_INVALID;
            err = AIM_ERROR_UNEXPECTED_DATA; /* Data stored on the flash is reliable, but contains invalid value */
        }

        /* Check for uncorrectable ECC errors */
        if (aim_ota_status_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
        {
            SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", aim_ota_status_ecc_error_address);
            *out_state = AIMIS_INVALID; /* Override any state since data read from flash is unreliable */
            err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t store_install_state(const aim_install_state_t state, const uint32_t active_image_size_pages, const uint32_t install_image_size_pages)
{
    aim_error_t err = AIM_ERROR_GENERIC;

    /**
     * Important note: while update status uses two flash lines, any state transition shall be atomic. This means only one line can be updated
     * at any given transition. The only exception is the transition to the Idle state when both lines are erased. But this operation can still
     * be considered as atomic since the lines are not erased one by one, instead a full page erase is executed.
     */

    do
    {
        aim_install_state_t current_state;
        aim_flash_line_t    new_status_word;
        uint32_t            status_word_write_addr = 0u;

        /* Read current state */
        err = read_install_state(&current_state, NULL, NULL);
        if ((err != AIM_ERROR_NONE) && (state != AIMIS_IDLE))
        {
            /* Idle state can be entered even if the current state stored on flash is invalid or unknown */
            break;
        }

        switch (state)
        {
            case AIMIS_IDLE:
                /* Idle state can be entered from any state */
                err = erase_flash_pages(0u, APP_CONFIG_AIM_OTA_STATUS_AREA_SIZE_PAGES, AIMFAA_INSTALL_METADATA);
                break;

            case AIMIS_UPDATE_ONGOING:
                if (current_state != AIMIS_IDLE)
                {
                    /* Invalid transition - update can only be initiated from the Idle state */
                    err = AIM_ERROR_INVALID_STATE_TRANSITION;
                    break;
                }

                /* Update status word 0 with new state and image size info */
                SID_STM32_UTIL_fast_memcpy(new_status_word, aim_state_update_ongoing_magic_word, sizeof(aim_state_update_ongoing_magic_word));

                new_status_word[SID_STM32_UTIL_ARRAY_SIZE(new_status_word) - 1u] = (active_image_size_pages << AIM_ACTIVE_IMAGE_BLOCK_NUM_OFFSET) & AIM_ACTIVE_IMAGE_BLOCK_NUM_MASK;
                new_status_word[SID_STM32_UTIL_ARRAY_SIZE(new_status_word) - 1u] |= (install_image_size_pages << AIM_INSTALL_IMAGE_BLOCK_NUM_OFFSET) & AIM_INSTALL_IMAGE_BLOCK_NUM_MASK;

                status_word_write_addr = (uint32_t)(void *)&install_metadata.install_state.status_word_0;
                err = AIM_ERROR_NONE;
                break;

            case AIMIS_ROLLBACK_ONGOING:
                if (current_state != AIMIS_IDLE)
                {
                    /* Invalid transition - rollback can only be initiated from the Idle state */
                    err = AIM_ERROR_INVALID_STATE_TRANSITION;
                    break;
                }

                /* Update status word 0 with new state and image size info */
                SID_STM32_UTIL_fast_memcpy(new_status_word, aim_state_rollback_ongoing_magic_word, sizeof(aim_state_rollback_ongoing_magic_word));

                new_status_word[SID_STM32_UTIL_ARRAY_SIZE(new_status_word) - 1u] = (active_image_size_pages << AIM_ACTIVE_IMAGE_BLOCK_NUM_OFFSET) & AIM_ACTIVE_IMAGE_BLOCK_NUM_MASK;
                new_status_word[SID_STM32_UTIL_ARRAY_SIZE(new_status_word) - 1u] |= (install_image_size_pages << AIM_INSTALL_IMAGE_BLOCK_NUM_OFFSET) & AIM_INSTALL_IMAGE_BLOCK_NUM_MASK;

                status_word_write_addr = (uint32_t)(void *)&install_metadata.install_state.status_word_0;
                err = AIM_ERROR_NONE;
                break;

            case AIMIS_UPDATE_CONFIRMATION_PENDING:
                if ((current_state != AIMIS_UPDATE_ONGOING) && (current_state != AIMIS_ROLLBACK_ONGOING))
                {
                    /* Invalid transition - Confirmation Pending state can only be entered from Update Ongoing state */
                    err = AIM_ERROR_INVALID_STATE_TRANSITION;
                    break;
                }

                /* Update status word 0 with new state */
                SID_STM32_UTIL_fast_memset(new_status_word, 0u, sizeof(new_status_word));

                status_word_write_addr = (uint32_t)(void *)&install_metadata.install_state.status_word_0;
                err = AIM_ERROR_NONE;
                break;

            case AIMIS_FINISHED:
                if (current_state != AIMIS_UPDATE_CONFIRMATION_PENDING)
                {
                    /* Invalid transition - Update Finished can only be entered from Confirmation Pending state */
                    err = AIM_ERROR_INVALID_STATE_TRANSITION;
                    break;
                }

                /* Update status word 0 with new state and image size info */
                SID_STM32_UTIL_fast_memcpy(new_status_word, aim_state_finished_magic_word, sizeof(aim_state_finished_magic_word));

                status_word_write_addr = (uint32_t)(void *)&install_metadata.install_state.status_word_1;
                err = AIM_ERROR_NONE;
                break;

            case AIMIS_ROLLBACK_REQUIRED:
                if (current_state != AIMIS_UPDATE_CONFIRMATION_PENDING)
                {
                    /* Invalid transition - Update Finished can only be entered from Confirmation Pending state */
                    err = AIM_ERROR_INVALID_STATE_TRANSITION;
                    break;
                }

                /* Update status word 0 with new state */
                SID_STM32_UTIL_fast_memset(new_status_word, 0u, sizeof(new_status_word));

                status_word_write_addr = (uint32_t)(void *)&install_metadata.install_state.status_word_1;
                err = AIM_ERROR_NONE;
                break;

            default:
                err = AIM_ERROR_INVALID_ARGS;
                break;
        }

        /* Terminate if error occurred */
        if (err != AIM_ERROR_NONE)
        {
            break;
        }

        /* Write new status word to the flash if needed */
        if (status_word_write_addr != 0u)
        {
            HAL_StatusTypeDef status = HAL_ERROR;

            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); /* Clear all Flash flags before write operation*/

            HAL_FLASH_Unlock();
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, status_word_write_addr, (uint32_t)(void *)&new_status_word);
            HAL_FLASH_Lock();

            /* Invalidate ICACHE to ensure flash erase is actually committed */
            invalidate_icache();

            if (status != HAL_OK)
            {
                err = AIM_ERROR_FLASH_WRITE_FAIL;
                break;
            }
        }

        /* Done */
        err = AIM_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_error_t install_image(const aim_fw_image_type_t image_type, const uint32_t active_image_size_pages, const uint32_t install_image_size_pages)
{
    aim_error_t err;

    do
    {
        aim_install_state_t prev_state;
        uint32_t            prev_active_image_size_pages;
        uint32_t            prev_install_image_size_pages;
        uint32_t            prev_install_resumed          = FALSE;
        uint32_t            prev_install_processed_blocks = 0u;

        /* Check for past non-finished updates and resume if available */
        err = read_install_state(&prev_state, &prev_active_image_size_pages, &prev_install_image_size_pages);
        if (AIM_ERROR_NONE == err)
        {
            if (AIMIS_UPDATE_ONGOING == prev_state)
            {
                /* There's unfinished FW update installation - check if it can be resumed */
                if ((AIMFWIT_UPDATE == image_type) && (active_image_size_pages == prev_active_image_size_pages) && (install_image_size_pages == prev_install_image_size_pages))
                {
                    /* Determine the resume point */
                    err = get_processed_image_blocks_count(image_type, &prev_install_processed_blocks);
                    if (err != AIM_ERROR_NONE)
                    {
                        /**
                         * This is a potentially unrecoverable situation. The blocks from the active slot and the staging slot were extensively being swapped and the borderline is now lost.
                         * The content of the old firmware image and the new firmware image are now mixed and there's no inputs to determine the split point. The best effort tht can be done
                         * is to give it a new try on the next boot. This will help from any intermittent read failures, but permanent failures are non-recoverable
                         */
                        break;
                    }

                    if (prev_install_processed_blocks > install_image_size_pages)
                    {
                        /* It seems that metadata of the previous install is not related to the current one */
                        err = AIM_ERROR_INVALID_STATE;
                        break;
                    }

                    /* Indicate that previous install can be resumed */
                    prev_install_resumed = TRUE;
                }
                else
                {
                    // TODO: mismatch, decide how to handle that
                }
            }
            else if (AIMIS_ROLLBACK_ONGOING == prev_state)
            {
                /* There's unfinished rollback installation - check if it can be resumed */
                if ((AIMFWIT_ROLL_BACK == image_type) && (active_image_size_pages == prev_active_image_size_pages) && (install_image_size_pages == prev_install_image_size_pages))
                {
                    /* Determine the resume point */
                    err = get_processed_image_blocks_count(image_type, &prev_install_processed_blocks);
                    if (err != AIM_ERROR_NONE)
                    {
                        /* Abort the rollback and reset the state. If the active app is valid, it will boot. If not, the backup image is intact and rollback can be restarted from scratch */
                        SID_PAL_LOG_ERROR("Unable to resume install. Error %u", (uint32_t)err);
                        store_install_state(AIMIS_IDLE, 0u, 0u);
                        break;
                    }

                    if (prev_install_processed_blocks > install_image_size_pages)
                    {
                        /* It seems that metadata of the previous install is not related to the current one */
                        err = AIM_ERROR_INVALID_STATE;
                        break;
                    }

                    /* Indicate that previous install can be resumed */
                    prev_install_resumed = TRUE;
                }
                else
                {
                    /* Mismatch between the requested update and the stored update metadata - abort update */
                    err = AIM_ERROR_INVALID_STATE;
                    break;
                }
            }
            else
            {
                /* Nothing to resume */
            }
        }

        /* Perform update initialization steps if we are not resuming any unfinished installation */
        if (FALSE == prev_install_resumed)
        {
            /* Initialize update status info area */
            err = store_install_state(AIMIS_IDLE, 0u, 0u);
            if (err != AIM_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to set install state transition %u->%u. Error %u", (uint32_t)prev_state, (uint32_t)AIMIS_IDLE, (uint32_t)err);
                break;
            }

            /* Store the type of the update we are going to perform */
            if (AIMFWIT_UPDATE == image_type)
            {
                err = store_install_state(AIMIS_UPDATE_ONGOING, active_image_size_pages, install_image_size_pages);
                if (err != AIM_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Failed to set install state transition %u->%u. Error %u", (uint32_t)prev_state, (uint32_t)AIMIS_UPDATE_ONGOING, (uint32_t)err);
                }
            }
            else if (AIMFWIT_ROLL_BACK == image_type)
            {
                err = store_install_state(AIMIS_ROLLBACK_ONGOING, 0u /* Current image is invalid and there's no point to back it up */, install_image_size_pages);
                if (err != AIM_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Failed to set install state transition %u->%u. Error %u", (uint32_t)prev_state, (uint32_t)AIMIS_ROLLBACK_ONGOING, (uint32_t)err);
                }
            }
            else
            {
                /* Invalid image type */
                err = AIM_ERROR_UNKNOWN_IMAGE_TYPE;
                SID_PAL_LOG_ERROR("Unknow image type: 0x%08X", (uint32_t)image_type);
            }

            if (err != AIM_ERROR_NONE)
            {
                break;
            }
        }

        SID_PAL_LOG_INFO("Installing %s image...", (AIMFWIT_UPDATE == image_type) ? "update" : "rollback");

        if (prev_install_processed_blocks > 0u)
        {
            /* Printout status update */
            const uint32_t progress_nom   = prev_install_processed_blocks * 100u;
            const uint32_t progress_denom = MAX(active_image_size_pages, install_image_size_pages);
            const uint32_t progress_int   = progress_nom / progress_denom;
            const uint32_t progress_frac  = (progress_nom % progress_denom) * 100u / progress_denom;
            SID_PAL_LOG_INFO("Install progress: %u.%02u%%", progress_int, progress_frac);
        }

        uint32_t skip_copy_offset = 0u;
        uint32_t skip_copy_size   = 0u;

        for (uint32_t i = prev_install_processed_blocks; i < MAX(active_image_size_pages, install_image_size_pages); i++)
        {
            aim_image_block_state_t block_state;

            /* Use heuristic block state read routine to handle inconsistent block state reports (e.g., due to power loss during block state update) */
            err = estimate_image_block_state(i, image_type, &block_state);
            if (err != AIM_ERROR_NONE)
            {
                /* This is an unrecoverable situation */
                break;
            }

            /* Backup phase handling */
            if (AIMIBS_NOT_PROCESSED == block_state)
            {
                /**
                 * Perform backup steps only if we are installing an update and there are still some blocks of the current app left non-backed up. For rollbacks there's no point
                 * in creating a backup of a bad image, so it's enough to just mark this phase as competed without actually copying anything
                 */
                if ((AIMFWIT_UPDATE == image_type) && (i < active_image_size_pages))
                {
                    /* We are in the update phase and there are still some blocks in the current app that require a backup - proceed with creating a backup of the current block */
                    for (uint32_t attempt = 0u; attempt < AIM_BLOCK_BACKUP_RETRY_LIMIT; attempt++)
                    {
                        /* Backup step 1 - Erase backup receiving block */
                        err = erase_flash_pages(i, 1u, AIMFAA_BACKUP_SLOT);
                        if (err != AIM_ERROR_NONE)
                        {
                            /* Try once again */
                            continue;
                        }

                        /* Backup step 2 - Copy current app block to the backup receiving block */
                        err = copy_image_block(i, AIMFAA_BACKUP_SLOT, AIMFAA_ACTIVE_APP_SLOT, 0u, 0u);
                        if (err != AIM_ERROR_NONE)
                        {
                            /* Try once again */
                            continue;
                        }

                        /* Backup step 3 - Verify the copied content matches the source */
                        err = compare_image_block(i, AIMFAA_BACKUP_SLOT, AIMFAA_ACTIVE_APP_SLOT, 0u, 0u);
                        if (err != AIM_ERROR_NONE)
                        {
                            /* Try once again */
                            continue;
                        }

                        if (AIM_ERROR_NONE == err)
                        {
                            /* All backup steps completed with no errors, content is verified */
                            break;
                        }
                    }

                    if (err != AIM_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Failed to backup block #%u. Error %u", i, (uint32_t)err);
                        break;
                    }
                }

                /* Backup step 4 - Indicate the backup phase is completed for the current block */
                err = store_image_block_state(i, AIMIBS_BACKUP_COMPLETED);
                if (err != AIM_ERROR_NONE)
                {
                    /* Logs provided by store_image_block_state() */
                    /* Since AIMIBS_BACKUP_COMPLETED state is indicated by a magic word on the flash, retries here are not feasible. If the flash is already written with a wrong value, it's not possible to overwrite it with the correct one */
                    if (AIM_ERROR_FLASH_WRITE_FAIL == err)
                    {
                        /* Still proceed - if successful, the AIMIBS_PROCESSED (all 0s) still can be set, if not - at least we are not loosing anything */
                        SID_PAL_LOG_WARNING("Proceeding with unreliable state of block #%u", i);
                    }
                    else
                    {
                        break;
                    }
                }

                /* Backup phase is now completed successfully */
                block_state = AIMIBS_BACKUP_COMPLETED;
            }

            /* Write update phase handling */
            if (AIMIBS_BACKUP_COMPLETED == block_state)
            {
                for (uint32_t attempt = 0u; attempt < AIM_BLOCK_UPDATE_RETRY_LIMIT; attempt++)
                {
                    /* Update step 1 - Erase active block contents */
                    err = erase_flash_pages(i, 1u, AIMFAA_ACTIVE_APP_SLOT);
                    if (err != AIM_ERROR_NONE)
                    {
                        /* Try once again */
                        continue;
                    }

                    /* The very last block of the new image requires special handling */
                    if ((AIMFWIT_UPDATE == image_type) && ((install_image_size_pages - 1u) == i))
                    {
                        const sid_aim_ota_header_t *      fw_image_header;
                        const sid_aim_validity_marker_t * fw_validity_marker;

                        /* Assume the image header is already written to the active slot and verified - extract validity marker location from there */
                        fw_image_header = (const sid_aim_ota_header_t *)(void *)APP_CONFIG_AIM_ACTIVE_SLOT_HEADER_START;

                        /* Check if the image validation marker address stored in the image header belongs to the current slot */
                        app_flash_ecc_error_address = ECC_ERROR_ADDRESS_INVALID; __DSB(); /* Reset ECC error detection mechanism */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds" /* GCC 13 struggles with analyzing pointer arithmetic involving _PACKED_STRUCT correctly */
                        if (((uint32_t)(void *)fw_image_header->validity_marker_location < APP_CONFIG_AIM_ACTIVE_SLOT_START)
                         || ((uint32_t)(void *)fw_image_header->validity_marker_location > APP_CONFIG_AIM_ACTIVE_SLOT_END)
                         || (((uint32_t)(void *)fw_image_header->validity_marker_location + sizeof(*fw_validity_marker)) > APP_CONFIG_AIM_ACTIVE_SLOT_END))
#pragma GCC diagnostic pop
                        {
                            /* Check for uncorrectable ECC errors first as invalid footer address can be just a consequence */
                            if (app_flash_ecc_error_address != ECC_ERROR_ADDRESS_INVALID)
                            {
                                /* Retries are useless here because ECC error does not belong to the current block */
                                SID_PAL_LOG_ERROR("Uncorrectable ECC error at address 0x%08X", app_flash_ecc_error_address);
                                err = AIM_ERROR_FLASH_UNCORRECTABLE_ECC;
                                break;
                            }

                            /* The validity marker address is invalid */
                            err = AIM_ERROR_UNEXPECTED_DATA;
                            break;
                        }
                        /* Validity marker address is ok and can be used */
                        fw_validity_marker = fw_image_header->validity_marker_location;

                        /* Compute skip parameters and ensure skip parameters are aligned to the flash line size */
                        skip_copy_offset = (uint32_t)(void *)fw_validity_marker % FLASH_PAGE_SIZE;
                        skip_copy_offset = skip_copy_offset & (~(sizeof(aim_flash_line_t) - 1u));                                 /* Trim down to the nearest 16-byte aligned value */

                        skip_copy_size   = FLASH_PAGE_SIZE - skip_copy_offset;
                        skip_copy_size   = (skip_copy_size + sizeof(aim_flash_line_t) - 1u) & (~(sizeof(aim_flash_line_t) - 1u)); /* Scale up to the nearest 16-byte aligned value */
                    }

                    /* Update step 2 - Copy new data from the staging or backup area */
                    err = copy_image_block(i, AIMFAA_ACTIVE_APP_SLOT, (AIMFWIT_UPDATE == image_type) ? AIMFAA_STAGING_SLOT : AIMFAA_BACKUP_SLOT, skip_copy_offset, skip_copy_size);
                    if (err != AIM_ERROR_NONE)
                    {
                        /* Try once again */
                        continue;
                    }

                    /* Update step 3 - Verify the copied content matches the source */
                    err = compare_image_block(i, AIMFAA_ACTIVE_APP_SLOT, (AIMFWIT_UPDATE == image_type) ? AIMFAA_STAGING_SLOT : AIMFAA_BACKUP_SLOT, skip_copy_offset, skip_copy_size);
                    if (err != AIM_ERROR_NONE)
                    {
                        /* Try once again */
                        continue;
                    }

                    /* Update step 4 - Indicate the update phase is completed for the current block */
                    err = store_image_block_state(i, AIMIBS_PROCESSED);
                    if (err != AIM_ERROR_NONE)
                    {
                        /* Try once again. Unlike the backup phase, here a retry may work because AIMIBS_PROCESSED is indicated by all 0s in the flash line. This transition is allowed by flash controller from any state */
                        continue;
                    }

                    if (AIM_ERROR_NONE == err)
                    {
                        break;
                    }
                }

                if (err != AIM_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Failed to update block #%u. Error %u", i, (uint32_t)err);
                    break;
                }
            }

            /* Printout status update */
            const uint32_t progress_nom   = (i + 1u) * 100u;
            const uint32_t progress_denom = MAX(active_image_size_pages, install_image_size_pages);
            const uint32_t progress_int   = progress_nom / progress_denom;
            const uint32_t progress_frac  = (progress_nom % progress_denom) * 100u / progress_denom;
            SID_PAL_LOG_INFO("Install progress: %u.%02u%%", progress_int, progress_frac);
        }

        /* Propagate the error from the write loop */
        if (err != AIM_ERROR_NONE)
        {
            break;
        }

        /* Erase unused pages from the active and backup area */
        SID_PAL_LOG_INFO("Cleaning up unused space...");

        if (install_image_size_pages < APP_CONFIG_AIM_ACTIVE_SLOT_SIZE_PAGES)
        {
            /* Erase unused space in the active slot. Note: install_image_size_pages and active_image_size_pages are effectively swapped at this point */
            err = erase_flash_pages(install_image_size_pages, APP_CONFIG_AIM_ACTIVE_SLOT_SIZE_PAGES - install_image_size_pages, AIMFAA_ACTIVE_APP_SLOT);
            if (err != AIM_ERROR_NONE)
            {
                break;
            }
        }

        if (AIMFWIT_UPDATE == image_type)
        {
            if ((active_image_size_pages > 0u) && (active_image_size_pages < APP_CONFIG_AIM_ROLLBACK_SLOT_SIZE_PAGES))
            {
                /* Erase unused space in the backup slot. Note: install_image_size_pages and active_image_size_pages are effectively swapped at this point */
                err = erase_flash_pages(active_image_size_pages, APP_CONFIG_AIM_ROLLBACK_SLOT_SIZE_PAGES - active_image_size_pages, AIMFAA_BACKUP_SLOT);
                if (err != AIM_ERROR_NONE)
                {
                    break;
                }
            }

            /* Erase the non-overlapping portion of the update slot */
            const uint32_t overlapping_pages_num = (APP_CONFIG_AIM_STAGING_SLOT_START - APP_CONFIG_AIM_ROLLBACK_SLOT_START) / FLASH_PAGE_SIZE;
            err = erase_flash_pages((APP_CONFIG_AIM_STAGING_SLOT_SIZE_PAGES - overlapping_pages_num), overlapping_pages_num, AIMFAA_STAGING_SLOT);
            if (err != AIM_ERROR_NONE)
            {
                break;
            }

            if (0u == active_image_size_pages)
            {
                /* Erase any residue in the beginning of the backup slot if backup was not performed in this cycle */
                err = erase_flash_pages(0u, overlapping_pages_num, AIMFAA_BACKUP_SLOT);
                if (err != AIM_ERROR_NONE)
                {
                    break;
                }
            }
        }

        /* Final step - set update status in the flash */
        err = store_install_state(AIMIS_UPDATE_CONFIRMATION_PENDING, 0u, 0u);
        if (err != AIM_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set install state to %u. Error %u", (uint32_t)AIMIS_UPDATE_CONFIRMATION_PENDING, (uint32_t)err);
            SID_PAL_LOG_ERROR("Failed to set install state transition %u->%u. Error %u",
                (AIMFWIT_UPDATE == image_type) ? (uint32_t)AIMIS_UPDATE_ONGOING : (uint32_t)AIMIS_ROLLBACK_ONGOING,
                (uint32_t)AIMIS_UPDATE_CONFIRMATION_PENDING,
                (uint32_t)err);
            break;
        }

        /**
         * Transition to Finished state immediately if this is a rollback installation. Confirmation is passed already for a backup image by definition.
         * If this part fails for any reason, a regular confirmation mechanism will kick in and complete the transition.
         */
        if (AIMFWIT_ROLL_BACK == image_type)
        {
            err = store_install_state(AIMIS_FINISHED, 0u, 0u);
            if (err != AIM_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to set install state transition %u->%u. Error %u", (uint32_t)AIMIS_ROLLBACK_ONGOING, (uint32_t)AIMIS_FINISHED, (uint32_t)err);
                break;
            }
        }

        /* Image is now fully written into the active area. Indicate the we are now awaiting a confirmation from the app that it can run normally */
        SID_PAL_LOG_INFO("Installed %s image. Validity confirmation pending", (AIMFWIT_UPDATE == image_type) ? "update" : "rollback");
    } while (0);

    if (err != AIM_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to install %s image. Error %u", (AIMFWIT_UPDATE == image_type) ? "update" : "rollback", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static aim_fw_validity_state_t check_and_install_update_image(const uint32_t active_app_blocks_to_backup)
{
    aim_fw_validity_state_t fw_validity_status;
    uint32_t                fw_image_size_pages;
    aim_error_t             err;

    fw_validity_status = check_fw_image_validity(AIMBM_INSTALL_UPDATE, &fw_image_size_pages);

    if (AIMFWVS_VALID == fw_validity_status)
    {
        err = install_image(AIMFWIT_UPDATE, active_app_blocks_to_backup, fw_image_size_pages);
        if (err != AIM_ERROR_NONE)
        {
            Error_Handler();
        }
        else
        {
            /* Update image was installed successfully and the app can now be started. Rebooting to run all the consistency against the active app slot */
            SID_PAL_LOG_INFO("Rebooting to start the app...");
            SID_PAL_LOG_FLUSH();
            NVIC_SystemReset();
        }
    }

    return fw_validity_status;
}

/*----------------------------------------------------------------------------*/

static aim_fw_validity_state_t check_and_install_rollback_image(void)
{
    aim_fw_validity_state_t fw_validity_status;
    uint32_t                fw_image_size_pages;
    aim_error_t             err;

    fw_validity_status = check_fw_image_validity(AIMBM_ROLL_BACK, &fw_image_size_pages);

    if (AIMFWVS_VALID == fw_validity_status)
    {
        err = install_image(AIMFWIT_ROLL_BACK, 0u, fw_image_size_pages);
        if (err != AIM_ERROR_NONE)
        {
            Error_Handler();
        }
        else
        {
            /* Rollback image was installed successfully and the app can now be started. Rebooting to run all the consistency against the active app slot */
            SID_PAL_LOG_INFO("Rebooting to start the app...");
            SID_PAL_LOG_FLUSH();
            NVIC_SystemReset();
        }
    }

    return fw_validity_status;
}

/*----------------------------------------------------------------------------*/

static void enforce_rollback(void)
{
    aim_error_t err;

    err = store_install_state(AIMIS_ROLLBACK_REQUIRED, 0u, 0u);
    if (err != AIM_ERROR_NONE)
    {
        /* Unrecoverable situation */
        SID_PAL_LOG_ERROR("Failed to enforce rollback: %u", (uint32_t)err);
        Error_Handler();
    }
    else
    {
        SID_PAL_LOG_WARNING("Rebooting to enforce rollback...");
        SID_PAL_LOG_FLUSH();
        NVIC_SystemReset();
    }
}

/*----------------------------------------------------------------------------*/

static void jump_selection_on_power_up(void)
{
    do
    {
        aim_error_t             err;
        aim_install_state_t     stored_state;
        uint32_t                stored_active_image_size_pages;
        uint32_t                stored_install_image_size_pages;
        aim_fw_validity_state_t fw_validity_status = AIMFWVS_UNKNOWN;
        uint32_t                fw_image_size_pages;

        /* Check for past non-finished updates and resume if available */
        err = read_install_state(&stored_state, &stored_active_image_size_pages, &stored_install_image_size_pages);
        if (err != AIM_ERROR_NONE)
        {
            /* Unable to check firmware update status - do not terminate and try to verify and boot the app anyway */
            SID_PAL_LOG_WARNING("Unable to verify firmware update status. Error %u", (uint32_t)err);
        }
        else
        {
            if ((AIMIS_UPDATE_ONGOING == stored_state) || (AIMIS_ROLLBACK_ONGOING == stored_state))
            {
                SID_PAL_LOG_WARNING("Detected unfinished %s image installation...", (AIMIS_UPDATE_ONGOING == stored_state) ? "update" : "rollback");

                /* Resume the installation. Note: it's ok to resume without the image validation since the image integrity will be checked post-install anyway */
                err = install_image((AIMIS_UPDATE_ONGOING == stored_state) ? AIMFWIT_UPDATE : AIMFWIT_ROLL_BACK, stored_active_image_size_pages, stored_install_image_size_pages);
                if (err != AIM_ERROR_NONE)
                {
                    /* Logs provided by install_image() */
                    /* Terminate because the installation is not finished, it's unsafe to try to boot the app */
                    break;
                }

                /* Proceed with booting the app */
                SID_PAL_LOG_INFO("Rebooting to start the app...");
                SID_PAL_LOG_FLUSH();
                NVIC_SystemReset();
            }
            else if (AIMIS_UPDATE_CONFIRMATION_PENDING == stored_state)
            {
                sid_aim_app_validity_state_t app_validity_confirmation;

                /* Validate the active application before any interactions with it */
                fw_validity_status = check_fw_image_validity(AIMBM_START_ACTIVE_APP, &fw_image_size_pages);
                if (fw_validity_status != AIMFWVS_VALID)
                {
                    /* App content is tampered, request immediate rollback */
                    enforce_rollback();
                    break;
                }

                /* Check confirmation marker from the app */
                err = read_confirmation_state(&app_validity_confirmation);
                if (err != AIM_ERROR_NONE)
                {
                    /* Logs provided by read_confirmation_state */
                    /* Self-confirmation status cannot be determined. Enforce rollback since confirmation status area may be written with unexpected data and recovery won't be possible unless corresponding flash page is erased */
                    enforce_rollback();
                    break;
                }

                if (AIMAVS_CONFIRMED == app_validity_confirmation)
                {
                    SID_PAL_LOG_INFO("App confirmed its validity. Updating status in the AIM");
                    err = store_install_state(AIMIS_FINISHED, 0u, 0u);
                    if (err != AIM_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Failed to set install state transition %u->%u. Error %u", (uint32_t)AIMIS_UPDATE_CONFIRMATION_PENDING, (uint32_t)AIMIS_FINISHED, (uint32_t)err);

                        /* Try erasing the metadata area */
                        err = store_install_state(AIMIS_IDLE, 0u, 0u);
                        if (err != AIM_ERROR_NONE)
                        {
                            /* Unrecoverable situation */
                            SID_PAL_LOG_ERROR("Failed to set install state transition %u->%u. Error %u", (uint32_t)AIMIS_UPDATE_CONFIRMATION_PENDING, (uint32_t)AIMIS_IDLE, (uint32_t)err);
                            Error_Handler();
                            break;
                        }
                    }
                }
                else if (AIMAVS_REJECTED == app_validity_confirmation)
                {
                    SID_PAL_LOG_INFO("App self-rejected the update. Proceeding with rollback");
                    enforce_rollback();
                    break;
                }
                else if (AIMAVS_CONFIRMATION_PENDING == app_validity_confirmation)
                {
                    uint32_t boot_counter;

                    err = read_confirmation_boot_counter(&boot_counter);
                    if (err != AIM_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Unable to retrieve app startup counter. Error %u", (uint32_t)err);
                        enforce_rollback();
                        break;
                    }

                    /* Increment boot counter in NVM */
                    if (boot_counter < AIM_APP_CONFIRMATION_ATTEMPT_LIMIT)
                    {
                        err = increment_confirmation_boot_counter();
                        if (err != AIM_ERROR_NONE)
                        {
                            /* Request rollback immediately */
                            SID_PAL_LOG_ERROR("Failed to increment boot counter: %u", (uint32_t)err);
                            enforce_rollback();
                            break;
                        }
                    }

                    /* Ensure boot counter is within the limits - keep in mind that boot_counter value is now less by one than what is stored on the flash */
                    if (boot_counter < (AIM_APP_CONFIRMATION_ATTEMPT_LIMIT / 2))
                    {
                        SID_PAL_LOG_INFO("App is pending confirmation. Boot %u out of %u", (boot_counter + 1u), AIM_APP_CONFIRMATION_ATTEMPT_LIMIT);
                    }
                    else if (boot_counter < AIM_APP_CONFIRMATION_ATTEMPT_LIMIT)
                    {
                        /* Display the same message as a warning because half of the attempts are wasted already */
                        SID_PAL_LOG_WARNING("App is pending confirmation. Boot %u out of %u", (boot_counter + 1u), AIM_APP_CONFIRMATION_ATTEMPT_LIMIT);
                    }
                    else
                    {
                        SID_PAL_LOG_ERROR("App failed to confirm its validity. Enforcing rollback...");
                        enforce_rollback();
                        break;
                    }
                }

                /* Proceed with booting the app */
            }
            else if (AIMIS_ROLLBACK_REQUIRED == stored_state)
            {
                /* Attempt to install the rollback image */
                SID_PAL_LOG_WARNING("Enforced firmware rollback is requested");
                (void)check_and_install_rollback_image(); /* Return value is don't care. Either it deploys rollback and reboots or or there's no valid rollback image - terminate in both cases */
                break;
            }
            else
            {
                /* No traces of an interrupted update - proceed normally */
            }
        }

        /* Active app validity is already evaluated if this portion is reached in AIMIS_UPDATE_CONFIRMATION_PENDING state */
        if (stored_state != AIMIS_UPDATE_CONFIRMATION_PENDING)
        {
            /* Check if a valid active application is available */
            fw_validity_status = check_fw_image_validity(AIMBM_START_ACTIVE_APP, &fw_image_size_pages);
        }
        if (AIMFWVS_VALID == fw_validity_status)
        {
            if (AIMIS_INVALID == stored_state)
            {
                sid_aim_app_validity_state_t app_validity_confirmation;

                /* OTA status metadata is corrupted, by we found a valid active app - let's check the self-confirmation marker set by the app */
                err = read_confirmation_state(&app_validity_confirmation);
                if ((AIM_ERROR_NONE == err) && (AIMAVS_CONFIRMED == app_validity_confirmation))
                {
                    /* Try erasing the metadata area as a recovery action */
                    (void)store_install_state(AIMIS_IDLE, 0u, 0u);
                }
            }
            /* A valid active app is available. Boot it */
            user_app_pre_entry_actions();
            AIM_BootUserApp();
            break;
        }

        /* The image is the active slot is unusable, but no explicit update or rollback is requested - check for other options */

        /* Check if we can proceed with an update */
        SID_PAL_LOG_INFO("Active image is invalid, checking if an update image is available");
        fw_validity_status = check_and_install_update_image(0u);
        if (AIMFWVS_VALID == fw_validity_status)
        {
            /* Normally this shall not happen, but just in case - terminate the processing since the flow is abnormal */
            break;
        }

        /* Check if we can proceed with a rollback */
        SID_PAL_LOG_INFO("Update image is invalid, checking if a rollback image is available");
        fw_validity_status = check_and_install_rollback_image();
        if (AIMFWVS_VALID == fw_validity_status)
        {
            /* Normally this shall not happen, but just in case - terminate the processing since the flow is abnormal */
            break;
        }
    } while (0);
}

/*----------------------------------------------------------------------------*/

static void user_app_pre_entry_actions(void)
{
    SID_PAL_LOG_INFO("Starting user application at 0x%08X...", APP_CONFIG_AIM_ACTIVE_SLOT_START);

    /* De-initialize all the peripherals used by AIM to avoid conflicts with the application */
    (void)HAL_CRC_DeInit(&hcrc);

    /* Ensure all the logs are printed out */
    SID_PAL_LOG_FLUSH();

#if (CFG_LOG_SUPPORTED)
    /* Initialize logging subsystem */
    UTIL_ADV_TRACE_DeInit();
#endif /* CFG_LOG_SUPPORTED */

    /* Ensure the above actions are not reordered by the compiler */
    __COMPILER_BARRIER();

    /* Perform DMA deinitialization only after logging functionality is deactivated */
    __HAL_RCC_GPDMA1_CLK_DISABLE();

    /* Switch back to HSI clock as the app may expect to see it as SysClock source on boot */
    switch_to_hsi_clock();

    /* Disable ICACHE as the app may expect it to be disabled on boot */
    LL_ICACHE_ClearFlag_BSYEND();
    LL_ICACHE_Disable();
    while (LL_ICACHE_IsEnabled() != 0u)
    {
        __NOP();
    }

    // TODO: reset used peripherals?
    // TODO: erase RAM for security?
}

/* Global function definitions -----------------------------------------------*/

/**
 * Jump to existing FW App in flash
 * It never returns
 */
SID_STM32_STACKLESS_FUNCTION void AIM_BootUserApp(void)
{
    SCB->VTOR = APP_CONFIG_AIM_ACTIVE_SLOT_VTOR_START;
    __set_MSP(*(uint32_t*)(void *)APP_CONFIG_AIM_ACTIVE_SLOT_ESTACK);

    /* Use assembly instruction for unconditional jump to prevent C compiler from updating link register wit an address in AIM space */
    __ASM volatile(
        "mov r0, %1   \n" /* Store RCC reset flags on entry to r0 so that the user app can receive the original RCC state */
        "mov r1, %0   \n" /* Load the location of the Reset_Handler pointer */
        "ldr r1, [r1] \n" /* Load Reset_Handler address */
        "bx r1        \n" /* Jump into the app's Reset_Handler */
        :
        : "r" (APP_CONFIG_AIM_ACTIVE_SLOT_ENTRY_POINT), "r" (rcc_reset_flags)
        : "r1", "memory"
    );
}

/*----------------------------------------------------------------------------*/

void AIM_Init(void)
{
    /* Enable instruction cache in 1-way (direct mapped cache) */
    LL_ICACHE_SetMode(LL_ICACHE_1WAY);
    LL_ICACHE_Enable();

    /* Go to 96MHz clock since cryptographic functions put high computational load */
    switch_to_pll_clock();

    (void)TIMER_IF_Init();

#if (CFG_LOG_SUPPORTED)
    /* Initialize logging subsystem */
    UTIL_ADV_TRACE_Init();

    /* Ensure the output will start from a new line */
    UTIL_ADV_TRACE_FSend(SID_PAL_LOG_LINE_ENDING);

    /* Printout AIM version info */
    SID_PAL_LOG_INFO("AIM version %s", SID_APP_PROJECT_VERSION_STRING);
    SID_PAL_LOG_INFO("AIM build type: %s", SID_APP_PROJECT_BUILD_TYPE);
    SID_PAL_LOG_INFO("AIM commit hash: %s", SID_APP_PROJECT_COMMIT_HASH_STRING);
    SID_PAL_LOG_INFO("AIM commit description: %s", SID_APP_PROJECT_COMMIT_DESCRIPTION);

    /* CubeMX pack version */
    const hal_version_info_t cubemx_fw_pack_ver = {
        .raw = HAL_GetHalVersion(),
    };
    SID_PAL_LOG_INFO("STM32CubeWBA: %u.%u.%u", cubemx_fw_pack_ver.major, cubemx_fw_pack_ver.minor, cubemx_fw_pack_ver.patch);

    /* Printout MCU details */
    stm32_mcu_info_t mcu_info = stm32_mcu_info_describe_host();
    SID_PAL_LOG_INFO("Host MCU: %s (0x%X), revision: %s (0x%X)", mcu_info.device_name, mcu_info.device_id, mcu_info.rev_name, mcu_info.rev_id);
#endif /* CFG_LOG_SUPPORTED */

    /* Flash size check */
    const uint32_t actual_flash_size_kb = FLASH_SIZE / 1024u;
    const uint32_t expected_flash_size_kb = APP_CONFIG_TOTAL_FLASH_SIZE / 1024u;
    if (actual_flash_size_kb != expected_flash_size_kb)
    {
        SID_PAL_LOG_ERROR("Configuration mismatch: AIM configured for %u%s flash size, but the actual size is %u%s",
            expected_flash_size_kb >= 1024u ? expected_flash_size_kb / 1024u : expected_flash_size_kb,
            expected_flash_size_kb >= 1024u ? "MB" : "KB",
            actual_flash_size_kb >= 1024u ? actual_flash_size_kb / 1024u : actual_flash_size_kb,
            actual_flash_size_kb >= 1024u ? "MB" : "KB"
        );

        Error_Handler();
    }
    else
    {
        SID_PAL_LOG_INFO("Detected flash size: %u%s",
            actual_flash_size_kb >= 1024u ? actual_flash_size_kb / 1024u : actual_flash_size_kb,
            actual_flash_size_kb >= 1024u ? "MB" : "KB"
        );
    }

    /* Initialize cryptographic library */
    cmox_init_arg_t cmox_init_cfg = {
        .target = CMOX_INIT_TARGET_WBA,
        .pArg   = NULL,
    };
    cmox_init_retval_t cmox_ret = cmox_initialize(&cmox_init_cfg);
    if (cmox_ret != CMOX_INIT_SUCCESS)
    {
        SID_PAL_LOG_ERROR("Failed to initialize CMOX");
        Error_Handler();
    }
}

/*----------------------------------------------------------------------------*/

void AIM_Entry(void)
{
    const char * reset_reason_str;
    uint32_t     check_update_request = FALSE;

    /* Capture the reset flags reported by RCC */
    rcc_reset_flags = READ_REG(RCC->CSR);

    /* Normally RCC_CSR_PINRSTF is always set, so filter it out to simplify the checks below */
    const uint32_t rcc_reset_flags_without_pin_reset = rcc_reset_flags & (~RCC_CSR_PINRSTF);

    /* Clear the reset flags in RCC to ensure reliable reset reason detection on the next boot */
    LL_RCC_ClearResetFlags();

    if ((rcc_reset_flags & RCC_CSR_PINRSTF) == 0u)
    {
        SID_PAL_LOG_WARNING("Can't reliably detect reset reason, ambiguous reset flags: 0x%08X", rcc_reset_flags);
        reset_reason_str = NULL;
    }
    else if ((RCC_CSR_BORRSTF) == rcc_reset_flags_without_pin_reset)
    {
        reset_reason_str = "Power-on/Brownout";
    }
    else if ((RCC_CSR_PINRSTF) == rcc_reset_flags)
    {
        reset_reason_str = "External NRST";
    }
    else if ((RCC_CSR_SFTRSTF) == rcc_reset_flags_without_pin_reset)
    {
        reset_reason_str = "Software";
        check_update_request = TRUE;
    }
    else if (((RCC_CSR_OBLRSTF) == rcc_reset_flags_without_pin_reset) || ((RCC_CSR_OBLRSTF | RCC_CSR_SFTRSTF) == rcc_reset_flags_without_pin_reset))
    {
        /**
         * OBL reset is triggered when the option bytes are reloaded, including (but not limited to) the bank swap operation on STM32WBA6
         *
         * @attention If you decide to go with the bank swap, make sure you disable LSE clock before requesting the swap operation.
         *            LSE trim is loaded from the option bytes, keeping LSE enabled during the reload may result in undefined behavior.
         */
        reset_reason_str = "Option Byte Loader";
#  ifdef STM32WBA6x
        /* OBL reset may indicate a firmware update request for the WBA6 family only. WBA5 has no support for dual-bank mode */
        check_update_request = TRUE;
#  endif /* STM32WBA6x */
    }
    else if (((RCC_CSR_IWDGRSTF) == rcc_reset_flags_without_pin_reset) || ((RCC_CSR_WWDGRSTF) == rcc_reset_flags_without_pin_reset) || ((RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF) == rcc_reset_flags_without_pin_reset))
    {
        reset_reason_str = "Watchdog";
    }
    else if ((RCC_CSR_LPWRRSTF) == rcc_reset_flags_without_pin_reset)
    {
        reset_reason_str = "Illegal LPM";
    }
    else
    {
        /* This situation may happen if AIM or user app fail to reach the point at which the reset flags are cleared in RCC */
        SID_PAL_LOG_WARNING("Can't reliably detect reset reason, ambiguous reset flags: 0x%08X", rcc_reset_flags);
        reset_reason_str = NULL;
    }

    /* Printout the reset reason if it was successfully identified */
    if (reset_reason_str != NULL)
    {
        SID_PAL_LOG_INFO("Reset reason: %s Reset", reset_reason_str);
    }

    /**
     * Check the Boot mode request
     * Depending on the result, the CPU may either jump to an existing application in the user flash
     * or keep on running the code to start the OTA loader
     */
    if (FALSE == check_update_request)
    {
        /* This is a regular boot with no request to install an update, trying to start the active app directly */
        jump_selection_on_power_up();
    }
    else
    {
        /* A software reset is detected and RAM contents is preserved - check if tehre's an explicit request to run firmware update */
        if ((*(SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD0_ADDR) == SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD0) && (*(SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD1_ADDR) == SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD1))
        {
            aim_fw_validity_state_t active_fw_validity_status;
            aim_fw_validity_state_t update_fw_validity_status;
            uint32_t                active_fw_image_size_pages;
            uint32_t                update_fw_image_size_pages;
            aim_error_t             err;

            SID_PAL_LOG_INFO("Update install requested present in RAM");

            /* Clear RAM contents to avoid infinite reset loops */
            *SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD0_ADDR = 0u;
            *SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD1_ADDR = 0u;

            /* Check the validity of the update image */
            update_fw_validity_status = check_fw_image_validity(AIMBM_INSTALL_UPDATE, &update_fw_image_size_pages);
            if (update_fw_validity_status != AIMFWVS_VALID)
            {
                SID_PAL_LOG_ERROR("Aborting installation due to corrupted install image");

                /* Erase the entire staging slot */
                (void)erase_flash_pages(0u, (APP_CONFIG_AIM_STAGING_SLOT_END - APP_CONFIG_AIM_STAGING_SLOT_START) / FLASH_PAGE_SIZE, AIMFAA_STAGING_SLOT);

                /* Proceed with a regular startup */
                jump_selection_on_power_up();
            }

            /* Check the validity of the active app. Backup only makes sense if it is valid */
            active_fw_validity_status = check_fw_image_validity(AIMBM_START_ACTIVE_APP, &active_fw_image_size_pages);

            if (AIMFWVS_VALID == active_fw_validity_status)
            {
                // TODO: implement version check and downgrade protection
            }

            /* Proceed with the installation */
            err = install_image(AIMFWIT_UPDATE, active_fw_image_size_pages, update_fw_image_size_pages);
            if (err != AIM_ERROR_NONE)
            {
                Error_Handler();
            }
            else
            {
                /* Update image was installed successfully and the app can now be started. Rebooting to run all the consistency against the active app slot */
                SID_PAL_LOG_INFO("Rebooting to start the app...");
                SID_PAL_LOG_FLUSH();
                NVIC_SystemReset();
            }
        }
        else
        {
            /* RAM does not contain any recognizable request. Proceed with a regular boot procedure */
            jump_selection_on_power_up();
        }
    }

    SID_PAL_LOG_ERROR("Unable to find a bootable app image");
    Error_Handler();
}

/*----------------------------------------------------------------------------*/

cmox_init_retval_t cmox_ll_init(void * pArg)
{
    (void)pArg;
    return CMOX_INIT_SUCCESS;
}

/*----------------------------------------------------------------------------*/

cmox_init_retval_t cmox_ll_deInit(void * pArg)
{
    (void)pArg;
    return CMOX_INIT_SUCCESS;
}
