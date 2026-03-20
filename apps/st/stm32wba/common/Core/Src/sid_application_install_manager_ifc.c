/**
  ******************************************************************************
  * @file    sid_application_install_manager_ifc.c
  * @brief   Application Install Manager interface for Sidewalk applications
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

#include "sid_application_install_manager_ifc.h"

#include <sid_error.h>
#include <sid_hal_reset_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_critical_region_ifc.h>

#include <cmsis_compiler.h>
#include <cmsis_os.h>

#include <common_memory_symbols.h>

#include <flash_manager.h>
#include <stm32wbaxx.h>
#include <stm32_rtos.h>

/* Imported variables --------------------------------------------------------*/

extern sid_aim_validity_marker_t app_ota_validity_marker;

typedef struct {
    uint32_t erase_start_page;
    uint32_t erase_pages_num;
} sid_aim_ifc_flash_erase_op_params_t;

typedef struct {
    uint32_t * src_addr;
    uint32_t * dst_addr;
    uint32_t   size_words;
} sid_aim_ifc_flash_write_op_params_t;

/* Private constants ---------------------------------------------------------*/

static const uint32_t validity_magic_word[SID_AIM_IFC_FLASH_WORD_SIZE / sizeof(uint32_t)]   = SID_AIM_IFC_VALIDITY_MARKER_MAGIC_WORD_INITIALIZER;
static const uint32_t validity_failure_word[SID_AIM_IFC_FLASH_WORD_SIZE / sizeof(uint32_t)] = {0u};
static const uint32_t validity_pending_word[SID_AIM_IFC_FLASH_WORD_SIZE / sizeof(uint32_t)] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, };

static const osSemaphoreAttr_t flash_lock_sem_attributes = {
    .name       = "AIM Flash Lock Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t flash_erase_sem_attributes = {
    .name       = "AIM Erase Complete Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t flash_write_sem_attributes = {
    .name       = "AIM Write Complete Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

/* Private function prototypes -----------------------------------------------*/

static void aim_ifc_confirmation_fm_callback(FM_FlashOp_Status_t Status);
static void aim_ifc_ota_block_erase_fm_callback(FM_FlashOp_Status_t Status);
static void aim_ifc_ota_block_write_fm_callback(FM_FlashOp_Status_t Status);

/* Private variables ---------------------------------------------------------*/

static FM_CallbackNode_t aim_ifc_confirmation_fm_callback_node = {
    .Callback = aim_ifc_confirmation_fm_callback,
};
static uint8_t confirmation_write_pending = FALSE;

static osSemaphoreId_t flash_lock_semaphore;

static FM_CallbackNode_t aim_ifc_ota_block_erase_fm_callback_node = {
    .Callback = aim_ifc_ota_block_erase_fm_callback,
};
static sid_aim_ifc_flash_erase_op_params_t ota_erase_params;
static osSemaphoreId_t flash_erase_semaphore;
static FM_Cmd_Status_t fm_erase_cmd_status;

static FM_CallbackNode_t aim_ifc_ota_block_write_fm_callback_node = {
    .Callback = aim_ifc_ota_block_write_fm_callback,
};
static sid_aim_ifc_flash_write_op_params_t ota_write_params;
static osSemaphoreId_t flash_write_semaphore;
static FM_Cmd_Status_t fm_write_cmd_status;

static uint8_t sid_aim_ifc_initialized = FALSE;

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void aim_ifc_confirmation_fm_callback(FM_FlashOp_Status_t Status)
{
    if (FM_OPERATION_COMPLETE == Status)
    {
        if (sid_aim_ifc_get_app_verification_state() == AIMAVS_REJECTED)
        {
            /* Request an immediate reset to proceed with the rollback */
            SID_PAL_LOG_WARNING("Application OTA marked as rejected. Rebooting to start rollback");
            (void)sid_hal_reset(SID_HAL_RESET_NORMAL);
        }
        else
        {
            SID_PAL_LOG_INFO("Application OTA confirmation marker successfully set");
            confirmation_write_pending = FALSE;
        }
    }
    else
    {
        /* Reschedule the operation */
        (void)FM_Write((uint32_t *)&validity_magic_word[0], &app_ota_validity_marker.validity_magic_word[0], sizeof(app_ota_validity_marker.validity_magic_word) / sizeof(uint32_t), &aim_ifc_confirmation_fm_callback_node);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void aim_ifc_ota_block_erase_fm_callback(FM_FlashOp_Status_t Status)
{
    if (FM_OPERATION_COMPLETE == Status)
    {
        fm_erase_cmd_status = FM_OK;
        (void)osSemaphoreRelease(flash_erase_semaphore);
    }
    else
    {
        FM_Cmd_Status_t fm_status;

        /* Reschedule the operation */
        sid_pal_enter_critical_region();
        fm_status = FM_Erase(ota_erase_params.erase_start_page, ota_erase_params.erase_pages_num, &aim_ifc_ota_block_erase_fm_callback_node);
        sid_pal_exit_critical_region();

        if (fm_status != FM_OK)
        {
            /* Failed to reschedule the erase operation */
            fm_erase_cmd_status = fm_status;
            (void)osSemaphoreRelease(flash_erase_semaphore);
        }
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void aim_ifc_ota_block_write_fm_callback(FM_FlashOp_Status_t Status)
{
    if (FM_OPERATION_COMPLETE == Status)
    {
        fm_write_cmd_status = FM_OK;
        (void)osSemaphoreRelease(flash_write_semaphore);
    }
    else
    {
        FM_Cmd_Status_t fm_status;

        /* Reschedule the operation */
        sid_pal_enter_critical_region();
        fm_status = FM_Write(ota_write_params.src_addr, ota_write_params.dst_addr, ota_write_params.size_words, &aim_ifc_ota_block_write_fm_callback_node);
        sid_pal_exit_critical_region();

        if (fm_status != FM_OK)
        {
            /* Failed to reschedule the write operation */
            fm_write_cmd_status = fm_status;
            (void)osSemaphoreRelease(flash_write_semaphore);
        }
    }
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_aim_ifc_confirm_app_is_verified(void)
{
    sid_error_t     err;
    FM_Cmd_Status_t fm_err;

    sid_pal_enter_critical_region();

    do
    {
        /* Ensure write operation is not scheduled already */
        if (confirmation_write_pending != FALSE)
        {
            err = SID_ERROR_IN_PROGRESS;
            break;
        }

        const sid_aim_app_validity_state_t current_state = sid_aim_ifc_get_app_verification_state();

        /* Check if the marker actually needs to be set */
        if (AIMAVS_CONFIRMED == current_state)
        {
            /* Confirmation marker is set already, no actions required */
            err = SID_ERROR_NONE;
            break;
        }
        else if (current_state != AIMAVS_CONFIRMATION_PENDING)
        {
            /* Flash is already written with some value, writing the confirmation marker is not possible */
            err = SID_ERROR_INVALID_STATE;
            break;
        }
        else
        {
            /* Proceed with update */
        }

        /* Schedule flash write operation */
        fm_err = FM_Write((uint32_t *)&validity_magic_word[0], &app_ota_validity_marker.validity_magic_word[0], sizeof(app_ota_validity_marker.validity_magic_word) / sizeof(uint32_t), &aim_ifc_confirmation_fm_callback_node);
        if (FM_ERROR == fm_err)
        {
            SID_PAL_LOG_ERROR("Failed to mark app as verified for AIM");
            err = SID_ERROR_STORAGE_WRITE_FAIL;
            break;
        }

        /* Indicate flash write is pending */
        confirmation_write_pending = TRUE;
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_aim_ifc_set_app_verification_failed(void)
{
    sid_error_t     err;
    FM_Cmd_Status_t fm_err;

    sid_pal_enter_critical_region();

    do
    {
        /* Ensure write operation is not scheduled already */
        if (confirmation_write_pending != FALSE)
        {
            err = SID_ERROR_IN_PROGRESS;
            break;
        }

        if (sid_aim_ifc_get_app_verification_state() == AIMAVS_REJECTED)
        {
            /* Confirmation marker is set already, no actions required */
            err = SID_ERROR_NONE;
            break;
        }

        /* Schedule flash write operation */
        fm_err = FM_Write((uint32_t *)&validity_failure_word[0], &app_ota_validity_marker.validity_magic_word[0], sizeof(app_ota_validity_marker.validity_magic_word) / sizeof(uint32_t), &aim_ifc_confirmation_fm_callback_node);
        if (FM_ERROR == fm_err)
        {
            SID_PAL_LOG_ERROR("Failed to mark app as verified for AIM");
            err = SID_ERROR_STORAGE_WRITE_FAIL;
            break;
        }

        /* Indicate flash write is pending */
        confirmation_write_pending = TRUE;
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_aim_app_validity_state_t sid_aim_ifc_get_app_verification_state(void)
{
    sid_aim_app_validity_state_t state;

    sid_pal_enter_critical_region();

    if (SID_STM32_UTIL_fast_memcmp(validity_magic_word, app_ota_validity_marker.validity_magic_word, sizeof(app_ota_validity_marker.validity_magic_word)) == 0u)
    {
        state = AIMAVS_CONFIRMED;
    }
    else if (SID_STM32_UTIL_fast_memcmp(validity_failure_word, app_ota_validity_marker.validity_magic_word, sizeof(app_ota_validity_marker.validity_magic_word)) == 0u)
    {
        state = AIMAVS_REJECTED;
    }
    else if (SID_STM32_UTIL_fast_memcmp(validity_pending_word, app_ota_validity_marker.validity_magic_word, sizeof(app_ota_validity_marker.validity_magic_word)) == 0u)
    {
        state = AIMAVS_CONFIRMATION_PENDING;
    }
    else
    {
        state = AIMAVS_CONFIRMATION_INVALID;
    }

    sid_pal_exit_critical_region();

    return state;
}

/*----------------------------------------------------------------------------*/

__NO_RETURN SID_STM32_STACKLESS_FUNCTION void sid_aim_ifc_boot_in_dfu_mode(void)
{
    /**
     * @attention The interface between the app and AIM uses two first RAM words to store requests.
     * Since the reference design of the Sidewalk apps uses this area for stack, it is important to
     * ensure no stack operations are executed after the magic words are written.
     */
    *SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD0_ADDR = SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD0;
    *SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD1_ADDR = SID_AIM_IFC_OTA_REQUEST_MAGIC_WORD1;
    __DSB();

    __ASM volatile(
        "ldr r0, =%0\n"  /* load address of the NVIC_SystemReset() function */
        "bx  r0\n"       /* jump there, no link, no stack use */
        :
        : "i"(NVIC_SystemReset)
    );
}

/*----------------------------------------------------------------------------*/

int32_t sid_aim_ifc_init(void)
{
    sid_error_t err;

    do
    {
        if (sid_aim_ifc_initialized != FALSE)
        {
            err = SID_ERROR_NONE;
            break;
        }

        /* Create a semaphore to lock flash access */
        flash_lock_semaphore = osSemaphoreNew(1u, 1u, &flash_lock_sem_attributes);
        if (NULL == flash_lock_semaphore)
        {
            SID_PAL_LOG_ERROR("Can't create %s semaphore. No memory", flash_lock_sem_attributes.name);
            err = SID_ERROR_OOM;
            break;
        }

        /* Create a semaphore to monitor flash erase operation completion */
        flash_erase_semaphore = osSemaphoreNew(1u, 0u, &flash_erase_sem_attributes);
        if (NULL == flash_erase_semaphore)
        {
            SID_PAL_LOG_ERROR("Can't create %s semaphore. No memory", flash_erase_sem_attributes.name);
            err = SID_ERROR_OOM;
            break;
        }

        /* Create a semaphore to monitor Sidewalk registration status */
        flash_write_semaphore = osSemaphoreNew(1u, 0u, &flash_write_sem_attributes);
        if (NULL == flash_write_semaphore)
        {
            SID_PAL_LOG_ERROR("Can't create %s semaphore. No memory", flash_write_sem_attributes.name);
            err = SID_ERROR_OOM;
            break;
        }

        /* Done */
        sid_aim_ifc_initialized = TRUE;
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

int32_t sid_aim_ifc_deinit(void)
{
    sid_aim_ifc_initialized = FALSE;

    if (flash_erase_semaphore != NULL)
    {
        (void)osSemaphoreDelete(flash_erase_semaphore);
        flash_erase_semaphore = NULL;
    }

    if (flash_write_semaphore != NULL)
    {
        (void)osSemaphoreDelete(flash_write_semaphore);
        flash_write_semaphore = NULL;
    }

    if (flash_lock_semaphore != NULL)
    {
        (void)osSemaphoreDelete(flash_lock_semaphore);
        flash_lock_semaphore = NULL;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

int32_t sid_aim_ifc_store_ota_data_chunk(void * data, const uint32_t offset, const size_t size)
{
    sid_error_t err;
    uint32_t    lock_acquired = FALSE;

    do
    {
        osStatus_t      os_status;
        FM_Cmd_Status_t fm_status;
        const uint32_t  write_start_addr = APP_CONFIG_AIM_STAGING_SLOT_START + offset;

        /* Validate inputs */
        if ((NULL == data) || (0u == size) || ((offset + size) > APP_CONFIG_AIM_STAGING_SLOT_SIZE))
        {
            SID_PAL_LOG_ERROR("OTA chunk write rejected, invalid parameters. Offset: %u, size: %u, src addr: 0x%08X", offset, size, (uint32_t)data);
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* STM32WBA5x can write by 16 bytes chunks only */
        if ((write_start_addr % SID_AIM_IFC_FLASH_WORD_SIZE) != 0u)
        {
            SID_PAL_LOG_ERROR("OTA chunk write rejected, start address 0x%08X is not aligned to flash word size (16 bytes)", write_start_addr);
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if ((size % SID_AIM_IFC_FLASH_WORD_SIZE) != 0u)
        {
            SID_PAL_LOG_ERROR("OTA chunk write rejected, write length of %u is not aligned to flash word size (16 bytes)", size);
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        os_status = osSemaphoreAcquire(flash_lock_semaphore, osWaitForever);
        if (osErrorTimeout == os_status)
        {
            err = SID_ERROR_BUSY;
            break;
        }
        else if (os_status != osOK)
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }
        else
        {
            /* Proceed normally */
            lock_acquired = TRUE;
        }

        /* Erase new pages that will be written by current data chunk */
        const uint32_t page_aligned_erase_start_offset = (offset + (FLASH_PAGE_SIZE - 1u)) & (FLASH_PAGE_SIZE - 1u);         /* Offset of the first new page that will be written */
        const uint32_t page_aligned_erase_end_offset   = (offset + size + (FLASH_PAGE_SIZE - 1u)) & (FLASH_PAGE_SIZE - 1u);  /* Offset past the last page that is touched by current operation */
        const uint32_t pages_to_erase = (page_aligned_erase_end_offset - page_aligned_erase_start_offset) / FLASH_PAGE_SIZE; /* Total number of pages to be erased before writing */

        if (pages_to_erase != 0u)
        {
            const uint32_t erase_start_page_idx = ((APP_CONFIG_AIM_STAGING_SLOT_START - FLASH_BASE) + page_aligned_erase_start_offset) / FLASH_PAGE_SIZE;
            sid_pal_enter_critical_region();
            ota_erase_params.erase_start_page = erase_start_page_idx;
            ota_erase_params.erase_pages_num  = pages_to_erase;
            fm_status = FM_Erase(ota_erase_params.erase_start_page, ota_erase_params.erase_pages_num, &aim_ifc_ota_block_erase_fm_callback_node);
            sid_pal_exit_critical_region();
            if (fm_status != FM_OK)
            {
                err = SID_ERROR_STORAGE_ERASE_FAIL;
                break;
            }

            /* Wait for the erase operation to complete */
            os_status = osSemaphoreAcquire(flash_erase_semaphore, osWaitForever);
            if (osErrorTimeout == os_status)
            {
                err = SID_ERROR_BUSY;
                break;
            }
            else if (os_status != osOK)
            {
                err = SID_ERROR_INVALID_STATE;
                break;
            }
            else
            {
                /* Proceed normally */
            }

            /* Check write operation status */
            if (fm_erase_cmd_status != FM_OK)
            {
                SID_PAL_LOG_ERROR("Failed to erase OTA pages %u-%u", erase_start_page_idx, (erase_start_page_idx + pages_to_erase));
                err = SID_ERROR_STORAGE_WRITE_FAIL;
                break;
            }
        }

        /* Now proceed with writing data */
        sid_pal_enter_critical_region();
        ota_write_params.src_addr   = (uint32_t *)data;
        ota_write_params.dst_addr   = (uint32_t *)(void *)write_start_addr;
        ota_write_params.size_words = size / sizeof(uint32_t);
        fm_status = FM_Write(ota_write_params.src_addr, ota_write_params.dst_addr, ota_write_params.size_words, &aim_ifc_ota_block_write_fm_callback_node);
        sid_pal_exit_critical_region();
        if (fm_status != FM_OK)
        {
            err = SID_ERROR_STORAGE_WRITE_FAIL;
            break;
        }

        /* Wait for the write operation to complete */
        os_status = osSemaphoreAcquire(flash_write_semaphore, osWaitForever);
        if (osErrorTimeout == os_status)
        {
            err = SID_ERROR_BUSY;
            break;
        }
        else if (os_status != osOK)
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }
        else
        {
            /* Proceed normally */
        }

        /* Check write operation status */
        if (fm_write_cmd_status != FM_OK)
        {
            SID_PAL_LOG_ERROR("Failed to store OTA chunk (%u bytes) at address 0x%08X", size, write_start_addr);
            err = SID_ERROR_STORAGE_WRITE_FAIL;
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if (lock_acquired != FALSE)
    {
        /* Release the flash access lock to allow other flash operations */
        (void)osSemaphoreRelease(flash_lock_semaphore);
    }

    return err;
}
