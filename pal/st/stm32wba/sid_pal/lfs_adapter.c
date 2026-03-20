/**
  ******************************************************************************
  * @file    lfs_adapter.c
  * @brief   Bridge layer for LittleFS
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

#include <stdbool.h>
#include <string.h>

#include "lfs_adapter.h"
#include "app_conf.h"

/* Sidewalk Interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>

/* LittleFS Middleware */
#include "lfs.h"
#include "lfs_port.h"

/* Utilities */
#include <cmsis_os2.h>
#include <sid_stm32_common_utils.h>

#ifdef LFS_ADAPTER_USE_STDLIB_PRINT
#  include <stdio.h>
#else
#  include "stm32_tiny_vsnprintf.h"
#endif /* LFS_ADAPTER_USE_STDLIB_PRINT */

/* Private defines -----------------------------------------------------------*/

#define LFS_ADAPTER_MAX_FNAME_LEN    ((LFS_NAME_MAX) + 1u)
#define LFS_ADAPTER_MAX_FSIZE        (4096U - sizeof(FStoreHeader_t))

#ifdef LFS_ADAPTER_USE_STDLIB_PRINT
#  define LFS_ADAPTER_VSNPRINTF(...) vsnprintf(__VA_ARGS__)
#  define LFS_ADAPTER_SNPRINTF(...)  snprintf(__VA_ARGS__)
#else
#  define LFS_ADAPTER_VSNPRINTF(...) tiny_vsnprintf_like(__VA_ARGS__)
#  define LFS_ADAPTER_SNPRINTF(...)  _prv_snprintf_like(__VA_ARGS__)
#endif /* LFS_ADAPTER_USE_STDLIB_PRINT */

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
    uint16_t len;
} FStoreHeader_t;

/* Private variables ---------------------------------------------------------*/

// configuration of the filesystem is provided by this struct
static lfs_t lfs;
const struct lfs_config * p_cfg = NULL;
static bool sid_pal_internal_fstorage_initialized = false;

/* Private function prototypes -----------------------------------------------*/

static inline void lfs_size_to_err(int32_t * const p_return_value, const size_t expected_length);
#ifndef LFS_ADAPTER_USE_STDLIB_PRINT
static inline int  _prv_snprintf_like(char * buf, const size_t size, const char *fmt, ...);
#endif /* LFS_ADAPTER_USE_STDLIB_PRINT */

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void lfs_size_to_err(int32_t * const p_return_value, const size_t expected_length)
{
    if (*p_return_value == expected_length)
    {
        *p_return_value = LFS_ERR_OK;
    }
    else if (*p_return_value >= 0)
    {
        *p_return_value = LFS_ERR_CORRUPT;
    }
    else
    {
        /* Pass through the error code otherwise */
    }
}

/*----------------------------------------------------------------------------*/

#ifndef LFS_ADAPTER_USE_STDLIB_PRINT
SID_STM32_SPEED_OPTIMIZED static inline int _prv_snprintf_like(char * buf, const size_t size, const char *fmt, ...)
{
    va_list args;
    int print_len;

    va_start(args, fmt);
    print_len = LFS_ADAPTER_VSNPRINTF(buf, size, fmt, args);
    va_end(args);

    return print_len;
}
#endif /* LFS_ADAPTER_USE_STDLIB_PRINT */

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_init(void)
{
    int32_t err = LFS_ERR_OK;

    if(sid_pal_internal_fstorage_initialized)
    {
        return err;
    }

    /* internal flash initialize */
    p_cfg = initialize_internal_flash_fs_static();
    if(p_cfg == NULL)
    {
        err = LFS_ERR_IO;
    }

    /* mount the filesystem */
    if (err == LFS_ERR_OK)
    {
        err = lfs_mount(&lfs, p_cfg);
    }

    /* format if we can't mount the filesystem
     * this should only happen on the first boot
     */
    if (err != LFS_ERR_OK)
    {
        SID_PAL_LOG_DEBUG("lfs: Failed to mount LFS partition. Formatting...");

        err = lfs_format(&lfs, p_cfg);

        if (err == 0)
        {
            err = lfs_mount(&lfs, p_cfg);
        }

        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Failed to format LFS partition");
        }
    }

    if (err == LFS_ERR_OK)
    {
        sid_pal_internal_fstorage_initialized = true;
    }
    else
    {
        sid_pal_internal_fstorage_initialized = false;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_deinit(void)
{
    int32_t err = LFS_ERR_OK;
    if (!sid_pal_internal_fstorage_initialized)
    {
        return err;
    }

    /* unmount the filesystem */
    if (err == LFS_ERR_OK)
    {
        err = lfs_unmount(&lfs);
    }

    if (err == LFS_ERR_OK)
    {
        sid_pal_internal_fstorage_initialized = false;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_write_value(const char * const p_dir, const char * const p_name, const void * const p_buf, const uint32_t buf_len)
{
    struct lfs_info dir_info = { 0 };
    char file_name[ LFS_ADAPTER_MAX_FNAME_LEN + 1] = { 0 };
    bool file_open_flag = false;
    lfs_file_t fil = { 0 };
    int32_t err = LFS_ERR_INVAL;

    do
    {
        /* Validate inputs */
        if ((NULL == p_dir) || (NULL == p_name) || (NULL == p_buf) || (buf_len > LFS_ADAPTER_MAX_FSIZE))
        {
            err = LFS_ERR_INVAL;
            break;
        }

        /* Check if the directory exists */
        err = lfs_stat(&lfs, p_dir, &dir_info);

        /* Create directory if it does not exist */
        if (LFS_ERR_NOENT == err)
        {
            err = lfs_mkdir(&lfs, p_dir);
        }

        /* Failed to locate or create directory */
        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Failed to open/create directory %s. Error %d", p_dir, err);
            break;
        }

        /* Construct file name */
        (void)LFS_ADAPTER_SNPRINTF(file_name, sizeof(file_name), "%s/%s", p_dir, p_name);

        /* Open the file */
        err = lfs_file_open(&lfs, &fil, file_name, LFS_O_WRONLY | LFS_O_TRUNC | LFS_O_CREAT);
        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Error %d while opening file %s for writing", err, file_name);
            break;
        }

        /* Write the header */
        file_open_flag = true;
        FStoreHeader_t file_header = {
            .len = buf_len,
        };

        err = lfs_file_write(&lfs, &fil, &file_header, sizeof(FStoreHeader_t));
        lfs_size_to_err(&err, sizeof(FStoreHeader_t));
        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Error %d while writing header of file %s", err, file_name);
            break;
        }

        /* Write the data */
        err = lfs_file_write(&lfs, &fil, p_buf, buf_len);
        lfs_size_to_err(&err, buf_len);
        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Error %d while writing data of length %u bytes to file %s", err, buf_len, file_name);
            break;
        }
    } while (0);

    /* Cleanup */
    if (file_open_flag != false)
    {
        (void)lfs_file_close(&lfs, &fil);

        /* Delete partially written file if writing was not successful */
        if (err != LFS_ERR_OK)
        {
            (void)lfs_remove(&lfs, file_name);
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_read_value(const char * const p_dir, const char * const p_name, void * const p_buf, uint32_t * const buf_len)
{
    char           file_name[ LFS_ADAPTER_MAX_FNAME_LEN + 1 ] = { 0 };
    bool           file_open_flag = false;
    lfs_file_t     fil = { 0 };
    int32_t        err = LFS_ERR_INVAL;
    FStoreHeader_t file_header = { 0 };

    do
    {
        /* Validate inputs */
        if ((NULL == p_dir) || (NULL == p_name) || (p_buf == NULL))
        {
            err = LFS_ERR_INVAL;
            break;
        }

        /* Construct file name */
        (void)LFS_ADAPTER_SNPRINTF(file_name, sizeof(file_name), "%s/%s", p_dir, p_name);

        /* Open the file */
        err = lfs_file_open(&lfs, &fil, file_name, LFS_O_RDONLY);
        if (LFS_ERR_NOENT == err)
        {
            /* File does not exist - that's a valid use case */
            break;
        }
        else if (err != LFS_ERR_OK)
        {
            /* Something went unexpectedly wrong */
            SID_PAL_LOG_ERROR("lfs: Error %d while opening file %s for reading", err, file_name);
            break;
        }
        else
        {
            /* Proceed with file reading */
        }

        /* Read the header */
        file_open_flag = true;
        err = lfs_file_read(&lfs, &fil, &file_header, sizeof(FStoreHeader_t));
        lfs_size_to_err(&err, sizeof(FStoreHeader_t));
        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Error %d while reading header of file %s", err, file_name);
            break;
        }

        /* Read file data into the supplied buffer */
        err = lfs_file_read(&lfs, &fil, p_buf, file_header.len);
        lfs_size_to_err(&err, file_header.len);
        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Error %d while reading data of length %u bytes to file %s", err, file_header.len, file_name);
            break;
        }
    } while (0);

    /* Cleanup */
    if (file_open_flag != false)
    {
        (void)lfs_file_close(&lfs, &fil);
    }

    /* Update read length output */
    if (buf_len != NULL)
    {
        *buf_len = (LFS_ERR_OK == err) ? file_header.len : 0u;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_get_value_len(const char * const p_dir, const char * const p_name, uint32_t * const p_len)
{
    struct lfs_info file_info = { 0 };
    char            file_name[ LFS_ADAPTER_MAX_FNAME_LEN + 1 ] = { 0 };
    int32_t         err = LFS_ERR_INVAL;

    do
    {
        /* Validate inputs */
        if ((NULL == p_dir) || (NULL == p_name) || (NULL == p_len))
        {
            err = LFS_ERR_INVAL;
            break;
        }

        /* Construct file name */
        (void)LFS_ADAPTER_SNPRINTF(file_name, sizeof(file_name), "%s/%s", p_dir, p_name);

        err = lfs_stat(&lfs, file_name, &file_info);
        *p_len = (LFS_ERR_OK == err) ? (file_info.size - sizeof(FStoreHeader_t)) : 0u;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_delete_value(const char * const p_dir, const char * const p_name)
{
    char file_name[ LFS_ADAPTER_MAX_FNAME_LEN + 1 ] = { 0 };
    int32_t err = LFS_ERR_INVAL;

    /* Construct file name */
    (void)LFS_ADAPTER_SNPRINTF(file_name, sizeof(file_name), "%s/%s", p_dir, p_name);

    /* Delete file */
    err = lfs_remove(&lfs, file_name);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_delete_dir(const char * const p_dir)
{
    int32_t         err = LFS_ERR_INVAL;
    lfs_dir_t       dir;
    struct lfs_info dir_info;
    struct lfs_info file_info;
    char            file_name[ LFS_ADAPTER_MAX_FNAME_LEN + 1 ] = { 0 };
    bool            dir_open_flag = false;

    do
    {
        /* Validate inputs */
        if (NULL == p_dir)
        {
            err = LFS_ERR_INVAL;
            break;
        }

        /* check the directory */
        err = lfs_stat(&lfs, p_dir, &dir_info);
        if (err == LFS_ERR_NOENT)
        {
            /* directory doesn't exist */
            err = LFS_ERR_OK;
            break;
        }
        else if (err != LFS_ERR_OK)
        {
            break;
        }
        else
        {
            /* Proceed with deletion */
        }

        /* Open the directory */
        err = lfs_dir_open(&lfs, &dir, p_dir);
        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Error %d while opening directory %s", err, p_dir);
            break;
        }

        /* Indicate the dir is now open */
        dir_open_flag = true;

        /* Remove all files in the directory */
        while (lfs_dir_read(&lfs, &dir, &file_info) > 0)
        {
            /* Construct file name */
            (void)LFS_ADAPTER_SNPRINTF(file_name, sizeof(file_name), "%s/%s", p_dir, file_info.name);

            if (LFS_TYPE_REG == file_info.type)
            {
                /* Delete file */
                err = lfs_remove(&lfs, file_name);
            }
            else if (LFS_TYPE_DIR == file_info.type)
            {
                if ((strcmp(file_info.name, ".") != 0) && (strcmp(file_info.name, "..") != 0))
                {
                    err = sid_pal_internal_delete_dir(file_name);
                }
            }
            else
            {
                /* Normally shall never happen */
                SID_PAL_ASSERT(0);
                err = LFS_ERR_INVAL;
            }

            /* Terminate on error */
            if (err != LFS_ERR_OK)
            {
                break;
            }
        }

        /* Propagate the error from the loop above */
        if (err != LFS_ERR_OK)
        {
            break;
        }

        /* Close the dir before deletion */
        err = lfs_dir_close(&lfs, &dir);
        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Error %d while closing directory %s", err, p_dir);
            break;
        }

        dir_open_flag = false;

        /* Remove the directory */
        err = lfs_remove(&lfs, p_dir);
        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Error %d while removing directory %s", err, p_dir);
            break;
        }
    } while (0);

    /* Cleanup */
    if (dir_open_flag != false)
    {
        (void)lfs_dir_close(&lfs, &dir);
    }

    return err;
}
