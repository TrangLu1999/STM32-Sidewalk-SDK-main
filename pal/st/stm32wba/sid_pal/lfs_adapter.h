/**
  ******************************************************************************
  * @file    lfs_adapter.h
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

#ifndef __SID_STM32WBA_LFS_ADAPTER_INIT_H_
#define __SID_STM32WBA_LFS_ADAPTER_INIT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stddef.h>

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  File system initialization
  *
  * @retval LFS_ERR_OK if passed
  */
int32_t sid_pal_internal_fs_init(void);

/**
  * @brief  File system deinitialization
  *
  * @retval LFS_ERR_OK if passed
  */
int32_t sid_pal_internal_fs_deinit(void);

/**
  * @brief Create or open a file in dir and write a value
  *
  * @param [in] p_dir   Pointer to dir char string
  * @param [in] p_name  Pointer to file char string
  * @param [in] p_buf   Pointer to data buf to write
  * @param [in] buf_len Length of data to write
  * @retval LFS_ERR_OK if passed
  */
int32_t sid_pal_internal_fs_write_value(const char * p_dir, const char * p_name, const void * p_buf, uint32_t buf_len);

/**
  * @brief Read data from a file
  *
  * @param [in]  p_dir  Pointer to dir char string
  * @param [in]  p_name Pointer to file char string
  * @param [out] p_buf   Receiving buffer for the file data
  * @param [out] buf_len Actual size of the data read into the receiving buffer
  * @retval LFS_ERR_OK if passed
  */
int32_t sid_pal_internal_fs_read_value(const char * const p_dir, const char * const p_name, void * const p_buf, uint32_t * const buf_len);

/**
  * @brief Get length of data in the file
  *
  * @param [in]  p_dir  Pointer to dir char string
  * @param [in]  p_name Pointer to file char string
  * @param [out] p_len  Storage for the length of the file
  * @retval LFS_ERR_OK if passed
  */
int32_t sid_pal_internal_fs_get_value_len(const char * const p_dir, const char * const p_name, uint32_t * const p_len);

/**
  * @brief Delete file
  *
  * @param [in] p_dir  Pointer to dir char string
  * @param [in] p_name Pointer to file char string
  * @retval LFS_ERR_OK if passed
  */
int32_t sid_pal_internal_delete_value(const char * const p_dir, const char * const p_name);

/**
  * @brief Delete dir
  *
  * @param [in] p_dir Pointer to dir char string
  * @retval LFS_ERR_OK if passed
  */
int32_t sid_pal_internal_delete_dir(const char * const p_dir);


#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32WBA_LFS_ADAPTER_INIT_H_ */
