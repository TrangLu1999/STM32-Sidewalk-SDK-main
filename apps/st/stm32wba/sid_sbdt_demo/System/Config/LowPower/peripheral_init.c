/**
  ******************************************************************************
  * @file    peripheral_init.c
  * @brief   Peripheral re-initialization handling on leaving Standby LPM
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

#include "app_conf.h"
#include "app_entry.h"
#include "peripheral_init.h"
#include "main.h"
#include <sid_stm32_common_utils.h>

/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private types -------------------------------------------------------------*/

#if defined(HAL_UART_MODULE_ENABLED) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
typedef struct {
    void (* TxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Tx Half Complete Callback        */
    void (* TxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Tx Complete Callback             */
    void (* RxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Rx Half Complete Callback        */
    void (* RxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Rx Complete Callback             */
    void (* ErrorCallback)(struct __UART_HandleTypeDef *huart);             /*!< UART Error Callback                   */
    void (* AbortCpltCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Abort Complete Callback          */
    void (* AbortTransmitCpltCallback)(struct __UART_HandleTypeDef *huart); /*!< UART Abort Transmit Complete Callback */
    void (* AbortReceiveCpltCallback)(struct __UART_HandleTypeDef *huart);  /*!< UART Abort Receive Complete Callback  */
    void (* RxFifoFullCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Rx Fifo Full Callback            */
    void (* TxFifoEmptyCallback)(struct __UART_HandleTypeDef *huart);       /*!< UART Tx Fifo Empty Callback           */
    void (* RxEventCallback)(struct __UART_HandleTypeDef *huart, uint16_t Pos); /*!< UART Reception Event Callback     */

    void (* MspInitCallback)(struct __UART_HandleTypeDef *huart);           /*!< UART Msp Init callback                */
    void (* MspDeInitCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Msp DeInit callback              */
} UART_Callbacks_t;
#endif /* defined(HAL_UART_MODULE_ENABLED) && (USE_HAL_UART_REGISTER_CALLBACKS == 1) */

/* USER CODE BEGIN PT */

/* USER CODE END PT */

/* External variables --------------------------------------------------------*/

extern RAMCFG_HandleTypeDef hramcfg_SRAM1;
extern RAMCFG_HandleTypeDef hramcfg_SRAM2;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/

#if (CFG_LPM_LEVEL != 0) && (CFG_LPM_STDBY_SUPPORTED != 0)
SID_STM32_SPEED_OPTIMIZED void MX_StandbyExit_PeripheralInit(void)
{
    /* USER CODE BEGIN MX_STANDBY_EXIT_PERIPHERAL_INIT_1 */

    /* USER CODE END MX_STANDBY_EXIT_PERIPHERAL_INIT_1 */

#  if (CFG_DEBUGGER_LEVEL == 0)
    /* Setup GPIOA 13, 14, 15 in Analog no pull */
    GPIO_InitTypeDef DbgIOsInit = {0};
    DbgIOsInit.Mode = GPIO_MODE_ANALOG;
    DbgIOsInit.Pull = GPIO_NOPULL;
    DbgIOsInit.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_Init(GPIOA, &DbgIOsInit);

    /* Setup GPIOB 3, 4 in Analog no pull */
    DbgIOsInit.Mode = GPIO_MODE_ANALOG;
    DbgIOsInit.Pull = GPIO_NOPULL;
    DbgIOsInit.Pin = GPIO_PIN_3|GPIO_PIN_4;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_Init(GPIOB, &DbgIOsInit);
#  endif /* CFG_DEBUGGER_LEVEL */

    SID_STM32_UTIL_fast_memset(&hramcfg_SRAM1, 0u, sizeof(hramcfg_SRAM1));
    SID_STM32_UTIL_fast_memset(&hramcfg_SRAM2, 0u, sizeof(hramcfg_SRAM2));

#  if (CFG_LOG_SUPPORTED)
  if (LOG_UART_HANDLER.hdmarx != NULL)
  {
        SID_STM32_UTIL_fast_memset(LOG_UART_HANDLER.hdmarx, 0u, sizeof(*LOG_UART_HANDLER.hdmarx));
    }

    if (LOG_UART_HANDLER.hdmatx != NULL)
    {
      SID_STM32_UTIL_fast_memset(LOG_UART_HANDLER.hdmatx, 0u, sizeof(*LOG_UART_HANDLER.hdmatx));
    }

#    if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    /* Create a backup of the callbacks assigned by the log module */
    const UART_Callbacks_t backup_log_uart_callbacks = {
        .TxHalfCpltCallback        = LOG_UART_HANDLER.TxHalfCpltCallback,
        .TxCpltCallback            = LOG_UART_HANDLER.TxCpltCallback,
        .RxHalfCpltCallback        = LOG_UART_HANDLER.RxHalfCpltCallback,
        .RxCpltCallback            = LOG_UART_HANDLER.RxCpltCallback,
        .ErrorCallback             = LOG_UART_HANDLER.ErrorCallback,
        .AbortCpltCallback         = LOG_UART_HANDLER.AbortCpltCallback,
        .AbortTransmitCpltCallback = LOG_UART_HANDLER.AbortTransmitCpltCallback,
        .AbortReceiveCpltCallback  = LOG_UART_HANDLER.AbortReceiveCpltCallback,
        .RxFifoFullCallback        = LOG_UART_HANDLER.RxFifoFullCallback,
        .TxFifoEmptyCallback       = LOG_UART_HANDLER.TxFifoEmptyCallback,
        .RxEventCallback           = LOG_UART_HANDLER.RxEventCallback,
        .MspInitCallback           = LOG_UART_HANDLER.MspInitCallback,
        .MspDeInitCallback         = LOG_UART_HANDLER.MspDeInitCallback,
    };
#    endif  /* (USE_HAL_UART_REGISTER_CALLBACKS == 1) */

    SID_STM32_UTIL_fast_memset(&LOG_UART_HANDLER, 0u, sizeof(LOG_UART_HANDLER));
#  endif /* CFG_LOG_SUPPORTED */

    MX_ICACHE_Init();
    MX_GPIO_Init();
    MX_GPDMA1_Init();
    MX_RAMCFG_Init();
#  if (CFG_LOG_SUPPORTED)
    LOG_UART_MX_INIT_FUNC();
#    if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    /* Restore the original callbacks so the log module can continue to operate */
    LOG_UART_HANDLER.TxHalfCpltCallback        = backup_log_uart_callbacks.TxHalfCpltCallback;
    LOG_UART_HANDLER.TxCpltCallback            = backup_log_uart_callbacks.TxCpltCallback;
    LOG_UART_HANDLER.RxHalfCpltCallback        = backup_log_uart_callbacks.RxHalfCpltCallback;
    LOG_UART_HANDLER.RxCpltCallback            = backup_log_uart_callbacks.RxCpltCallback;
    LOG_UART_HANDLER.ErrorCallback             = backup_log_uart_callbacks.ErrorCallback;
    LOG_UART_HANDLER.AbortCpltCallback         = backup_log_uart_callbacks.AbortCpltCallback;
    LOG_UART_HANDLER.AbortTransmitCpltCallback = backup_log_uart_callbacks.AbortTransmitCpltCallback;
    LOG_UART_HANDLER.AbortReceiveCpltCallback  = backup_log_uart_callbacks.AbortReceiveCpltCallback;
    LOG_UART_HANDLER.RxFifoFullCallback        = backup_log_uart_callbacks.RxFifoFullCallback;
    LOG_UART_HANDLER.TxFifoEmptyCallback       = backup_log_uart_callbacks.TxFifoEmptyCallback;
    LOG_UART_HANDLER.RxEventCallback           = backup_log_uart_callbacks.RxEventCallback;
    LOG_UART_HANDLER.MspInitCallback           = backup_log_uart_callbacks.MspInitCallback;
    LOG_UART_HANDLER.MspDeInitCallback         = backup_log_uart_callbacks.MspDeInitCallback;
#    endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 1) */
#  endif /* CFG_LOG_SUPPORTED */
    MX_APPE_StandbyExit();
    /* USER CODE BEGIN MX_STANDBY_EXIT_PERIPHERAL_INIT_2 */

    /* USER CODE END MX_STANDBY_EXIT_PERIPHERAL_INIT_2 */
}
#endif /* (CFG_LPM_LEVEL != 0) && (CFG_LPM_STDBY_SUPPORTED != 0) */

/*----------------------------------------------------------------------------*/

#if (CFG_LPM_LEVEL != 0) && (CFG_LPM_STDBY_SUPPORTED != 0)
SID_STM32_SPEED_OPTIMIZED void MX_StandbyEntry_PeripheralBackup(void)
{
    /* Use this callback to backup peripheral states if needed */
}
#endif /* (CFG_LPM_LEVEL != 0) && (CFG_LPM_STDBY_SUPPORTED != 0) */
