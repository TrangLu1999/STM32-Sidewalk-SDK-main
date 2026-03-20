/**
  ******************************************************************************
  * @file    adv_trace_usart_if.c
  * @author  MCD Application Team
  * @brief : Source file for interfacing the stm32_adv_trace to hardware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022-2025 STMicroelectronics.
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
#include "stm32_adv_trace.h"
#include "adv_trace_usart_if.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#ifndef ADV_TRACE_USART_IF_SUPPORT_RX
#  define ADV_TRACE_USART_IF_SUPPORT_RX (0)
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */

/* External variables --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* list all the driver interface used by the trace application. */
const UTIL_ADV_TRACE_Driver_s UTIL_TraceDriver =
{
  UART_Init,
  UART_DeInit,
  UART_StartRx,
  UART_TransmitDMA
};

/* Private variables ---------------------------------------------------------*/

#if ADV_TRACE_USART_IF_SUPPORT_RX
/* Whether the UART should be in RX after a Transmit */
uint8_t receive_after_transmit = 0; 

/* Buffer to receive 1 character */
static uint8_t cCharRx;
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */

/* Exported macro ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static void (*TxCpltCallback)       ( void * );
#if ADV_TRACE_USART_IF_SUPPORT_RX
static void (*RxCpltCallback)       ( uint8_t * pData, uint16_t iSize, uint8_t cError );
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */

static void UsartIf_TxCpltCallback  ( UART_HandleTypeDef *huart );
#if ADV_TRACE_USART_IF_SUPPORT_RX
static void UsartIf_RxCpltCallback  ( UART_HandleTypeDef *huart );
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */
static IRQn_Type get_IRQn_Type_from_DMA_HandleTypeDef (DMA_HandleTypeDef * hDmaHandler );

/* Private user code ---------------------------------------------------------*/

/**
 *
 */
UTIL_ADV_TRACE_Status_t UART_Init(  void (*pCallbackFunction)(void *))
{
  TxCpltCallback = pCallbackFunction;

  LOG_UART_HANDLER.TxCpltCallback = UsartIf_TxCpltCallback;
  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  /* Ensure any formatting from the past output is reset and logs will start from a new line */
  UTIL_ADV_TRACE_FSend("\e[0m\n");
  /*^^^ END OF PATCH STMC-485 ^^^*/

  return UTIL_ADV_TRACE_OK;
}

/**
 *
 */
UTIL_ADV_TRACE_Status_t UART_DeInit( void )
{
  HAL_StatusTypeDef eResult;

  eResult = HAL_UART_DeInit( &LOG_UART_HANDLER );
  if ( eResult != HAL_OK )
  {
    LOG_UART_HANDLER.TxCpltCallback = NULL;
    return UTIL_ADV_TRACE_UNKNOWN_ERROR;
  }

  return UTIL_ADV_TRACE_OK;
}

/**
 *
 */
UTIL_ADV_TRACE_Status_t UART_StartRx( void (*pCallbackFunction)(uint8_t * pData, uint16_t iSize, uint8_t cError ) )
{
#if ADV_TRACE_USART_IF_SUPPORT_RX
  /* Configure UART in Receive mode */
  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  // HAL_UART_Receive_IT( &LOG_UART_HANDLER, &cCharRx, 1 );
  /*^^^ END OF PATCH STMC-485 ^^^*/
  LOG_UART_HANDLER.RxCpltCallback = &UsartIf_RxCpltCallback;

  if ( pCallbackFunction != NULL )
  {
    RxCpltCallback = pCallbackFunction;
  }
  /*vvv PATCH STMC-485 vvvvvvvvvv*/

  HAL_UART_Receive_IT( &LOG_UART_HANDLER, &cCharRx, 1 );
  /*^^^ END OF PATCH STMC-485 ^^^*/

  return UTIL_ADV_TRACE_OK;
#else
  return UTIL_ADV_TRACE_HW_ERROR;
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */
}

/**
 *
 */
UTIL_ADV_TRACE_Status_t UART_TransmitDMA ( uint8_t * pData, uint16_t iSize )
{
  UTIL_ADV_TRACE_Status_t eStatus = UTIL_ADV_TRACE_OK;
  HAL_StatusTypeDef eResult;
  IRQn_Type         eUseDmaTx;
#if ADV_TRACE_USART_IF_SUPPORT_RX
  IRQn_Type         eUseDmaRx;
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */

  eUseDmaTx = get_IRQn_Type_from_DMA_HandleTypeDef( LOG_UART_HANDLER.hdmatx );
#if ADV_TRACE_USART_IF_SUPPORT_RX
  eUseDmaRx = get_IRQn_Type_from_DMA_HandleTypeDef( LOG_UART_HANDLER.hdmarx );
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */

  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  // if ( ( eUseDmaTx == GPDMA1_Channel0_IRQn ) || ( eUseDmaTx == GPDMA1_Channel1_IRQn ) ||
  //      ( eUseDmaTx == GPDMA1_Channel2_IRQn ) || ( eUseDmaTx == GPDMA1_Channel3_IRQn ) ||
  //      ( eUseDmaTx == GPDMA1_Channel4_IRQn ) || ( eUseDmaTx == GPDMA1_Channel5_IRQn ) ||
  //      ( eUseDmaTx == GPDMA1_Channel6_IRQn ) || ( eUseDmaTx == GPDMA1_Channel7_IRQn ) ||
  //      ( eUseDmaRx == GPDMA1_Channel0_IRQn ) || ( eUseDmaRx == GPDMA1_Channel1_IRQn ) ||
  //      ( eUseDmaRx == GPDMA1_Channel2_IRQn ) || ( eUseDmaRx == GPDMA1_Channel3_IRQn ) ||
  //      ( eUseDmaRx == GPDMA1_Channel4_IRQn ) || ( eUseDmaRx == GPDMA1_Channel5_IRQn ) ||
  //      ( eUseDmaRx == GPDMA1_Channel6_IRQn ) || ( eUseDmaRx == GPDMA1_Channel7_IRQn ) )
  // {
  //   eResult = HAL_UART_Transmit_DMA( &LOG_UART_HANDLER, pData, iSize );
  // }
  // else
  // {
  //   eResult = HAL_UART_Transmit_IT( &LOG_UART_HANDLER, pData, iSize );
  // }
  if (    ( eUseDmaTx != GPDMA1_Channel0_IRQn ) && ( eUseDmaTx != GPDMA1_Channel1_IRQn )
       && ( eUseDmaTx != GPDMA1_Channel2_IRQn ) && ( eUseDmaTx != GPDMA1_Channel3_IRQn )
       && ( eUseDmaTx != GPDMA1_Channel4_IRQn ) && ( eUseDmaTx != GPDMA1_Channel5_IRQn )
       && ( eUseDmaTx != GPDMA1_Channel6_IRQn ) && ( eUseDmaTx != GPDMA1_Channel7_IRQn )
#if ADV_TRACE_USART_IF_SUPPORT_RX
       && ( eUseDmaRx != GPDMA1_Channel0_IRQn ) && ( eUseDmaRx != GPDMA1_Channel1_IRQn )
       && ( eUseDmaRx != GPDMA1_Channel2_IRQn ) && ( eUseDmaRx != GPDMA1_Channel3_IRQn )
       && ( eUseDmaRx != GPDMA1_Channel4_IRQn ) && ( eUseDmaRx != GPDMA1_Channel5_IRQn )
       && ( eUseDmaRx != GPDMA1_Channel6_IRQn ) && ( eUseDmaRx != GPDMA1_Channel7_IRQn )
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */
    )
  {
    return UTIL_ADV_TRACE_HW_ERROR;
  }
  else
  {
    eResult = HAL_UART_Transmit_DMA( &LOG_UART_HANDLER, pData, iSize );
  }
  /*^^^ END OF PATCH STMC-485 ^^^*/

  if ( eResult != HAL_OK )
  {
    eStatus = UTIL_ADV_TRACE_HW_ERROR;
  }

#if ADV_TRACE_USART_IF_SUPPORT_RX
  /* Check whether the UART should return in Receiver mode */
  if ( receive_after_transmit )
  {
    HAL_UART_Receive_IT( &LOG_UART_HANDLER, &cCharRx, 1 );
  }
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */

  return eStatus;
}

/**
 *
 */
static void UsartIf_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* ADV Trace callback */
  TxCpltCallback(NULL);
}

#if ADV_TRACE_USART_IF_SUPPORT_RX
/**
 *
 */
static void UsartIf_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  // RxCpltCallback( &cCharRx, 1, 0 );
  if (RxCpltCallback != NULL)
  {
    RxCpltCallback( &cCharRx, 1, 0 );
  }
  /*^^^ END OF PATCH STMC-485 ^^^*/
  HAL_UART_Receive_IT( &LOG_UART_HANDLER, &cCharRx, 1 );
}
#endif /* ADV_TRACE_USART_IF_SUPPORT_RX */

/**
  * The purpose of this function is to match a DMA_HandleTypeDef as key with the corresponding IRQn_Type as value.
  *
  * TAKE CARE : in case of an invalid parameter or e.g. an usart/lpuart not initialized, this will lead to hard fault.
  *             it is up to the user to ensure the serial link is in a valid state.
  */
static IRQn_Type get_IRQn_Type_from_DMA_HandleTypeDef( DMA_HandleTypeDef * hDmaHandler )
{
  if ( hDmaHandler->Instance == GPDMA1_Channel0 ) 
    { return GPDMA1_Channel0_IRQn; }
  
  if ( hDmaHandler->Instance == GPDMA1_Channel1 ) 
    { return GPDMA1_Channel1_IRQn; }
  
  if ( hDmaHandler->Instance == GPDMA1_Channel2 ) 
    { return GPDMA1_Channel2_IRQn; }
  
  if ( hDmaHandler->Instance == GPDMA1_Channel3 ) 
    { return GPDMA1_Channel3_IRQn; }
  
  if ( hDmaHandler->Instance == GPDMA1_Channel4 ) 
    { return GPDMA1_Channel4_IRQn; }
  
  if ( hDmaHandler->Instance == GPDMA1_Channel5 ) 
    { return GPDMA1_Channel5_IRQn; }
  
  if  (hDmaHandler->Instance == GPDMA1_Channel6 ) 
    { return GPDMA1_Channel6_IRQn; }
  
  if ( hDmaHandler->Instance == GPDMA1_Channel7 ) 
    { return GPDMA1_Channel7_IRQn; }

  /* Values from (-1) to (-15) are already in used. This value isn't used so it should be safe.
     So, if you see this value, it means you used an invalid DMA handler as input. */
  return (IRQn_Type)(-666);
}

