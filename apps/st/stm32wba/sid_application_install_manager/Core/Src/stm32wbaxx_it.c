/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wbaxx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "common_memory_symbols.h"
#include "stm32wbaxx_it.h"
#include "sid_pal_log_like.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;

#if CFG_LOG_SUPPORTED
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern UART_HandleTypeDef huart1;
#endif /* CFG_LOG_SUPPORTED */

extern volatile uint32_t app_flash_ecc_error_address;
extern volatile uint32_t aim_ota_status_ecc_error_address;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* Check if NMI is due to flash ECCD (error detection) */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCD))
  {
#if defined(FLASH_DBANK_SUPPORT)
    const uint32_t physical_bank_id = READ_BIT(FLASH->ECCR, FLASH_ECCR_BK_ECC);
    const uint32_t bank_swap_active = READ_BIT (FLASH->OPTR, FLASH_OPTR_SWAP_BANK_Msk);
    const uint32_t flash_base_addr = FLASH_BASE + (
      (((physical_bank_id == 0u) && (bank_swap_active == OB_SWAP_BANK_DISABLE)) || ((physical_bank_id != 0u) && (bank_swap_active != OB_SWAP_BANK_DISABLE)))
      ? 0u /* Physical bank 1 and no bank swap OR physical bank 2 and bank swap active */
      : (FLASH_SIZE >> 1u) /* Physical bank 2 and no bank swap OR physical bank 1 and bank swap active */
    );
#else
    const uint32_t flash_base_addr = FLASH_BASE;
#endif /* FLASH_DBANK_SUPPORT */
    const uint32_t error_address = ((READ_BIT(FLASH->ECCR, FLASH_ECCR_SYSF_ECC) == 0u) ? flash_base_addr : SYSTEM_FLASH_BASE_NS)
      + ((FLASH->ECCR & FLASH_ECCR_ADDR_ECC_Msk) >> FLASH_ECCR_ADDR_ECC_Pos);

    if ((error_address >= APP_CONFIG_AIM_ACTIVE_SLOT_START) && (error_address < APP_CONFIG_AIM_STAGING_SLOT_END))
    {
      /* ECC error detected in the app or staging flash area - store the address of the problematic quad-word */
      app_flash_ecc_error_address = error_address;
      __DSB(); /* Ensure app_flash_ecc_error_address is actually written before we return from here */

      /* Clear ECC detection flag to allow the program to proceed */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);

      /* Return from here and let the LittleFS port to handle the ECC error */
      return;
    }
    else if ((error_address >= APP_CONFIG_AIM_OTA_STATUS_AREA_START) && (error_address < APP_CONFIG_AIM_OTA_STATUS_AREA_END))
    {
      /* ECC error detected in the AIM's install status metadata - store the address of the problematic quad-word */
      aim_ota_status_ecc_error_address = error_address;
      __DSB(); /* Ensure app_flash_ecc_error_address is actually written before we return from here */

      /* Clear ECC detection flag to allow the program to proceed */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);

      /* Return from here and let the LittleFS port to handle the ECC error */
      return;
    }
    else
    {
      /* This is a fatal error since it takes place in AIM code area */
      SID_PAL_LOG_ERROR("Uncorrectable ECC error detected in flash at address 0x%08X", error_address);
      Error_Handler();
    }
  }
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while(1)
  {
  }

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */

    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */

    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */

    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */

    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WBAxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wbaxx.s).                    */
/******************************************************************************/

#if CFG_LOG_SUPPORTED
/**
  * @brief This function handles GPDMA1 Channel 4 global interrupt.
  */
void GPDMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 0 */

  /* USER CODE END GPDMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel4);
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 1 */

  /* USER CODE END GPDMA1_Channel4_IRQn 1 */
}
#endif /* CFG_LOG_SUPPORTED */

#if CFG_LOG_SUPPORTED
/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
#endif /* CFG_LOG_SUPPORTED */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
