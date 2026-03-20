/**
  ******************************************************************************
  * @file    stm32wlxx_app_radio_default_timings.h
  * @brief   Reference values for Tx/Rx processing delays
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

#ifndef __STM32WLXX_APP_RADIO_DEFAULT_TIMINGS_H_
#define __STM32WLXX_APP_RADIO_DEFAULT_TIMINGS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

#ifndef STM32WLxx_RADIO_APP_LPM_SUPPORT
#  define STM32WLxx_RADIO_APP_LPM_SUPPORT                        (1u)          /*!< 0 - LPM on STM32WLxx side is not supported, 1 - STM32WLxx may enter LPM and the driver needs to perform wakeup procedures whenever necessary */
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

#define STM32WLxx_RADIO_CFG_USE_DEFAULT_TIMINGS                  (0xFFFFFFFFu) /*!< Indicates that the radio driver should apply default Tx/Rx processing timings (if available) for the current configuration */
#define STM32WLxx_RADIO_CFG_UNUSED_TIMING_VALUE                  (0u)          /*!< Value for the RF timing settings that are not in use by this driver and Sidewalk stack */

/* STM32WBAxx definitions ----------------------------------------------------*/
#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined(NUCLEO_WBA65_BOARD) || defined(GENERIC_WBA5x_BOARD) || defined(GENERIC_WBA6x_BOARD)
/**
 * @attention The timings below are applicable to the NUCLEO-WBAxx boards running Sidewalk reference apps with 96MHz core clock and instruction cache enabled.
 *            Using STM32WBA MCUs with altered configuration (e.g., lower core or peripheral clock, disabled ICACHE, different configuration of SRAM and/or
 *            flash wait states, etc.) may require adjustments to these values.
 */
#  define STM32WLxx_LORA_TX_DEFAULT_PROCESS_DELAY_US             (1572u) /*!< Time difference (in useconds) between the moment when Sidewalk MAC schedules to send data and the first bit of radio Tx in air */
#  define STM32WLxx_LORA_RX_DEFAULT_PROCESS_DELAY_US             ( 628u) /*!< Time difference (in useconds) between the moment when Sidewalk MAC starts Rx and the actual opening of the Rx window by the radio */

#  define STM32WLxx_FSK_TX_DEFAULT_PROCESS_DELAY_US              (4651u) /*!< Time difference (in useconds) between the moment when Sidewalk MAC schedules to send data and the first bit of radio Tx in air (must also cover Carrier Sense before actual Tx) */
#  define STM32WLxx_FSK_RX_DEFAULT_PROCESS_DELAY_US              ( 817u) /*!< Time difference (in useconds) between the moment when Sidewalk MAC starts Rx and the actual opening of the Rx window by the radio */

#  if STM32WLxx_RADIO_APP_LPM_SUPPORT
#    define STM32WLxx_RADIO_SLEEP_TO_FULL_POWER_DEFAULT_DELAY_US (1831u) /*!< Time (in useconds) required for the radio to reach STDBY_XOSC state from the SLEEP state. IMPORTANT: this settings shall not account for TCXO startup delay as it is handled separately */
#    define STM32WLxx_HAL_STM32WLxx_DEFAULT_WAKEUP_TIME_US       ( 700u) /*!< STM32WLxx wakeup delay to be able to accept SPI transfers - HAL will wait at least for this time to allow STM32WLxx to wakeup and restart the AHB/APB clocks. IMPORTANT: this is not a full wakeup, it's just the time required to restart the clocks */
#else
#    define STM32WLxx_RADIO_SLEEP_TO_FULL_POWER_DEFAULT_DELAY_US ( 820u)
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

#  if defined(GENERIC_WBA5x_BOARD) || defined(GENERIC_WBA6x_BOARD)
#    warning "using the default radio processing delays. You need to measure radio commands processing delay for your board and specify them in the radio config"
#  endif /* GENERIC_WBA5x_BOARD || GENERIC_WBA6x_BOARD */
#endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD || GENERIC_WBA5x_BOARD || GENERIC_WBA6x_BOARD */
/*----------------------------------------------------------------------------*/


/* Consistency checks --------------------------------------------------------*/
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 && (!defined(STM32WLxx_LORA_TX_DEFAULT_PROCESS_DELAY_US) || !defined(STM32WLxx_LORA_RX_DEFAULT_PROCESS_DELAY_US))
#  error "Default processing delays for LoRa (CSS) link are not specified while the link is enabled in the config. Please set the defaults"
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 && (!defined(STM32WLxx_LORA_TX_DEFAULT_PROCESS_DELAY_US) || !defined(STM32WLxx_LORA_RX_DEFAULT_PROCESS_DELAY_US)) */

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 && (!defined(STM32WLxx_FSK_TX_DEFAULT_PROCESS_DELAY_US) || !defined(STM32WLxx_FSK_RX_DEFAULT_PROCESS_DELAY_US))
#  error "Default processing delays for FSK link are not specified while the link is enabled in the config. Please set the defaults"
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 && (!defined(STM32WLxx_FSK_TX_DEFAULT_PROCESS_DELAY_US) || !defined(STM32WLxx_FSK_RX_DEFAULT_PROCESS_DELAY_US)) */

#if !defined(STM32WLxx_RADIO_SLEEP_TO_FULL_POWER_DEFAULT_DELAY_US)
#  error "Default wakeup duration from SLEEP to STDBY_XOSC state is not defined. Please set the defaults"
#endif /* !defined(STM32WLxx_RADIO_SLEEP_TO_FULL_POWER_DEFAULT_DELAY_US) */
/*----------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __STM32WLXX_APP_RADIO_DEFAULT_TIMINGS_H_ */
