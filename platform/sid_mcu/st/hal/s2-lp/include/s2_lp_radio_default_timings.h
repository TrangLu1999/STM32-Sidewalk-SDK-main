/**
  ******************************************************************************
  * @file    s2_lp_radio_default_timings.h
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

#ifndef __S2_LP_RADIO_DEFAULT_TIMINGS_H_
#define __S2_LP_RADIO_DEFAULT_TIMINGS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

#define S2LP_RADIO_CFG_USE_DEFAULT_TIMINGS               (0xFFFFFFFFu) /*!< Indicates that the radio driver should apply default Tx/Rx processing timings (if available) for the current configuration */
#define S2LP_RADIO_CFG_UNUSED_TIMING_VALUE               (0u)          /*!< Value for the RF timing settings that are not in use by this driver and Sidewalk stack */

/* STM32WBAxx definitions ----------------------------------------------------*/
#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined(NUCLEO_WBA65_BOARD) || defined(GENERIC_WBA5x_BOARD) || defined(GENERIC_WBA6x_BOARD)
/**
 * @attention The timings below are applicable to the NUCLEO-WBAxx boards running Sidewalk reference apps with 96MHz core clock and instruction cache enabled.
 *            Using STM32WBA MCUs with altered configuration (e.g., lower core or peripheral clock, disabled ICACHE, different configuration of SRAM and/or
 *            flash wait states, etc.) may require adjustments to these values.
 */
#  define S2LP_RADIO_FSK_TX_DEFAULT_PROCESS_DELAY_US      (3118u)      /*!< Time difference (in useconds) between the moment when Sidewalk MAC schedules to send data and the first bit of radio Tx in air (must also cover Carrier Sense before actual Tx) */
#  define S2LP_RADIO_FSK_RX_DEFAULT_PROCESS_DELAY_US      ( 784u)      /*!< Time difference (in useconds) between the moment when Sidewalk MAC starts Rx and the actual opening of the Rx window by the radio */

#  define S2LP_RADIO_SLEEP_TO_FULL_POWER_DEFAULT_DELAY_US ( 363u)      /*!< Time (in useconds) required for the radio to reach ready state from sleep state */

#  if defined(GENERIC_WBA5x_BOARD) || defined(GENERIC_WBA6x_BOARD)
#    warning "Using the default radio processing delays. You need to measure radio commands processing delay for your board and specify them in the radio config"
#  endif /* GENERIC_WBA5x_BOARD || GENERIC_WBA6x_BOARD */
#endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD || GENERIC_WBA5x_BOARD || GENERIC_WBA6x_BOARD */
/*----------------------------------------------------------------------------*/


/* Consistency checks --------------------------------------------------------*/
#if !defined(S2LP_RADIO_FSK_TX_DEFAULT_PROCESS_DELAY_US) || !defined(S2LP_RADIO_FSK_RX_DEFAULT_PROCESS_DELAY_US)
#  error "Default processing delays for FSK link are not specified while the link is enabled in the config. Please set the defaults"
#endif /* !defined(S2LP_RADIO_FSK_TX_DEFAULT_PROCESS_DELAY_US) || !defined(S2LP_RADIO_FSK_RX_DEFAULT_PROCESS_DELAY_US) */

#if !defined(S2LP_RADIO_SLEEP_TO_FULL_POWER_DEFAULT_DELAY_US)
#  error "Default wakeup duration from Sleep to Ready state is not defined. Please set the defaults"
#endif /* !defined(S2LP_RADIO_SLEEP_TO_FULL_POWER_DEFAULT_DELAY_US) */
/*----------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __S2_LP_RADIO_DEFAULT_TIMINGS_H_ */
