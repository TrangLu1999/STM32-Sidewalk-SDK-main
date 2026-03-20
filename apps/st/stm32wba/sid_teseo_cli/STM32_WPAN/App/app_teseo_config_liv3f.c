
/**
  ******************************************************************************
  * @file    app_teseo_config_liv3f.c
  * @brief   Teseo LIV3F (X-NUCLEO-GNSS1A1) configuration for Sidewalk
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
#include "app_teseo_config.h"

/* Sidewalk interfaces */
#include <sid_pal_gpio_ext_ifc.h>

/* Platform interfaces */
#include <stm32wbaxx.h>

/* Private defines -----------------------------------------------------------*/

#define TESEO_UART_RX_BUFFER_SIZE (512u) /*!< Size of the UART Rx buffer on the MCU side to communicate with the Teseo module */
#define TESEO_UART_TX_BUFFER_SIZE (128u) /*!< Size of the UART Tx buffer on the MCU side to communicate with the Teseo module */

/* External variables --------------------------------------------------------*/

#if defined(STM32WBA5x)
extern UART_HandleTypeDef hlpuart;
#elif defined(STM32WBA6x)
extern UART_HandleTypeDef huart2;
#endif /* STM32WBAxx */

/* Private constants ---------------------------------------------------------*/

static uint8_t teseo_uart_line_start[]  = {'$'};
static uint8_t teseo_uart_line_ending[] = {'\r', '\n'};

/**
 * @brief UART interface configuration for communication with the Teseo module
 */
static const sid_pal_uart_config_t teseo_uart_cfg = {
#if defined(STM32WBA5x)
    .huart                = &hlpuart,
    .hw_instance          = LPUART1,
#elif defined(STM32WBA6x)
    /**
     * @attention On the NUCLEO-WBA65RI board the USART2 lines (D0-D1 pins on CN8) are used by ST-Link's VCP2, preventing the normal operation of Teseo since both
     *            Teseo and ST-Link use D0 line as UART Tx. To avoid any electrical damages you need to cut the connection of ST-Link VCP2 Tx line by removing the
     *            SB7 bridge resistor. Removing SB8 (Rx input for both Teseo and ST-Link VCP2) is optional since there's no electrical conflict and having ST-Link
     *            listening on MCU transmissions does not affect Teseo operation in any way
     */
    .huart                = &huart2,
    .hw_instance          = USART2,
#endif /* STM32WBAxx */
    .baud_rate            = 115200u,                         /* This is the targeted baud rate, but this CLI application will not reconfigure Teseo automatically. Teseo driver will run baud rate auto-detection instead and switch to the actual baud rate of the Teseo unit */
    .data_bits            = 8u,
    .stop_bits            = SID_PAL_UART_STOPBITS_1,
    .parity               = SID_PAL_UART_PARITY_NONE,
    .flow_control         = SID_PAL_UART_HW_FLOW_CTRL_NONE,
    .mode                 = SID_PAL_UART_MODE_TX_RX,
    .clock_source         = UART_CLOCKSOURCE_HSI,            /* HSI clock is required to allow UART and DMA run while the MCU is in Sleep or Stop low-power mode */
    .autonomous_mode_en   = TRUE,                            /* This is required to allow UART and DMA run while the MCU is in Sleep or Stop low-power mode */
    .rx_fifo_threshold    = SID_PAL_UART_FIFO_THRESHOLD_1_8,
    .tx_fifo_threshold    = SID_PAL_UART_FIFO_THRESHOLD_1_2,
    .rx_buffer_size       = TESEO_UART_RX_BUFFER_SIZE,
    .tx_buffer_size       = TESEO_UART_TX_BUFFER_SIZE,
    .frame_start_word     = teseo_uart_line_start,
    .frame_start_word_len = sizeof(teseo_uart_line_start),
    .frame_end_word       = teseo_uart_line_ending,
    .frame_end_word_len   = sizeof(teseo_uart_line_ending),
    .skip_tx_auto_framing = TRUE,                            /* Teseo driver handles adding '$' at the beginning and "\r\n" at the end, so auto framing can be disabled in the UART driver to save some runtime */
    .rx_check_period_us   = 100000u,                         /* Override the default Rx inspection period since Teseo is not transmitting non-stop. Using this override allows reduce the number of MCU wake-ups */
};

/**
 * @brief Board-specific Teseo driver configuration
 */
static const teseo_gnss_device_config_t teseo_liv3f_cfg = {
    .uart_cfg   = &teseo_uart_cfg,

    .gpio       = {
#if defined(NUCLEO_WBA52_BOARD)
#  if defined(SID_RADIO_PLATFORM_SX126X)
        .reset  = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_9),
#  else
        .reset  = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_9),
#  endif /* SID_RADIO_PLATFORM_SX126X */
        .wakeup = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_13),

#elif defined(NUCLEO_WBA55_BOARD)
#  if defined(SID_RADIO_PLATFORM_SX126X)
        .reset  = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_12), /* IMPORTANT: This maps to CN6.3 / CN4.17 (D10) since CN6.2 / CN4.19 (D9, Teseo reset line) is not connected to the MCU on NUCLEO-WBA55 board. You need to bridge CN4.17 and CN4.19 pins with a jumper to make this configuration work */
#  else
        .reset  = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_9),
#  endif /* SID_RADIO_PLATFORM_SX126X */
        .wakeup = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_11),

#elif defined(NUCLEO_WBA65_BOARD)
#  if defined(SID_RADIO_PLATFORM_SX126X)
        .reset  = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_11),
#  else
        .reset  = GPIO_PORT_PIN_TO_NUM(GPIOD, GPIO_PIN_14),
#  endif /* SID_RADIO_PLATFORM_SX126X */
        .wakeup = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_3),

#else
#  error "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the pin assignment here if you are using another (e.g. custom) board"
#endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
    },

    .constellations_to_use = {
        .use_gps     = TRUE,
        .use_galileo = FALSE,
        .use_beidou  = FALSE,
        .use_qzss    = FALSE,
        .use_glonass = FALSE,
    },
};

/* Global function definitions -----------------------------------------------*/

const teseo_gnss_device_config_t * get_teseo_cfg(void)
{
    return &teseo_liv3f_cfg;
}
