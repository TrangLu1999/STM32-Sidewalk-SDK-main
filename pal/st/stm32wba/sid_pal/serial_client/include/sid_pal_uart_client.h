/**
  ******************************************************************************
  * @file    sid_pal_uart_client.h
  * @brief   Universal UART communication engine for STM32WBAxx MCUs
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

#ifndef __SID_PAL_UART_CLIENT_STM32WBAXX_H_
#define __SID_PAL_UART_CLIENT_STM32WBAXX_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Sidewalk interfaces */
#include <sid_error.h>
#include <sid_pal_serial_client_ifc.h>

/* MCU platform interfaces */
#include <stm32wbaxx.h>

#ifndef SID_PAL_UART_CLIENT_OVERRIDE_TOO_LONG_RX_INSPECTION_PERIOD
#  define SID_PAL_UART_CLIENT_OVERRIDE_TOO_LONG_RX_INSPECTION_PERIOD (1u) /*!< When enabled, the driver will check that user-provided Rx inspection period is not longer than the time needed to fill 1/4 of the buffer. If user config is too long, the driver will print a warning and override user selection. See rx_check_period_us in @ref sid_pal_uart_config_t */
#endif /* SID_PAL_UART_CLIENT_OVERRIDE_TOO_LONG_RX_INSPECTION_PERIOD */

/* Exported types ------------------------------------------------------------*/

typedef enum {
    SID_PAL_UART_STOPBITS_0_5 = 0u, /*!< UART frame with 0.5 stop bit  */
    SID_PAL_UART_STOPBITS_1   = 1u, /*!< UART frame with 1 stop bit    */
    SID_PAL_UART_STOPBITS_1_5 = 2u, /*!< UART frame with 1.5 stop bits */
    SID_PAL_UART_STOPBITS_2   = 3u, /*!< UART frame with 2 stop bits   */
} sid_pal_uart_cfg_stopbits_t;

typedef enum {
    SID_PAL_UART_PARITY_NONE = 0u, /*!< No parity   */
    SID_PAL_UART_PARITY_EVEN = 1u, /*!< Even parity */
    SID_PAL_UART_PARITY_ODD  = 2u, /*!< Odd parity  */
} sid_pal_uart_cfg_parity_t;

typedef enum {
    SID_PAL_UART_HW_FLOW_CTRL_NONE    = 0u, /*!< No hardware control       */
    SID_PAL_UART_HW_FLOW_CTRL_RTS     = 1u, /*!< Request To Send           */
    SID_PAL_UART_HW_FLOW_CTRL_CTS     = 2u, /*!< Clear To Send             */
    SID_PAL_UART_HW_FLOW_CTRL_RTS_CTS = 3u, /*!< Request and Clear To Send */
} sid_pal_uart_cfg_hw_flow_ctrl_t;

typedef enum {
    SID_PAL_UART_MODE_RX    = 0u, /*!< RX mode        */
    SID_PAL_UART_MODE_TX    = 1u, /*!< TX mode        */
    SID_PAL_UART_MODE_TX_RX = 2u, /*!< RX and TX mode */
} sid_pal_uart_cfg_mode_t;

typedef enum {
    SID_PAL_UART_FIFO_DISABLED      = 0u, /*!< FIFO mode is disabled and not used */
    SID_PAL_UART_FIFO_THRESHOLD_1_8 = 1u, /*!< Rx FIFO is filled for at least 1/8 of its depth, Tx FIFO has 1/8 of its depth or less bytes remaining */
    SID_PAL_UART_FIFO_THRESHOLD_1_4 = 2u, /*!< Rx FIFO is filled for at least 1/4 of its depth, Tx FIFO has 1/4 of its depth or less bytes remaining */
    SID_PAL_UART_FIFO_THRESHOLD_1_2 = 3u, /*!< Rx FIFO is filled for at least 1/2 of its depth, Tx FIFO has 1/2 of its depth or less bytes remaining */
    SID_PAL_UART_FIFO_THRESHOLD_3_4 = 4u, /*!< Rx FIFO is filled for at least 3/4 of its depth, Tx FIFO has 3/4 of its depth or less bytes remaining */
    SID_PAL_UART_FIFO_THRESHOLD_7_8 = 5u, /*!< Rx FIFO is filled for at least 7/8 of its depth, Tx FIFO has 7/8 of its depth or less bytes remaining */
    SID_PAL_UART_FIFO_THRESHOLD_8_8 = 6u, /*!< Rx FIFO is completely full, Tx FIFO is absolutely empty */
} sid_pal_uart_cfg_fifo_lvl_t;

typedef struct sid_pal_uart_config {
    UART_HandleTypeDef * const      huart;                /*!< Pointer to the UART HAL peripheral descriptor to use. There's no need to call MX_USARTn_Init() or MX_LPUARTn_Init() functions since this descriptor will be populated by the Sidewalk UART driver. Any HAL/CubeMX configuration will be overwritten */
    USART_TypeDef * const           hw_instance;          /*!< Pointer to the actual U(S)ART IP to use */
    uint32_t                        baud_rate;            /*!< Desired baud rate in bps */
    uint8_t                         data_bits;            /*!< Desired word length in bits */
    sid_pal_uart_cfg_stopbits_t     stop_bits;            /*!< Amount of stop bits */
    sid_pal_uart_cfg_parity_t       parity;               /*!< Parity check selection */
    sid_pal_uart_cfg_hw_flow_ctrl_t flow_control;         /*!< Hardware flow control type */
    sid_pal_uart_cfg_mode_t         mode;                 /*!< UART operating mode (e.g., Tx-only, Rx-only, or Tx+Rx) */
    UART_ClockSourceTypeDef         clock_source;         /*!< Desired kernel Domain clock source. This has to be UART_CLOCKSOURCE_HSI or UART_CLOCKSOURCE_LSE if autonomous operation is intended */
    uint8_t                         autonomous_mode_en;   /*!< Enable operation in autonomous mode. When enabled, UART kernel clock will be kept active even when the CPU enters Sleep or Stop mode */
    sid_pal_uart_cfg_fifo_lvl_t     rx_fifo_threshold;    /*!< Rx FIFO threshold at which IRQ or DMA request is generated */
    sid_pal_uart_cfg_fifo_lvl_t     tx_fifo_threshold;    /*!< Tx FIFO threshold at which IRQ or DMA request is generated */
    uint16_t                        rx_buffer_size;       /*!< Size of the Rx buffer. The buffer is dynamically allocated (if Rx is enabled) on client creation */
    uint16_t                        tx_buffer_size;       /*!< Size of the Tx buffer. The buffer is dynamically allocated (if Tx is enabled) on client creation */
    uint8_t *                       frame_start_word;     /*!< Frame start marker. Set to NULL if not used */
    uint8_t *                       frame_end_word;       /*!< Frame end marker. Set to NULL if not used */
    uint8_t                         frame_start_word_len; /*!< Length of the frame start marker in bytes */
    uint8_t                         frame_end_word_len;   /*!< Length of the frame end marker in bytes */
    uint8_t                         skip_tx_auto_framing; /*!< 0/FALSE (default): frame start and end words are automatically added to the Tx frame if not present there already. Non-zero/TRUE: Tx frame is transmitted as is. This setting can be useful if start/end markers for Rx and Tx frames are different */
    uint32_t                        rx_check_period_us;   /*!< Period of the Rx buffer inspection. The buffer is checked for any completed frames with this period. Lower values provide faster response time, but result in higher energy consumption. Set to 0 for automatic selection */
} sid_pal_uart_config_t;

/**
 * @brief Pointer to an extended UART interface that is backward-compatible with the Sidewalk's sid_pal_serial_ifc_t and can be safely casted to it
 */
typedef const struct sid_pal_uart_ext_ifc_s * sid_pal_uart_ext_ifc_t;

/**
 * @brief Extended UART interface that is backward-compatible with the Sidewalk's struct sid_pal_serial_ifc_s
 */
struct sid_pal_uart_ext_ifc_s {
    /* Section for backward compatibility with Sidewalk's serial client interface */
    union {
        struct sid_pal_serial_ifc_s sid_ifc; /*!< Sidewalk native serial client interface */
        struct {
            sid_error_t             (*send)                 (const sid_pal_uart_ext_ifc_t * _this, const uint8_t *frame_to_send, size_t frame_size);
            sid_error_t             (*get_frame)            (const sid_pal_uart_ext_ifc_t * _this, uint8_t **frame_received, size_t *frame_size);
            sid_error_t             (*process)              (const sid_pal_uart_ext_ifc_t * _this);
            sid_error_t             (*get_mtu)              (const sid_pal_uart_ext_ifc_t * _this, uint16_t * mtu);
            void                    (*destroy)              (const sid_pal_uart_ext_ifc_t * _this);
        };                                   /*!< Transparent mapping to the Sidewalk-native API */
    };

    /* Section with the extended API for UART control */
    /**
     * @brief Start continuous UART reception
     * @note The received data gets into a ring buffer and it is continuously checked to identify completed frames. As soon as
     *       a valid frame is received, a callback specified at the serial client creation time is invoked
     *
     * @param [in] _this Pointer to the Extended Serial Client interface instance
     * @return SID_ERROR_NONE if the reception was started successfully
     */
    sid_error_t                     (*start_rx)             (const sid_pal_uart_ext_ifc_t * _this);

    /**
     * @brief Stop continuous UART reception
     * @note This disables the UART Rx path, but the Tx path remains operational (if enabled in the UART config)
     *
     * @param [in] _this Pointer to the Extended Serial Client interface instance
     * @return SID_ERROR_NONE if the reception was stopped successfully
     */
    sid_error_t                     (*stop_rx)              (const sid_pal_uart_ext_ifc_t * _this);

    /**
     * @brief Stop all UART operations and disable it
     * @note This method disables both Tx and Rx paths and suspends the UART peripheral, allowing the MCU to save power
     *
     * @param [in] _this Pointer to the Extended Serial Client interface instance
     * @return SID_ERROR_NONE if the reception was stopped successfully
     */
    sid_error_t                     (*suspend_uart)         (const sid_pal_uart_ext_ifc_t * _this);

    /**
     * @brief Resume UART peripheral
     * @note This method re-activates the UART, but it does not restart the continuos Rx automatically if it was running
     *       before calling the suspend_uart() method. To resume Rx you need to additionally call start_rx() method
     *
     * @param [in] _this Pointer to the Extended Serial Client interface instance
     * @return SID_ERROR_NONE if the reception was stopped successfully
     */
    sid_error_t                     (*resume_uart)          (const sid_pal_uart_ext_ifc_t * _this);

    /**
     * @brief Get the actual baud rate that is currently configured in the UART peripheral
     *
     * @param [in]  _this         Pointer to the Extended Serial Client interface instance
     * @param [out] out_baud_rate Storage for the baud rate value
     * @return SID_ERROR_NONE if the reception was stopped successfully
     */
    sid_error_t                     (*get_current_baud_rate)(const sid_pal_uart_ext_ifc_t * _this, uint32_t * const out_baud_rate);

    /**
     * @brief Get the intended (initial) baud rate provided in the UART driver configuration
     * @note This method does not read the actual baud rate from the UART peripheral, but rather provides the value specified in the driver config.
     *       The actual baud rate may be different
     *
     * @param [in]  _this         Pointer to the Extended Serial Client interface instance
     * @param [out] out_baud_rate Storage for the baud rate value
     * @return SID_ERROR_NONE if the reception was stopped successfully
     */
    sid_error_t                     (*get_default_baud_rate)(const sid_pal_uart_ext_ifc_t * _this, uint32_t * const out_baud_rate);

    /**
     * @brief Change the UART baud rate
     * @note This method dynamically reconfigures the UART baud rate, allowing auto baud rate detection and dynamic speed adjustment scenarios.
     *       It's vital to ensure the UART is not actively transmitting or receiving anything whenever you want to call this API. If an Rx or Tx
     *       is running, the driver will ignore this request and return an error
     *
     * @param [in] _this         Pointer to the Extended Serial Client interface instance
     * @param [in] new_baud_rate New baud rate to be applied to the UART
     * @return SID_ERROR_NONE if the reception was stopped successfully
     */
    sid_error_t                     (*set_baud_rate)        (const sid_pal_uart_ext_ifc_t * _this, const uint32_t new_baud_rate);
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Creates a standard Sidewalk serial client interface that can be used to communicate over a UART
 *
 * @param [out] _this  Storage for the created Serial Client interface instance
 * @param [in]  config Platform-specific UART driver configuration
 * @param [in]  params Platform-independent serial client parameters (e.g., event callbacks, user-defined context to be passed to the callbacks, etc.)
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_pal_uart_client_create(sid_pal_serial_ifc_t const ** _this, const void * config, const sid_pal_serial_params_t * params);

/**
 * @brief Creates an ST's proprietary extended version of  Sidewalk serial client interface that can be used to communicate over a UART
 * @note While this interface is backward-compatible with the standard sid_pal_serial_ifc_t, it provides additional API for finer UART
 *       control and better power savings options. If you need to share this interface between the users expecting extended API and the
 *       users expecting the standard sid_pal_serial_ifc_t API, it is safe to cast sid_pal_uart_ext_ifc_t pointer to sid_pal_serial_ifc_t
 *       pointer. This backward compatibility is by design and ensured via static assertions at compile time
 *
 * @param [out] _this  Storage for the created Extended Serial Client interface instance
 * @param [in]  config Platform-specific UART driver configuration
 * @param [in]  params Platform-independent serial client parameters (e.g., event callbacks, user-defined context to be passed to the callbacks, etc.)
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_pal_uart_client_create_ext(sid_pal_uart_ext_ifc_t const ** _this, const void * config, const sid_pal_serial_params_t * params);

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_UART_CLIENT_STM32WBAXX_H_ */
