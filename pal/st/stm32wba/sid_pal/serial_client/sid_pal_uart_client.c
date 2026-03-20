/**
  ******************************************************************************
  * @file    sid_pal_uart_client.c
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

/* Includes ------------------------------------------------------------------*/

#include <assert.h>
#include <stdlib.h>

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_serial_client_ifc.h>
#include <sid_pal_timer_ifc.h>
#include <sid_pal_uart_client.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>

/* Platform interfaces */
#include <sid_stm32_common_utils.h>
#include <stm_list.h>
#include <stm32wbaxx_hal.h>

#include <cmsis_compiler.h>

/* Private defines -----------------------------------------------------------*/

#define STM_LIST_INITIAL_VALUE(list)              {.next = &(list), .prev = &(list)}

#define SID_PAL_UART_RX_CHECK_PERIOD_MIN          (500u) /*!< Minimum allowed Rx ring buffer processing interval (in us) */

#define SID_PAL_UART_ORIGINATOR_ID_ERROR_CALLBACK (0x0BADCA11u)

/* Private macro -------------------------------------------------------------*/

#ifndef containerof
#  define containerof(ptr, type, member)          ((type *)((uintptr_t)(ptr) - offsetof(type, member)))
#endif

#ifndef SID_PAL_UART_EXTRA_LOGGING
/* Set SID_PAL_UART_EXTRA_LOGGING to 1 to enable extended logs */
#  define SID_PAL_UART_EXTRA_LOGGING              (0)
#endif

#if SID_PAL_UART_EXTRA_LOGGING
#  define SID_PAL_UART_LOG_ERROR(...)             SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SID_PAL_UART_LOG_WARNING(...)           SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SID_PAL_UART_LOG_INFO(...)              SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SID_PAL_UART_LOG_DEBUG(...)             SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SID_PAL_UART_LOG_TRACE(...)             SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SID_PAL_UART_LOG_ERROR(...)             ((void)0u)
#  define SID_PAL_UART_LOG_WARNING(...)           ((void)0u)
#  define SID_PAL_UART_LOG_INFO(...)              ((void)0u)
#  define SID_PAL_UART_LOG_DEBUG(...)             ((void)0u)
#  define SID_PAL_UART_LOG_TRACE(...)             ((void)0u)
#endif

#define SID_PAL_UART_GET_IMPL_FROM_IFC(_IFC_)     (containerof((_IFC_), uart_serial_client_impl_t, public_ifc))

/* Static assertions ---------------------------------------------------------*/

/* Ensure consistency between struct sid_pal_serial_ifc_s and struct sid_pal_uart_ext_ifc_s definitions */
static_assert(offsetof(struct sid_pal_uart_ext_ifc_s, send)      == offsetof(struct sid_pal_serial_ifc_s, send));
static_assert(offsetof(struct sid_pal_uart_ext_ifc_s, get_frame) == offsetof(struct sid_pal_serial_ifc_s, get_frame));
static_assert(offsetof(struct sid_pal_uart_ext_ifc_s, process)   == offsetof(struct sid_pal_serial_ifc_s, process));
static_assert(offsetof(struct sid_pal_uart_ext_ifc_s, get_mtu)   == offsetof(struct sid_pal_serial_ifc_s, get_mtu));
static_assert(offsetof(struct sid_pal_uart_ext_ifc_s, destroy)   == offsetof(struct sid_pal_serial_ifc_s, destroy));

/* Private types -------------------------------------------------------------*/

typedef struct {
    sid_pal_uart_ext_ifc_t             public_ifc;               /*!< Public UART interface */
    const sid_pal_serial_callbacks_t * callbacks;                /*!< Public UART event callbacks */
    void *                             cb_ctx;                   /*!< Context to pass to event callbacks */
    const struct sid_pal_uart_config * config;                   /*!< Static configuration of this instance */

    tListNode                          node;                     /*!< Linked list node used to maintain the collection of UART clients */

    uint8_t *                          rx_buffer;                /*!< Rx ring buffer */
    const uint8_t *                    rx_buf_frame_start_ptr;   /*!< Pointer to the detected frame start position. If a frame start is not identified, this pointer is NULL */
    const uint8_t *                    rx_buf_frame_end_ptr;     /*!< Pointer to the detected frame end position. If a frame end is not identified, this pointer is NULL */
    const uint8_t *                    rx_buf_inspect_ptr;       /*!< Location of the last checked symbol in the Rx ring buffer when searching for frame start and end points */
    uint32_t                           rx_buf_write_wrap_cnt;    /*!< Internal counter of the write wrap events in the Rx ring buffer. Used for buffer overflow monitoring, incremented every time the DMA write pointer wraps over to the buffer start */
    uint32_t                           rx_buf_inspect_period_us;
    sid_pal_timer_t                    rx_buf_inspection_timer;  /*!< Timer to periodically check if the Rx buffer received a full frame */
    uint8_t *                          rx_frame_linear_storage;  /*!< Dynamically allocated linear buffer to store a single frame extracted from the ring buffer */

    uint8_t *                          tx_buffer;                /*!< Tx ring buffer */
    uint8_t *                          tx_buf_ingest_ptr;        /*!< Position at which new data will be added to the Tx buffer */
    uint8_t *                          tx_buf_sendout_ptr;       /*!< Position of the first byte to be sent in the Tx buffer */
    uint8_t                            tx_buf_ingest_wrap_flag;  /*!< Flag indicating that there was a roll over when ingesting new data into the Tx buffer, meaning a portion of that data is placed at the very end of the Tx buffer and the other portion - at the very beginning */
    uint16_t                           tx_mtu_size;              /*!< MTU size for teh UART Tx. This is typically limited by the size of the Tx buffer */
} uart_serial_client_impl_t;

/* Private variables ---------------------------------------------------------*/

static tListNode uart_client_list = STM_LIST_INITIAL_VALUE(uart_client_list); /*!< List of UART clients that are managed by this driver */

extern const uint16_t UARTPrescTable[12];

/* Private function prototypes -----------------------------------------------*/

static        void            _prv_destroy_impl(uart_serial_client_impl_t * const impl);

static inline uint32_t        _prv_client_in_list(uart_serial_client_impl_t * const impl);
static inline uart_serial_client_impl_t * _prv_find_client_by_uart(UART_HandleTypeDef * const huart);
static inline void            _prv_remove_client_from_list(uart_serial_client_impl_t * const impl);

static inline uint32_t        _prv_advance_ring_buffer_pointer(const uint8_t ** const ptr, const uint32_t increment, const uint8_t * const buffer, const uint32_t buffer_size);
static inline uint32_t        _prv_rewind_ring_buffer_pointer(const uint8_t ** const ptr, const uint32_t decrement, const uint8_t * const buffer, const uint32_t buffer_size);
static inline uint32_t        _prv_compute_data_length_in_ring_buffer(const uint8_t * const start_ptr, const uint8_t * const end_ptr, const uint8_t * const buffer, const uint32_t buffer_size);
static inline uint32_t        _prv_detect_pattern(const uint8_t * const pattern, uint32_t pattern_len, const uint8_t * const buffer, const uint32_t buffer_size, const uint8_t * const start_ptr);
static inline const uint8_t * _prv_search_pattern_in_ring_buf(const uint8_t * const pattern, uint32_t pattern_len, const uint8_t * const buffer, const uint32_t buffer_size, const uint8_t ** const search_ptr, const uint8_t * const search_end_ptr);
static inline uint32_t        _prv_ingest_into_ring_buffer(const uint8_t * const data, const uint32_t data_len, uint8_t * const buffer, const uint32_t buffer_size, uint8_t ** const ingest_ptr);

static        sid_error_t     _prv_uart_hw_init(const struct sid_pal_uart_config * const config);
static        sid_error_t     _prv_uart_apply_fifo_config(const struct sid_pal_uart_config * const config);
static        const char *    _prv_get_uart_display_name(const USART_TypeDef * const instance);
static        void            _prv_calc_rx_inspection_period(uart_serial_client_impl_t * const impl);
static        sid_error_t     _prv_schedule_rx_inspection(uart_serial_client_impl_t * const impl, const uint32_t start_delay_us);
static        uint32_t        _prv_get_kernel_clock_source_freq(const struct sid_pal_uart_config * const config);
static        uint32_t        _prv_calc_clock_prescaler(const uint32_t kernel_clock_freq, const uint32_t baudrate, const uint32_t oversampling);
static        sid_error_t     _prv_apply_uart_clock_source(const struct sid_pal_uart_config * const config);
static        sid_error_t     _prv_start_continuous_rx(uart_serial_client_impl_t * const impl, const uint32_t inspect_rx_buf);
static        sid_error_t     _prv_stop_continuous_rx(uart_serial_client_impl_t * const impl);
static        void            _prv_rx_inspection_event_handler(void * arg, sid_pal_timer_t * originator);
static        void            _prv_rx_buffer_write_wrap_event_handler(UART_HandleTypeDef * huart);
static        void            _prv_on_uart_xfer_error(UART_HandleTypeDef * huart);
static        void            _prv_on_uart_tx_completed(UART_HandleTypeDef * huart);

static        sid_error_t     _uart_ifc_send(const sid_pal_serial_ifc_t * _this, const uint8_t * frame_to_send, size_t frame_size);
static        sid_error_t     _uart_ifc_get_frame(const sid_pal_serial_ifc_t *_this, uint8_t ** frame_received, size_t * frame_size);
static        sid_error_t     _uart_ifc_get_mtu(const sid_pal_serial_ifc_t * _this, uint16_t * mtu);
static        void            _uart_ifc_destroy(const sid_pal_serial_ifc_t * _this);

static        sid_error_t     _uart_ext_ifc_start_rx(const sid_pal_uart_ext_ifc_t * _this);
static        sid_error_t     _uart_ext_ifc_stop_rx(const sid_pal_uart_ext_ifc_t * _this);
static        sid_error_t     _uart_ext_ifc_suspend_uart(const sid_pal_uart_ext_ifc_t * _this);
static        sid_error_t     _uart_ext_ifc_resume_uart(const sid_pal_uart_ext_ifc_t * _this);
static        sid_error_t     _uart_ext_ifc_get_current_baud_rate(const sid_pal_uart_ext_ifc_t * _this, uint32_t * const out_baud_rate);
static        sid_error_t     _uart_ext_ifc_get_default_baud_rate(const sid_pal_uart_ext_ifc_t * _this, uint32_t * const out_baud_rate);
static        sid_error_t     _uart_ext_ifc_set_baud_rate(const sid_pal_uart_ext_ifc_t * _this, const uint32_t new_baud_rate);

static        sid_error_t     _uart_client_create_common(sid_pal_uart_ext_ifc_t const ** _this, const void * config, const sid_pal_serial_params_t * params);

/* Private constants ---------------------------------------------------------*/

static const struct sid_pal_uart_ext_ifc_s uart_client_ifc_methods = {
    .sid_ifc = {
        .send              = _uart_ifc_send,
        .get_frame         = _uart_ifc_get_frame,
        .get_mtu           = _uart_ifc_get_mtu,
        .destroy           = _uart_ifc_destroy,
    },

    .start_rx              = _uart_ext_ifc_start_rx,
    .stop_rx               = _uart_ext_ifc_stop_rx,
    .suspend_uart          = _uart_ext_ifc_suspend_uart,
    .resume_uart           = _uart_ext_ifc_resume_uart,
    .get_current_baud_rate = _uart_ext_ifc_get_current_baud_rate,
    .get_default_baud_rate = _uart_ext_ifc_get_default_baud_rate,
    .set_baud_rate         = _uart_ext_ifc_set_baud_rate,
};

/* Private function definitions ----------------------------------------------*/

static void _prv_destroy_impl(uart_serial_client_impl_t * const impl)
{
    if (impl != NULL)
    {
        /* Delete the Rx inspection timer */
        if ((impl->config != NULL) && ((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_RX == impl->config->mode)))
        {
            (void)sid_pal_timer_deinit(&impl->rx_buf_inspection_timer);
        }

        /* Free Rx buffer */
        if (impl->rx_buffer != NULL)
        {
            free(impl->rx_buffer);
        }
        if (impl->rx_frame_linear_storage != NULL)
        {
            free(impl->rx_frame_linear_storage);
        }

        /* Free Tx buffer */
        if (impl->tx_buffer != NULL)
        {
            free(impl->tx_buffer);
        }

        /* Free instance memory */
        free(impl);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _prv_client_in_list(uart_serial_client_impl_t * const impl)
{
    uint32_t in_list;

    SID_PAL_ASSERT(impl != NULL);

    do
    {
        /* Check if the client node is in any list at all */
        if ((NULL == impl->node.prev) || (NULL == impl->node.next))
        {
            /* Not in a list at all or the node links are invalid */
            in_list = FALSE;
            break;
        }

        /* Check if this client node belongs to the list of the UART clients maintained by this driver */
        tListNode * current_node;

        in_list = FALSE; /* Assume the node won't be found */
        LST_get_next_node(&uart_client_list, &current_node); /* If the list is empty this will set current_node back to the uart_client_list */
        while ((current_node != &uart_client_list) && (current_node != NULL))
        {
            if (&impl->node == current_node)
            {
                /* Found current client in the list */
                in_list = TRUE;
                break;
            }

            /* Move to the next list item */
            LST_get_next_node(current_node, &current_node);
        }
    } while (0);

    return in_list;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uart_serial_client_impl_t * _prv_find_client_by_uart(UART_HandleTypeDef * const huart)
{
    uart_serial_client_impl_t * impl = NULL;

    sid_pal_enter_critical_region();

    do
    {
        tListNode * current_node;

        /* Iterate over the list of clients and try to find the client associated with the specified UART peripheral */
        LST_get_next_node(&uart_client_list, &current_node); /* If the list is empty this will set current_node back to the uart_client_list */
        while ((current_node != &uart_client_list) && (current_node != NULL))
        {
            uart_serial_client_impl_t * impl_candidate = containerof(current_node, uart_serial_client_impl_t, node);
            if (huart == impl_candidate->config->huart)
            {
                /* Found the associated client in the list */
                impl = impl_candidate;
                break;
            }

            /* Move to the next list item */
            LST_get_next_node(current_node, &current_node);
        }
    } while (0);

    sid_pal_exit_critical_region();

    return impl;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _prv_remove_client_from_list(uart_serial_client_impl_t * const impl)
{
    SID_PAL_ASSERT(impl != NULL);

    if (_prv_client_in_list(impl) != FALSE)
    {
        LST_remove_node(&impl->node);
        impl->node.prev = NULL;
        impl->node.next = NULL;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _prv_advance_ring_buffer_pointer(const uint8_t ** const ptr, const uint32_t increment, const uint8_t * const buffer, const uint32_t buffer_size)
{
    uint32_t wrapped = FALSE;

    *ptr += (increment % buffer_size);
    if (*ptr >= (buffer + buffer_size))
    {
        /* Wrap the pointer */
        *ptr -= buffer_size;

        /* Set wrap flag on read pointer wrap */
        wrapped = TRUE;
    }

    return wrapped;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _prv_rewind_ring_buffer_pointer(const uint8_t ** const ptr, const uint32_t decrement, const uint8_t * const buffer, const uint32_t buffer_size)
{
    uint32_t wrapped = FALSE;

    *ptr -= (decrement % buffer_size);
    if (*ptr < buffer)
    {
        /* Wrap the pointer */
        *ptr += buffer_size;

        /* Set wrap flag on read pointer wrap */
        wrapped = TRUE;
    }

    return wrapped;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _prv_compute_data_length_in_ring_buffer(const uint8_t * const start_ptr, const uint8_t * const end_ptr, const uint8_t * const buffer, const uint32_t buffer_size)
{
    uint32_t data_length;

    if (start_ptr < end_ptr)
    {
        data_length = (uint32_t)(void *)end_ptr - (uint32_t)(void *)start_ptr;
     }
    else
    {
        data_length = (uint32_t)(void *)end_ptr - (uint32_t)(void *)start_ptr + buffer_size;
    }

    return data_length;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _prv_detect_pattern(const uint8_t * const pattern, uint32_t pattern_len, const uint8_t * const buffer, const uint32_t buffer_size, const uint8_t * const start_ptr)
{
    uint32_t pattern_detected = TRUE;
    const uint8_t * current_buffer_pos = start_ptr;
    const uint8_t * current_pattern_pos = pattern;

    for (uint32_t i = 0u; i < pattern_len; i++)
    {
        if (*current_pattern_pos != *current_buffer_pos)
        {
            pattern_detected = FALSE;
            break;
        }

        current_pattern_pos++;
        (void)_prv_advance_ring_buffer_pointer(&current_buffer_pos, 1u, buffer, buffer_size);
    }

    return pattern_detected;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline const uint8_t * _prv_search_pattern_in_ring_buf(const uint8_t * const pattern, uint32_t pattern_len, const uint8_t * const buffer, const uint32_t buffer_size, const uint8_t ** const search_ptr, const uint8_t * const search_end_ptr)
{
    const uint8_t * pattern_location_ptr = NULL;

    do
    {
        /* Check if there's enough data to search in */
        uint32_t available_bytes = _prv_compute_data_length_in_ring_buffer(*search_ptr, search_end_ptr, buffer, buffer_size);
        if (available_bytes < pattern_len)
        {
            /* Not enough data is available, skip end word search and wait for more bytes to arrive */
            break;
        }

        /* Compute search position limit */
        const uint8_t * inspection_limit_ptr = search_end_ptr; /* End pointer is the address right after the area of interest */

        /* Roll back by the length of the end pattern */
        (void)_prv_rewind_ring_buffer_pointer(&inspection_limit_ptr, pattern_len, buffer, buffer_size);
        uint32_t inspection_warp_flag = (*search_ptr <= inspection_limit_ptr) ? FALSE : TRUE;

        SID_PAL_UART_LOG_DEBUG("INSP: start: %u, end: %u, bytes: %u, warp_flag: %u, limit: %u", (*search_ptr - buffer), (search_end_ptr - buffer), available_bytes, inspection_warp_flag, (inspection_limit_ptr - buffer));

        while ((*search_ptr <= inspection_limit_ptr) || (inspection_warp_flag != FALSE))
        {
            if (_prv_detect_pattern(pattern, pattern_len, buffer, buffer_size, *search_ptr) == FALSE)
            {
                /* Pattern not detected at current position, move to the next byte */
                const uint32_t wrap_on_advance = _prv_advance_ring_buffer_pointer(search_ptr, 1u, buffer, buffer_size);
                if (wrap_on_advance != FALSE)
                {
                    if (inspection_warp_flag != FALSE)
                    {
                        /* Clear the buffer write wrap flag on read pointer wrap */
                        inspection_warp_flag = FALSE;
                    }
                    else
                    {
                        /* Corner case: inspection_limit_ptr is at the very end of the buffer and search_ptr wraps around to the buffer start on increment - terminate as the pattern was not found */
                        break;
                    }
                }
            }
            else
            {
                /* Frame end detected at current position */
                pattern_location_ptr = *search_ptr;

                /* Move inspection pointer to the first byte after the current frame */
                (void)_prv_advance_ring_buffer_pointer(search_ptr, pattern_len, buffer, buffer_size);

                /* Search completed */
                break;
            }
        }
    } while (0);

    return pattern_location_ptr;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _prv_ingest_into_ring_buffer(const uint8_t * const data, const uint32_t data_len, uint8_t * const buffer, const uint32_t buffer_size, uint8_t ** const ingest_ptr)
{
    SID_PAL_ASSERT(data_len <= buffer_size);

    if (((uint32_t)(void *)(*ingest_ptr) + data_len) <= ((uint32_t)(void *)buffer + buffer_size))
    {
        SID_STM32_UTIL_fast_memcpy(*ingest_ptr, data, data_len);
    }
    else
    {
        const uint32_t bytes_before_wrap = buffer_size - ((uint32_t)(void *)(*ingest_ptr) - (uint32_t)(void *)buffer);
        SID_STM32_UTIL_fast_memcpy(*ingest_ptr, data, bytes_before_wrap);
        SID_STM32_UTIL_fast_memcpy(buffer, &data[bytes_before_wrap], (data_len - bytes_before_wrap));
    }

    uint32_t wrap_flag = _prv_advance_ring_buffer_pointer((const uint8_t **)ingest_ptr, data_len, buffer, buffer_size);
    return wrap_flag;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_uart_hw_init(const struct sid_pal_uart_config * const config)
{
    sid_error_t err;
    HAL_StatusTypeDef hal_status;

    do
    {
        UART_HandleTypeDef * const huart = config->huart;

        /* Build UART initialization config for HAL driver */
        huart->Instance                    = config->hw_instance;
        huart->Init.BaudRate               = config->baud_rate;
        huart->Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
        huart->Init.OverSampling           = UART_OVERSAMPLING_16;
        huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        huart->FifoMode                    = UART_FIFOMODE_DISABLE; /* Keep disabledby default. It will be enabled on a later stage if FIFO mode is requested in config */

        /* Select clock prescaler based on the targeted baud rate and oversampling mode */
        huart->Init.ClockPrescaler = _prv_calc_clock_prescaler(_prv_get_kernel_clock_source_freq(config), huart->Init.BaudRate, huart->Init.OverSampling);

        /* Configure word length */
        switch (config->data_bits)
        {
            case 7u:
                huart->Init.WordLength = UART_WORDLENGTH_7B;
                err = SID_ERROR_NONE;
                break;

            case 8u:
                huart->Init.WordLength = UART_WORDLENGTH_8B;
                err = SID_ERROR_NONE;
                break;

            case 9u:
                huart->Init.WordLength = UART_WORDLENGTH_9B;
                err = SID_ERROR_NONE;
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("%s init failed. Usupported data word length: %u", _prv_get_uart_display_name(huart->Instance), (uint32_t)config->data_bits);
            break;
        }

        /* Configure stop bits */
        switch (config->stop_bits)
        {
            case SID_PAL_UART_STOPBITS_0_5:
                huart->Init.StopBits = UART_STOPBITS_0_5;
                break;

            case SID_PAL_UART_STOPBITS_1:
                huart->Init.StopBits = UART_STOPBITS_1;
                break;

            case SID_PAL_UART_STOPBITS_1_5:
                huart->Init.StopBits = UART_STOPBITS_1_5;
                break;

            case SID_PAL_UART_STOPBITS_2:
                huart->Init.StopBits = UART_STOPBITS_2;
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("%s init failed. Usupported stop bits selection: %u", _prv_get_uart_display_name(huart->Instance), (uint32_t)config->stop_bits);
            break;
        }

        /* Configure parity */
        switch (config->parity)
        {
            case SID_PAL_UART_PARITY_NONE:
                huart->Init.Parity = UART_PARITY_NONE;
                break;

            case SID_PAL_UART_PARITY_EVEN:
                huart->Init.Parity = UART_PARITY_EVEN;
                break;

            case SID_PAL_UART_PARITY_ODD:
                huart->Init.Parity = UART_PARITY_ODD;
                break;

            default:
                err = SID_ERROR_INVALID_ARGS;
                break;
        }
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("%s init failed. Invalid parity selection: %u", _prv_get_uart_display_name(huart->Instance), (uint32_t)config->parity);
            break;
        }

        /* Configure flow control */
        switch (config->flow_control)
        {
            case SID_PAL_UART_HW_FLOW_CTRL_NONE:
                huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
                break;

            case SID_PAL_UART_HW_FLOW_CTRL_RTS:
                huart->Init.HwFlowCtl = UART_HWCONTROL_RTS;
                break;

            case SID_PAL_UART_HW_FLOW_CTRL_CTS:
                huart->Init.HwFlowCtl = UART_HWCONTROL_CTS;
                break;

            case SID_PAL_UART_HW_FLOW_CTRL_RTS_CTS:
                huart->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
                break;

            default:
                err = SID_ERROR_INVALID_ARGS;
                break;
        }
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("%s init failed. Invalid flow control selection: %u", _prv_get_uart_display_name(huart->Instance), (uint32_t)config->flow_control);
            break;
        }

        /* Select operating mode */
        switch (config->mode)
        {
            case SID_PAL_UART_MODE_RX:
                huart->Init.Mode = UART_MODE_RX;
                break;

            case SID_PAL_UART_MODE_TX:
                huart->Init.Mode = UART_MODE_TX;
                break;

            case SID_PAL_UART_MODE_TX_RX:
                huart->Init.Mode = UART_MODE_TX_RX;
                break;

            default:
                err = SID_ERROR_INVALID_ARGS;
                break;
        }
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("%s init failed. Invalid operating mode selection: %u", _prv_get_uart_display_name(huart->Instance), (uint32_t)config->mode);
            break;
        }

        /* Initialize UART peripheral */
        hal_status = HAL_UART_Init(huart);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("%s peripheral initialization failed. Error 0x%02X", _prv_get_uart_display_name(huart->Instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Enable Character Match event to get faster frame end detection */
        if (((SID_PAL_UART_MODE_TX_RX == config->mode) || (SID_PAL_UART_MODE_RX == config->mode))                  /* Rx mode is enabled for the current UART instance */
            && (config->frame_end_word != NULL) && (config->frame_end_word_len > ((config->data_bits - 1u) / 8u))) /* And frame end detection pattern is valid (not NULL, length is at least 1 byte for 7 and 8-bit character size or 2 bytes for 9-bit character */
        {
            const uint32_t last_byte = (config->data_bits > 8u) ? (uint32_t)config->frame_end_word[config->frame_end_word_len - 2u] : (uint32_t)config->frame_end_word[config->frame_end_word_len - 1u];

            /* Ensure UE bit is cleared because Character Match cannot be configured when UARt is enabled */
            __HAL_UART_DISABLE(huart);

            /* Enable Character Match event IRQ */
            SET_BIT(huart->Instance->CR1, USART_CR1_CMIE);

            /* Configure the character to match */
            MODIFY_REG(huart->Instance->CR2, (USART_CR2_ADD | USART_CR2_ADDM7), (((last_byte << USART_CR2_ADD_Pos) & USART_CR2_ADD_Msk) | USART_CR2_ADDM7)); /* Setting ADDM7 is optional here since it mainly applies to multiprocessor mode, which is not used by ths driver. But keeping it for any possible future support */

            /* Re-enable UART */
            __HAL_UART_ENABLE(huart);

            /* Wait for TEACK and/or REACK before proceeding */
            hal_status = UART_CheckIdleState(huart);
            if (hal_status != HAL_OK)
            {
                /* Indicate error since UART state is now undefined, requiring partial or full re-initialization */
                config->huart->gState  = HAL_UART_STATE_ERROR;

                SID_PAL_LOG_ERROR("Failed to enable Character Match for %s. HAL error 0x%02X", _prv_get_uart_display_name(config->hw_instance), hal_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }

        /* Ensure the HAL_UART_MspInit() function selected the right clock source for the UART */
        UART_ClockSourceTypeDef actual_clock_source;
        UART_GETCLOCKSOURCE(huart, actual_clock_source);
        if (config->clock_source != actual_clock_source)
        {
            SID_PAL_LOG_WARNING("%s source clock is set to 0x%02X instead of 0x%02X. Forcing switch to 0x%02X", _prv_get_uart_display_name(huart->Instance), actual_clock_source, config->clock_source, config->clock_source);

            err = _prv_apply_uart_clock_source(config);
            if (err != SID_ERROR_NONE)
            {
                /* Logs provided by _prv_apply_uart_clock_source() */
                break;
            }
        }

        /* Enable Autonomous mode if requested in the config */
        if (config->autonomous_mode_en != FALSE)
        {
            uint32_t clock_source;

            /* Ensure Kernel Domain clock source is HSI or LSE and the clock is not gated in Stop mode */
            switch ((uint32_t)(void *)huart->Instance)
            {
#ifdef LPUART1
                case (uint32_t)(void *)LPUART1:
                    clock_source = __HAL_RCC_GET_LPUART1_SOURCE();
                    if ((RCC_LPUART1CLKSOURCE_HSI == clock_source) || (RCC_LPUART1CLKSOURCE_LSE == clock_source))
                    {
                        __HAL_RCC_LPUART1_CLK_SLEEP_ENABLE();
                        err = SID_ERROR_NONE;
                    }
                    else
                    {
                        err = SID_ERROR_INVALID_STATE;
                    }
                    break;
#endif /* LPUART1 */

#ifdef USART1
                case (uint32_t)(void *)USART1:
                    clock_source = __HAL_RCC_GET_USART1_SOURCE();
                    if ((RCC_USART1CLKSOURCE_HSI == clock_source) || (RCC_USART1CLKSOURCE_LSE == clock_source))
                    {
                        __HAL_RCC_USART1_CLK_SLEEP_ENABLE();
                        err = SID_ERROR_NONE;
                    }
                    else
                    {
                        err = SID_ERROR_INVALID_STATE;
                    }
                    break;
#endif /* USART1 */

#ifdef USART2
                case (uint32_t)(void *)USART2:
                    clock_source = __HAL_RCC_GET_USART2_SOURCE();
                    if ((RCC_USART2CLKSOURCE_HSI == clock_source) || (RCC_USART2CLKSOURCE_LSE == clock_source))
                    {
                        __HAL_RCC_USART2_CLK_SLEEP_ENABLE();
                        err = SID_ERROR_NONE;
                    }
                    else
                    {
                        err = SID_ERROR_INVALID_STATE;
                    }
                    break;
#endif /* USART2 */

#ifdef USART3
                case (uint32_t)(void *)USART3:
                    clock_source = __HAL_RCC_GET_USART3_SOURCE();
                    if ((RCC_USART3CLKSOURCE_HSI == clock_source) || (RCC_USART3CLKSOURCE_LSE == clock_source))
                    {
                        __HAL_RCC_USART3_CLK_SLEEP_ENABLE();
                        err = SID_ERROR_NONE;
                    }
                    else
                    {
                        err = SID_ERROR_INVALID_STATE;
                    }
                    break;
#endif /* USART3 */

                default:
                    SID_PAL_LOG_ERROR("Unknown UART instance: 0x%08X. Can't enable clock in LPM", huart->Instance);
                    err = SID_ERROR_NOSUPPORT;
                    break;
            }
            if (err != SID_ERROR_NONE)
            {
                if (SID_ERROR_INVALID_STATE == err)
                {
                    SID_PAL_LOG_ERROR("%s: wrong clock source (0x%08X) for autonomous mode", _prv_get_uart_display_name(huart->Instance), clock_source);
                }
                break;
            }

#ifdef HAL_DMA_MODULE_ENABLED
            /* Ensure DMA Kernel Domain clock is enabled in LPM as well if DMA Rx/Tx is used by this UART */
            if ((huart->hdmarx != NULL) || (huart->hdmatx != NULL))
            {
                if (__HAL_RCC_GPDMA1_IS_CLK_SLEEP_ENABLED() == 0u)
                {
                    SID_PAL_LOG_ERROR("%s uses DMA and autonomous mode, but DMA clock is gated in LPM", _prv_get_uart_display_name(huart->Instance));
                    err = SID_ERROR_INVALID_STATE;
                    break;
                }
            }
#endif /* HAL_DMA_MODULE_ENABLED */

            /* Enable autonomous mode in UART peripheral itself */
            hal_status = HAL_UARTEx_EnableStopMode(huart);
            if (hal_status != HAL_OK)
            {
                SID_PAL_LOG_ERROR("%s: failed to enable autonomous mode. Error 0x%02X", _prv_get_uart_display_name(huart->Instance), hal_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }

        /* Configure FIFO thresholds */
        err = _prv_uart_apply_fifo_config(config);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _prv_uart_apply_fifo_config() */
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_uart_apply_fifo_config(const struct sid_pal_uart_config * const config)
{
    sid_error_t err;
    HAL_StatusTypeDef hal_status;

    do
    {
        UART_HandleTypeDef * const huart = config->huart;

        if (((SID_PAL_UART_MODE_TX_RX == config->mode) && ((SID_PAL_UART_FIFO_DISABLED == config->tx_fifo_threshold) && (SID_PAL_UART_FIFO_DISABLED == config->rx_fifo_threshold)))
         || ((SID_PAL_UART_MODE_TX == config->mode) && (SID_PAL_UART_FIFO_DISABLED == config->tx_fifo_threshold))
         || ((SID_PAL_UART_MODE_RX == config->mode) && (SID_PAL_UART_FIFO_DISABLED == config->rx_fifo_threshold))
        )
        {
            /* FIFO is not used */
            err = SID_ERROR_NONE;
            break;
        }

        /* Tx FIFO */
        if ((SID_PAL_UART_MODE_TX_RX == config->mode) || (SID_PAL_UART_MODE_TX == config->mode))
        {
            uint32_t tx_fifo_threshold_val;

            switch (config->tx_fifo_threshold)
            {
                case SID_PAL_UART_FIFO_THRESHOLD_1_8:
                    tx_fifo_threshold_val = UART_TXFIFO_THRESHOLD_1_8;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_1_4:
                    tx_fifo_threshold_val = UART_TXFIFO_THRESHOLD_1_4;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_1_2:
                    tx_fifo_threshold_val = UART_TXFIFO_THRESHOLD_1_2;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_3_4:
                    tx_fifo_threshold_val = UART_TXFIFO_THRESHOLD_3_4;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_7_8:
                    tx_fifo_threshold_val = UART_TXFIFO_THRESHOLD_7_8;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_8_8:
                case SID_PAL_UART_FIFO_DISABLED:
                default:
                    /* With this setting an IRQ or DMA request is generated when FIFO is completely empty */
                    tx_fifo_threshold_val = UART_TXFIFO_THRESHOLD_8_8;
                    break;
            }

            hal_status = HAL_UARTEx_SetTxFifoThreshold(huart, tx_fifo_threshold_val);
            if (hal_status != HAL_OK)
            {
                SID_PAL_LOG_ERROR("%s Tx FIFO configuration failed. Error 0x%02X", _prv_get_uart_display_name(huart->Instance), hal_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }

        /* Rx FIFO */
        if ((SID_PAL_UART_MODE_TX_RX == config->mode) || (SID_PAL_UART_MODE_RX == config->mode))
        {
            uint32_t rx_fifo_threshold_val;

            switch (config->rx_fifo_threshold)
            {
                case SID_PAL_UART_FIFO_THRESHOLD_1_4:
                    rx_fifo_threshold_val = UART_RXFIFO_THRESHOLD_1_4;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_1_2:
                    rx_fifo_threshold_val = UART_RXFIFO_THRESHOLD_1_2;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_3_4:
                    rx_fifo_threshold_val = UART_RXFIFO_THRESHOLD_3_4;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_7_8:
                    rx_fifo_threshold_val = UART_RXFIFO_THRESHOLD_7_8;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_8_8:
                    rx_fifo_threshold_val = UART_RXFIFO_THRESHOLD_8_8;
                    break;

                case SID_PAL_UART_FIFO_THRESHOLD_1_8:
                case SID_PAL_UART_FIFO_DISABLED:
                default:
                    /* With this setting an IRQ or DMA request is generated as soon as a single byte gets into the FIFO */
                    rx_fifo_threshold_val = UART_RXFIFO_THRESHOLD_1_8;
                    break;
            }

            hal_status = HAL_UARTEx_SetRxFifoThreshold(huart, rx_fifo_threshold_val);
            if (hal_status != HAL_OK)
            {
                SID_PAL_LOG_ERROR("%s Rx FIFO configuration failed. Error 0x%02X", _prv_get_uart_display_name(huart->Instance), hal_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }

        /* Globally enable FIFO mode if at least Rx or Tx path requires it */
        hal_status = HAL_UARTEx_EnableFifoMode(huart);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to enable FIFO for %s. Error 0x%02X", _prv_get_uart_display_name(huart->Instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static const char * _prv_get_uart_display_name(const USART_TypeDef * const instance)
{
    const char * display_name;

    switch ((uint32_t)(void *)instance)
    {
#ifdef USART1
        case (uint32_t)(void *)USART1:
            display_name = "USART1";
            break;
#endif /* USART1 */

#ifdef USART2
        case (uint32_t)(void *)USART2:
            display_name = "USART2";
            break;
#endif /* USART2 */

#ifdef USART3
        case (uint32_t)(void *)USART3:
            display_name = "USART3";
            break;
#endif /* USART3 */

#ifdef LPUART1
        case (uint32_t)(void *)LPUART1:
            display_name = "LPUART1";
            break;
#endif /* LPUART1 */

        default:
            /* Normally this shall not ever happen */
            SID_PAL_ASSERT(0);
            display_name = NULL;
            break;
    }

    return display_name;
}

/*----------------------------------------------------------------------------*/

static void _prv_calc_rx_inspection_period(uart_serial_client_impl_t * const impl)
{
    SID_PAL_ASSERT(impl != NULL);
    SID_PAL_ASSERT(impl->config != NULL);

    /* Compute check interval based on the UART communication speed and Rx buffer size */
    const uint32_t baud_rate_to_use         = (impl->config->huart->gState != HAL_UART_STATE_RESET) ? impl->config->huart->Init.BaudRate : impl->config->baud_rate;
    const uint32_t bytes_per_uart_word      = (((uint32_t)impl->config->data_bits + 7u) / 8u); /* Amount of storage bytes required per single UART word. E.g., if UART word length is 9 bits it requires 2 bytes to store it */
    const uint32_t uart_words_per_second    = (baud_rate_to_use + ((uint32_t)impl->config->data_bits - 1u)) / (uint32_t)impl->config->data_bits; /* UART words per second rate with ceiling. Don't account for star/stop bits since they will only lower this value, current formula considers worst case */
    const uint32_t storage_bytes_per_second = bytes_per_uart_word * uart_words_per_second;
    const uint32_t check_interval_max_us    = (uint32_t)(((uint64_t)impl->config->rx_buffer_size * (uint64_t)1000000u) / (uint64_t)storage_bytes_per_second) / 4u;
    uint32_t check_interval_us;

    if (0u == impl->config->rx_check_period_us)
    {
        /* Period is not set explicitly, use the default value */
        check_interval_us = check_interval_max_us;
    }
#if SID_PAL_UART_CLIENT_OVERRIDE_TOO_LONG_RX_INSPECTION_PERIOD
    else if (impl->config->rx_check_period_us > check_interval_max_us)
    {
        /* Requested check interval is too long, buffer overrun is highly possible if we proceed with the requested value */
        SID_PAL_LOG_WARNING("%s: requested Rx inspection period of %uus is too slow for selected baud rate", _prv_get_uart_display_name(impl->config->hw_instance), impl->config->rx_check_period_us);
        check_interval_us = check_interval_max_us;
    }
#endif /* SID_PAL_UART_CLIENT_OVERRIDE_TOO_LONG_RX_INSPECTION_PERIOD */
    else if (impl->config->rx_check_period_us < SID_PAL_UART_RX_CHECK_PERIOD_MIN)
    {
        /* Requested check interval is too short, CPU load will be too high */
        SID_PAL_LOG_WARNING("%s: requested Rx inspection period of %uus is too short and will be extended", _prv_get_uart_display_name(impl->config->hw_instance), impl->config->rx_check_period_us);
        check_interval_us = SID_PAL_UART_RX_CHECK_PERIOD_MIN;
    }
    else
    {
        /* Requested check interval is withing the range, proceed with it */
        check_interval_us = impl->config->rx_check_period_us;
    }

    SID_PAL_LOG_INFO("%s: Rx inspection period set to %uus", _prv_get_uart_display_name(impl->config->hw_instance), check_interval_us);

    impl->rx_buf_inspect_period_us = check_interval_us;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_schedule_rx_inspection(uart_serial_client_impl_t * const impl, const uint32_t start_delay_us)
{
    sid_error_t err;

    /* This method may be triggered from various ISRs with different priorities. Unpredictable nested IRQs will result in race conditions and inconsistent timer state, so a critical section is a must */
    sid_pal_enter_critical_region();

    do
    {
        /* Arrange periodic Rx buffer checks for full frame detection */
        struct sid_timespec check_start_time, check_period, check_delay;
        sid_us_to_timespec(impl->rx_buf_inspect_period_us, &check_period);
        sid_us_to_timespec(start_delay_us, &check_delay);

        /* Compute first check time */
        err = sid_pal_uptime_now(&check_start_time);
        if (err != SID_ERROR_NONE)
        {
            break;
        }
        sid_time_add(&check_start_time, &check_delay);
        __COMPILER_BARRIER();

        (void)sid_pal_timer_cancel(&impl->rx_buf_inspection_timer);
        __COMPILER_BARRIER();

        /* Start timer for Rx buffer periodic checks */
        err = sid_pal_timer_arm(&impl->rx_buf_inspection_timer, SID_PAL_TIMER_PRIO_CLASS_PRECISE, &check_start_time, &check_period);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to start Rx processing timer for %s. Error %d", _prv_get_uart_display_name(impl->config->hw_instance), (int32_t)err);
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

static uint32_t _prv_get_kernel_clock_source_freq(const struct sid_pal_uart_config * const config)
{
    uint32_t                clock_source_frequency;
    UART_ClockSourceTypeDef clock_source;

    /* Try to read the actual clock source from the hardware */
    SID_PAL_ASSERT(config->hw_instance == config->huart->Instance);
    if (HAL_UART_STATE_RESET == config->huart->gState)
    {
        /* This may happen if UART peripheral is not enabled or not configured yet. Fall back to the static configuration */
        clock_source = config->clock_source;
    }
    else
    {
        /* Get the actual clock source from RCC */
        UART_GETCLOCKSOURCE(config->huart, clock_source);
    }

    /* Now get the frequency of the Kernel Domain clock source. Note - this is NOT the frequency of the Kernel Domain clock itself, it does not account for UART clock prescaler */
    switch (clock_source)
    {
#if defined(USART2) || defined(USART3)
        case UART_CLOCKSOURCE_PCLK1:
            clock_source_frequency = HAL_RCC_GetPCLK1Freq();
            break;
#endif /* USART2 || USART3 */

#ifdef USART1
        case UART_CLOCKSOURCE_PCLK2:
            clock_source_frequency = HAL_RCC_GetPCLK2Freq();
            break;
#endif /* USART1 */

#ifdef LPUART1
        case UART_CLOCKSOURCE_PCLK7:
            clock_source_frequency = HAL_RCC_GetPCLK7Freq();
            break;
#endif /* LPUART1 */

        case UART_CLOCKSOURCE_HSI:
            clock_source_frequency = (uint32_t)HSI_VALUE;
            break;

        case UART_CLOCKSOURCE_SYSCLK:
            clock_source_frequency = HAL_RCC_GetSysClockFreq();
            break;

        case UART_CLOCKSOURCE_LSE:
            clock_source_frequency = (uint32_t)LSE_VALUE;
            break;

        case UART_CLOCKSOURCE_UNDEFINED:
        default:
            clock_source_frequency = 0u;
            break;
    }

    return clock_source_frequency;
}

/*----------------------------------------------------------------------------*/

static uint32_t _prv_calc_clock_prescaler(const uint32_t kernel_clock_freq, const uint32_t baudrate, const uint32_t oversampling)
{
    uint32_t prescaler_idx; /** See @ref UART_ClockPrescaler */

    do
    {
        const uint32_t oversampling_ratio = (UART_OVERSAMPLING_8 == oversampling) ? 8u : 16u;
        const uint32_t sampling_freq      = baudrate * oversampling_ratio;

        if (sampling_freq > kernel_clock_freq)
        {
            /* Invalid configuration - the requested sampling frequency exceeds kernel clock */
            prescaler_idx = UINT32_MAX;
            break;
        }

        const uint32_t div_factor = kernel_clock_freq / sampling_freq; /* Using intentional round-down here since the resulting clock shall be greater or equal to the sampling frequency */

        /* Look up for the prescaler index. The first element of UARTPrescTable is not evaluated since that is the minimum prescaler value. If the code gets to the first index, we have to use it regardless if it satisfies the condition or not */
        for (prescaler_idx = (SID_STM32_UTIL_ARRAY_SIZE(UARTPrescTable) - 1u); prescaler_idx > 0u; prescaler_idx--)
        {
            if (div_factor >= UARTPrescTable[prescaler_idx])
            {
                /* Found suitable prescaler */
                break;
            }
        }
    } while (0u);

    return prescaler_idx;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _prv_apply_uart_clock_source(const struct sid_pal_uart_config * const config)
{
    sid_error_t              err;
    HAL_StatusTypeDef        hal_status;
    RCC_PeriphCLKInitTypeDef clock_init_cfg;

    do
    {
        err = SID_ERROR_NONE;

        switch ((uint32_t)(void *)config->hw_instance)
        {
#ifdef USART1
            case (uint32_t)(void *)USART1:
                clock_init_cfg.PeriphClockSelection = RCC_PERIPHCLK_USART1;
                if (UART_CLOCKSOURCE_PCLK2 == config->clock_source)
                {
                    clock_init_cfg.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
                }
                else if (UART_CLOCKSOURCE_HSI == config->clock_source)
                {
                    clock_init_cfg.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
                }
                else if (UART_CLOCKSOURCE_SYSCLK == config->clock_source)
                {
                    clock_init_cfg.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
                }
                else if (UART_CLOCKSOURCE_LSE == config->clock_source)
                {
                    clock_init_cfg.Usart1ClockSelection = RCC_USART1CLKSOURCE_LSE;
                }
                else
                {
                    SID_PAL_LOG_ERROR("Invalid clock source selection (0x%02X) for %s", config->clock_source, _prv_get_uart_display_name(config->hw_instance));
                    err = SID_ERROR_INVALID_ARGS;
                }
                break;
#endif /* USART1 */

#ifdef USART2
            case (uint32_t)(void *)USART2:
                clock_init_cfg.PeriphClockSelection = RCC_PERIPHCLK_USART2;
                if (UART_CLOCKSOURCE_PCLK1 == config->clock_source)
                {
                    clock_init_cfg.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
                }
                else if (UART_CLOCKSOURCE_HSI == config->clock_source)
                {
                    clock_init_cfg.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
                }
                else if (UART_CLOCKSOURCE_SYSCLK == config->clock_source)
                {
                    clock_init_cfg.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
                }
                else if (UART_CLOCKSOURCE_LSE == config->clock_source)
                {
                    clock_init_cfg.Usart2ClockSelection = RCC_USART2CLKSOURCE_LSE;
                }
                else
                {
                    SID_PAL_LOG_ERROR("Invalid clock source selection (0x%02X) for %s", config->clock_source, _prv_get_uart_display_name(config->hw_instance));
                    err = SID_ERROR_INVALID_ARGS;
                }
                break;
#endif /* USART2 */

#ifdef USART3
            case (uint32_t)(void *)USART3:
                clock_init_cfg.PeriphClockSelection = RCC_PERIPHCLK_USART3;
                if (UART_CLOCKSOURCE_PCLK1 == config->clock_source)
                {
                    clock_init_cfg.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
                }
                else if (UART_CLOCKSOURCE_HSI == config->clock_source)
                {
                    clock_init_cfg.Usart3ClockSelection = RCC_USART3CLKSOURCE_HSI;
                }
                else if (UART_CLOCKSOURCE_SYSCLK == config->clock_source)
                {
                    clock_init_cfg.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
                }
                else if (UART_CLOCKSOURCE_LSE == config->clock_source)
                {
                    clock_init_cfg.Usart3ClockSelection = RCC_USART3CLKSOURCE_LSE;
                }
                else
                {
                    SID_PAL_LOG_ERROR("Invalid clock source selection (0x%02X) for %s", config->clock_source, _prv_get_uart_display_name(config->hw_instance));
                    err = SID_ERROR_INVALID_ARGS;
                }
                break;
#endif /* USART3 */

#ifdef LPUART1
            case (uint32_t)(void *)LPUART1:
                clock_init_cfg.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
                if (UART_CLOCKSOURCE_PCLK7 == config->clock_source)
                {
                    clock_init_cfg.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK7;
                }
                else if (UART_CLOCKSOURCE_HSI == config->clock_source)
                {
                    clock_init_cfg.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
                }
                else if (UART_CLOCKSOURCE_SYSCLK == config->clock_source)
                {
                    clock_init_cfg.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_SYSCLK;
                }
                else if (UART_CLOCKSOURCE_LSE == config->clock_source)
                {
                    clock_init_cfg.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_LSE;
                }
                else
                {
                    SID_PAL_LOG_ERROR("Invalid clock source selection (0x%02X) for %s", config->clock_source, _prv_get_uart_display_name(config->hw_instance));
                    err = SID_ERROR_INVALID_ARGS;
                }
                break;
#endif /* LPUART1 */

            default:
                SID_PAL_LOG_WARNING("Unknown UART instance: 0x%08X. Can't select clock source", config->hw_instance);
                err = SID_ERROR_INVALID_ARGS;
                break;
        }
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Select the specified clock source in RCC */
        hal_status = HAL_RCCEx_PeriphCLKConfig(&clock_init_cfg);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set clock source for %s. HAL error 0x%02X", _prv_get_uart_display_name(config->hw_instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Ensure clock prescaler is valid */
        config->huart->Init.ClockPrescaler = _prv_calc_clock_prescaler(_prv_get_kernel_clock_source_freq(config), config->huart->Init.BaudRate, config->huart->Init.OverSampling);

        /* Ensure UART UE bit is cleared before proceeding */
        const uint32_t uart_was_enabled = (READ_BIT(config->huart->Instance->CR1, USART_CR1_UE_Msk) != 0u) ? TRUE : FALSE;
        __HAL_UART_DISABLE(config->huart);

        /* Re-apply UART hardware config after the clock switch */
        hal_status = UART_SetConfig(config->huart);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to reconfigure %s after clock source change. HAL error 0x%02X", _prv_get_uart_display_name(config->hw_instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Re-enable the UART after clock switch if it was previously enabled */
        if (uart_was_enabled != FALSE)
        {
            __HAL_UART_ENABLE(config->huart);

            /* Wait for TEACK and/or REACK before proceeding */
            hal_status = UART_CheckIdleState(config->huart);
            if (hal_status != HAL_OK)
            {
                /* Indicate error since UART state is now undefined, requiring partial or full re-initialization */
                config->huart->gState  = HAL_UART_STATE_ERROR;

                SID_PAL_LOG_ERROR("Failed to reconfigure baud rate for %s. HAL error 0x%02X", _prv_get_uart_display_name(config->hw_instance), hal_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_start_continuous_rx(uart_serial_client_impl_t * const impl, const uint32_t inspect_rx_buf)
{
    sid_error_t err;
    HAL_StatusTypeDef hal_status;

    do
    {
        /* Make sure this operation is performed over a UART that can run Rx */
        if ((impl->config->mode != SID_PAL_UART_MODE_TX_RX) && (impl->config->mode != SID_PAL_UART_MODE_RX))
        {
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Ensure the UART state is valid for initiating Rx */
        if (READ_BIT(impl->config->huart->Instance->CR1, USART_CR1_UE) == 0u)
        {
            /* UART peripheral is disabled */
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        if (impl->config->huart->RxState != HAL_UART_STATE_READY)
        {
            if (HAL_UART_STATE_BUSY_RX == impl->config->huart->RxState)
            {
                /* UART is already in the Rx mode */
                err = SID_ERROR_NONE;
                break;
            }
            else
            {
                /* UART is in a state when Rx cannot be initiated */
                SID_PAL_LOG_ERROR("Unable to start %s Rx, invalid UART Rx state: 0x%08X", _prv_get_uart_display_name(impl->config->hw_instance), impl->config->huart->RxState);
                err = SID_ERROR_INVALID_STATE;
                break;
            }
        }

        /* Ensure Rx processing timer is stopped */
        (void)sid_pal_timer_cancel(&impl->rx_buf_inspection_timer);
        __COMPILER_BARRIER();

        /* If requested, check the Rx buffer for any valid data before flushing it */
        if (inspect_rx_buf != FALSE)
        {
            _prv_rx_inspection_event_handler(impl, (void *)SID_PAL_UART_ORIGINATOR_ID_ERROR_CALLBACK);
        }

        /* Reset Rx ring buffer state */
        impl->rx_buf_write_wrap_cnt  = 0u;
        impl->rx_buf_inspect_ptr     = impl->rx_buffer;
        impl->rx_buf_frame_start_ptr = NULL;
        impl->rx_buf_frame_end_ptr   = NULL;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /* Configure UART callback for Rx ring buffer write position roll-over */
        impl->config->huart->RxCpltCallback = _prv_rx_buffer_write_wrap_event_handler;
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        __COMPILER_BARRIER();

        /* Flush Rx shift reg and/or Rx FIFO */
        while (READ_BIT(impl->config->huart->Instance->ISR, USART_ISR_RXNE_RXFNE) != 0u)
        {
            /* Note: it assumed that CPU core and APB clock speeds are much higher than UART baud rate, so this loop won't execute indefinitely if UART is actively receiving non-stop data stream */
            (void)impl->config->huart->Instance->RDR;
        }
        __COMPILER_BARRIER();

#ifdef HAL_DMA_MODULE_ENABLED
        if (impl->config->huart->hdmarx != NULL)
        {
            /* Start DMA-driven Rx */
            hal_status = HAL_UART_Receive_DMA(impl->config->huart, impl->rx_buffer, impl->config->rx_buffer_size);
        }
        else
#endif /* HAL_DMA_MODULE_ENABLED */
        {
            /* Start IRQ-driven Rx */
            hal_status = HAL_UART_Receive_IT(impl->config->huart, impl->rx_buffer, impl->config->rx_buffer_size);
        }

        /* Check if continuous Rx was started successfully */
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to start %s Rx. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        __COMPILER_BARRIER();

        /* Enable IDLE interrupt */
        __HAL_UART_CLEAR_FLAG(impl->config->huart, UART_CLEAR_IDLEF);
        ATOMIC_SET_BIT(impl->config->huart->Instance->CR1, USART_CR1_IDLEIE);

        /* Arrange periodic Rx buffer checks for full frame detection */
        err = _prv_schedule_rx_inspection(impl, impl->rx_buf_inspect_period_us);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _prv_schedule_rx_inspection() */
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _prv_stop_continuous_rx(uart_serial_client_impl_t * const impl)
{
    sid_error_t err;
    HAL_StatusTypeDef hal_status;

    do
    {
        /* Make sure this operation is performed over a UART that can run Rx */
        if ((impl->config->mode != SID_PAL_UART_MODE_TX_RX) && (impl->config->mode != SID_PAL_UART_MODE_RX))
        {
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Ensure Rx processing timer is stopped */
        (void)sid_pal_timer_cancel(&impl->rx_buf_inspection_timer);
        __COMPILER_BARRIER();

        /* Disable UART IDLE interrupt */
        ATOMIC_CLEAR_BIT(impl->config->huart->Instance->CR1, USART_CR1_IDLEIE);
        __HAL_UART_CLEAR_FLAG(impl->config->huart, UART_CLEAR_IDLEF);

        /* Abort any ongoing Rx operation */
        hal_status = HAL_UART_AbortReceive(impl->config->huart);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to stop %sRx. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _prv_rx_inspection_event_handler(void * arg, sid_pal_timer_t * originator)
{
    uart_serial_client_impl_t * const impl = (uart_serial_client_impl_t *)arg;

    if (NULL == impl)
    {
        return;
    }

    sid_pal_enter_critical_region();

    do
    {
        register uint32_t buffer_write_ptr;

#ifdef HAL_DMA_MODULE_ENABLED
        if (impl->config->huart->hdmarx != NULL)
        {
            /* Check if DMA Transfer Complete IRQ is pending and this inspection is not originating from the UART error callback */
            if (((uint32_t)(void *)originator != SID_PAL_UART_ORIGINATOR_ID_ERROR_CALLBACK) && (__HAL_DMA_GET_FLAG(impl->config->huart->hdmarx, DMA_FLAG_TC) != 0U))
            {
                /* Let the Transfer Complete IRQ to be processed before proceeding */
                break;
            }

            /* Capture DMA write pointer */
            buffer_write_ptr = impl->config->huart->hdmarx->Instance->CDAR;
        }
        else
#endif /* HAL_DMA_MODULE_ENABLED */
        {
            /* Check if UART Rx / Rx FIFO Not Empty IRQ is pending and we are close to buffer wrap point */
            if ((impl->config->huart->RxXferCount <= 8u) && (READ_BIT(impl->config->huart->Instance->ISR, USART_ISR_RXNE_RXFNE) != 0u))
            {
                /* Let the Rx / Rx FIFO Not Empty IRQ to be processed before proceeding */
                break;
            }

            /* Capture UART Rx buffer write pointer */
            buffer_write_ptr = (uint32_t)(void *)impl->config->huart->pRxBuffPtr;
        }

        /* Check for buffer overflow: either write pointer is ahead of read pointer with wrap indication or wrap count is more than 1 regardless of read and write pointer relation */
        const uint32_t buf_overrun_border_ptr = (impl->rx_buf_frame_start_ptr != NULL) ? (uint32_t)(void *)impl->rx_buf_frame_start_ptr : (uint32_t)(void *)impl->rx_buf_inspect_ptr; /* Use frame start as a borderline if detected, otherwise use inspection pointer */
        if (((buffer_write_ptr > buf_overrun_border_ptr) && (impl->rx_buf_write_wrap_cnt != 0u)) || (impl->rx_buf_write_wrap_cnt > 1u))
        {
            /* Buffer overflow, new data was written on top of read space - flush the buffer by resetting the read pointers and continue */
            SID_PAL_LOG_ERROR("%s Rx buffer overrun. Data loss is possible", _prv_get_uart_display_name(impl->config->hw_instance));
            impl->rx_buf_inspect_ptr     = (uint8_t *)(void *)buffer_write_ptr;
            impl->rx_buf_frame_start_ptr = NULL;
            impl->rx_buf_frame_end_ptr   = NULL;
            impl->rx_buf_write_wrap_cnt  = 0u;
            break;
        }

        /* Check if there's a valid frame detected, but not read out yet by the UART client user */
        if ((impl->rx_buf_frame_start_ptr != NULL) && (impl->rx_buf_frame_end_ptr != NULL))
        {
            break;
        }

        /* Check if there's any new data to inspect */
        while ((buffer_write_ptr > (uint32_t)(void *)impl->rx_buf_inspect_ptr) || ((impl->rx_buf_write_wrap_cnt != 0u) && (buffer_write_ptr < (uint32_t)(void *)impl->rx_buf_inspect_ptr)))
        {
            /* Save a copy of the inspection start position */
            const uint8_t * inspection_start_ptr = impl->rx_buf_inspect_ptr;

            /* Perform frame start detection if required */
            if (NULL == impl->rx_buf_frame_start_ptr)
            {
                if (impl->config->frame_start_word != NULL)
                {
                    impl->rx_buf_frame_start_ptr = _prv_search_pattern_in_ring_buf(impl->config->frame_start_word, impl->config->frame_start_word_len, impl->rx_buffer, impl->config->rx_buffer_size, &impl->rx_buf_inspect_ptr, (const uint8_t *)(void *)buffer_write_ptr);
                    if (NULL == impl->rx_buf_frame_start_ptr)
                    {
                        /* Terminate processing if all the available data was checked and frame start was not detected, otherwise continue processing since full frame may already be in the buffer */
                        break;
                    }
                }
                else
                {
                    /* Frame start pattern is not used, just set frame start to the current buffer position */
                    impl->rx_buf_frame_start_ptr = impl->rx_buf_inspect_ptr;
                }

                SID_PAL_UART_LOG_DEBUG("Frame start: %u", (uint32_t)(void *)impl->rx_buf_frame_start_ptr - (uint32_t)(void *)impl->rx_buffer);

                /* Restart the loop to evaluate if we have enough new data available to proceed with frame end search */
                continue;
            }

            /* Perform frame end detection */
            impl->rx_buf_frame_end_ptr = _prv_search_pattern_in_ring_buf(impl->config->frame_end_word, impl->config->frame_end_word_len, impl->rx_buffer, impl->config->rx_buffer_size, &impl->rx_buf_inspect_ptr, (const uint8_t *)(void *)buffer_write_ptr);
            if (impl->rx_buf_frame_end_ptr != NULL)
            {
                /* Make frame end pointer to point to the first byte after the frame */
                impl->rx_buf_frame_end_ptr = impl->rx_buf_inspect_ptr; /* Not using _prv_advance_ring_buffer_pointer() here for performance reasons - calling _prv_advance_ring_buffer_pointer() will always result in the same value as impl->rx_buf_inspect_ptr at this point */
            }
            else
            {
                /* End word pattern is not detected in the available data */

                /* Additional check when frame start word is used - verify frame start word is not repeated before frame end arrives */
                if ((impl->config->frame_start_word != NULL) && (impl->rx_buf_frame_start_ptr != NULL))
                {
                    /* Re-inspect the same part of the ring buffer */
                    const uint8_t * repeated_start_ptr = _prv_search_pattern_in_ring_buf(impl->config->frame_start_word, impl->config->frame_start_word_len, impl->rx_buffer, impl->config->rx_buffer_size, &inspection_start_ptr, (const uint8_t *)(void *)buffer_write_ptr);
                    if (repeated_start_ptr != NULL)
                    {
                        /* Found a repeated frame start word. This may happen if the previous frame was not sent in full (e.g., remote side sudden reset) */

                        /* Adjust DMA write wrap counter if applicable */
                        if (repeated_start_ptr < impl->rx_buf_frame_start_ptr)
                        {
                            /* The readout from the original frame start pointer should have cleared the wrap indication */
                            impl->rx_buf_write_wrap_cnt  = 0u;
                        }

                        /* Move the frame start pointer to the new position and discard the previous incomplete frame data */
                        impl->rx_buf_frame_start_ptr = repeated_start_ptr;

                        SID_PAL_LOG_WARNING("%s: detected repetitive frame start before frame end detection", _prv_get_uart_display_name(impl->config->hw_instance));
                    }
                }

                /* Terminate the search and wait for more bytes to arrive */
                break;
            }

            /* Check if a full frame is now available */
            if ((impl->rx_buf_frame_start_ptr != NULL) && (impl->rx_buf_frame_end_ptr != NULL))
            {
                SID_PAL_UART_LOG_DEBUG("Frame end: %u", (uint32_t)(void *)impl->rx_buf_frame_end_ptr - (uint32_t)(void *)impl->rx_buffer);

                sid_error_t err = (impl->callbacks->new_rx_done_cb != NULL) ? impl->callbacks->new_rx_done_cb(impl->cb_ctx) : SID_ERROR_NULL_POINTER;
                if (err != SID_ERROR_NONE)
                {
                    /* Something went wrong, discard the received data */
                    if ((1u == impl->rx_buf_write_wrap_cnt)
#ifdef HAL_DMA_MODULE_ENABLED
                      && ((NULL == impl->config->huart->hdmarx) || (__HAL_DMA_GET_FLAG(impl->config->huart->hdmarx , DMA_FLAG_TC) == 0U))
#endif /* HAL_DMA_MODULE_ENABLED */
                    )
                    {
                        /* This readout should have cleared the wrap indication on success */
                        impl->rx_buf_write_wrap_cnt  = 0u;
                    }
                    impl->rx_buf_frame_start_ptr = NULL;
                    impl->rx_buf_frame_end_ptr   = NULL;
                }
            }
        }
    } while (0);

    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _prv_rx_buffer_write_wrap_event_handler(UART_HandleTypeDef * huart)
{
    sid_error_t       err;
    HAL_StatusTypeDef hal_status;

    sid_pal_enter_critical_region();

    do
    {
        /* Search for the associated UART client instance */
        uart_serial_client_impl_t * const impl = _prv_find_client_by_uart(huart);

        if (NULL == impl)
        {
            /* Normally should never happen */
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by non-related UART (%s)", _prv_get_uart_display_name(huart->Instance));
            break;
        }

        /* Increment wrap counter to monitor ring buffer overrun */
        impl->rx_buf_write_wrap_cnt++;
        __COMPILER_BARRIER();

        /* If reception is IRQ-driven, restart Rx ASAP */
        if (NULL == impl->config->huart->hdmarx)
        {
            hal_status = HAL_UART_Receive_IT(impl->config->huart, impl->rx_buffer, impl->config->rx_buffer_size);
            if (hal_status != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to start %s Rx. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
                /* Still proceed to at least process what is in the Rx buffer already */
            }
        }

        SID_PAL_UART_LOG_DEBUG("WRAP %u", impl->rx_buf_write_wrap_cnt);

        /* Reschedule Rx buffer full frame detection to ASAP */
        err = _prv_schedule_rx_inspection(impl, 0u);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _prv_schedule_rx_inspection() */
            break;
        }
    } while (0);

    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _prv_on_uart_xfer_error(UART_HandleTypeDef * huart)
{
    do
    {
        /* Search for the associated UART client instance */
        uart_serial_client_impl_t * const impl = _prv_find_client_by_uart(huart);

        if (NULL == impl)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by non-related UART (%s)", _prv_get_uart_display_name(huart->Instance));
            break;
        }

        /* Reset Tx buffer state */
        if ((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_TX == impl->config->mode))
        {
            impl->tx_buf_ingest_ptr       = impl->tx_buffer;
            impl->tx_buf_sendout_ptr      = impl->tx_buffer;
            impl->tx_buf_ingest_wrap_flag = FALSE;
        }

        /* Restart Rx if needed */
        if (((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_RX == impl->config->mode)) && (sid_pal_timer_is_armed(&impl->rx_buf_inspection_timer) != false))
        {
            /* Capture the original error before restarting */
            const uint32_t comm_error_code = huart->ErrorCode;

            /* Restart the Rx */
            sid_error_t err = _prv_start_continuous_rx(impl, TRUE);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to restart %s after comm error 0x%08X. UART error: 0x%08X, restart error: %d", _prv_get_uart_display_name(huart->Instance), comm_error_code, huart->ErrorCode, (int32_t)err);
                break;
            }
            else
            {
                SID_PAL_UART_LOG_WARNING("Restarted %s due to comm error 0x%08X", _prv_get_uart_display_name(huart->Instance), comm_error_code);
            }
        }
    } while (0);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _prv_on_uart_tx_completed(UART_HandleTypeDef * huart)
{
    uint32_t more_bytes_to_send;

    do
    {
        /* Search for the associated UART client instance */
        uart_serial_client_impl_t * const impl = _prv_find_client_by_uart(huart);

        if (NULL == impl)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by non-related UART (%s)", _prv_get_uart_display_name(huart->Instance));
            break;
        }

        if ((impl->config->mode != SID_PAL_UART_MODE_TX_RX) && (impl->config->mode != SID_PAL_UART_MODE_TX))
        {
            SID_PAL_LOG_ERROR("Tx Done event triggered for %s, but Tx was not enabled for this UART", _prv_get_uart_display_name(huart->Instance));
            break;
        }

        sid_pal_enter_critical_region();
        uint32_t wrap_on_advance = _prv_advance_ring_buffer_pointer((const uint8_t **)&impl->tx_buf_sendout_ptr, huart->TxXferSize, impl->tx_buffer, impl->config->tx_buffer_size);
        if (wrap_on_advance != FALSE)
        {
            /* Wrap on send out clears ingest wrap flag */
            impl->tx_buf_ingest_wrap_flag = FALSE;
        }

        /* Restart Tx immediately if there's more data to send */
        more_bytes_to_send = (impl->tx_buf_sendout_ptr <= impl->tx_buf_ingest_ptr) ?
                                ((uint32_t)(void *)impl->tx_buf_ingest_ptr - (uint32_t)(void *)impl->tx_buf_sendout_ptr) :
                                (impl->config->tx_buffer_size - ((uint32_t)(void *)impl->tx_buf_sendout_ptr - (uint32_t)(void *)impl->tx_buffer));
        if (more_bytes_to_send > 0u)
        {
            HAL_StatusTypeDef hal_status;
#ifdef HAL_DMA_MODULE_ENABLED
            if (impl->config->huart->hdmatx != NULL)
            {
                /* Start DMA Tx */
                hal_status = HAL_UART_Transmit_DMA(impl->config->huart, impl->tx_buf_sendout_ptr, more_bytes_to_send);
            }
            else
#endif /* HAL_DMA_MODULE_ENABLED */
            {
                /* Start IRQ-based Tx */
                hal_status = HAL_UART_Transmit_IT(impl->config->huart, impl->tx_buf_sendout_ptr, more_bytes_to_send);
            }
            if (hal_status != HAL_OK)
            {
                /* Report an error. Keep ingest and send-out pointers intact - this will give a chance to restart Tx without data drop outs */
                SID_PAL_LOG_ERROR("Failed to start %s Tx. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
            }
        }
        sid_pal_exit_critical_region();

        /* Finally call user callback to notify about Transfer end */
        if ((0u == more_bytes_to_send) && (impl->callbacks->tx_done_cb != NULL))
        {
            (void)impl->callbacks->tx_done_cb(impl->cb_ctx);
        }
    } while (0);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ifc_send(const sid_pal_serial_ifc_t * _this, const uint8_t * frame_to_send, size_t frame_size)
{
    sid_error_t       err;
    HAL_StatusTypeDef hal_status;
    uint32_t          need_to_add_start_word;
    uint32_t          need_to_add_end_word;

    sid_pal_enter_critical_region();

    do
    {
        /* Validate inputs */
        if ((NULL == _this) || (NULL == frame_to_send) || (0u == frame_size))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        /* Store the original Tx buffer ingestion position */
        uint8_t * const ingest_ptr_on_entry       = impl->tx_buf_ingest_ptr;
        const uint8_t   ingest_wrap_flag_on_entry = impl->tx_buf_ingest_wrap_flag;

        /* Check if supplied frame includes valid start and stop sequences */
        if ((FALSE == impl->config->skip_tx_auto_framing) && (impl->config->frame_start_word_len != 0u))
        {
            need_to_add_start_word = SID_STM32_UTIL_fast_memcmp(impl->config->frame_start_word, frame_to_send, impl->config->frame_start_word_len) != 0u ? TRUE : FALSE;
        }
        else
        {
            need_to_add_start_word = FALSE;
        }

        if ((FALSE == impl->config->skip_tx_auto_framing) && (impl->config->frame_end_word_len != 0u))
        {
            need_to_add_end_word = SID_STM32_UTIL_fast_memcmp(impl->config->frame_end_word, &frame_to_send[frame_size - impl->config->frame_end_word_len], impl->config->frame_end_word_len) != 0u ? TRUE : FALSE;
        }
        else
        {
            need_to_add_end_word = FALSE;
        }

        /* Compute the required space in the buffer, depending on the need to add start or stop sequences to the original frame payload */
        const uint32_t bytes_to_ingest = frame_size
                                            + (need_to_add_start_word != FALSE ? impl->config->frame_start_word_len : 0u)
                                            + (need_to_add_end_word != FALSE ? impl->config->frame_end_word_len : 0u);

        /* Check the full frame fits into MTU */
        if (bytes_to_ingest > impl->tx_mtu_size)
        {
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Calculate the available space in the buffer */
        uint32_t available_space = _prv_compute_data_length_in_ring_buffer(impl->tx_buf_ingest_ptr, impl->tx_buf_sendout_ptr, impl->tx_buffer, impl->config->tx_buffer_size);

        if ((available_space < bytes_to_ingest) || ((impl->config->tx_buffer_size == available_space) && (impl->tx_buf_ingest_wrap_flag != FALSE)))
        {
            /* Ring buffer currently doesn't have enough space to fit the frame, try later when some of the data is transferred and the buffer hasmore free space */
            err = SID_ERROR_BUSY;
            break;
        }

        uint32_t wrap_on_ingest = FALSE;

        /* Add frame start marker to the buffer if it's not part of the supplied frame */
        if (need_to_add_start_word != FALSE)
        {
            wrap_on_ingest |= _prv_ingest_into_ring_buffer(impl->config->frame_start_word, (uint32_t)impl->config->frame_start_word_len, impl->tx_buffer, impl->config->tx_buffer_size, &impl->tx_buf_ingest_ptr);
        }

        /* Ingest the supplied frame itself */
        wrap_on_ingest |= _prv_ingest_into_ring_buffer(frame_to_send, (uint32_t)frame_size, impl->tx_buffer, impl->config->tx_buffer_size, &impl->tx_buf_ingest_ptr);

        /* Add frame end marker to the buffer if it's not part of the supplied frame */
        if (need_to_add_end_word != FALSE)
        {
            wrap_on_ingest |= _prv_ingest_into_ring_buffer(impl->config->frame_end_word, (uint32_t)impl->config->frame_end_word_len, impl->tx_buffer, impl->config->tx_buffer_size, &impl->tx_buf_ingest_ptr);
        }

        /* Set Tx ring buffer ingestion position wrap flag if the wrap was indicated on any ingestion step above */
        if (wrap_on_ingest != FALSE)
        {
            impl->tx_buf_ingest_wrap_flag = TRUE;
        }

        /* Calculate the amount of bytes to be sent. This generally may differ from the bytes_to_ingest value because the ring buffer may contain some more bytes pending the Tx */
        const uint32_t bytes_to_send = (impl->tx_buf_sendout_ptr < impl->tx_buf_ingest_ptr) ?
                                          ((uint32_t)(void *)impl->tx_buf_ingest_ptr - (uint32_t)(void *)impl->tx_buf_sendout_ptr) :
                                          (impl->config->tx_buffer_size - ((uint32_t)(void *)impl->tx_buf_sendout_ptr - (uint32_t)(void *)impl->tx_buffer));

        /* Start transfer immediately if UART is free */
        if (HAL_UART_STATE_READY == impl->config->huart->gState)
        {
#ifdef HAL_DMA_MODULE_ENABLED
            if (impl->config->huart->hdmatx != NULL)
            {
                /* Start DMA Tx */
                hal_status = HAL_UART_Transmit_DMA(impl->config->huart, impl->tx_buf_sendout_ptr, bytes_to_send);
            }
            else
#endif /* HAL_DMA_MODULE_ENABLED */
            {
                /* Start IRQ-based Tx */
                hal_status = HAL_UART_Transmit_IT(impl->config->huart, impl->tx_buf_sendout_ptr, bytes_to_send);
            }
            if (hal_status != HAL_OK)
            {
                /* Roll back the ingestion pointer as we can't send the data due to UART failure */
                impl->tx_buf_ingest_ptr       = ingest_ptr_on_entry;
                impl->tx_buf_ingest_wrap_flag = ingest_wrap_flag_on_entry;

                SID_PAL_LOG_ERROR("Failed to start %s Tx. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }
        else
        {
            /* UART is already transmitting something, new data will handled in TxCpltCallback */
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ifc_get_frame(const sid_pal_serial_ifc_t * _this, uint8_t ** frame_received, size_t * frame_size)
{
    sid_error_t err;

    sid_pal_enter_critical_region();

    do
    {
        if ((NULL == _this) || (NULL == frame_received) || (NULL == frame_size))
        {
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        if ((NULL == impl->rx_buf_frame_start_ptr) || (NULL == impl->rx_buf_frame_end_ptr))
        {
            /* No data available */
            err = SID_ERROR_GENERIC;
            break;
        }

        /* Compute frame size */
        uint32_t bytes_available = _prv_compute_data_length_in_ring_buffer(impl->rx_buf_frame_start_ptr, impl->rx_buf_frame_end_ptr, impl->rx_buffer, impl->config->rx_buffer_size);

        if (impl->rx_frame_linear_storage != NULL)
        {
            free(impl->rx_frame_linear_storage);
        }

        /* Current implementation uses a ring buffer, but this method is expected to return a pointer to a contiguous (linear) buffer - allocate RAM for that and extract the data from the ring buffer */
        impl->rx_frame_linear_storage = malloc(bytes_available + 1u); /* Add one byte for string termination - this is a convenience feature when UART is used to transfer text data */
        if (NULL == impl->rx_frame_linear_storage)
        {
            /* No free heap space available */
            err = SID_ERROR_OOM;
            break;
        }

        if (impl->rx_buf_frame_start_ptr < impl->rx_buf_frame_end_ptr)
        {
            /* Frame data is contiguous in the Rx ring buffer */
            SID_STM32_UTIL_fast_memcpy(impl->rx_frame_linear_storage, impl->rx_buf_frame_start_ptr, bytes_available);
        }
        else
        {
            /* Frame data is wrapped in the Rx ring buffer, we have to copy it in two steps */
            const uint32_t bytes_before_wrap = (uint32_t)impl->config->rx_buffer_size - ((uint32_t)(void *)impl->rx_buf_frame_start_ptr - (uint32_t)(void *)impl->rx_buffer);
            SID_STM32_UTIL_fast_memcpy(impl->rx_frame_linear_storage, impl->rx_buf_frame_start_ptr, bytes_before_wrap);
            SID_STM32_UTIL_fast_memcpy(&impl->rx_frame_linear_storage[bytes_before_wrap], impl->rx_buffer, (bytes_available - bytes_before_wrap));

            /* This readout clears the wrap flag */
            if ((1u == impl->rx_buf_write_wrap_cnt)
#ifdef HAL_DMA_MODULE_ENABLED
              && ((NULL == impl->config->huart->hdmarx) || (__HAL_DMA_GET_FLAG(impl->config->huart->hdmarx , DMA_FLAG_TC) == 0U))
#endif /* HAL_DMA_MODULE_ENABLED */
            )
            {
                impl->rx_buf_write_wrap_cnt = 0u;
                SID_PAL_UART_LOG_DEBUG("WRAP cleared");
            }
            else
            {
                /* Buffer overflow, new data was written on top of read space - flush the buffer by resetting the read pointers and continue */
                SID_PAL_LOG_ERROR("%s Rx buffer overrun. Data loss is possible", _prv_get_uart_display_name(impl->config->hw_instance));
                impl->rx_buf_frame_start_ptr = NULL;
                impl->rx_buf_frame_end_ptr   = NULL;
                impl->rx_buf_write_wrap_cnt  = 0u;
                err = SID_ERROR_BUFFER_OVERFLOW;
                break;
            }
        }

        /* Add string terminator to the very end of the buffer (that's the extra byte reserved at allocation time, this does not modify the actually received data) */
        impl->rx_frame_linear_storage[bytes_available] = '\0';

        /* Populate output parameters */
        *frame_received = impl->rx_frame_linear_storage;
        *frame_size = (size_t)bytes_available;

        /* Reset frame state data after readout */
        impl->rx_buf_frame_start_ptr = NULL;
        impl->rx_buf_frame_end_ptr   = NULL;

        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ifc_get_mtu(const sid_pal_serial_ifc_t * _this, uint16_t * mtu)
{
    sid_error_t err;

    sid_pal_enter_critical_region();

    do
    {
        if ((NULL == _this) || (NULL == mtu))
        {
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        *mtu = impl->tx_mtu_size;
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

static void _uart_ifc_destroy(const sid_pal_serial_ifc_t * _this)
{
    HAL_StatusTypeDef hal_status;

    do
    {
        if (NULL == _this)
        {
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            break;
        }

        if ((impl->config != NULL) && ((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_RX == impl->config->mode)))
        {
            /* Stop the Rx inspection timer */
            (void)sid_pal_timer_cancel(&impl->rx_buf_inspection_timer);
        }

        /* Abort any ongoing Tx/Rx */
        hal_status = HAL_UART_Abort(impl->config->huart);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("%s: failed to abort ongoing Tx/Rx. Error 0x%02X", _prv_get_uart_display_name(impl->config->huart->Instance), hal_status);
            /* Proceed with de-initialization even if this step has failed */
        }

        /* Disable Autonomous mode */
        hal_status = HAL_UARTEx_DisableStopMode(impl->config->huart);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("%s: failed to disable autonomous mode. Error 0x%02X", _prv_get_uart_display_name(impl->config->huart->Instance), hal_status);
            /* Proceed with de-initialization even if this step has failed */
        }
        /* Disable Kernel Domain clock in Stop mode */
        switch ((uint32_t)(void *)(impl->config->huart->Instance))
        {
#ifdef LPUART1
            case (uint32_t)(void *)LPUART1:
                __HAL_RCC_LPUART1_CLK_SLEEP_DISABLE();
                break;
#endif /* LPUART1 */

#ifdef USART1
            case (uint32_t)(void *)USART1:
                __HAL_RCC_USART1_CLK_SLEEP_DISABLE();
                break;
#endif /* USART1 */

#ifdef USART2
            case (uint32_t)(void *)USART2:
                __HAL_RCC_USART2_CLK_SLEEP_DISABLE();
                break;
#endif /* USART2 */

#ifdef USART3
            case (uint32_t)(void *)USART3:
                __HAL_RCC_USART2_CLK_SLEEP_DISABLE();
                break;
#endif /* USART3 */

            default:
                SID_PAL_LOG_WARNING("Unknown UART instance: 0x%08X. Can't gate clock in LPM", impl->config->hw_instance);
                /* Proceed with de-initialization anyway */
                break;
        }

        /* De-initialize UART hardware */
        hal_status = HAL_UART_DeInit(impl->config->huart);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to de-initialize %s. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
            /* Proceed with de-initialization even if this step has failed */
        }

        sid_pal_enter_critical_region();

        /* Remove this client from the list of the clients served by this driver */
        _prv_remove_client_from_list(impl);

        /* Free the dynamically allocated memory */
        _prv_destroy_impl(impl);

        sid_pal_exit_critical_region();
    } while (0);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ext_ifc_start_rx(const sid_pal_uart_ext_ifc_t * _this)
{
    sid_error_t err;

    do
    {
        if (NULL == _this)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        /* Start Rx */
        err = _prv_start_continuous_rx(impl, FALSE);
        if (err != SID_ERROR_NONE)
        {
            /* Logs are provided by _prv_start_continuous_rx() */
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ext_ifc_stop_rx(const sid_pal_uart_ext_ifc_t * _this)
{
    sid_error_t err;

    do
    {
        if (NULL == _this)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        /* Stop Rx */
        err = _prv_stop_continuous_rx(impl);
        if (err != SID_ERROR_NONE)
        {
            /* Logs are provided by _prv_stop_continuous_rx() */
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ext_ifc_suspend_uart(const sid_pal_uart_ext_ifc_t * _this)
{
    sid_error_t       err;
    HAL_StatusTypeDef hal_status;

    do
    {
        if (NULL == _this)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        SID_PAL_ASSERT(impl->config        != NULL);
        SID_PAL_ASSERT(impl->config->huart != NULL);

        if (READ_BIT(impl->config->huart->Instance->CR1, USART_CR1_UE) == 0u)
        {
            /* UART is disabled already */
            err = SID_ERROR_NONE;
            break;
        }

        /* For Rx-capble UART we need to stop not onl;y the perihperal, but associated periodic Rx buffer inspection routine */
        if ((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_RX == impl->config->mode))
        {
            err = _prv_stop_continuous_rx(impl);
            if (err != SID_ERROR_NONE)
            {
                /* Logs are provided by _prv_stop_continuous_rx() */
                break;
            }
        }

        hal_status = HAL_UART_Abort(impl->config->huart);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to abort %s comm. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Disable UART so it stops operating and won't produce clock requests to RCC */
        __HAL_UART_DISABLE(impl->config->huart);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ext_ifc_resume_uart(const sid_pal_uart_ext_ifc_t * _this)
{
    sid_error_t       err;
    HAL_StatusTypeDef hal_status;

    do
    {
        if (NULL == _this)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        SID_PAL_ASSERT(impl->config        != NULL);
        SID_PAL_ASSERT(impl->config->huart != NULL);

        if (READ_BIT(impl->config->huart->Instance->CR1, USART_CR1_UE) != 0u)
        {
            /* UART is enabled already */
            err = SID_ERROR_NONE;
            break;
        }

        /* Re-enable UART operation */
        __HAL_UART_ENABLE(impl->config->huart);

        /* Wait for TEACK and/or REACK flags indicate UART readiness */
        hal_status = UART_CheckIdleState(impl->config->huart);
        if (hal_status != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to abort %s comm. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ext_ifc_get_current_baud_rate(const sid_pal_uart_ext_ifc_t * _this, uint32_t * const out_baud_rate)
{
    sid_error_t err;

    sid_pal_enter_critical_region();

    do
    {
        if ((NULL == _this) || (NULL == out_baud_rate))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        *out_baud_rate = impl->config->huart->Init.BaudRate;

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ext_ifc_get_default_baud_rate(const sid_pal_uart_ext_ifc_t * _this, uint32_t * const out_baud_rate)
{
    sid_error_t err;

    sid_pal_enter_critical_region();

    do
    {
        if ((NULL == _this) || (NULL == out_baud_rate))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        *out_baud_rate = impl->config->baud_rate;

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _uart_ext_ifc_set_baud_rate(const sid_pal_uart_ext_ifc_t * _this, const uint32_t new_baud_rate)
{
    sid_error_t       err;
    HAL_StatusTypeDef hal_status;

    sid_pal_enter_critical_region();

    do
    {
        if ((NULL == _this) || (IS_UART_BAUDRATE(new_baud_rate) == FALSE))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Locate the full instance object in RAM */
        uart_serial_client_impl_t * const impl = SID_PAL_UART_GET_IMPL_FROM_IFC(_this);

        if (_prv_client_in_list(impl) == FALSE)
        {
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by unknown client (0x%08X)", (uint32_t)(void *)_this);
            err = SID_ERROR_PORT_INVALID;
            break;
        }

        UART_HandleTypeDef * const huart = impl->config->huart;

        /* Ensure the UART is not actively transmitting or receiving anything */
        if ((huart->gState != HAL_UART_STATE_READY) || ((impl->config->mode != SID_PAL_UART_MODE_TX) && (huart->RxState != HAL_UART_STATE_READY)))
        {
            err = (HAL_UART_STATE_RESET == huart->gState) ? SID_ERROR_UNINITIALIZED : SID_ERROR_BUSY;
            break;
        }

        /* Select clock prescaler based on the targeted baud rate and oversampling mode */
        huart->Init.BaudRate       = new_baud_rate;
        huart->Init.ClockPrescaler = _prv_calc_clock_prescaler(_prv_get_kernel_clock_source_freq(impl->config), new_baud_rate, huart->Init.OverSampling);

        /* Ensure UE bit in CR1 is cleared before proceeding */
        __HAL_UART_DISABLE(huart);

        /* Apply new configuration */
        hal_status = UART_SetConfig(huart);
        if (hal_status != HAL_OK)
        {
            /* Indicate error since UART state is now undefined, requiring partial or full re-initialization */
            huart->gState  = HAL_UART_STATE_ERROR;

            SID_PAL_LOG_ERROR("Failed to reconfigure baud rate for %s. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Re-apply FIFO configuration since it may have been  */
        err = _prv_uart_apply_fifo_config(impl->config);
        if (err != SID_ERROR_NONE)
        {
            /* Indicate error since UART state is now undefined, requiring partial or full re-initialization */
            huart->gState  = HAL_UART_STATE_ERROR;
            break;
        }

        /* Re-enable UART */
        __HAL_UART_ENABLE(huart);

        /* Wait for TEACK and/or REACK before proceeding */
        hal_status = UART_CheckIdleState(huart);
        if (hal_status != HAL_OK)
        {
            /* Indicate error since UART state is now undefined, requiring partial or full re-initialization */
            huart->gState  = HAL_UART_STATE_ERROR;

            SID_PAL_LOG_ERROR("Failed to reconfigure baud rate for %s. HAL error 0x%02X", _prv_get_uart_display_name(impl->config->hw_instance), hal_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Display baud rate update message*/
        SID_PAL_LOG_INFO("%s baud rate changed to %ubps", _prv_get_uart_display_name(impl->config->hw_instance), new_baud_rate);

        /* Recalculate Rx inspection period based on the new baud rate */
        _prv_calc_rx_inspection_period(impl);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _uart_client_create_common(sid_pal_uart_ext_ifc_t const ** _this, const void * config, const sid_pal_serial_params_t * params)
{
    sid_error_t                              err;
    const struct sid_pal_uart_config * const uart_cfg = config;
    uart_serial_client_impl_t *              impl     = NULL;

    do
    {
        /* Validate inputs */
        if ((NULL == uart_cfg) || (NULL == params) || (NULL == params->callbacks))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if ((NULL == uart_cfg->huart) || (NULL == uart_cfg->hw_instance))
        {
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        /* Ensure that frame start/end words are aligned with data bits (at least one byte in 7 and 8 bit mode, and at least two bytes in 9-bit mode) */
        if (((SID_PAL_UART_MODE_TX_RX == uart_cfg->mode) || (SID_PAL_UART_MODE_RX == uart_cfg->mode)) /* This check is only relevant if Rx is expected for the current UART */
            && (
                    ((uart_cfg->frame_start_word != NULL) && (uart_cfg->frame_start_word_len <= ((uart_cfg->data_bits - 1u) / 8u)))
                 || ((uart_cfg->frame_end_word != NULL) && (uart_cfg->frame_end_word_len <= ((uart_cfg->data_bits - 1u) / 8u)))
                ))
        {
            SID_PAL_LOG_ERROR("Invalid %s config: mismatch between data size and start/end pattern size", _prv_get_uart_display_name(uart_cfg->hw_instance));
            err = SID_ERROR_INCOMPATIBLE_PARAMS;
            break;
        }

        /* Low-level UART init */
        err = _prv_uart_hw_init(uart_cfg);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Allocate memory for the UART client instance */
        uart_serial_client_impl_t * impl = calloc(1u, sizeof(*impl));
        if (NULL == impl)
        {
            err = SID_ERROR_OOM;
            break;
        }

        /* Set instance configuration */
        impl->callbacks = params->callbacks;
        impl->cb_ctx    = params->user_ctx;
        impl->config    = uart_cfg;

        /* Allocate memory for UART Rx buffer (if used) */
        if ((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_RX == impl->config->mode))
        {
            /* Allocate RAM for Rx buffer */
            if (0u == uart_cfg->rx_buffer_size)
            {
                SID_PAL_LOG_ERROR("Invalid %s config: Rx is enabled, but Rx buffer size is set to zero", _prv_get_uart_display_name(uart_cfg->hw_instance));
                err = SID_ERROR_INCOMPATIBLE_PARAMS;
                break;
            }

            impl->rx_buffer = malloc(uart_cfg->rx_buffer_size);
            if (NULL == impl->rx_buffer)
            {
                SID_PAL_LOG_ERROR("Failed to create %s Rx buffer. Out of memory", _prv_get_uart_display_name(uart_cfg->hw_instance));
                err = SID_ERROR_OOM;
                break;
            }

            /* Create timer to periodically check received data for valid frames */
            err = sid_pal_timer_init(&impl->rx_buf_inspection_timer, _prv_rx_inspection_event_handler, impl);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to create timer for %s Rx processing. Error %d", _prv_get_uart_display_name(uart_cfg->hw_instance), (int32_t)err);
                break;
            }

            /* Calculate inspection period based on baud rate, Rx buffer size, and other user configuration */
            _prv_calc_rx_inspection_period(impl);
        }

        /* Allocate memory for UART Tx buffer (if used) */
        if ((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_TX == impl->config->mode))
        {
            if (0u == uart_cfg->tx_buffer_size)
            {
                SID_PAL_LOG_ERROR("Invalid %s config: Tx is enabled, but Tx buffer size is set to zero", _prv_get_uart_display_name(uart_cfg->hw_instance));
                err = SID_ERROR_INCOMPATIBLE_PARAMS;
                break;
            }

            impl->tx_buffer = malloc(uart_cfg->tx_buffer_size);
            if (NULL == impl->tx_buffer)
            {
                SID_PAL_LOG_ERROR("Failed to create %s Tx buffer. Out of memory", _prv_get_uart_display_name(uart_cfg->hw_instance));
                err = SID_ERROR_OOM;
                break;
            }

            /* Initialize Tx buffer pointers */
            impl->tx_buf_ingest_ptr  = impl->tx_buffer;
            impl->tx_buf_sendout_ptr = impl->tx_buffer;

            /* Compute Tx MTU size */
            impl->tx_mtu_size = uart_cfg->tx_buffer_size;
            if (uart_cfg->frame_start_word != NULL)
            {
                SID_PAL_ASSERT(impl->tx_mtu_size > uart_cfg->frame_start_word_len);
                impl->tx_mtu_size -= uart_cfg->frame_start_word_len;
            }
            if (uart_cfg->frame_end_word != NULL)
            {
                SID_PAL_ASSERT(impl->tx_mtu_size > uart_cfg->frame_end_word_len);
                impl->tx_mtu_size -= uart_cfg->frame_end_word_len;
            }
        }

        /* Associate API methods with current instance */
        impl->public_ifc = &uart_client_ifc_methods;

        /* Set interface pointer */
        *_this = &impl->public_ifc;

        /* Add the client to the list of clients before the actual Tx/Rx starts */
        LST_insert_tail(&uart_client_list, &impl->node);

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /* Configure UART event callbacks */
        impl->config->huart->ErrorCallback = _prv_on_uart_xfer_error;
        if ((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_TX == impl->config->mode))
        {
            impl->config->huart->TxCpltCallback = _prv_on_uart_tx_completed;
        }
        else
        {
            impl->config->huart->TxCpltCallback = NULL;
        }
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if (err != SID_ERROR_NONE)
    {
        /* Release any dynamically allocated resources */
        if (impl != NULL)
        {
            _prv_destroy_impl(impl);
        }

        /* Ensure UART peripheral is disabled */
        if ((uart_cfg != NULL) && (uart_cfg->huart != NULL) && (uart_cfg->hw_instance == uart_cfg->huart->Instance))
        {
            (void)HAL_UART_DeInit(uart_cfg->huart);
        }
    }

    return err;
}

/* Global function definitions -----------------------------------------------*/

sid_error_t sid_pal_uart_client_create(sid_pal_serial_ifc_t const ** _this, const void * config, const sid_pal_serial_params_t * params)
{
    sid_error_t                 err;
    uart_serial_client_impl_t * impl = NULL;

    do
    {
        /* Validate inputs */
        if (NULL == _this)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Run common UART initialization */
        err = _uart_client_create_common((sid_pal_uart_ext_ifc_t const **)_this, config, params);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _uart_client_create_common() */
            break;
        }

        impl = SID_PAL_UART_GET_IMPL_FROM_IFC(*_this);

        /* Start UART communication automatically since Sidewalk serial client API has no methods to control Rx */
        if ((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_RX == impl->config->mode))
        {
            err = _prv_start_continuous_rx(impl, FALSE);
            if (err != SID_ERROR_NONE)
            {
                /* Logs are provided by _prv_start_continuous_rx() */
                break;
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if ((err != SID_ERROR_NONE) && (impl != NULL))
    {
        _prv_destroy_impl(impl);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

sid_error_t sid_pal_uart_client_create_ext(sid_pal_uart_ext_ifc_t const ** _this, const void * config, const sid_pal_serial_params_t * params)
{
    sid_error_t                 err;
    uart_serial_client_impl_t * impl = NULL;

    do
    {
        /* Validate inputs */
        if (NULL == _this)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Run common UART initialization */
        err = _uart_client_create_common((sid_pal_uart_ext_ifc_t const **)_this, config, params);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _uart_client_create_common() */
            break;
        }

        impl = SID_PAL_UART_GET_IMPL_FROM_IFC(*_this);

        /* Do not start Rx automatically since the extended API provides methods to control Rx */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if ((err != SID_ERROR_NONE) && (impl != NULL))
    {
        _prv_destroy_impl(impl);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_pal_uart_irq_handler(UART_HandleTypeDef * huart)
{
    do
    {
        /* Search for the associated UART client instance */
        uart_serial_client_impl_t * const impl = _prv_find_client_by_uart(huart);

        if (NULL == impl)
        {
            /* Normally should never happen */
            SID_PAL_LOG_WARNING("Sidewalk UART driver triggered by non-related UART (%s)", _prv_get_uart_display_name(huart->Instance));
            break;
        }

        /* Use IDLE and Character Match events as an indication for triggering UART Rx buffer processing without waiting for a regular scheduled inspection - this helps to minimize the delay between actual Rx end and invocation of Rx Done callback */
        if ((SID_PAL_UART_MODE_TX_RX == impl->config->mode) || (SID_PAL_UART_MODE_RX == impl->config->mode))
        {
            const uint32_t isrflags   = READ_REG(huart->Instance->ISR);
            const uint32_t cr1its     = READ_REG(huart->Instance->CR1);
            const uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));

            /* If no error occurs */
            if (errorflags == 0u)
            {
                uint32_t need_rx_inspection = FALSE;

                /*  IDLE event is indicated */
                if (((isrflags & USART_ISR_IDLE) != 0u) && ((cr1its & USART_CR1_IDLEIE) != 0u))
                {
                    /* Clear IDLE event flag */
                    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
                    need_rx_inspection = TRUE;
                }

                /*  Character Match event is indicated */
                if (((isrflags & USART_ISR_CMF) != 0u) && ((cr1its & USART_CR1_CMIE) != 0u))
                {
                    /* Clear Character Match event flag */
                    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_CMF);
                    need_rx_inspection = TRUE;
                }

                if (need_rx_inspection != FALSE)
                {
                    /* Schedule Rx buffer inspection to ASAP */
                (void)_prv_schedule_rx_inspection(impl, 0u);
                }
            }
        }

        /* Call regular ISR handler to drive HAL layer */
        HAL_UART_IRQHandler(huart);
    } while (0);
}

/*----------------------------------------------------------------------------*/

#if (USE_HAL_UART_REGISTER_CALLBACKS == 0)
#  error "This module cannot run without UART callbacks enabled"
// TODO: implement unified HAL wrappers for this configuration
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
