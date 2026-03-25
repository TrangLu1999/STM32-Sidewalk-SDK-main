/**
  ******************************************************************************
  * @file    app_sidewalk.c
  * @brief   Sidewalk SubGHz link sample app implementation
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
#include <stdint.h>
#include <string.h>

#include "app_freertos.h"

#include <sid_api.h>
#include <sid_hal_reset_ifc.h>
#include <sid_log_control.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_common_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_900_cfg.h>
#include <sid_stm32_common_utils.h>

#include SID_APP_VERSION_HEADER
#include <sid_sdk_version.h>

#include "target/memory.h"

#include "app_ble_config.h"
#include "app_900_config.h"
#include "app_common.h"
#include <stm32_mcu_info.h>
#if CFG_LED_SUPPORTED
#  include "led_indication.h"
#endif /* CFG_LED_SUPPORTED */
#include "sid_pal_gpio_ext_ifc.h"
#include <stm32wlxx_app_radio_ext_ifc.h>
#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined(NUCLEO_WBA65_BOARD)
#  include "stm32wbaxx_nucleo.h"
#endif

#if (CFG_LPM_LEVEL != 0)
#  include "stm32_lpm.h"
#endif /* (CFG_LPM_LEVEL != 0) */
#if (CFG_BUTTON_SUPPORTED != 0) && (CFG_LPM_STDBY_SUPPORTED != 0)
#  include <sid_pal_gpio_ext_ifc.h>
#endif /* (CFG_BUTTON_SUPPORTED != 0) && (CFG_LPM_STDBY_SUPPORTED != 0) */

/* Private defines -----------------------------------------------------------*/

#define SID_MSG_QUEUE_LEN                           (10u)

#define DEMO_MESSAGE_DELAY_MS                       (60000u)

#define BUTTON_LONG_PRESS_MS                        (1500u)
#define BUTTON_TIMESTAMP_INVALID_VAL                (0u)

/* Select Sidewalk link type for communication - LoRa or FSK */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 && !SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
#  define SID_COMM_LINK_TYPE                        SID_LINK_TYPE_2 /* FSK link */
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
#  define SID_COMM_LINK_TYPE                        SID_LINK_TYPE_3 /* LoRa link */
#else
#  error "Invalid configuration. This sample app is designed to use either FSK or LoRa Sidewalk links for normal operation"
#endif

/* Select Sidewalk link type for initial device registration - BLE or FSK */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_1
#  define SID_REGISTRATION_LINK_TYPE                SID_LINK_TYPE_1 /* BLE link */
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 && !SID_SDK_CONFIG_ENABLE_LINK_TYPE_1
#  define SID_REGISTRATION_LINK_TYPE                SID_LINK_TYPE_2 /* FSK link */
#else
#  error "Sidewalk device registration is only possible via BLE or FSK link, but none of them is enabled in the configuration"
#endif

#if !SID_SDK_CONFIG_ENABLE_LINK_TYPE_1 && SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 && !SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
#  define SID_FSK_ONLY_LINK                         (1)
#else
#  define SID_FSK_ONLY_LINK                         (0)
#endif

/* Private macro -----------------------------------------------------------*/

#define OS_MS_TO_TICKS( xTimeInMs )                 ( ( uint32_t ) ( ( ( uint32_t ) ( xTimeInMs ) * ( uint32_t ) osKernelGetTickFreq() ) / ( uint32_t ) 1000U ) )

/* UART2 Command Protocol from STM32G0B1 */
#define UART2_CMD_INIT_LORA    0x01
#define UART2_CMD_SEND_DATA    0x02
#define UART2_CMD_STOP_LINK    0x03
#define UART2_CMD_GET_STATUS   0x04
#define UART2_CMD_SET_BAUDRATE 0x05

/* BLE Status GPIO (PB1) - output to G0B1 */
#define BLE_STATUS_GPIO_PORT   GPIOB
#define BLE_STATUS_GPIO_PIN    GPIO_PIN_1

/* UART2 Response codes (WBA55 → G0B1) */
#define UART2_RSP_OK           0xA0
#define UART2_RSP_ERR          0xA1
#define UART2_RSP_BUSY         0xA2

/* Private typedef -----------------------------------------------------------*/

enum event_type
{
    EVENT_TYPE_COLD_START_LINK,
    EVENT_TYPE_HOT_START_LINK,
    EVENT_TYPE_STOP_LINK,
    EVENT_TYPE_TOGGLE_LINK,
    EVENT_TYPE_SIDEWALK,
    EVENT_TYPE_SEND_HELLO,
    EVENT_TYPE_UART2_DATA,
    EVENT_FACTORY_RESET,
    EVENT_TYPE_SET_DEVICE_PROFILE,
    EVENT_TYPE_REGISTRATION_COMPLETED,
#if (CFG_LPM_STDBY_SUPPORTED != 0)
    EVENT_TYPE_ACTIVE_STANDBY,
    EVENT_TYPE_FULL_STANDBY,
#endif /* CFG_LPM_STDBY_SUPPORTED */
};

enum app_state
{
    STATE_INIT,
    STATE_SIDEWALK_READY,
    STATE_SIDEWALK_NOT_READY,
    STATE_SIDEWALK_SECURE_CONNECTION,
};

struct link_status
{
    uint32_t link_mask;
    uint32_t supported_link_mode[SID_LINK_TYPE_MAX_IDX];
};

typedef struct app_context
{
    osThreadId_t main_task;
    osMessageQueueId_t event_queue;
    struct sid_handle *sidewalk_handle;
    enum app_state state;
    struct link_status link_status;
    uint8_t counter;
} app_context_t;

enum sid_subghz_profile_idx {
    SID_LORA_PROFILE_A         = 0,
    SID_LORA_PROFILE_B         = 1,
    SID_FSK_PROFILE_1          = 2,
    SID_FSK_PROFILE_2          = 3,
    SID_SUBGHZ_PROFILE_UNKNOWN = 4,
    PROFILE_LAST               = SID_SUBGHZ_PROFILE_UNKNOWN,
};

typedef __PACKED_UNION {
    __PACKED_STRUCT {
        uint8_t build;
        uint8_t patch;
        uint8_t minor;
        uint8_t major;
    };
    uint32_t raw;
} hal_version_info_t;

/* Private variables ---------------------------------------------------------*/

static osMessageQueueId_t g_event_queue;

static osSemaphoreId_t registration_ready_semaphore;
static osSemaphoreId_t connection_ready_semaphore;
static osThreadId_t demo_counter_task;

/* Indicates if Sidewalk registration process is pending */
static bool registration_pending = false;

/* UART2 data buffer for forwarding to Sidewalk */
static uint8_t uart2_tx_buffer[20];
static uint8_t uart2_tx_len;

/* UART2 handle for sending responses to G0B1 */
extern UART_HandleTypeDef huart2;

/* Private constants ---------------------------------------------------------*/

static const osThreadAttr_t sidewalk_stack_task_attributes = {
    .name         = "Sidewalk Stack Task",
    .priority     = SID_MAIN_TASK_PRIO,
    .stack_size   = SID_MAIN_TASK_STACK_SIZE,
    .attr_bits    = TASK_DEFAULT_ATTR_BITS,
    .cb_mem       = TASK_DEFAULT_CB_MEM,
    .cb_size      = TASK_DEFAULT_CB_SIZE,
    .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

static const osSemaphoreAttr_t registration_ready_sem_attributes = {
    .name       = "Sidewalk Registration Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t connection_ready_sem_attributes = {
    .name       = "Connection Ready Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osThreadAttr_t demo_counter_task_attributes = {
    .name         = "Demo Counter Task",
    .priority     = DEMO_COUNTER_TASK_PRIO,
    .stack_size   = DEMO_COUNTER_TASK_STACK_SIZE,
    .attr_bits    = TASK_DEFAULT_ATTR_BITS,
    .cb_mem       = TASK_DEFAULT_CB_MEM,
    .cb_size      = TASK_DEFAULT_CB_SIZE,
    .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

#if SID_PAL_LOG_ENABLED
static const char * const sid_subghz_profile_strings[] = {
    [SID_LORA_PROFILE_A]         = "LoRa Profile A",
    [SID_LORA_PROFILE_B]         = "LoRa Profile B",
    [SID_FSK_PROFILE_1]          = "FSK Profile 1",
    [SID_FSK_PROFILE_2]          = "FSK Profile 2",
    [SID_SUBGHZ_PROFILE_UNKNOWN] = "Unknown",
};

static const char * const sid_subghz_wakeup_type_strings[] = {
    [SID_NO_WAKEUP]        = "No Wakeup",
    [SID_TX_ONLY_WAKEUP]   = "Tx Only",
    [SID_RX_ONLY_WAKEUP]   = "Rx Only",
    [SID_TX_AND_RX_WAKEUP] = "Tx & Rx",
};
#endif /* SID_PAL_LOG_ENABLED */

/* Private function prototypes -----------------------------------------------*/

static void         queue_event(osMessageQueueId_t queue, enum event_type event);
static void         on_sidewalk_event(bool in_isr, void *context);
static void         on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context);
static void         on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context);
static void         on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context);
static void         on_sidewalk_status_changed(const struct sid_status *status, void *context);
static void         on_sidewalk_factory_reset(void *context);
static void         send_ping(app_context_t *app_context);
static void         factory_reset(app_context_t * const context);
static void         print_subghz_link_profile(const app_context_t * const context);
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
static void         adjust_sidewalk_link_log_level(const uint32_t link_mask);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
static sid_error_t  init_and_start_link(app_context_t * const context, struct sid_config * const config, const bool cold_start);

static void         sidewalk_stack_task_entry(void *context);
static void         demo_counter_task_entry(void *context);
#if SID_PAL_LOG_ENABLED
static const char * sid_subghz_profile_code_to_str(enum sid_device_profile_id code);
#endif /* SID_PAL_LOG_ENABLED */
#if (CFG_BUTTON_SUPPORTED == 1)
#  if (defined(STM32WBA6x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
static void         button1_irq_handler(uint32_t pin, void * callback_arg);
#  endif /* (STM32WBA6x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA6x */
#  if (defined(STM32WBA5x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
static void         button2_irq_handler(uint32_t pin, void * callback_arg);
#  endif /* (STM32WBA5x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA5x */
static void         button3_irq_handler(uint32_t pin, void * callback_arg);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
static void         button3_wakeup_handler(uint32_t pin, void * callback_arg);
#  endif /* (CFG_LPM_STDBY_SUPPORTED != 0) */
static void         buttons_init(void);
#endif /* (CFG_BUTTON_SUPPORTED == 1) */

/* Private function definitions ----------------------------------------------*/

static void queue_event(osMessageQueueId_t queue, enum event_type event)
{
    osMessageQueuePut(queue, &event, 0u, 0u);
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_event(bool in_isr, void *context)
{
    (void)in_isr;

    app_context_t *app_context = (app_context_t *)context;
    queue_event(app_context->event_queue, EVENT_TYPE_SIDEWALK);
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context)
{
    SID_PAL_LOG_INFO("Received message(type: %d, link_mode: %d, id: %u size %u)", (int)msg_desc->type,
                             (int)msg_desc->link_mode, msg_desc->id, msg->size);
    SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, msg->data, msg->size);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_RCV_OK);
#endif /* CFG_LED_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_INFO("Sent message(type: %d, id: %u)", (int)msg_desc->type, msg_desc->id);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SENT_OK);
#endif /* CFG_LED_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_ERROR("Failed to send message(type: %d, id: %u), err:%d",
                  (int)msg_desc->type, msg_desc->id, (int)error);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SEND_ERROR);
#endif /* CFG_LED_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_status_changed(const struct sid_status *status, void *context)
{
    app_context_t *app_context = (app_context_t *)context;
    SID_PAL_LOG_INFO("Sidewalk status changed: %d", (int)status->state);
    switch (status->state)
    {
        case SID_STATE_READY:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_CONNECTED);
#endif /* CFG_LED_SUPPORTED */
            if (false == registration_pending)
            {
                /* Notify the user app that Sidewalk link can now be used */
                app_context->state = STATE_SIDEWALK_READY;
                osSemaphoreRelease(connection_ready_semaphore);
            }
            break;

        case SID_STATE_NOT_READY:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_BONDING);
#endif /* CFG_LED_SUPPORTED */
            app_context->state = STATE_SIDEWALK_NOT_READY;
            break;

        case SID_STATE_ERROR:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_ERROR);
#endif /* CFG_LED_SUPPORTED */
            SID_PAL_LOG_ERROR("Sidewalk error: %d", (int)sid_get_error(app_context->sidewalk_handle));
            SID_PAL_ASSERT(0);
            break;

        case SID_STATE_SECURE_CHANNEL_READY:
            app_context->state = STATE_SIDEWALK_SECURE_CONNECTION;
            break;

        default:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_ERROR);
#endif /* CFG_LED_SUPPORTED */
            SID_PAL_LOG_ERROR("Unknown Sidewalk state received: %d", (int)status->state);
            SID_PAL_ASSERT(0);
            break;
    }

    SID_PAL_LOG_INFO("Registration Status = %d, Time Sync Status = %d and Link Status Mask = %x",
                 status->detail.registration_status, status->detail.time_sync_status,
                 status->detail.link_status_mask);

    if ((registration_pending != false) && (SID_STATUS_REGISTERED == status->detail.registration_status))
    {
        /* Device registration completed, restart Sidewalk link to switch to 900MHz radio */
        SID_PAL_LOG_INFO("Device registration done, restarting Sidewalk link");
        registration_pending = false;
        queue_event(g_event_queue, EVENT_TYPE_REGISTRATION_COMPLETED);
    }

    app_context->link_status.link_mask = status->detail.link_status_mask;
    for (uint32_t i = 0u; i < SID_LINK_TYPE_MAX_IDX; i++)
    {
        app_context->link_status.supported_link_mode[i] = status->detail.supported_link_modes[i];
        SID_PAL_LOG_INFO("Link %d Mode %x", i, status->detail.supported_link_modes[i]);
    }
    if ((status->detail.link_status_mask & (SID_LINK_TYPE_2 | SID_LINK_TYPE_3)) != 0u)
    {
        print_subghz_link_profile(context);
    }
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_factory_reset(void *context)
{
    (void)context;

    SID_PAL_LOG_INFO("Factory reset notification received from sid api");
    (void)sid_hal_reset(SID_HAL_RESET_NORMAL);
}

/*----------------------------------------------------------------------------*/

static void send_ping(app_context_t *app_context)
{
    SID_PAL_LOG_INFO("Sending ping...");
    if (app_context->state == STATE_SIDEWALK_READY ||
        app_context->state == STATE_SIDEWALK_SECURE_CONNECTION)
    {
        SID_PAL_LOG_INFO("Sending counter update: %d", app_context->counter);
        struct sid_msg msg = {.data = (uint8_t*)&app_context->counter, .size = sizeof(uint8_t)};
        struct sid_msg_desc desc = {
            .type = SID_MSG_TYPE_NOTIFY,
            .link_type = SID_LINK_TYPE_ANY,
            .link_mode = SID_LINK_MODE_CLOUD,
        };

        if ((app_context->link_status.link_mask & SID_LINK_TYPE_1) &&
            (app_context->link_status.supported_link_mode[SID_LINK_TYPE_1_IDX] & SID_LINK_MODE_MOBILE))
        {
            desc.link_mode = SID_LINK_MODE_MOBILE;
        }

        sid_error_t ret = sid_put_msg(app_context->sidewalk_handle, &msg, &desc);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed queueing data, err:%d", (int) ret);
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_SEND_ERROR);
#endif /* CFG_LED_SUPPORTED */
        }
        else
        {
            SID_PAL_LOG_DEBUG("Queued data message id:%u", desc.id);
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_SEND_ENQUEUED);
#endif /* CFG_LED_SUPPORTED */
        }
        app_context->counter++;
    }
    else
    {
        SID_PAL_LOG_WARNING("Send_ping: Sidewalk is not ready yet!");
#if CFG_LED_SUPPORTED
        (void)led_indication_set(LED_INDICATE_SEND_ERROR);
#endif /* CFG_LED_SUPPORTED */
    }
}

static sid_error_t send_uart2_data(app_context_t *app_context)
{
    (void)app_context;
    SID_PAL_LOG_INFO("Sending UART2 data via UDT (%d bytes)", uart2_tx_len);
    SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, uart2_tx_buffer, uart2_tx_len);

    sid_error_t ret = sid_pal_radio_stm32wlxx_send_user_data(uart2_tx_buffer, uart2_tx_len, 0);
    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("UDT send failed, err:%d", (int)ret);
    }
    else
    {
        SID_PAL_LOG_INFO("UDT data sent to WL55");
    }
    return ret;
}

/**
 * @brief Send response to G0B1 via UART2
 * @param cmd   Original command ID
 * @param status Response code (UART2_RSP_OK, UART2_RSP_ERR, UART2_RSP_BUSY)
 * @param data  Optional payload (can be NULL)
 * @param len   Payload length
 */
static void uart2_send_response(uint8_t cmd, uint8_t status, const uint8_t *data, uint8_t len)
{
    uint8_t rsp[20] = {0};
    rsp[0] = status;
    rsp[1] = cmd;
    rsp[2] = len;
    if ((data != NULL) && (len > 0) && (len <= 17))
    {
        memcpy(&rsp[3], data, len);
    }
    HAL_UART_Transmit(&huart2, rsp, sizeof(rsp), 50);
}

static void parse_uart2_command(app_context_t *app_context)
{
    if (uart2_tx_len < 1) return;

    uint8_t cmd = uart2_tx_buffer[0];
    uint8_t payload_len = (uart2_tx_len > 1) ? uart2_tx_buffer[1] : 0;

    SID_PAL_LOG_INFO("UART2 CMD: 0x%02X, payload_len: %d", cmd, payload_len);

    switch (cmd)
    {
        case UART2_CMD_INIT_LORA:
            SID_PAL_LOG_INFO("CMD: INIT_LORA received from G0B1");
            if (app_context->sidewalk_handle == NULL)
            {
                queue_event(g_event_queue, EVENT_TYPE_COLD_START_LINK);
                uart2_send_response(cmd, UART2_RSP_OK, NULL, 0);
            }
            else
            {
                SID_PAL_LOG_WARNING("Sidewalk already initialized, ignoring INIT_LORA");
                uart2_send_response(cmd, UART2_RSP_BUSY, NULL, 0);
            }
            break;

        case UART2_CMD_SEND_DATA:
        {
            SID_PAL_LOG_INFO("CMD: SEND_DATA (%d bytes) from G0B1", payload_len);
            /* Shift payload: skip cmd + len bytes */
            memmove(uart2_tx_buffer, &uart2_tx_buffer[2], payload_len);
            uart2_tx_len = payload_len;
            sid_error_t send_ret = send_uart2_data(app_context);
            uart2_send_response(cmd, (send_ret == SID_ERROR_NONE) ? UART2_RSP_OK : UART2_RSP_ERR, NULL, 0);
            break;
        }

        case UART2_CMD_STOP_LINK:
            SID_PAL_LOG_INFO("CMD: STOP_LINK received from G0B1");
            queue_event(g_event_queue, EVENT_TYPE_STOP_LINK);
            uart2_send_response(cmd, UART2_RSP_OK, NULL, 0);
            break;

        case UART2_CMD_GET_STATUS:
        {
            SID_PAL_LOG_INFO("CMD: GET_STATUS - state=%d, handle=%s",
                app_context->state,
                (app_context->sidewalk_handle != NULL) ? "active" : "null");
            uint8_t status_data[2] = {
                (uint8_t)app_context->state,
                (app_context->sidewalk_handle != NULL) ? 1 : 0
            };
            uart2_send_response(cmd, UART2_RSP_OK, status_data, 2);
            break;
        }

        case UART2_CMD_SET_BAUDRATE:
        {
            if (payload_len != 4)
            {
                SID_PAL_LOG_ERROR("CMD: SET_BAUDRATE invalid payload_len=%d (need 4)", payload_len);
                uart2_send_response(cmd, UART2_RSP_ERR, NULL, 0);
                break;
            }
            uint32_t new_baud = (uint32_t)uart2_tx_buffer[2]
                              | ((uint32_t)uart2_tx_buffer[3] << 8)
                              | ((uint32_t)uart2_tx_buffer[4] << 16)
                              | ((uint32_t)uart2_tx_buffer[5] << 24);
            SID_PAL_LOG_INFO("CMD: SET_BAUDRATE to %lu", new_baud);

            /* Send OK response at current baud rate first */
            uart2_send_response(cmd, UART2_RSP_OK, (const uint8_t *)&new_baud, 4);

            /* Wait for TX to complete before switching baud */
            osDelay(10);

            /* Disable DMA channel */
            LL_DMA_DisableChannel(GPDMA1, LL_DMA_CHANNEL_3);
            LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_3);
            LL_DMA_ClearFlag_DTE(GPDMA1, LL_DMA_CHANNEL_3);

            /* Reinit UART with new baud rate */
            HAL_UART_DeInit(&huart2);
            huart2.Init.BaudRate = new_baud;
            HAL_UART_Init(&huart2);
            HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8);
            HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8);
            HAL_UARTEx_EnableFifoMode(&huart2);
            __HAL_UART_ENABLE(&huart2);

            /* Restart DMA reception */
            DMA_InitStruct.SrcAddress = LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_RECEIVE);
            DMA_InitStruct.DestAddress = (uint32_t)aUSART2RxBuffer;
            LL_DMA_Init(GPDMA1, LL_DMA_CHANNEL_3, &DMA_InitStruct);
            LL_DMA_SetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_3, ubUSART2NbDataToReceive);
            NVIC_SetPriority(GPDMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
            NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
            LL_DMA_EnableIT_TC(GPDMA1, LL_DMA_CHANNEL_3);
            LL_DMA_EnableIT_DTE(GPDMA1, LL_DMA_CHANNEL_3);
            LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_3);

            SID_PAL_LOG_INFO("UART2 baud rate changed to %lu", new_baud);
            break;
        }

        default:
            SID_PAL_LOG_ERROR("UART2: Unknown command 0x%02X", cmd);
            uart2_send_response(cmd, UART2_RSP_ERR, NULL, 0);
            break;
    }
}

/*----------------------------------------------------------------------------*/

static void factory_reset(app_context_t * const context)
{
    /* Validate inputs */
    if (NULL == context)
    {
        SID_PAL_LOG_ERROR("Factory reset request failed - context cannot be null");
        return;
    }

    sid_error_t ret = sid_set_factory_reset(context->sidewalk_handle);
    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Factory reset request failed: %d", (int32_t)ret);
    }
    else
    {
        SID_PAL_LOG_DEBUG("Factory reset initiated");
    }
}

/*----------------------------------------------------------------------------*/

static void print_subghz_link_profile(const app_context_t * const context)
{
    sid_error_t ret;
    struct sid_device_profile dev_cfg;

    do
    {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        dev_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_A;
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        dev_cfg.unicast_params.device_profile_id = SID_LINK2_PROFILE_2;
#else
        ret = SID_ERROR_NOSUPPORT;
        break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

        ret = sid_option(context->sidewalk_handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE, &dev_cfg, sizeof(dev_cfg));
        if (ret != SID_ERROR_NONE)
        {
            break;
        }

        SID_PAL_LOG_INFO("Current Sidewalk SubGHz mode: %s, rx_wndw_cnt: %u, wkup_type: %s, rx_interv: %ums",
                         sid_subghz_profile_code_to_str(dev_cfg.unicast_params.device_profile_id),
                         (uint32_t)dev_cfg.unicast_params.rx_window_count,
                         dev_cfg.unicast_params.wakeup_type < SID_STM32_UTIL_ARRAY_SIZE(sid_subghz_wakeup_type_strings) ? sid_subghz_wakeup_type_strings[dev_cfg.unicast_params.wakeup_type] : "Unknown",
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                         (uint32_t)dev_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms
#else
                         (uint32_t)dev_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
        );

        ret = SID_ERROR_NONE;
    } while (0);

    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to get Sidewalk SubGHz link profile info. Error %d", (int32_t)ret);
    }
}

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
static void adjust_sidewalk_link_log_level(const uint32_t link_mask)
{
    if (SID_PAL_LOG_LEVEL > SID_PAL_LOG_SEVERITY_INFO)
    {
        struct sid_log_control_severity sid_log_settings;
        sid_log_control_get_severity(&sid_log_settings);
        if ((link_mask & SID_LINK_TYPE_2) != 0u)
        {
            sid_log_settings.level = SID_PAL_LOG_SEVERITY_INFO;
        }
        else
        {
            sid_log_settings.level = SID_PAL_LOG_LEVEL;
        }
        sid_log_control_set_severity(&sid_log_settings);
    }
}
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

/*----------------------------------------------------------------------------*/

static sid_error_t init_and_start_link(app_context_t * const context, struct sid_config * const config, const bool cold_start)
{
    sid_error_t ret = SID_ERROR_GENERIC;
    struct sid_handle *sid_handle = NULL;
    struct sid_status status;

    /* Validate inputs */
    if (NULL == context)
    {
        SID_PAL_LOG_ERROR("Sidewalk context cannot be null");
        ret = SID_ERROR_NULL_POINTER;
        goto error;
    }

    if (NULL == config)
    {
        SID_PAL_LOG_ERROR("Sidewalk config cannot be null");
        ret = SID_ERROR_NULL_POINTER;
        goto error;
    }

    /* Make sure Sidewalk is not initialized already */
    if (context->sidewalk_handle != NULL)
    {
        SID_PAL_LOG_ERROR("Sidewalk is initialized already");
        ret = SID_ERROR_ALREADY_INITIALIZED;
        goto error;
    }

    /* Initialize state indication to the user app */
    context->state = STATE_SIDEWALK_NOT_READY;

    if (cold_start != false)
    {
#if SID_FSK_ONLY_LINK
        /* For FSK-only mode start with FSK mode and handle the device registration over the FSK link */
        config->link_mask = SID_COMM_LINK_TYPE;
#else
        /* Initialize Sidewalk in BLE + LoRa mode to enable both registration and SubGHz radio */
        /* Note: LoRa link added to force STM32WL55 radio PAL init for UDT support */
        config->link_mask = SID_REGISTRATION_LINK_TYPE | SID_COMM_LINK_TYPE;
#endif /* SID_FSK_ONLY_LINK */
        ret = sid_init(config, &sid_handle);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to initialize Sidewalk stack, link_mask:%x, err:%d", (int)config->link_mask, (int)ret);
            goto error;
        }

        /* Register sidewalk handle to the application context */
        context->sidewalk_handle = sid_handle;

        ret = sid_get_status(sid_handle, &status);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to get Sidewalk stack status, err:%d", (int)ret);
            goto error;
        }

        SID_PAL_LOG_INFO("Sidewalk registration status: %s", status.detail.registration_status == SID_STATUS_REGISTERED ? "Registered" : "Not registered");

        if (SID_STATUS_REGISTERED == status.detail.registration_status)
        {
            /* Indicate to the app that Sidewalk device registration is completed */
            queue_event(g_event_queue, EVENT_TYPE_REGISTRATION_COMPLETED);
#if SID_FSK_ONLY_LINK
            /* Proceed with starting FSK link as stack re-initialization is not required */
#else
            /* Terminate from here, Sidewalk stack should be re-initialzed to switch from registration mode link to normal mode link */
            return SID_ERROR_NONE;
#endif /* SID_FSK_ONLY_LINK */
        }
        else
        {
            /* Indicate registration process is pending */
            registration_pending = true;
        }
    }
    else
    {
        /* This is a re-initialization attempt, we assume that registration is completed at this point */
        config->link_mask = SID_COMM_LINK_TYPE;
        ret = sid_init(config, &sid_handle);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to initialize Sidewalk link_mask:%x, err:%d", (int)config->link_mask, (int)ret);
            goto error;
        }

        /* Register new Sidewalk handle to the application context */
        context->sidewalk_handle = sid_handle;
    }

#if (SID_SDK_CONFIG_ENABLE_LINK_TYPE_2)
    /* Reduce log level for FSK connection since it produces excessive logs that affect communication timings */
    adjust_sidewalk_link_log_level(config->link_mask);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

    /* Start the Sidewalk stack with the selected link type */
    ret = sid_start(sid_handle, config->link_mask);
    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to start Sidewalk stack, link_mask:%x, err:%d", (int)config->link_mask, (int)ret);
        goto error;
    }

    return ret;

error:
    context->sidewalk_handle = NULL;
    config->link_mask = 0;
    return ret;
}

/*----------------------------------------------------------------------------*/

static sid_error_t set_device_profile(app_context_t * const context, struct sid_device_profile * const set_dp_cfg)
{
    sid_error_t ret;
    struct sid_device_profile dev_cfg;

    do
    {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        dev_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_A;
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        dev_cfg.unicast_params.device_profile_id = SID_LINK2_PROFILE_2;
#else
        ret = SID_ERROR_NOSUPPORT;
        break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

        ret = sid_option(context->sidewalk_handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE, &dev_cfg, sizeof(dev_cfg));
        if (ret != SID_ERROR_NONE)
        {
            break;
        }

        if ((set_dp_cfg->unicast_params.device_profile_id != dev_cfg.unicast_params.device_profile_id)
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
         || (set_dp_cfg->unicast_params.unicast_window_interval.async_rx_interval_ms != dev_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms)
#else
         || (set_dp_cfg->unicast_params.unicast_window_interval.sync_rx_interval_ms != dev_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms)
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
         || (set_dp_cfg->unicast_params.rx_window_count   != dev_cfg.unicast_params.rx_window_count))
        {
            ret = sid_option(context->sidewalk_handle, SID_OPTION_900MHZ_SET_DEVICE_PROFILE, set_dp_cfg, sizeof(dev_cfg));
        }
        else
        {
            ret = SID_ERROR_NONE;
            SID_PAL_LOG_INFO("Sidewalk SubGHz mode is already set to the desired configuration");
        }
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

static sid_error_t destroy_link(app_context_t *app_context, struct sid_config *const config)
{
    sid_error_t ret;
    do
    {
        if (NULL == app_context)
        {
            ret = SID_ERROR_UNINITIALIZED;
            break;
        }

        if ((NULL == app_context->sidewalk_handle) || (NULL == config))
        {
            ret = SID_ERROR_INVALID_ARGS;
            break;
        }

        ret = sid_stop(app_context->sidewalk_handle, config->link_mask);
        if(ret != SID_ERROR_NONE)
        {
            break;
        }
        ret = sid_deinit(app_context->sidewalk_handle);
        if(ret != SID_ERROR_NONE)
        {
            break;
        }
        app_context->sidewalk_handle = NULL;
#if CFG_LED_SUPPORTED
        (void)led_indication_set(LED_INDICATE_IDLE);
#endif /* CFG_LED_SUPPORTED */
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

#if (CFG_BUTTON_SUPPORTED == 1)
#  if (defined(STM32WBA6x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
static void button1_irq_handler(uint32_t pin, void * callback_arg)
{
    static uint32_t uStart = BUTTON_TIMESTAMP_INVALID_VAL;
    static uint32_t uEnd   = BUTTON_TIMESTAMP_INVALID_VAL;
    uint8_t pinState;
    sid_error_t ret;

    (void)callback_arg;

    ret = sid_pal_gpio_read(pin, &pinState);
    if (SID_ERROR_NONE == ret)
    {
        if (GPIO_PIN_RESET == pinState)
        {
            uStart = osKernelGetTickCount();
        }
        else if (uStart != BUTTON_TIMESTAMP_INVALID_VAL)
        {
            uEnd = osKernelGetTickCount();

            if((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                queue_event(g_event_queue, EVENT_TYPE_TOGGLE_LINK);
            }
            else
            {
                /* Long-press event */
                queue_event(g_event_queue, EVENT_FACTORY_RESET);
            }

            /* Reset start timestamp */
            uStart = BUTTON_TIMESTAMP_INVALID_VAL;
        }
        else
        {
            /**
             * Invalid state - rising edge detected but preceding falling edge was not.
             * This may happen due to GPIO transitions when leaving Standby LPM. The options are
             * either to ignore such events (like it is done here) or enable GPIO pin retention
             * for the respective GPIO pin before entering Standby LPM, but this will result in
             * higher quiescent current.
             */
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B1 state. Error %d", (int32_t)ret);
    }
}
#  endif /* (STM32WBA6x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA6x */

/*----------------------------------------------------------------------------*/

#  if (defined(STM32WBA5x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
static void button2_irq_handler(uint32_t pin, void * callback_arg)//
{
    (void)pin;
    (void)callback_arg;

    static uint32_t uStart = BUTTON_TIMESTAMP_INVALID_VAL;
    static uint32_t uEnd   = BUTTON_TIMESTAMP_INVALID_VAL;
    uint8_t pinState;
    sid_error_t ret;

     ret = sid_pal_gpio_read(pin, &pinState);
    if (SID_ERROR_NONE == ret)
    {
        if (GPIO_PIN_RESET == pinState)
        {
            uStart = osKernelGetTickCount();
        }
        else if (uStart != BUTTON_TIMESTAMP_INVALID_VAL)
        {
            uEnd = osKernelGetTickCount();

            if((uEnd - uStart) >= (4u * OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS)))
            {
                /* Extra-long-press event */
#    if (CFG_LPM_STDBY_SUPPORTED != 0)
                /* Enter Standby LPM without retention */
                queue_event(g_event_queue, EVENT_TYPE_FULL_STANDBY);
#    endif /* CFG_LPM_STDBY_SUPPORTED */
            }
            else if((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                queue_event(g_event_queue, EVENT_TYPE_SET_DEVICE_PROFILE);
            }
            else
            {
                /* Long-press event */
#    if  defined(STM32WBA6x) && (defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X)) /* Semtech shields occupy B1 EXTI line for BUSY signal, use long B2 press instead */
                queue_event(g_event_queue, EVENT_TYPE_TOGGLE_LINK);
#    endif  /* STM32WBA6x && (SID_RADIO_PLATFORM_LR11XX || SID_RADIO_PLATFORM_SX126X) */
            }

            /* Reset start timestamp */
            uStart = BUTTON_TIMESTAMP_INVALID_VAL;
        }
        else
        {
            /**
             * Invalid state - rising edge detected but preceding falling edge was not.
             * This may happen due to GPIO transitions when leaving Standby LPM. The options are
             * either to ignore such events (like it is done here) or enable GPIO pin retention
             * for the respective GPIO pin before entering Standby LPM, but this will result in
             * higher quiescent current.
             */
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B2 state. Error %d", (int32_t)ret);
    }
}
#  endif /* (STM32WBA5x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA5x */

/*----------------------------------------------------------------------------*/

static void button3_irq_handler(uint32_t pin, void * callback_arg)
{
    static uint32_t uStart = BUTTON_TIMESTAMP_INVALID_VAL;
    static uint32_t uEnd   = BUTTON_TIMESTAMP_INVALID_VAL;
    uint8_t pinState;
    sid_error_t ret;

    (void)callback_arg;

    ret = sid_pal_gpio_read(pin, &pinState);
    if (SID_ERROR_NONE == ret)
    {
        if (GPIO_PIN_RESET == pinState)
        {
            uStart = osKernelGetTickCount();
        }
        else if (uStart != BUTTON_TIMESTAMP_INVALID_VAL)
        {
            uEnd = osKernelGetTickCount();

#  if defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X) /* Semtech shields occupy B2 GPIO pin for BUSY signal, use extra-long B3 press instead */
            if((uEnd - uStart) >= (4u * OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS)))
            {
                /* Extra-long-press event */
#    if defined(STM32WBA6x)
                queue_event(g_event_queue, EVENT_FACTORY_RESET);
#    elif defined(STM32WBA5x)
                queue_event(g_event_queue, EVENT_TYPE_SET_DEVICE_PROFILE);
#    endif /* STM32WBA6x || STM32WBA5x */
            }
            else
#  endif /* SID_RADIO_PLATFORM_LR11XX || SID_RADIO_PLATFORM_SX126X */
            if((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                queue_event(g_event_queue, EVENT_TYPE_SEND_HELLO);
            }
            else
            {
                /* Long-press event */
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
                /* Enter Standby LPM with retention and keep Sidewalk running */
                queue_event(g_event_queue, EVENT_TYPE_ACTIVE_STANDBY);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
            }

            /* Reset start timestamp */
            uStart = BUTTON_TIMESTAMP_INVALID_VAL;
        }
        else
        {
            /**
             * Invalid state - rising edge detected but preceding falling edge was not.
             * This may happen due to GPIO transitions when leaving Standby LPM. The options are
             * either to ignore such events (like it is done here) or enable GPIO pin retention
             * for the respective GPIO pin before entering Standby LPM, but this will result in
             * higher quiescent current.
             */
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B3 state. Error %d", (int32_t)ret);
    }
}

/*----------------------------------------------------------------------------*/

#  if (CFG_LPM_STDBY_SUPPORTED != 0)
static void button3_wakeup_handler(uint32_t pin, void * callback_arg)
{
    (void)pin;
    (void)callback_arg;

    /* Disable Standby LPM */
    UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_DISABLE);

    /* Prevent B3 from triggering WKUP IRQ on pressing till the next entry to Standby mode */
    (void)sid_pal_gpio_ext_ifc_wakeup_disable(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN));

    SID_PAL_LOG_INFO("Standby mode deactivated");
}
#  endif /* (CFG_LPM_STDBY_SUPPORTED != 0) */

/*----------------------------------------------------------------------------*/

static void buttons_init(void)//
{
    sid_error_t ret_code;

#  if (defined(STM32WBA6x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* (STM32WBA6x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA6x */

#  if (defined(STM32WBA5x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* (STM32WBA5x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA5x */

    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);

    /* Give the GPIO pins some time to reach a stable state before enabling IRQs */
    sid_pal_delay_us(75u);

#  if (defined(STM32WBA6x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button1_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* (STM32WBA6x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA6x */

#  if (defined(STM32WBA5x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button2_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* (STM32WBA5x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA5x */

    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button3_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
}
#endif /* CFG_BUTTON_SUPPORTED */

/*----------------------------------------------------------------------------*/

static void sidewalk_stack_task_entry(void *context)
{
    SID_PAL_LOG_INFO("Sidewalk demo started");

    app_context_t *app_context = (app_context_t *)context;
    sid_error_t ret;
    osStatus_t os_status;

    struct sid_event_callbacks event_callbacks = {
        .context           = app_context,
        .on_event          = on_sidewalk_event,          /* Called from ISR context */
        .on_msg_received   = on_sidewalk_msg_received,   /* Called from sid_process() */
        .on_msg_sent       = on_sidewalk_msg_sent,       /* Called from sid_process() */
        .on_send_error     = on_sidewalk_send_error,     /* Called from sid_process() */
        .on_status_changed = on_sidewalk_status_changed, /* Called from sid_process() */
        .on_factory_reset  = on_sidewalk_factory_reset,  /* Called from sid_process */
    };

    struct sid_config config = {
        .link_mask           = 0u,
        .callbacks           = &event_callbacks,
        .link_config         = app_get_sidewalk_ble_config(),
        .sub_ghz_link_config = app_get_sub_ghz_config(),
    };

    /* Performing a cold start of the Sidewalk since device registration status is unknown at this point */
    queue_event(g_event_queue, EVENT_TYPE_COLD_START_LINK);

    while (1)
    {
        enum event_type event;

        if (osMessageQueueGet (app_context->event_queue, &event, 0u, osWaitForever) == osOK)
        {
            switch (event)
            {
                case EVENT_TYPE_COLD_START_LINK:
                    /* Performing a cold start of the Sidewalk since device registration status is unknown at this point */
                    if (NULL == app_context->sidewalk_handle)
                    {
                        ret = init_and_start_link(app_context, &config, true);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to initialize Sidewalk. Error code: %d", (int)ret);
                            goto error;
                        }
                    }
                    else
                    {
                        /* Skip the request, Sidewalk is already running */
                    }
                    break;

                case EVENT_TYPE_HOT_START_LINK:
                    /* Performing a hot start of the Sidewalk */
                    if (NULL == app_context->sidewalk_handle)
                    {
                        ret = init_and_start_link(app_context, &config, false);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to initialize Sidewalk. Error code: %d", (int)ret);
                            goto error;
                        }
                    }
                    else
                    {
                        /* Skip the request, Sidewalk is already running */
                    }
                    break;

                case EVENT_TYPE_TOGGLE_LINK:
                    if(app_context->sidewalk_handle == NULL)
                    {
                        queue_event(g_event_queue, (false == registration_pending) ? EVENT_TYPE_HOT_START_LINK : EVENT_TYPE_COLD_START_LINK);
                    }
                    else
                    {
                        queue_event(g_event_queue, EVENT_TYPE_STOP_LINK);
                    }
                    break;

                case EVENT_TYPE_STOP_LINK:
                    if (app_context->sidewalk_handle != NULL)
                    {
                        ret = destroy_link(app_context, &config);
                        SID_PAL_ASSERT(ret == SID_ERROR_NONE);
                    }
                    break;

                case EVENT_TYPE_SIDEWALK:
                    if ((NULL == app_context) || (NULL == app_context->sidewalk_handle))
                    {
                        /*  This may happen if Sidewalk link was terminated but the event queue already contained some events - just ignore them */
                        break;
                    }
                    ret = sid_process(app_context->sidewalk_handle);
                    if ((ret != SID_ERROR_NONE) && (ret != SID_ERROR_STOPPED))
                    {
                        SID_PAL_LOG_ERROR("Error processing Sidewalk event, err:%d", (int)ret);
                        goto error;
                    }
                    else if (SID_ERROR_STOPPED == ret)
                    {
                        SID_PAL_LOG_DEBUG("Sidewalk processing discarded - Sidewalk stack is stopped");
                    }
                    else
                    {
                        /* Nothing to do, everything is fine */
                    }
                    break;

                case EVENT_TYPE_SEND_HELLO:
                    SID_PAL_LOG_INFO("EVENT_TYPE_SEND_HELLO");

                    if (app_context->state == STATE_SIDEWALK_READY)
                    {
                        send_ping(app_context);
                    }
                    else
                    {
                        SID_PAL_LOG_DEBUG("Sidewalk not ready!");;
                    }
                    break;

                case EVENT_TYPE_UART2_DATA:
                    SID_PAL_LOG_INFO("EVENT_TYPE_UART2_DATA");
                    parse_uart2_command(app_context);
                    break;

                case EVENT_TYPE_SET_DEVICE_PROFILE:
                    {
                        SID_PAL_LOG_INFO("EVENT_TYPE_SET_DEVICE_PROFILE");
                        struct sid_device_profile set_dp_cfg = {};
                        struct sid_device_profile dev_cfg;

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                        dev_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_A;
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
                        dev_cfg.unicast_params.device_profile_id = SID_LINK2_PROFILE_2;
#else
                        SID_PAL_LOG_ERROR("SubGHz profile switch failed - unknown link mode");
                        break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

                        sid_option(app_context->sidewalk_handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE, &dev_cfg, sizeof(dev_cfg));

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                        if (SID_LINK3_PROFILE_A == dev_cfg.unicast_params.device_profile_id)
                        {
                            set_dp_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_B;
                            set_dp_cfg.unicast_params.rx_window_count   = SID_RX_WINDOW_CNT_INFINITE;
                            set_dp_cfg.unicast_params.wakeup_type       = SID_TX_AND_RX_WAKEUP;
                            set_dp_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms = SID_LINK3_RX_WINDOW_SEPARATION_3;
                        }
                        else if (SID_LINK3_PROFILE_B == dev_cfg.unicast_params.device_profile_id)
                        {
                            set_dp_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_A;
                            set_dp_cfg.unicast_params.rx_window_count   = SID_RX_WINDOW_CNT_2;
                            set_dp_cfg.unicast_params.wakeup_type       = SID_TX_AND_RX_WAKEUP;
                            set_dp_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms = SID_LINK3_RX_WINDOW_SEPARATION_3;
                        }
                        else
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
                        if (SID_LINK2_PROFILE_1 == dev_cfg.unicast_params.device_profile_id)
                        {
                            set_dp_cfg.unicast_params.device_profile_id    = SID_LINK2_PROFILE_2;
                            set_dp_cfg.unicast_params.rx_window_count      = SID_RX_WINDOW_CNT_INFINITE;
                            set_dp_cfg.unicast_params.beacon_interval_unit = 1u;
                            set_dp_cfg.unicast_params.wakeup_type          = SID_TX_AND_RX_WAKEUP;
                            set_dp_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms = SID_LINK2_RX_WINDOW_SEPARATION_2;
                        }
                        else if (SID_LINK2_PROFILE_2 == dev_cfg.unicast_params.device_profile_id)
                        {
                            set_dp_cfg.unicast_params.device_profile_id    = SID_LINK2_PROFILE_1;
                            set_dp_cfg.unicast_params.rx_window_count      = SID_RX_WINDOW_CNT_INFINITE;
                            set_dp_cfg.unicast_params.beacon_interval_unit = 1u;
                            set_dp_cfg.unicast_params.wakeup_type          = SID_TX_AND_RX_WAKEUP;
                            set_dp_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms = SID_LINK2_RX_WINDOW_SEPARATION_1;
                        }
                        else
#else
                        {
                            SID_PAL_LOG_ERROR("Unsupported SubGHz link profile - %s (%u)",
                                              sid_subghz_profile_code_to_str(dev_cfg.unicast_params.device_profile_id),
                                              (uint32_t)dev_cfg.unicast_params.device_profile_id);
                            break;
                        }
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

                        SID_PAL_LOG_INFO("Changing Sidewalk SubGHz link mode: %s -> %s, rx_interval %u",
                                         sid_subghz_profile_code_to_str(dev_cfg.unicast_params.device_profile_id),
                                         sid_subghz_profile_code_to_str(set_dp_cfg.unicast_params.device_profile_id),
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                                         (uint32_t)set_dp_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms
#else
                                         (uint32_t)set_dp_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
                        );

                        ret = set_device_profile(app_context, &set_dp_cfg);
                        if (SID_ERROR_NONE == ret)
                        {
                            SID_PAL_LOG_INFO("Successfully switched Sidewalk SubGHz mode to %s",
                                             sid_subghz_profile_code_to_str(set_dp_cfg.unicast_params.device_profile_id));
                        }
                        else
                        {
                            SID_PAL_LOG_ERROR("Failed to switch Sidewalk SubGHz link mode to %s. Error %d",
                                              sid_subghz_profile_code_to_str(set_dp_cfg.unicast_params.device_profile_id),
                                              (int32_t)ret);
                        }
                    }
                    break;

                case EVENT_TYPE_REGISTRATION_COMPLETED:
#if SID_FSK_ONLY_LINK
                    /* Stack re-initialization is not required, we are in the FSK mode already */
#else
                    /* Switch to SubGHz-only configuration since device registration is done */
                    ret = sid_deinit(app_context->sidewalk_handle);
                    if (ret != SID_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Failed to deinitialize Sidewalk, link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                        break;
                    }

                    config.link_mask = SID_COMM_LINK_TYPE;
                    ret = sid_init(&config, &app_context->sidewalk_handle);
                    if (ret != SID_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Failed to initialize Sidewalk link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                        break;
                    }

#  if (SID_SDK_CONFIG_ENABLE_LINK_TYPE_2)
                    /* Reduce log level for FSK connection since it produces excessive logs that affect communication timings */
                    adjust_sidewalk_link_log_level(config.link_mask);
#  endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

                    ret = sid_start(app_context->sidewalk_handle, config.link_mask);
                    if (ret != SID_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Failed to start Sidewalk stack, link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                        break;
                    }
#endif /* SID_FSK_ONLY_LINK */

                    /* Indicate to the app that Sidewalk device registration is completed */
                    os_status = osSemaphoreRelease(registration_ready_semaphore);
                    if (os_status != osOK)
                    {
                        SID_PAL_LOG_ERROR("Failed to indicate Sidewalk registration completion. Error code : %d", (int32_t)os_status);
                        goto error;
                    }
                    break;

                case EVENT_FACTORY_RESET:
                    factory_reset(app_context);
                    break;

#if (CFG_BUTTON_SUPPORTED != 0) && (CFG_LPM_STDBY_SUPPORTED != 0)
                case EVENT_TYPE_ACTIVE_STANDBY:
                    /**
                     * Note: Sidewalk keeps running in this mode. However, LEDs and buttons will be inactive (except the wakeup source)
                     * due to GPIO being powered down. Periodic wakeups to handle Sidewalk events are indicated by LEDs activation while
                     * the MCU is in Run mode. As soon as Sidewalk radio event is processed, the LEDs are turned off and the system goes
                     * back into Standby LPM.
                     */

                    /* Use B3 button release as a wakeup trigger */
                    ret = sid_pal_gpio_ext_ifc_wakeup_enable(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_IRQ_TRIGGER_RISING, button3_wakeup_handler, NULL);
                    if (ret != SID_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Error entering in standby mode: unable to configure wakeup pin, err:%d", (int)ret);
                        goto error;
                    }

                    /* Enable Standby LPM */
                    UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
                    SID_PAL_LOG_INFO("Standby with retention mode enabled");
                    break;

                case EVENT_TYPE_FULL_STANDBY:
                    /**
                     * Note: Sidewalk connectivity is not available in this mode since neither RAM nor peripheral states are retained.
                     * This mode allows to achieve the lowest power consumption since the only running unit is RTC. System can be woken
                     * up by pressing B3 or scheduling an RTC event. Wakeup startup process is identical to MCU reset flow since all
                     * units have to be initialized from scratch (except RTC).
                     */
                    if ((app_context != NULL) && (app_context->sidewalk_handle != NULL))
                    {
                        ret = destroy_link(app_context, &config);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Error entering in standby mode: unable to terminate Sidewalk link. Error code: %d", (int)ret);
                            goto error;
                        }
                    }

                    /* Use B3 button release as a wakeup trigger */
                    ret = sid_pal_gpio_ext_ifc_wakeup_enable(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_IRQ_TRIGGER_RISING, button3_wakeup_handler, NULL);
                    if (ret != SID_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Error entering in standby mode: unable to configure wakeup pin, err:%d", (int)ret);
                        goto error;
                    }

                    /* Suspend demo task to avoid periodic wakeups from Standby mode */
                    if (demo_counter_task != NULL)
                    {
                        (void)osThreadSuspend(demo_counter_task);
                    }

#  if CFG_LED_SUPPORTED
                    /* Turn off LEDs */
                    led_indication_set(LED_INDICATE_OFF);
#  endif /* CFG_LED_SUPPORTED */

                    /* Disable RAM retention to achieve lowest possible power consumption */
                    HAL_PWREx_DisableSRAM1ContentStandbyRetention();
                    HAL_PWREx_DisableSRAM2ContentStandbyRetention();

                    /* Enable Standby LPM */
                    UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
                    SID_PAL_LOG_INFO("Entering Standby w/o retention mode...");
                    break;
#endif /* CFG_BUTTON_SUPPORTED && CFG_LPM_STDBY_SUPPORTED */

                default:
                    /* Ignore unknown events */
                    break;
            }
        }
    }

error:
    if (demo_counter_task != NULL)
    {
        (void)osThreadTerminate(demo_counter_task);
        demo_counter_task = NULL;
    }

    if (app_context->sidewalk_handle != NULL)
    {
        sid_stop(app_context->sidewalk_handle, config.link_mask);
        sid_deinit(app_context->sidewalk_handle);
        app_context->sidewalk_handle = NULL;
    }

#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_OFF);
#endif /* CFG_LED_SUPPORTED */

    SID_PAL_LOG_INFO("Sidewalk demo terminated due to error");
    Error_Handler();

    osThreadExit(); /* Normally this line is not reachable, but keep it if Error_Handler somehow returns */
}

/*----------------------------------------------------------------------------*/

static void demo_counter_task_entry(void *context)
{
    app_context_t *app_context = (app_context_t *)context;
    osStatus_t status;

    SID_PAL_LOG_INFO("Demo Counter Thread: waiting for the Sidewalk registration to be completed...");
    status = osSemaphoreAcquire(registration_ready_semaphore, osWaitForever);
    if (status != osOK)
    {
        SID_PAL_LOG_ERROR("Demo Counter Thread - failed to wait for Sidewalk device registration to be completed");
        goto error;
    }
    SID_PAL_LOG_INFO("Demo Counter Thread: Sidewalk device is registered, proceeding with the demo...");

    while (1)
    {
        if ((app_context->state != STATE_SIDEWALK_READY) && (app_context->state != STATE_SIDEWALK_SECURE_CONNECTION))
        {
            SID_PAL_LOG_DEBUG("Demo Counter Thread: waiting for the Sidewalk connection");
        }

        osStatus_t status = osSemaphoreAcquire(connection_ready_semaphore, 15000u);
        if (status != osOK)
        {
            if (osErrorTimeout == status)
            {
                SID_PAL_LOG_WARNING("Demo Counter Thread: Sidewalk connection is not available");
            }
            else
            {
                SID_PAL_LOG_ERROR("Demo Counter Thread: unable to establish Sidewalk connection. Error code: %d", status);
            }
            continue;
        }

        while ((STATE_SIDEWALK_READY == app_context->state) || (STATE_SIDEWALK_SECURE_CONNECTION == app_context->state))
        {
            SID_PAL_LOG_INFO("Demo Counter Thread: sending out counter update");
            queue_event(g_event_queue, EVENT_TYPE_SEND_HELLO);
            osDelay(DEMO_MESSAGE_DELAY_MS);
            SID_PAL_LOG_INFO("Demo Counter Thread: sending out counter update - DONE");
        }

        SID_PAL_LOG_DEBUG("Demo Counter Thread: Sidewalk connection lost");
    }

error:
    SID_PAL_LOG_INFO("Demo Counter Thread: Terminated due to error");
    SID_PAL_LOG_FLUSH();
    Error_Handler();

    /* Normally the lines below are not reachable, but keep them if Error_Handler somehow returns */
    demo_counter_task = NULL;
    osThreadExit();
}

/*----------------------------------------------------------------------------*/

#if SID_PAL_LOG_ENABLED
static const char * sid_subghz_profile_code_to_str(enum sid_device_profile_id code)
{
    const char * profile_str = NULL;

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    assert_param(code >= SID_LINK3_PROFILE_A);
    assert_param(code < SID_LINK3_PROFILE_LAST);
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    assert_param(code >= SID_LINK2_PROFILE_1);
    assert_param(code < SID_LINK2_PROFILE_LAST);
#else
    assert_param(0); /* Invalid configuration */
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

    /*Converting from sid_profile_id to index*/
    switch (code)
    {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        case SID_LINK3_PROFILE_A:
            profile_str = sid_subghz_profile_strings[SID_LORA_PROFILE_A];
            break;

        case SID_LINK3_PROFILE_B:
            profile_str = sid_subghz_profile_strings[SID_LORA_PROFILE_B];
            break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        case SID_LINK2_PROFILE_1:
            profile_str = sid_subghz_profile_strings[SID_FSK_PROFILE_1];
            break;

        case SID_LINK2_PROFILE_2:
            profile_str = sid_subghz_profile_strings[SID_FSK_PROFILE_2];
            break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

        default:
            profile_str = sid_subghz_profile_strings[SID_SUBGHZ_PROFILE_UNKNOWN];
            break;
    }

    return profile_str;
}
#endif /* SID_PAL_LOG_ENABLED */

/* UDT callback - data received from WL55 via SPI */
static void _on_wl55_user_data(const uint8_t * const data, const uint32_t data_len, void * user_arg)
{
    (void)user_arg;
    SID_PAL_LOG_INFO("UDT from WL55 (%u bytes): %.*s", data_len, data_len, (const char *)data);
    SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, data, data_len);
}

/* Global function definitions -----------------------------------------------*/

void SID_APP_Init(void)
{
    /* Adjust Sidewalk log level dynamically - this is needed if the firmware is compiled using
     * Sidewalk SDK static library and the library was assembled with different debug level
     */
    struct sid_log_control_severity sid_log_settings;
    sid_log_control_get_severity(&sid_log_settings);
    if (SID_PAL_LOG_LEVEL != sid_log_settings.level)
    {
        sid_log_settings.level = SID_PAL_LOG_LEVEL;
        sid_log_control_set_severity(&sid_log_settings);
    }

    /* Printout application version info */
    SID_PAL_LOG_INFO("Application name: %s", SID_APP_PROJECT_NAME);
    SID_PAL_LOG_INFO("Application version %s", SID_APP_PROJECT_VERSION_STRING);
    SID_PAL_LOG_INFO("Application build type: %s", SID_APP_PROJECT_BUILD_TYPE);
    SID_PAL_LOG_INFO("Application commit hash: %s", SID_APP_PROJECT_COMMIT_HASH_STRING);
    SID_PAL_LOG_INFO("Application commit description: %s", SID_APP_PROJECT_COMMIT_DESCRIPTION);
    SID_PAL_LOG_INFO("Sidewalk SDK: %u.%u.%u-%u", SID_SDK_MAJOR_VERSION, SID_SDK_MINOR_VERSION, SID_SDK_PATCH_VERSION, SID_SDK_BUILD_VERSION);
    SID_PAL_LOG_INFO("FreeRTOS Kernel: %s", tskKERNEL_VERSION_NUMBER);

    /* CubeMX pack version */
    const hal_version_info_t cubemx_fw_pack_ver = {
        .raw = HAL_GetHalVersion(),
    };
    SID_PAL_LOG_INFO("STM32CubeWBA: %u.%u.%u", cubemx_fw_pack_ver.major, cubemx_fw_pack_ver.minor, cubemx_fw_pack_ver.patch);

    /* Printout MCU details */
    stm32_mcu_info_t mcu_info = stm32_mcu_info_describe_host();
    SID_PAL_LOG_INFO("Host MCU: %s (0x%X), revision: %s (0x%X)", mcu_info.device_name, mcu_info.device_id, mcu_info.rev_name, mcu_info.rev_id);

    platform_parameters_t platform_parameters = {
        .mfg_store_region.addr_start = MANUFACTURE_FLASH_START,
        .mfg_store_region.addr_end   = MANUFACTURE_FLASH_END,

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        .platform_init_parameters.radio_cfg = get_radio_cfg(),
#endif
    };

    sid_error_t ret_code = sid_platform_init(&platform_parameters);
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk Platform Init err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }

    /* Register UDT callback to receive data from WL55 */
    ret_code = sid_pal_radio_stm32wlxx_set_user_data_received_cb(_on_wl55_user_data, NULL);
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set UDT RX callback, err:%d", (int)ret_code);
    }
    else
    {
        SID_PAL_LOG_INFO("UDT RX callback registered");
    }

#if CFG_LED_SUPPORTED
    ret_code = led_indication_init();
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk Init LED indication err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }
#endif /* CFG_LED_SUPPORTED */

    g_event_queue = osMessageQueueNew(SID_MSG_QUEUE_LEN, sizeof(enum event_type), NULL);
    if (g_event_queue == NULL)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk message queue. No memory");
        SID_PAL_ASSERT(0);
    }

    static app_context_t app_context = {
        .event_queue = NULL,
        .main_task = NULL,
        .sidewalk_handle = NULL,
        .state = STATE_INIT,
    };

#if (CFG_BUTTON_SUPPORTED == 1)
    buttons_init();
#endif /* (CFG_BUTTON_SUPPORTED == 1) */

    app_context.event_queue = g_event_queue;
    app_context.main_task = osThreadNew(sidewalk_stack_task_entry, &app_context, &sidewalk_stack_task_attributes);
    if (NULL == app_context.main_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk processing thread. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to monitor Sidewalk registration status */
    registration_ready_semaphore = osSemaphoreNew(1u, 0u, &registration_ready_sem_attributes);
    if (NULL == registration_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk Registration Status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to monitor Sidewalk connection status */
    connection_ready_semaphore = osSemaphoreNew(1u, 0u, &connection_ready_sem_attributes);
    if (NULL == connection_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk connection status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

    demo_counter_task = osThreadNew(demo_counter_task_entry, &app_context, &demo_counter_task_attributes);
    if (NULL == demo_counter_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk demo thread. No memory");
        SID_PAL_ASSERT(0);
    }
}

/*----------------------------------------------------------------------------*/

void SID_APP_StandbyExit(void)
{
#if (CFG_BUTTON_SUPPORTED == 1)
    /* Restore GPIO and IRQ config for ther buttons */
    buttons_init();
#endif /* CFG_BUTTON_SUPPORTED */
}

void app_sidewalk_forward_uart2_data(const uint8_t *data, uint8_t len)
{
    if (len > sizeof(uart2_tx_buffer))
    {
        len = sizeof(uart2_tx_buffer);
    }
    memcpy(uart2_tx_buffer, data, len);
    uart2_tx_len = len;
    queue_event(g_event_queue, EVENT_TYPE_UART2_DATA);
}
