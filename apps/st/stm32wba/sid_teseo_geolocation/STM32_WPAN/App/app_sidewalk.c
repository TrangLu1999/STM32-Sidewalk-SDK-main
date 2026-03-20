/**
  ******************************************************************************
  * @file    app_sidewalk.c
  * @brief   Sidewalk SubGHz link sample app implementation
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <app_freertos.h>

/* Sidewalk SDK headers */
#include <sid_api.h>
#include <sid_hal_reset_ifc.h>
#include <sid_log_control.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_common_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_crypto_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_temperature_ifc.h>
#include <sid_900_cfg.h>
#include <sid_stm32_common_utils.h>

/* Teseo interface */
#include <teseo_gnss.h>

/* Version information */
#include SID_APP_VERSION_HEADER
#include <sid_sdk_version.h>

#include "target/memory.h"

#include "app_ble_config.h"
#include "app_900_config.h"
#include "app_teseo_config.h"
#include "app_common.h"
#if CFG_LED_SUPPORTED
#  include "led_indication.h"
#endif /* CFG_LED_SUPPORTED */
#include "sid_pal_gpio_ext_ifc.h"
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

#define GEOLOCATION_DEMO_UPDATE_INTERVAL_S          (120u) /*!< Interval (in seconds) between two successful location scans. The app will idle after a successful scan for this duration before initiating a new scan */
#define GEOLOCATION_DEMO_UPLINK_TTL                 (300u) /*!< TTL (in seconds) for a Sidewalk acknowledged uplink message */
#define GEOLOCATION_DEMO_UPLINK_RETRIES             (5u)   /*!< Number of retries for Sidewalk acknowledged uplink delivery */

#define SID_LINK_TYPE_NONE                          (0u)

#define GEOLOCATION_DEMO_TLV_TAG_COUNTER            (0xC7u) /*!< Demo counter value. This has no relation to geolocation. Demo Counter is an incrementing value that is sent to the cloud whenever user presses a button */
#define GEOLOCATION_DEMO_TLV_TAG_MCU_TEMP           (0x37u) /*!< MCU temperature value */
#define GEOLOCATION_DEMO_TLV_TAG_GNSS_POSITION      (0x60u) /*!< GNSS positioning information (latitude, longitude, elevation, timestamp, etc.) */

#define SID_MAX_MTU                                 (200u)  /*!< This app uses either FSK or LoRa link to send geolocation data. LoRa maximum MTU is 19 and FSK is 200 as per Sidewalk specification */

/* Private macro -----------------------------------------------------------*/

#define OS_MS_TO_TICKS( xTimeInMs )                 ( ( uint32_t ) ( ( ( uint32_t ) ( xTimeInMs ) * ( uint32_t ) osKernelGetTickFreq() ) / ( uint32_t ) 1000U ) )

/* Private typedef -----------------------------------------------------------*/

enum event_type
{
    EVENT_TYPE_SIDEWALK_INIT,
    EVENT_TYPE_SIDEWALK_START_LINK,
    EVENT_TYPE_SIDEWALK_STOP_LINK,
    EVENT_TYPE_SIDEWALK_TOGGLE_LINK,
    EVENT_TYPE_SIDEWALK_PROCESS,
    EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER,
    EVENT_TYPE_SIDEWALK_SEND_GEOLOCATION_DATA,
    EVENT_TYPE_SIDEWALK_FACTORY_RESET,
    EVENT_TYPE_SIDEWALK_SET_DEVICE_PROFILE,
    EVENT_TYPE_SIDEWALK_REGISTRATION_COMPLETED,
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
    osThreadId_t               main_task;
    osMessageQueueId_t         event_queue;
    struct sid_handle *        sidewalk_handle;
    enum app_state             state;
    struct link_status         link_status;
    uint8_t                    counter;
    int16_t                    mcu_temperature;
    teseo_gnss_position_data_t gnss_position;
} app_context_t;

enum sid_subghz_profile_idx {
    SID_LORA_PROFILE_A         = 0,
    SID_LORA_PROFILE_B         = 1,
    SID_FSK_PROFILE_1          = 2,
    SID_FSK_PROFILE_2          = 3,
    SID_SUBGHZ_PROFILE_UNKNOWN = 4,
    PROFILE_LAST               = SID_SUBGHZ_PROFILE_UNKNOWN,
};

typedef __PACKED_STRUCT {
    uint32_t timestamp;  /*!< Scan timestamp in Unix epoch seconds */
    float    latitude;   /*!< latitude in degrees */
    float    longitude;  /*!< Longitude in degrees */
    float    elevation;  /*!< Elevation over WGS84 ellipsoid (in meters) */
    float    h_accuracy; /*!< Horizontal 1-sigma accuracy (0.68 confidence level) */
    float    v_accuracy; /*!< Vertical 1-sigma accuracy (0.68 confidence level) */
} sid_uplink_gnss_position_record_t;

typedef __PACKED_STRUCT {
    uint8_t counter_value; /*!< Counter value that is sent to the cloud each time the user presses the button */
} sid_uplink_demo_counter_record_t;

typedef __PACKED_STRUCT {
    int16_t temperature_value; /*!< MCU temperature value that is sent to the cloud together with the position information */
} sid_uplink_mcu_temperature_record_t;

typedef uint16_t sid_uplink_token_t;

typedef __PACKED_STRUCT {
    __PACKED_STRUCT {
        uint8_t tag;
        uint8_t length;
    } header;
    __PACKED_UNION {
        uint8_t                             raw[1];
        sid_uplink_gnss_position_record_t   gnss_position;
        sid_uplink_demo_counter_record_t    demo_counter;
        sid_uplink_mcu_temperature_record_t mcu_temperature;
    } payload; /*!< TLV record payload. The actual amount of raw bytes or scan records is variable */
} sid_uplink_tlv_record_t;

typedef __PACKED_STRUCT {
    __PACKED_STRUCT {
        sid_uplink_token_t      token;            /*!< A reasonably unique value that is used to identify all fragments belonging to the same uplink message. All fragments should share the same token value */
        uint8_t                 total_fragments;  /*!< Total number of fragments for the current uplink message */
        uint8_t                 current_fragment; /*!< Index of the current fragment */
    } header;
    __PACKED_UNION {
        uint8_t                 raw[1];
        sid_uplink_tlv_record_t tlv_record[1];
    } tlv_payload; /*!< Uplink message payload (collection of TLV records). The actual amount of raw bytes or TLV records is variable */
} sid_uplink_message_t;

/* Private variables ---------------------------------------------------------*/

static osMessageQueueId_t g_event_queue;

static osSemaphoreId_t sidewalk_registration_ready_semaphore;
static osSemaphoreId_t sidewalk_connection_ready_semaphore;
static osSemaphoreId_t sidewalk_connection_stopped_semaphore;
static osSemaphoreId_t sidewalk_message_delivered_semaphore;

static osSemaphoreId_t teseo_gnss_fix_semaphore;

static osThreadId_t geolocation_demo_task;

static uint8_t geodata_serialization_buffer[256]; /*!< A working buffer to build the serialized payload from application data */

/* Indicates if Sidewalk registration process is pending */
static bool registration_pending = false;

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

static const osSemaphoreAttr_t teseo_gnss_fix_sem_attributes = {
    .name       = "Teseo GNSS Fix Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t sidewalk_registration_ready_sem_attributes = {
    .name       = "Sidewalk Registration Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t sidewalk_connection_ready_sem_attributes = {
    .name       = "Sidewalk Conn Ready Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t sidewalk_connection_stopped_sem_attributes = {
    .name       = "Sidewalk Conn Stop Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t sidewalk_msg_delivered_sem_attributes = {
    .name       = "Sidewalk Msg Delivered Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osThreadAttr_t geolocation_demo_task_attributes = {
    .name         = "Geolocation Demo Task",
    .priority     = GEOLOCATION_DEMO_TASK_PRIO,
    .stack_size   = GEOLOCATION_DEMO_TASK_STACK_SIZE,
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

static sid_error_t  send_sidewalk_uplink(app_context_t * const app_context, const uint8_t * const data, const size_t data_size);
static sid_error_t  serialize_gnss_position_data(const teseo_gnss_position_data_t * const gnss_position, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len);
static sid_error_t  serialize_demo_counter_data(const uint8_t counter_value, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len);
static sid_error_t  serialize_mcu_temperature_data(const int16_t temperature_value, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len);

static void         factory_reset(app_context_t * const context);
static void         print_subghz_link_profile(const app_context_t * const context);
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
static void         adjust_sidewalk_link_log_level(const uint32_t link_mask);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
static sid_error_t  init_link_and_check_sid_registration(app_context_t * const context, struct sid_config * const config);

static void         on_gnss_fix_obtained(void * user_arg);

static void         sidewalk_stack_task_entry(void *context);
static void         geolocation_demo_task_entry(void *context);

static const char * McuRevName(const uint32_t rev_id);
#if SID_PAL_LOG_ENABLED
static const char * sid_subghz_profile_code_to_str(enum sid_device_profile_id code);
#endif /* SID_PAL_LOG_ENABLED */
#if (CFG_BUTTON_SUPPORTED == 1)
static void         toggle_teseo_log_mirroring(void);
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
    osStatus_t os_status = osMessageQueuePut(queue, &event, 0u, 0u);

    if (os_status != osOK)
    {
        SID_PAL_LOG_ERROR("Failed to queue event %u. Error %d", (uint32_t)event, (int32_t)os_status);
    }
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_event(bool in_isr, void *context)
{
    (void)in_isr;

    app_context_t *app_context = (app_context_t *)context;
    queue_event(app_context->event_queue, EVENT_TYPE_SIDEWALK_PROCESS);
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context)
{
    if ((SID_MSG_TYPE_RESPONSE == msg_desc->type) && (msg_desc->msg_desc_attr.rx_attr.is_msg_ack != false))
    {
        SID_PAL_LOG_INFO("Received Sidewalk ACK for message ID %u (link_mode: %d)", msg_desc->id, (int32_t)msg_desc->link_mode);
        osSemaphoreRelease(sidewalk_message_delivered_semaphore);
    }
    else
    {
        SID_PAL_LOG_INFO("Received Sidewalk downlink (type: %d, link_mode: %d, id: %u size %u)", (int32_t)msg_desc->type,
                                 (int32_t)msg_desc->link_mode, msg_desc->id, msg->size);
        SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, msg->data, msg->size);
    }
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_RCV_OK);
#endif /* CFG_LED_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_INFO("Sent Sidewalk uplink (type: %d, id: %u)", (int32_t)msg_desc->type, msg_desc->id);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SENT_OK);
#endif /* CFG_LED_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_ERROR("Failed to send Sidewalk uplink (type: %d, id: %u), err:%d",
                  (int32_t)msg_desc->type, msg_desc->id, (int32_t)error);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SEND_ERROR);
#endif /* CFG_LED_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_status_changed(const struct sid_status *status, void *context)
{
    app_context_t *app_context = (app_context_t *)context;
    SID_PAL_LOG_INFO("Sidewalk status changed: %d", (int32_t)status->state);
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
                osSemaphoreRelease(sidewalk_connection_ready_semaphore);
            }
            break;

        case SID_STATE_NOT_READY:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_BONDING);
#endif /* CFG_LED_SUPPORTED */
            app_context->state = STATE_SIDEWALK_NOT_READY;
            osSemaphoreRelease(sidewalk_connection_stopped_semaphore);
            break;

        case SID_STATE_ERROR:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_ERROR);
#endif /* CFG_LED_SUPPORTED */
            SID_PAL_LOG_ERROR("sidewalk error: %d", (int32_t)sid_get_error(app_context->sidewalk_handle));
            SID_PAL_ASSERT(0);
            break;

        case SID_STATE_SECURE_CHANNEL_READY:
            app_context->state = STATE_SIDEWALK_SECURE_CONNECTION;
            break;

        default:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_ERROR);
#endif /* CFG_LED_SUPPORTED */
            SID_PAL_LOG_ERROR("Unknown Sidewalk state received: %d", (int32_t)status->state);
            SID_PAL_ASSERT(0);
            break;
    }

    SID_PAL_LOG_INFO("Registration Status = %d, Time Sync Status = %d and Link Status Mask = %x",
                 status->detail.registration_status, status->detail.time_sync_status,
                 status->detail.link_status_mask);

    if ((registration_pending != false) && (SID_STATUS_REGISTERED == status->detail.registration_status))
    {
        /* Device registration completed */
        SID_PAL_LOG_INFO("Sidewalk Device Registration done");
        registration_pending = false;
        queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_REGISTRATION_COMPLETED);
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

    SID_PAL_LOG_INFO("factory reset notification received from sid api");
    (void)sid_hal_reset(SID_HAL_RESET_NORMAL);
}

/*----------------------------------------------------------------------------*/

static sid_error_t send_sidewalk_uplink(app_context_t * const app_context, const uint8_t * const data, const size_t data_size)
{
    sid_error_t ret;

    do
    {
        size_t mtu;

        /* Ensure Sidewalk link is good */
        if ((app_context->state != STATE_SIDEWALK_READY) &&
            (app_context->state != STATE_SIDEWALK_SECURE_CONNECTION))
        {
            SID_PAL_LOG_ERROR("Can't send Sidewalk uplink, Sidewalk connection is not ready");
            ret = SID_ERROR_PORT_NOT_OPEN;
            break;
        }

        /* Query Sidewalk MTU size */
        ret = sid_get_mtu(app_context->sidewalk_handle, app_context->link_status.link_mask, &mtu);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Can't get MTU size for Sidewalk link %u. Error %d", app_context->link_status.link_mask, (int32_t)ret);
            break;
        }

        /* Generate uplink token */
        sid_uplink_token_t uplink_token;
        ret = sid_pal_crypto_rand((uint8_t *)(void *)&uplink_token, sizeof(uplink_token));
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Can't generate token for Sidewalk uplink. Error %d", (int32_t)ret);
            break;
        }
        static_assert(sizeof(sid_uplink_token_t) == sizeof(uint16_t));
        const sid_uplink_token_t uplink_token_big_endian = SID_STM32_UTIL_SWAP_BYTES_16(uplink_token); /* Token should be sent as Big Endian value */

        /* Calculate the number of fragments */
        const uint32_t fragment_mtu = mtu - SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_uplink_message_t, header); /* This is how many bytes of user data we can put into a single fragment */
        const uint32_t total_fragments =  (data_size + (fragment_mtu - 1u)) / fragment_mtu; /* Use math ceiling */
        SID_PAL_LOG_INFO("Sending %u bytes of app data in %u fragment(s) with token 0x%04X. Sidewalk MTU: %u", data_size, total_fragments, uplink_token, mtu);

        const uint8_t * data_ptr = data;
        size_t remaining_data_bytes = data_size;
        for (uint32_t i = 0u; i < total_fragments; i++)
        {
            uint8_t                sendout_buf[SID_MAX_MTU];
            size_t                 sendout_len = SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_uplink_message_t, header);
            sid_uplink_message_t * uplink_msg = (sid_uplink_message_t *)(void *)sendout_buf;

            /* Populate uplink message header */
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Warray-bounds"
            uplink_msg->header.token            = uplink_token_big_endian;
            uplink_msg->header.total_fragments  = (uint8_t)total_fragments;
            uplink_msg->header.current_fragment = (uint8_t)(i + 1u);

            /* Copy portion of the app data */
            size_t bytes_to_copy = (remaining_data_bytes > fragment_mtu) ? fragment_mtu : remaining_data_bytes;
            SID_STM32_UTIL_fast_memcpy(&uplink_msg->tlv_payload.raw[0], data_ptr, bytes_to_copy);
            #pragma GCC diagnostic pop

            /* Adjust pointers and counters */
            remaining_data_bytes -= bytes_to_copy;
            data_ptr             += bytes_to_copy;
            sendout_len          += bytes_to_copy;

            /* Prepare Sidewalk message descriptor */
            struct sid_msg msg = {
                .data = sendout_buf,
                .size = sendout_len,
            };
            struct sid_msg_desc desc = {
                .type = SID_MSG_TYPE_NOTIFY,
                .link_type = SID_LINK_TYPE_ANY,
                .link_mode = SID_LINK_MODE_CLOUD,
                /**
                 * Note: Whenever Sidewalk and LBM run concurrently it is highly recommended to send messages with acknowledgments
                 *       because the chances of data loss are significantly higher in this mode, especially when GNSS scan runs
                 *       concurrently as the radio is completely blocked during the scan. Selection of the message delivery attempts
                 *       and TTL is up to the end user and depends on the application and importance of the data. Sidewalk stack
                 *       will send out the message every (ttl_in_seconds / num_retries) seconds until either ACK is received or TTL
                 *       expires
                 */
                .msg_desc_attr = {
                    .tx_attr = {
                        .request_ack     = true,
                        .num_retries     = GEOLOCATION_DEMO_UPLINK_RETRIES,
                        .additional_attr = SID_MSG_DESC_TX_ADDITIONAL_ATTRIBUTES_NONE,
                        .ttl_in_seconds  = GEOLOCATION_DEMO_UPLINK_TTL,
                    },
                },
            };

            /* Adjust link mode for smartphone connections (if used) */
            if ((app_context->link_status.link_mask & SID_LINK_TYPE_1) &&
                (app_context->link_status.supported_link_mode[SID_LINK_TYPE_1_IDX] & SID_LINK_MODE_MOBILE))
            {
                desc.link_mode = SID_LINK_MODE_MOBILE;
            }

            /* Push the message to the Sidewalk stack queue */
            ret = sid_put_msg(app_context->sidewalk_handle, &msg, &desc);
            if (ret != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed queueing Sidewalk uplink fragment %u out of %u for token 0x%04X. Error: %d", (i + 1u), total_fragments, uplink_token, (int32_t)ret);
                break;
            }

            SID_PAL_LOG_INFO("Queued Sidewalk uplink fragment %u out of %u for token 0x%04X, message ID: %u", (i + 1u), total_fragments, uplink_token, desc.id);
        }

        /* Terminate if the for loop above failed */
        if (ret != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        ret = SID_ERROR_NONE;
    } while (0);

#if CFG_LED_SUPPORTED
    if (SID_ERROR_NONE == ret)
    {
        (void)led_indication_set(LED_INDICATE_SEND_ENQUEUED);
    }
    else
    {
        (void)led_indication_set(LED_INDICATE_SEND_ERROR);
    }
#endif /* CFG_LED_SUPPORTED */

    return ret;
}

/*----------------------------------------------------------------------------*/

static sid_error_t serialize_gnss_position_data(const teseo_gnss_position_data_t * const gnss_position, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len)
{
    sid_error_t ret;

    /**
     * Serialized payload structure:
     * vvv Universal TLV Header vvvvvvvvvvvv
     * Byte  0: tag (GNSS Position Data)
     * Byte  1: payload length (excluding Tag and Payload Length bytes)
     * ^^^ End of Universal TLV Header ^^^^^
     * vvv Position Data (Payload) vvvvvvvvvvvvv
     * Bytes 2-5: UTC Timestamp (Big Endian) - Unix epoch seconds
     * Bytes 6-9: Latitude in degrees (32-bit float, Big Endian)
     * Bytes 10-13: Longitude in degrees (32-bit float, Big Endian)
     * Bytes 14-17: Elevation in meters (32-bit float, Big Endian)
     * Bytes 18-21: Horizontal accuracy in meters - 1-sigma-based, corresponds to 0.68 confidence level (32-bit float, Big Endian)
     * Bytes 22-25: Vertical accuracy in meters - 1-sigma-based, corresponds to 0.68 confidence level (32-bit float, Big Endian)
     * ^^^ End of Scan Data (Payload) ^^^^^^
     */
    do
    {
        sid_uplink_tlv_record_t * serialized_payload = (sid_uplink_tlv_record_t *)(void *)buffer;

        /* Validate inputs */
        if ((NULL == gnss_position) || (NULL == buffer) || (0u == buffer_size) || (NULL == out_serialized_len))
        {
            ret = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Calculate the required buffer size and ensure we have enough space */
        const size_t serialized_data_length = sizeof(serialized_payload->header) + sizeof(sid_uplink_gnss_position_record_t);
        if (buffer_size < serialized_data_length)
        {
            ret = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Fill-in the header */
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Warray-bounds"
        #pragma GCC diagnostic ignored "-Wstrict-aliasing"
        serialized_payload->header.tag = GEOLOCATION_DEMO_TLV_TAG_GNSS_POSITION;
        serialized_payload->header.length = (uint8_t)(serialized_data_length - sizeof(serialized_payload->header));

        /* Populate data */
        sid_uplink_gnss_position_record_t * const serialized_gnss_position_data =  &serialized_payload->payload.gnss_position;
        serialized_gnss_position_data->timestamp                         = SID_STM32_UTIL_SWAP_BYTES_32(gnss_position->posix_time);
        *((uint32_t*)(void *)&serialized_gnss_position_data->latitude)   = SID_STM32_UTIL_SWAP_BYTES_32(*((uint32_t*)(void *)&gnss_position->latitude));
        *((uint32_t*)(void *)&serialized_gnss_position_data->longitude)  = SID_STM32_UTIL_SWAP_BYTES_32(*((uint32_t*)(void *)&gnss_position->longitude));
        *((uint32_t*)(void *)&serialized_gnss_position_data->elevation)  = SID_STM32_UTIL_SWAP_BYTES_32(*((uint32_t*)(void *)&gnss_position->elevation));
        *((uint32_t*)(void *)&serialized_gnss_position_data->h_accuracy) = SID_STM32_UTIL_SWAP_BYTES_32(*((uint32_t*)(void *)&gnss_position->horizontal_accuracy));
        *((uint32_t*)(void *)&serialized_gnss_position_data->v_accuracy) = SID_STM32_UTIL_SWAP_BYTES_32(*((uint32_t*)(void *)&gnss_position->vertical_accuracy));
        #pragma GCC diagnostic pop

        *out_serialized_len = serialized_data_length;
        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

static sid_error_t serialize_demo_counter_data(const uint8_t counter_value, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len)
{
    sid_error_t ret;

    /**
     * Serialized payload structure:
     * vvv Universal TLV Header vvvvvvvvvvvv
     * Byte  0: tag (Demo Counter)
     * Byte  1: payload length (excluding Tag and Payload Length bytes)
     * ^^^ End of Universal TLV Header ^^^^^
     * vvv Scan Data (Payload) vvvvvvvvvvvvv
     * Byte 2: Counter value reported by the application
     * ^^^ End of Scan Data (Payload) ^^^^^^
     *
     * Note: the number of the discovered WiFi networks can be calculated based on the overall payload length in TLV header and a fixed length of a single record (7 bytes)
     */
    do
    {
        sid_uplink_tlv_record_t * serialized_payload = (sid_uplink_tlv_record_t *)(void *)buffer;

        /* Validate inputs */
        if ((NULL == buffer) || (0u == buffer_size) || (NULL == out_serialized_len))
        {
            ret = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Calculate the required buffer size and ensure we have enough space */
        const size_t serialized_data_length = sizeof(serialized_payload->header) + sizeof(sid_uplink_demo_counter_record_t);
        if (buffer_size < serialized_data_length)
        {
            ret = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Fill-in the header */
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Warray-bounds"
        serialized_payload->header.tag = GEOLOCATION_DEMO_TLV_TAG_COUNTER;
        serialized_payload->header.length = (uint8_t)(serialized_data_length - sizeof(serialized_payload->header));

        /* Populate data */
        sid_uplink_demo_counter_record_t * const serialized_demo_counter_record = &serialized_payload->payload.demo_counter;
        serialized_demo_counter_record->counter_value = counter_value;
        #pragma GCC diagnostic pop

        *out_serialized_len = serialized_data_length;
        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

static sid_error_t serialize_mcu_temperature_data(const int16_t temperature_value, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len)
{
    sid_error_t ret;

    /**
     * Serialized payload structure:
     * vvv Universal TLV Header vvvvvvvvvvvv
     * Byte  0: tag (MCU Temperature)
     * Byte 1: payload length (excluding Tag and Payload Length bytes)
     * ^^^ End of Universal TLV Header ^^^^^
     * vvv MCu Temperature (Payload) vvvvvvvvvvvvv
     * Bytes 2-3: measured MCU temperature value (int16, Big Endian)
     * ^^^ End of Scan Data (Payload) ^^^^^^
     *
     * Note: the number of the discovered WiFi networks can be calculated based on the overall payload length in TLV header and a fixed length of a single record (7 bytes)
     */
    do
    {
        sid_uplink_tlv_record_t * serialized_payload = (sid_uplink_tlv_record_t *)(void *)buffer;

        /* Validate inputs */
        if ((NULL == buffer) || (0u == buffer_size) || (NULL == out_serialized_len))
        {
            ret = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Calculate the required buffer size and ensure we have enough space */
        const size_t serialized_data_length = sizeof(serialized_payload->header) + sizeof(sid_uplink_mcu_temperature_record_t);
        if (buffer_size < serialized_data_length)
        {
            ret = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Fill-in the header */
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Warray-bounds"
        serialized_payload->header.tag = GEOLOCATION_DEMO_TLV_TAG_MCU_TEMP;
        serialized_payload->header.length = (uint8_t)(serialized_data_length - sizeof(serialized_payload->header));

        /* Populate data */
        sid_uplink_mcu_temperature_record_t * const serialized_mcu_temperature_record = &serialized_payload->payload.mcu_temperature;
        serialized_mcu_temperature_record->temperature_value = SID_STM32_UTIL_SWAP_BYTES_16(temperature_value);
        #pragma GCC diagnostic pop

        *out_serialized_len = serialized_data_length;
        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
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

static sid_error_t init_link_and_check_sid_registration(app_context_t * const context, struct sid_config * const config)
{
    sid_error_t ret = SID_ERROR_GENERIC;
    struct sid_handle *sid_handle = NULL;
    struct sid_status sid_status;

    do
    {
        /* Validate inputs */
        if (NULL == context)
        {
            SID_PAL_LOG_ERROR("Sidewalk context cannot be null");
            ret = SID_ERROR_NULL_POINTER;
            break;
        }

        if (NULL == config)
        {
            SID_PAL_LOG_ERROR("Sidewalk config cannot be null");
            ret = SID_ERROR_NULL_POINTER;
            break;
        }

        /* Make sure Sidewalk is not initialized already */
        if (context->sidewalk_handle != NULL)
        {
            SID_PAL_LOG_ERROR("Sidewalk is initialized already");
            ret = SID_ERROR_ALREADY_INITIALIZED;
            break;
        }

        /* Initialize state indication to the user app */
        context->state = STATE_SIDEWALK_NOT_READY;

#if SID_FSK_ONLY_LINK
        /* For FSK-only mode start with FSK mode and handle the device registration over the FSK link */
        config->link_mask = SID_COMM_LINK_TYPE;
#else
        /* Initialize Sidewalk in BLE or FSK mode to check device registration status */
        /* WARNING: starting in LoRa-only mode is not allowed if the device is not registered. Calling sid_init() will always return an error */
        config->link_mask = SID_REGISTRATION_LINK_TYPE;
#endif /* SID_FSK_ONLY_LINK */
        ret = sid_init(config, &sid_handle);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to initialize Sidewalk stack, link_mask:%x, err:%d", (int32_t)config->link_mask, (int32_t)ret);
            break;
        }

        /* Register sidewalk handle to the application context */
        context->sidewalk_handle = sid_handle;

        ret = sid_get_status(sid_handle, &sid_status);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to get Sidewalk stack status, err:%d", (int32_t)ret);
            break;
        }

        SID_PAL_LOG_INFO("Sid registration status: %s", sid_status.detail.registration_status == SID_STATUS_REGISTERED ? "Registered" : "Not registered");

#if (SID_SDK_CONFIG_ENABLE_LINK_TYPE_2)
        /* Reduce log level for FSK connection since it produces excessive logs that affect communication timings */
        adjust_sidewalk_link_log_level(config->link_mask);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

        if (SID_STATUS_REGISTERED == sid_status.detail.registration_status)
        {
            /* Indicate to the app that Sidewalk device registration is completed */
            queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_REGISTRATION_COMPLETED);
        }
        else
        {
            /* Indicate registration process is pending */
            registration_pending = true;

            /* Start the Sidewalk stack with the selected link type for registration */
            ret = sid_start(sid_handle, config->link_mask);
            if (ret != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to start Sidewalk stack, link_mask:%x, err:%d", (int32_t)config->link_mask, (int32_t)ret);
                break;
            }
        }

        /* Done */
        ret = SID_ERROR_NONE;
    } while (0);

    if (ret != SID_ERROR_NONE)
    {
        context->sidewalk_handle = NULL;
        config->link_mask = 0;
    }

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

static void on_gnss_fix_obtained(void * user_arg)
{
    (void)user_arg;
    (void)osSemaphoreRelease(teseo_gnss_fix_semaphore);
}

/*----------------------------------------------------------------------------*/

#if (CFG_BUTTON_SUPPORTED == 1)
static void toggle_teseo_log_mirroring(void)
{
    sid_error_t err;
    uint8_t is_enabled;

    do
    {
        err = teseo_gnss_get_log_mirroring_status(&is_enabled);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        if (FALSE == is_enabled)
        {
            err = teseo_gnss_enable_log_mirroring();
            if (err != SID_ERROR_NONE)
            {
                break;
            }
            SID_PAL_LOG_INFO("Teseo messages logging enabled");
        }
        else
        {
            err = teseo_gnss_disable_log_mirroring();
            if (err != SID_ERROR_NONE)
            {
                break;
            }
            SID_PAL_LOG_INFO("Teseo messages logging disabled");
        }
    } while (0);
}

/*----------------------------------------------------------------------------*/

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
                queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_TOGGLE_LINK);
            }
            else
            {
                /* Long-press event */
                queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_FACTORY_RESET);
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
static void button2_irq_handler(uint32_t pin, void * callback_arg)
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
                queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_SET_DEVICE_PROFILE);
            }
            else
            {
                /* Long-press event */
#    if  defined(STM32WBA6x) && (defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X)) /* Semtech shields occupy B1 EXTI line for BUSY signal, use long B2 press instead */
                queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_TOGGLE_LINK);
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

            if((uEnd - uStart) >= (4u * OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS)))
            {
                /* Extra-long-press event */
                toggle_teseo_log_mirroring();
            }
            else if((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER);
            }
            else
            {
                /* Long-press event */
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
                queue_event(g_event_queue, EVENT_TYPE_ACTIVE_STANDBY);
#else
                toggle_teseo_log_mirroring();
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

static void buttons_init(void)
{
    sid_error_t ret_code;

#  if (defined(STM32WBA6x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
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
    SID_PAL_LOG_INFO("Sidewalk Teseo geolocation demo started");

    app_context_t *app_context = (app_context_t *)context;
    sid_error_t ret;
    osStatus_t os_status;

    struct sid_event_callbacks event_callbacks = {
        .context = app_context,
        .on_event = on_sidewalk_event,                   /* Called from ISR context */
        .on_msg_received = on_sidewalk_msg_received,     /* Called from sid_process() */
        .on_msg_sent = on_sidewalk_msg_sent,             /* Called from sid_process() */
        .on_send_error = on_sidewalk_send_error,         /* Called from sid_process() */
        .on_status_changed = on_sidewalk_status_changed, /* Called from sid_process() */
        .on_factory_reset = on_sidewalk_factory_reset,   /* Called from sid_process */
    };

    struct sid_config config = {
        .link_mask = 0,
        .callbacks = &event_callbacks,
        .link_config = app_get_sidewalk_ble_config(),
        .sub_ghz_link_config = app_get_sub_ghz_config(),
    };

    /* Performing a cold start of the Sidewalk since device registration status is unknown at this point */
    queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_INIT);

    while (1)
    {
        enum event_type event;

        if (osMessageQueueGet (app_context->event_queue, &event, 0u, osWaitForever) == osOK)
        {
            switch (event)
            {
                case EVENT_TYPE_SIDEWALK_INIT:
                    {
                        /* Performing a cold start of the Sidewalk since device registration status is unknown at this point */
                        if (NULL == app_context->sidewalk_handle)
                        {
                            ret = init_link_and_check_sid_registration(app_context, &config);
                            if (ret != SID_ERROR_NONE)
                            {
                                SID_PAL_LOG_ERROR("Failed to initialize Sidewalk. Error code: %d", (int32_t)ret);
                                goto error;
                            }
                        }
                        else
                        {
                            /* Skip the request, Sidewalk is already running */
                        }
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_START_LINK:
                    {
                        /* Start selected Sidewalk link */
                        ret = sid_start(app_context->sidewalk_handle, config.link_mask);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to start Sidewalk link, link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                            goto error;
                        }
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_STOP_LINK:
                    {
                        /* Start selected Sidewalk link */
                        ret = sid_stop(app_context->sidewalk_handle, config.link_mask);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to stop Sidewalk link, link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                            goto error;
                        }
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_TOGGLE_LINK:
                    if(app_context->sidewalk_handle == NULL)
                    {
                        queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_START_LINK);
                    }
                    else
                    {
                        queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_STOP_LINK);
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_PROCESS:
                    if ((NULL == app_context) || (NULL == app_context->sidewalk_handle))
                    {
                        /*  This may happen if Sidewalk link was terminated but the event queue already contained some events - just ignore them */
                        break;
                    }
                    ret = sid_process(app_context->sidewalk_handle);
                    if ((ret != SID_ERROR_NONE) && (ret != SID_ERROR_STOPPED))
                    {
                        SID_PAL_LOG_ERROR("Error processing Sidewalk event, err:%d", (int32_t)ret);
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

                case EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER:
                    SID_PAL_LOG_INFO("EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER");

                    if (app_context->state == STATE_SIDEWALK_READY)
                    {
                        size_t serialized_data_len = 0u;

                        SID_PAL_LOG_INFO("Sending counter update: %d", app_context->counter);
                        ret = serialize_demo_counter_data(app_context->counter, geodata_serialization_buffer, sizeof(geodata_serialization_buffer), &serialized_data_len);
                        if (ret != SID_ERROR_NONE)
                        {
                            break;
                        }
                        ret = send_sidewalk_uplink(app_context, geodata_serialization_buffer, serialized_data_len);
                        if (SID_ERROR_NONE == ret)
                        {
                            app_context->counter++;
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_WARNING("Can't send counter update - Sidewalk is not ready");;
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_SEND_GEOLOCATION_DATA:
                    SID_PAL_LOG_INFO("EVENT_TYPE_SIDEWALK_SEND_GEOLOCATION_DATA");

                    if (app_context->state == STATE_SIDEWALK_READY)
                    {
                        size_t serialized_data_len = 0u;
                        size_t remaining_buf_space = sizeof(geodata_serialization_buffer);
                        uint8_t * working_buf_ptr = geodata_serialization_buffer;

                        /* Run in a critical section to ensure scan data is not modified in the background */
                        sid_pal_enter_critical_region();
                        {
                            size_t serialized_mcu_temp_data_len;
                            ret = serialize_mcu_temperature_data(app_context->mcu_temperature, working_buf_ptr, remaining_buf_space, &serialized_mcu_temp_data_len);
                            if (SID_ERROR_NONE == ret)
                            {
                                SID_PAL_LOG_DEBUG("Serialized MCU temperature into %u bytes of payload", serialized_mcu_temp_data_len);
                                serialized_data_len += serialized_mcu_temp_data_len;
                                remaining_buf_space -= serialized_mcu_temp_data_len;
                                working_buf_ptr += serialized_mcu_temp_data_len;
                            }
                            else
                            {
                                SID_PAL_LOG_ERROR("Failed to serialize MCU temperature data, sending out MCU temperature data is skipped. Error %d", (int32_t)ret);
                                /* Don't terminate, we still need to leave the critical section */
                            }
                        }

                        {
                            size_t serialized_gnss_data_len;
                            ret = serialize_gnss_position_data(&app_context->gnss_position, working_buf_ptr, remaining_buf_space, &serialized_gnss_data_len);
                            if (SID_ERROR_NONE == ret)
                            {
                                SID_PAL_LOG_DEBUG("Serialized GNSS position data into %u bytes of payload", serialized_gnss_data_len);
                                serialized_data_len += serialized_gnss_data_len;
                                remaining_buf_space -= serialized_gnss_data_len;
                                working_buf_ptr += serialized_gnss_data_len;
                            }
                            else
                            {
                                SID_PAL_LOG_ERROR("Failed to serialize GNSS position data, sending out GNSS data is skipped. Error %d", (int32_t)ret);
                                /* Don't terminate, we still need to leave the critical section */
                            }
                        }
                        sid_pal_exit_critical_region();

                        if (0u == serialized_data_len)
                        {
                            /* Terminate if serialization failed */
                            SID_PAL_LOG_ERROR("Failed to send out geolocation scan data - no serialized payload available");
                            break;
                        }

                        SID_PAL_LOG_INFO("Sending out %u bytes of aggregated geolocation data", serialized_data_len);
                        ret = send_sidewalk_uplink(app_context, geodata_serialization_buffer, serialized_data_len);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Geolocation update sendout failed. Error %d", (int32_t)ret);
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_WARNING("Can't send geolocation data - Sidewalk is not ready");;
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_SET_DEVICE_PROFILE:
                    {
                        SID_PAL_LOG_INFO("EVENT_TYPE_SIDEWALK_SET_DEVICE_PROFILE");
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

                case EVENT_TYPE_SIDEWALK_REGISTRATION_COMPLETED:
                    {
#if SID_FSK_ONLY_LINK
                        /* Stack re-initialization is not required, we are in the FSK mode already */
                        /* Stop any running links */
                        ret = sid_stop(app_context->sidewalk_handle, config.link_mask);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to stop Sidewalk link, link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                            goto error;
                        }
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
#endif /* SID_FSK_ONLY_LINK */

                        /* Indicate to the app that Sidewalk device registration is completed */
                        os_status = osSemaphoreRelease(sidewalk_registration_ready_semaphore);
                        if (os_status != osOK)
                        {
                            SID_PAL_LOG_ERROR("Failed to indicate Sidewalk registration completion. Error code : %d", (int32_t)os_status);
                            goto error;
                        }
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_FACTORY_RESET:
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
                    if (geolocation_demo_task != NULL)
                    {
                        (void)osThreadSuspend(geolocation_demo_task);
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
    if (geolocation_demo_task != NULL)
    {
        (void)osThreadTerminate(geolocation_demo_task);
        geolocation_demo_task = NULL;
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
    SID_PAL_LOG_FLUSH();
    Error_Handler();

    osThreadExit(); /* Normally this line is not reachable, but keep it if Error_Handler somehow returns */
}

/*----------------------------------------------------------------------------*/

static void geolocation_demo_task_entry(void *context)
{
    app_context_t *          app_context = (app_context_t *)context;
    osStatus_t               os_status;
    sid_error_t              sid_err;

    /* 1. Initialize Teseo and let it run in the background ----------------------*/
    const teseo_gnss_device_config_t * teseo_driver_config = get_teseo_cfg();
    teseo_gnss_event_callbacks_t teseo_event_callbacks = {
        .on_gnss_fix_obtained = on_gnss_fix_obtained,
    };
    sid_err = teseo_gnss_init(teseo_driver_config, &teseo_event_callbacks);
    if (sid_err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Geolocation Demo Thread - failed to initialize Teseo driver");
        goto error;
    }
    /*----------------------------------------------------------------------------*/

    /* 2. Wait for Sidewalk device registration process to be completed - this is required to ensure Sidewalk stack won't reset the radio anymore */
    if (osSemaphoreGetCount(sidewalk_registration_ready_semaphore) == 0u)
    {
        SID_PAL_LOG_INFO("Geolocation Demo: waiting for the Sidewalk registration to be completed...");
        os_status = osSemaphoreAcquire(sidewalk_registration_ready_semaphore, osWaitForever);
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Geolocation Demo Thread - failed to wait for Sidewalk device registration to be completed");
            goto error;
        }
    }
    SID_PAL_LOG_INFO("Geolocation Demo: Sidewalk device is registered, proceeding with the demo...");
    /*----------------------------------------------------------------------------*/

    /* 3. Establish initial Sidewalk connection ----------------------------------*/
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    /* For FSK link only: limit debug logging during Sidewalk FSK connection since the amount of message is large and it may affect radio timings */
    adjust_sidewalk_link_log_level(SID_COMM_LINK_TYPE);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

    /* Start the Sidewalk stack with the selected link type */
    queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_START_LINK);
    /*----------------------------------------------------------------------------*/

    /* 4. Wait for Sidewalk connection readiness ---------------------------------*/
    SID_PAL_LOG_INFO("Geolocation Demo: waiting for the Sidewalk connection...");
    os_status = osSemaphoreAcquire(sidewalk_connection_ready_semaphore, osWaitForever);
    if (os_status != osOK)
    {
        SID_PAL_LOG_ERROR("Geolocation Demo: Unable to establish Sidewalk connection. Error code: %d", os_status);
        goto error;
    }

    /* Send a test message to verify the device and Sidewalk link are both ok. Getting the geolocation data, especially from cold start, may take some time */
    SID_PAL_LOG_INFO("Geolocation Demo: send Demo Counter value to test Sidewalk link...");
    queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER);
    os_status = osSemaphoreAcquire(sidewalk_message_delivered_semaphore, osWaitForever);
    if (os_status != osOK)
    {
        SID_PAL_LOG_ERROR("Geolocation Demo: Unable to establish Sidewalk connection. Error code: %d", os_status);
        goto error;
    }
    SID_PAL_LOG_INFO("Geolocation Demo: Demo Counter value acknowledged by the cloud, proceeding to geolocation scan");
    /*----------------------------------------------------------------------------*/

    do
    {
        /* 5. Wait for GNSS fix availability and read position -----------------------*/
        uint8_t gnss_fix_valid;
        sid_err = teseo_gnss_is_gnss_fix_available(&gnss_fix_valid);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to retrieve GNSS fix status from Teseo driver. Error %d", (int32_t)sid_err);
            break;
        }

        if (FALSE == gnss_fix_valid)
        {
            SID_PAL_LOG_INFO("Geolocation Demo: Waiting for GNSS fix...");
            os_status = osSemaphoreAcquire(teseo_gnss_fix_semaphore, osWaitForever);
            if (os_status != osOK)
            {
                SID_PAL_LOG_ERROR("Geolocation Demo: failed to wait for GNSS fix. Error code: %d", os_status);
                break;
            }
        }
        SID_PAL_LOG_INFO("Geolocation Demo: GNSS fix is valid");

        sid_err = teseo_gnss_get_position(&app_context->gnss_position);
        if (SID_ERROR_NOT_FOUND == sid_err)
        {
            SID_PAL_LOG_WARNING("Geolocation Demo: GNSS fix lost, will wait for a new fix");
            continue;
        }
        else if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Geolocation Demo: failed read GNSS position. Error code: %d", (int32_t)sid_err);
            break;
        }
        else
        {
            /* Everything is fine */
        }

        SID_PAL_LOG_INFO("Geolocation Demo: Device position: %d.%07u, %d.%07u, elevation: %d.%02um",
            (int32_t)app_context->gnss_position.latitude,  abs((int32_t)(fmodf(app_context->gnss_position.latitude, 1.f) * 1e7f)), /* Have to do this because logging API uses tiny_vsnprintf and does not support %f format */
            (int32_t)app_context->gnss_position.longitude, abs((int32_t)(fmodf(app_context->gnss_position.longitude, 1.f) * 1e7f)),
            (int32_t)app_context->gnss_position.elevation, abs((int32_t)(fmodf(app_context->gnss_position.elevation, 1.f) * 1e2f))
        );
        /*----------------------------------------------------------------------------*/

        /* 6. Get MCU temperature to send it together with GNSS data ----------------*/
        app_context->mcu_temperature = sid_pal_temperature_get();
        SID_PAL_LOG_INFO("Geolocation Demo: Measured MCU temperature: %d", app_context->mcu_temperature);
        /*----------------------------------------------------------------------------*/

        /* 7. Send out geolocation scan results via Sidewalk link -------------------*/
        if (app_context->state != STATE_SIDEWALK_READY)
        {
            SID_PAL_LOG_INFO("Geolocation Demo: waiting for the Sidewalk connection...");
            os_status = osSemaphoreAcquire(sidewalk_connection_ready_semaphore, osWaitForever);
            if (os_status != osOK)
            {
                SID_PAL_LOG_ERROR("Geolocation Demo: unable to establish Sidewalk connection. Error code: %d", os_status);
                break;
            }
        }

        SID_PAL_LOG_INFO("Geolocation Demo: Sending out collected geolocation data");
        queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_SEND_GEOLOCATION_DATA);
        /*----------------------------------------------------------------------------*/

        /* 8. Idle time, do nothing till the next location update cycle -------------*/
        // TODO: for a production-ready solution it's better to explicitly check that all messages were sent rather than just waiting for a set delay
        SID_PAL_LOG_INFO("Geolocation Demo: Idling for %u seconds while Sidewalk stack processes uplinks", GEOLOCATION_DEMO_UPDATE_INTERVAL_S);
        os_status = osDelay(GEOLOCATION_DEMO_UPDATE_INTERVAL_S * 1000u);
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Geolocation Demo: Failed to idle. Error code: %d", os_status);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* New scan iteration starts from top */
        SID_PAL_LOG_INFO("Geolocation Demo: New location update cycle started");
    } while (1);

error:
    SID_PAL_LOG_INFO("Geolocation Demo: Terminated due to error");
    SID_PAL_LOG_FLUSH();
    Error_Handler();

    /* Normally the lines below are not reachable, but keep them if Error_Handler somehow returns */
    geolocation_demo_task = NULL;
    osThreadExit();
}

/*----------------------------------------------------------------------------*/

static const char * McuRevName(const uint32_t rev_id)
{
  const char * rev_name;

  switch (rev_id)
  {
    case REV_ID_A:
      rev_name = "Cut1.0";
      break;

    case REV_ID_B:
      rev_name = "Cut2.0";
      break;

    default:
      /* Unknown revision ID */
      rev_name = NULL;
      break;
  }

  return rev_name;
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
    SID_PAL_LOG_INFO("Using Sidewalk SDK version %u.%u.%u.%u", SID_SDK_MAJOR_VERSION, SID_SDK_MINOR_VERSION, SID_SDK_PATCH_VERSION, SID_SDK_BUILD_VERSION);
    SID_PAL_LOG_INFO("FreeRTOS Kernel: %s", tskKERNEL_VERSION_NUMBER);

    /* Printout MCU details */
    const uint32_t mcu_rev_id  = LL_DBGMCU_GetRevisionID();
    const char * const mcu_rev_name = McuRevName(mcu_rev_id);
    if (mcu_rev_name != NULL)
    {
        SID_PAL_LOG_INFO("Detected MCU rev: %s", mcu_rev_name);
    }
    else
    {
        SID_PAL_LOG_INFO("Detected MCU rev: 0x%04X", mcu_rev_id);
    }

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

    ret_code = sid_pal_temperature_init();
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk MCU temperature measurement init err: %d", ret_code);
        SID_PAL_ASSERT(0);
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

    /* Create a semaphore to monitor readiness of GNSS scan results */
    teseo_gnss_fix_semaphore = osSemaphoreNew(1u, 0u, &teseo_gnss_fix_sem_attributes);
    if (NULL == teseo_gnss_fix_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create GNSS Fix Readiness semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to monitor Sidewalk registration status */
    sidewalk_registration_ready_semaphore = osSemaphoreNew(1u, 0u, &sidewalk_registration_ready_sem_attributes);
    if (NULL == sidewalk_registration_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk Registration Status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to monitor Sidewalk connection status */
    sidewalk_connection_ready_semaphore = osSemaphoreNew(1u, 0u, &sidewalk_connection_ready_sem_attributes);
    if (NULL == sidewalk_connection_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk connection status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }
    sidewalk_connection_stopped_semaphore = osSemaphoreNew(1u, 0u, &sidewalk_connection_stopped_sem_attributes);
    if (NULL == sidewalk_connection_stopped_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk connection status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to control Sidewalk message delivery */
    sidewalk_message_delivered_semaphore = osSemaphoreNew(1u, 0u, &sidewalk_msg_delivered_sem_attributes);
    if (NULL == sidewalk_message_delivered_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk Message Delivered semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

   geolocation_demo_task = osThreadNew(geolocation_demo_task_entry, &app_context, &geolocation_demo_task_attributes);
   if (NULL == geolocation_demo_task)
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
