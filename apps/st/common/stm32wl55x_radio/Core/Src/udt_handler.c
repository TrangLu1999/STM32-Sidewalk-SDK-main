/**
  ******************************************************************************
  * @file    udt_handler.c
  * @brief   User Data Transfer handler for WL55 radio co-processor
  *          Overrides weak sid_host_comm_udt_user_init() to receive and log
  *          raw data from WBA55 host via SPI (opcode 0x19)
  ******************************************************************************
  */

#include <stdint.h>
#include "host_comm.h"
#include "sid_pal_log_like.h"

/* Private function prototypes -----------------------------------------------*/
static void _on_incoming_user_data_cb(const uint8_t * const data, const uint32_t data_len, void * user_arg);

/* Global function definitions -----------------------------------------------*/

/**
 * @brief Override weak function from host_comm.c
 *        Called during WL55 initialization when UDT is enabled
 */
sid_host_comm_error_t sid_host_comm_udt_user_init(void)
{
    sid_host_comm_error_t err;

    err = sid_host_comm_set_user_data_received_cb(_on_incoming_user_data_cb, NULL);
    if (err != SID_HOST_COMM_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to register UDT callback, err:%u", (uint32_t)err);
        return err;
    }

    SID_PAL_LOG_INFO("UDT initialized on WL55");
    return SID_HOST_COMM_ERROR_NONE;
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief Callback invoked when user data arrives from WBA55 via SPI
 * @note  Called from radio driver context - keep processing minimal
 */
static void _on_incoming_user_data_cb(const uint8_t * const data, const uint32_t data_len, void * user_arg)
{
    (void)user_arg;

    if ((data != NULL) && (data_len > 0u))
    {
        /* Log received data as readable string */
        SID_PAL_LOG_INFO("UDT from WBA55 (%u bytes): %.*s", data_len, data_len, (const char *)data);

        /* Echo data back to WBA55 */
        sid_host_comm_send_user_data(data, data_len, 0);
    }
}
