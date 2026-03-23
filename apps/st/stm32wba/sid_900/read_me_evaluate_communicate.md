# SID_900 Evaluate Communication: STM32G0B1 → STM32WBA55 → STM32WL55JC

## Architecture

```
STM32G0B1 (Sensor MCU)
    │
    │ UART2 (PB8=RX, PB0=TX, 115200, 8N1, DMA, 20-byte ping-pong buffer)
    ▼
STM32WBA55 (Host MCU - Sidewalk + BLE)
    │
    │ SPI1 (UDT opcode 0x19, bypass Sidewalk registration)
    ▼
STM32WL55JC (Radio Co-processor - LoRa SubGHz)
```

## Command Protocol (G0B1 → WBA55)

Format: `[CMD_ID (1B)] [LEN (1B)] [PAYLOAD (max 18B)]` — always send **20 bytes total** (pad with 0x00).

| CMD_ID | Name             | Payload     | WBA55 Action                          |
|--------|------------------|-------------|---------------------------------------|
| 0x01   | CMD_INIT_LORA    | none        | Cold start Sidewalk (ignored if already running) |
| 0x02   | CMD_SEND_DATA    | data bytes  | Forward payload to WL55 via UDT/SPI   |
| 0x03   | CMD_STOP_LINK    | none        | Stop Sidewalk link                     |
| 0x04   | CMD_GET_STATUS   | none        | Log current state to USART1            |

### G0B1 Example Code

```c
// Wait for WBA55 to fully boot (~5s after reset)
HAL_Delay(5000);

// Send "Hello" to WL55 via WBA55
uint8_t cmd_send[20] = {0x02, 0x05, 'H', 'e', 'l', 'l', 'o'};
HAL_UART_Transmit(&huart2, cmd_send, 20, 100);

// Check status
uint8_t cmd_status[20] = {0x04, 0x00};
HAL_UART_Transmit(&huart2, cmd_status, 20, 100);
```

---

## All Changes from Original SDK

### 1. NVIC Priority Fix (FreeRTOS safety)

**Problem:** GPDMA1_Channel3 (UART2 RX DMA) had priority 1, violating FreeRTOS `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` = 5.
Caused debug target crash: "Target is not responding, retrying... GdbSession, Lost target connection".

**Files changed:**

| File | Change |
|------|--------|
| `Core/Src/main.c` line ~319 | `HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 1, 0)` → **6, 0** |
| `Core/Src/main.c` MX_USART2_UART_ReInit() | `NVIC_EncodePriority(..., 0, 0)` → **6, 0** |

---

### 2. Low Power Mode Disabled

**Problem:** Stop mode shuts down UART2/DMA clocks → cannot receive data.

| File | Change | Original |
|------|--------|----------|
| `Config/FreeRTOSConfig.h` | `configUSE_TICKLESS_IDLE` = **0** | 2 |
| `Config/app_conf.h` | `CFG_LPM_LEVEL` = **0** | (was enabled) |

**Note:** For production, re-enable LPM and configure `HAL_UARTEx_EnableStopMode()` with wakeup on start bit.

---

### 3. FreeRTOS Heap Size Increase

**Problem:** UDT task + message queue need ~1.5KB additional heap. `osThreadNew()` failed with "No memory".

| File | Change | Original |
|------|--------|----------|
| `Config/FreeRTOSConfig.h` | `configTOTAL_HEAP_SIZE` = **30400** | 28400 |

---

### 4. UART2 DMA → Sidewalk Event Bridge

**Problem:** Need to pass UART2 data from DMA ISR context to Sidewalk FreeRTOS task.

**Pattern:** ISR → `osMessageQueuePut()` (ISR-safe) → task processes event.

| File | Change |
|------|--------|
| `Core/Src/main.c` | Added `#include "app_sidewalk.h"` |
| `Core/Src/main.c` USART1_GPDMA1_ReceiveComplete_Callback() | Save completed buffer pointer, call `app_sidewalk_forward_uart2_data()` |
| `STM32_WPAN/App/app_sidewalk.h` | Added `void app_sidewalk_forward_uart2_data(const uint8_t *data, uint8_t len)` |
| `STM32_WPAN/App/app_sidewalk.c` | Added `EVENT_TYPE_UART2_DATA` enum, `uart2_tx_buffer[20]`, `uart2_tx_len` |
| `STM32_WPAN/App/app_sidewalk.c` | Added `parse_uart2_command()` — dispatches CMD_INIT_LORA, CMD_SEND_DATA, etc. |
| `STM32_WPAN/App/app_sidewalk.c` | Added `app_sidewalk_forward_uart2_data()` — copies data + queues event (ISR-safe) |

---

### 5. User Data Transfer (UDT) — WBA55 ↔ WL55 Raw SPI Data

**Problem:** `sid_put_msg()` requires Sidewalk registration (BLE + phone app). UDT sends raw data via SPI opcode 0x19, no registration needed.

**Reference implementation:** `apps/st/stm32wba/sid_dut_udt/`

#### WBA55 side (ble_mcu)

| File | Change |
|------|--------|
| `STM32CubeIDE/STM32WBA55_STM32WL55/ble_mcu/.cproject` | Added `STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER=1` to all build configs |
| `STM32_WPAN/App/app_sidewalk.c` | Added `#include <stm32wlxx_app_radio_ext_ifc.h>` |
| `STM32_WPAN/App/app_sidewalk.c` send_uart2_data() | Replaced `sid_put_msg()` with `sid_pal_radio_stm32wlxx_send_user_data()` |
| `STM32_WPAN/App/app_sidewalk.c` | Added `_on_wl55_user_data()` callback for data from WL55 |
| `STM32_WPAN/App/app_sidewalk.c` SID_APP_Init() | Register UDT RX callback via `sid_pal_radio_stm32wlxx_set_user_data_received_cb()` |

#### WL55 side (subghz_mcu)

| File | Change |
|------|--------|
| `STM32CubeIDE/STM32WBA55_STM32WL55/subghz_mcu/.cproject` | Added `STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER=1` to all build configs |
| `apps/st/common/stm32wl55x_radio/Core/Src/udt_handler.c` | **NEW FILE** — overrides weak `sid_host_comm_udt_user_init()`, logs received data + echoes back |

**Important:** After changing `.cproject` defines, you MUST do a **Clean Build** (delete Debug_* folder or Project → Clean).

---

### 6. Force SubGHz Radio Init During Cold Start

**Problem:** Cold start only initialized BLE link (`SID_REGISTRATION_LINK_TYPE = SID_LINK_TYPE_1`).
STM32WL55 radio PAL (`sid_pal_radio_init()`) only runs when SubGHz link is active.
Without it, UDT returns `SID_ERROR_INVALID_STATE (-40)` because `drv_ctx->init_done == FALSE`.

| File | Change | Original |
|------|--------|----------|
| `STM32_WPAN/App/app_sidewalk.c` init_and_start_link() | `config->link_mask = SID_REGISTRATION_LINK_TYPE \| SID_COMM_LINK_TYPE` | `SID_REGISTRATION_LINK_TYPE` only |

---

### 7. GPIO Pin Conflict Fix

**Problem:** PB8 is used for UART2_RX (AF3) but also configured as radio status LED (GPIO output) by LoRa radio driver during init. This reconfigures PB8 from alternate function to GPIO output → UART2 DMA stops working.

| File | Change | Original |
|------|--------|----------|
| `STM32_WPAN/App/app_900_config_stm32wlxx_app.c` | `tx_led` = `HALO_GPIO_NOT_CONNECTED` | `GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_8)` |
| `STM32_WPAN/App/app_900_config_stm32wlxx_app.c` | `rx_led` = `HALO_GPIO_NOT_CONNECTED` | `GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_8)` |

**Known pin conflicts on NUCLEO-WBA55 + NUCLEO-WL55:**

| Pin | UART2 Function | Conflict With |
|-----|---------------|---------------|
| PB8 | UART2_RX (AF3) | Radio TX/RX LED (app_900_config_stm32wlxx_app.c, app_900_config_sx126x.c) |
| PB0 | UART2_TX (AF3) | S2LP radio_shutdown (app_900_config_s2_lp.c) — not active with STM32WL55 |

---

## Data Flow

```
G0B1 sends 20 bytes: {0x02, 0x05, 'H', 'e', 'l', 'l', 'o', 0...}
    │
    ▼ UART2 DMA Transfer Complete ISR
GPDMA1_Channel3_IRQHandler()
    → USART1_GPDMA1_ReceiveComplete_Callback()         [main.c, ISR context]
        → save completed_buf pointer
        → ping-pong buffer switch (MX_USART2_UART_ReInit)
        → app_sidewalk_forward_uart2_data(buf, 20)      [app_sidewalk.c]
            → memcpy to uart2_tx_buffer
            → osMessageQueuePut(EVENT_TYPE_UART2_DATA)   [ISR-safe]
    │
    ▼ FreeRTOS Task context
sidewalk_stack_task_entry()
    → osMessageQueueGet(EVENT_TYPE_UART2_DATA)
    → parse_uart2_command()                              [app_sidewalk.c]
        → CMD 0x02: memmove payload, call send_uart2_data()
            → sid_pal_radio_stm32wlxx_send_user_data()  [UDT API]
    │
    ▼ SPI1 (opcode 0x19 USER_DATA)
STM32WL55 receives via host_comm.c
    → sid_radio_process_app_frame(OPCODE_USER_DATA)
    → _on_incoming_user_data_cb()                        [udt_handler.c]
        → SID_PAL_LOG_INFO("UDT from WBA55...")
        → sid_host_comm_send_user_data() (echo back)
    │
    ▼ SPI1 response
STM32WBA55 receives echo
    → _on_wl55_user_data()                               [app_sidewalk.c]
        → SID_PAL_LOG_INFO("UDT from WL55...")
```

---

## Build Instructions

### WL55 (subghz_mcu) — build and flash FIRST
1. Open `STM32CubeIDE/STM32WBA55_STM32WL55/subghz_mcu/`
2. Clean Build
3. Flash to NUCLEO-WL55JC

### WBA55 (ble_mcu) — build and flash SECOND
1. Open `STM32CubeIDE/STM32WBA55_STM32WL55/ble_mcu/`
2. Clean Build (important after .cproject changes)
3. Flash to NUCLEO-WBA55CG

### Debug UART Log Ports
- **WBA55:** USART1 (ST-Link VCP) — COM16
- **WL55:** LPUART1 (ST-Link VCP) — COM18
- **Baud:** 115200, 8N1

---

## Known Limitations

1. **DMA buffer fixed 20 bytes** — G0B1 must send exactly 20 bytes per command. For variable-length messages, implement UART IDLE line detection (`LL_USART_EnableIT_IDLE`).
2. **Low power disabled** — `configUSE_TICKLESS_IDLE=0`, `CFG_LPM_LEVEL=0`. For production, re-enable with UART wakeup configuration.
3. **Sidewalk not registered** — Data goes WBA55→WL55 via UDT (raw SPI), not through LoRa cloud. To send via Sidewalk cloud, complete BLE registration with Amazon Sidewalk phone app first.
4. **UDT max payload 256 bytes** — per `STM32WLxx_RADIO_COMM_USER_DATA_MAX_SIZE`.
5. **`%.*s` format not supported** — `tiny_printf` on this platform does not support `%.*s`. Use hexdump for data logging.
