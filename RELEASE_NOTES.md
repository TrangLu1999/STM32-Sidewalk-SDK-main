
# Release Notes

The STM32-Sidewalk-SDK is an integration of the Sidewalk SDK v1.19.4.20 provided by Amazon, specifically designed to run on the STM32WBAxx MCUs. This package offers a comprehensive set of tools and services to facilitate the development of Amazon Sidewalk applications. See [README.md](README.md) file for more details.

## [V1.1.0] - 2025-12-08

### Main changes

- Upgraded to Sidewalk SDK v1.19.4.20
- Sidewalk Location functionality support (limited to BLE mode for non-LR11xx radio platforms)
- Support for basic and advanced geolocation scenarios using ST's Teseo [LIV3x](https://www.st.com/en/positioning/teseo-liv3f.html) and [VIC3x](https://www.st.com/en/positioning/teseo-vic3da.html) GNSS receivers. Full integration with the Sidewalk stack and full support for low power operation. Teseo driver brings in the ease of configuration and operation via high level API functions and event callbacks while all the heavy lifting of NMEA protocol parsing and Teseo UART commands handling is encapsulated in the driver.
- Standby with Retention LPM mode is now supported with active Sidewalk connections, allowing ultra-low power consumption while staying connected to the network
- Migrated to [LittleFS v2.11.1](https://github.com/littlefs-project/littlefs/releases/tag/v2.11.1) (backward-compatible with existing LFS/KV storage format)
- Universal, easy-to-use UART driver featuring continuous Rx via a ring buffer, automatic frame start and end detection, built-in error handling and recovery, and support for autonomous operation in low-power modes, allowing high power efficiency without a risk of loosing data when CPU core is not operational
- BLE:
  - Support for external PAs and LNAs for BLE
  - BLE antenna gain, Tx and Rx paths loss/gain are now automatically accounted for by the driver when configuring Tx power or reading RSSI, allowing to use EIRP values for power configuration and providing more accurate RSSI readings
  - BLE Tx power can now be adjusted dynamically in runtime for both Sidewalk and user mode
  - Added support for user-defined dynamic configuration of Sidewalk advertisement and connection parameters (new feature of Sidewalk SDK v1.19)
- STM32WLxx Radio Platform:
  - STM32WBAxx-STM32WLxx inter-MCU protocol updated to v1.1.0 to include new features and additional command parameters. Protocol v1.1.0 is backward compatible with the previous v1.0.x
  - Added user-defined callbacks to perform inter-MCU communication protocol and application version compatibility checks for STM32WLxx radio platform. If user app does not define compatibility check methods, the radio driver ignores app version and reports a warning on any mismatch of inter-MCU protocol versions between host MCU and STM32WLxx. By default, a warning does not prevent inter-MCU communication, allowing to proceed with inter-MCU data exchange (e.g., OTA)
  - Additional `sid_diagnostics` features added to STM32WLxx platform: instant RSSI, continuous preamble Tx, channel noise detection, channel free check, LoRa CAD, Rx Duty Cycle mode

### Improvements
- Added startup validity check for MFG storage. The stack will now report an error on boot if manufacturing data is empty or corrupted
- sid_pal_delay module now supports RTOS kernel tick frequencies other than 1000Hz
- sid_hal_reset now ensures ICACHE is invalidated and all flash write operations are finished before triggering the NVIC MCU reset
- sid_hal_reset now ensures logs are fully printed out before the reset
- LoRa Basics Modem middleware modified to excluded all LoRaWAN-related code when LoRaWAN is not used. This reduces flash footprint by about 57KB and RAM footprint by 4KB
- SMPS enabled on supported MCUs (WBA55, WBA6x)
- LPM wakeup scheduling improved to achieve consistent RF timings for BLE and sub-GHz links
- STM32WLxx User Data Transfer (UDT) callbacks now include user-defined argument
- STM32WLxx User Data Transfer (UDT) state polling replaced with task notifications, allowing finer control over LPM and longer uninterrupted sleep times

### Fixes
- Fixed duplicated flash page erase when uncorrectable ECC error is detected in BLE NVM area (SNVMA module)
- Setting radio frequency in 900-902MHz range on SX126x, LR11xx, and STM32WLxx resulted in applying calibration image for 868MHz band
- Entering Standby LPM could fail if BLE stack was never initialized
- Sporadic SPI Rx buffer overrun errors on STM32WLxx
- STM32WLxx SPI Rx could fail if initiated while the MCU was in Stop 1 low power mode

### Content

Refer to Release Notes of [STM32CubeWBA v1.7.0](https://github.com/STMicroelectronics/STM32CubeWBA/tree/v1.7.0) and  [STM32CubeWL v1.3.1](https://github.com/STMicroelectronics/STM32CubeWL/tree/v1.3.1) packages for details of Components (Drivers, Cortex-M CMSIS, STM32 CMSIS, STM32WBAxx_HAL_Driver and STM32WLxx_HAL_Driver, STM32WBAxx_Nucleo and STM32WLxx_Nucleo, Middlewares, Utilities) and to the [LICENSE.md](LICENSE.md) file for the complete SBOM.

### Development Toolchains and Compilers

- STM32Cube IDE 1.18.0 + ST-Link

### Supported Devices and Boards

- STM32WBAxx MCU
  - [NUCLEO-WBA65RI](https://www.st.com/en/evaluation-tools/nucleo-wba65ri.html) board
  - [NUCLEO-WBA55CG](https://www.st.com/en/evaluation-tools/nucleo-wba55cg.html) board

- STM32WLxx MCU
  - [NUCLEO-WL55CJ1](https://www.st.com/en/evaluation-tools/nucleo-wl55jc.html) board

- S2-LP radio
  - [X-NUCLEO-S2915A1](https://www.st.com/en/evaluation-tools/x-nucleo-s2915a1.html) expansion board

- Semtech SX126x and LR11xx transceivers
  - [SX1262MB2CAS](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262mb2cas) board
  - [LR1110MB1LCKS](https://www.semtech.com/products/wireless-rf/lora-edge/lr1110mb1lcks) board

- Teseo GNSS receivers
  - [X-NUCLEO-GNSS1A1](https://www.st.com/en/evaluation-tools/x-nucleo-gnss1a1.html) board
  - [X-NUCLEO-GNSS2A1](https://www.st.com/en/evaluation-tools/x-nucleo-gnss2a1.html) board

### Known Limitations

- Teseo VIC3x (X-NUCLEO-GNSS2A1) support currently does not cover dead reckoning scenarios.

## [V1.0.0] - 2025-08-21

### Main changes

First release of STM32WBA-Sidewalk-SDK, based on official STM32CubeWBA v1.7.0 and STM32CubeWL v1.3.1 packages.

### Content

Refer to Release Notes of [STM32CubeWBA v1.7.0](https://github.com/STMicroelectronics/STM32CubeWBA/tree/v1.7.0) and  [STM32CubeWL v1.3.1](https://github.com/STMicroelectronics/STM32CubeWL/tree/v1.3.1) packages for details of Components (Drivers, Cortex-M CMSIS, STM32 CMSIS, STM32WBAxx_HAL_Driver and STM32WLxx_HAL_Driver, STM32WBAxx_Nucleo and STM32WLxx_Nucleo, Middlewares, Utilities) and to the [LICENSE.md](LICENSE.md) file for the complete SBOM.

### Development Toolchains and Compilers

- STM32Cube IDE 1.18.0 + ST-Link

### Supported Devices and Boards

- STM32WBA5x MCU
  - NUCLEO-WBA65RI board
  - NUCLEO-WBA55CG board

- STM32WLxx MCU
  - NUCLEO-WL55CJ1 board

- S2-LP radio
  - X-NUCLEO-S2915A1 expansion board

- Semtech SX126x and LR11xx transceivers
  - SX1262MB2CAS board
  - LR1110MB1LCKS board

### Known Limitations

None
