# Teseo GNSS Module CLI Application Documentation

### Overview
This document provides a brief description of the available CLI commands to interact with Teseo units using the UART interface over Sidewalk platform.

### CLI commands usage and format
Arguments to the command are shown in <>
```
    reboot                                      - initiate host MCU reset (not the reset of the Teseo module)

    teseo init                                  - initialize Teseo driver and underlying peripherals (e.g., UART, GPIO)
                                                    This command invokes teseo_gnss_init() method and tries to detect the attached Teseo device

    teseo deinit                                - deinitialize Teseo driver and release all the associated hardware and software resources

    teseo reset <reset_type>                    - Resets the Teseo module only (not the host MCU)
                                                    No <reset_type> specified: hardware reset is performed by toggling nRESET pin of the Teseo module

                                                    <reset_type> specified: selects a certain type of reset:
                                                        --hard or -h: hardware reset via nRESET pin
                                                        --soft or -s: software reset via $PSTMSRR command
                                                        --gnss or -g: reset only the GNSS engine via the $PSTMGPSRESET command

    teseo version                               - read out firmware and hardware version information from Teseo device

    teseo echo <on|off>                         - Read or configure forwarding of raw Teseo messages to Sidewalk log console
                                                    If neither "on" nor "off is specified the command queries the current status of message forwarding
                                                    on: enable raw Teseo messages forwarding to Sidewalk log console
                                                    off: disable raw Teseo messages forwarding to Sidewalk log console

    teseo get_position                          - Gets the latest position data from Teseo module and prints it out into the log console. If no GNSS
                                                    fix is available an error will be reported

    teseo standby <duration>                    - Put Teseo into Software Standby mode via the $PSTMFROCESTANDBY command
                                                    No <duration> specified: Teseo will reside in the Standby state indefinitely util the wake-up pin is triggered

                                                    <duration> specified: Teseo will remain in the Standby mode for the specified amount of seconds. After that it will wake up automatically. Setting duration to zero puts Teseo into indefinite Standby state

                                                    Note: no commands will accepted after Teseo enters Standby state since its UART will be deactivated

    teseo wakeup                                - Wake up Teseo from Standby state by toggling the Wake-up pin

    teseo cmd "<cmd>"                           - Forwards a raw command string to the Teseo module directly
                                                    <cmd> must be a valid NMEA or PSTM command string, enclosed in quotation marks to be recognized as a single command parameter

                                                    Examples:
                                                        teseo cmd "$PSTMGETSWVER,255"
                                                        teseo cmd "$PSTMGETPAR,1201"

    teseo restore_factory_defaults              - Restore factory-defaults for all configuration settings
                                                    This command erases Teseo NVM and loads CDB with factory defaults
```
