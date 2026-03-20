# Command Line Interface for Teseo GNSS Modules

This app provides direct CLI access to Teseo modules, allowing manual configuration and full control over Teseo.

## Hardware Connections and Configuration

>[!NOTE]
>Any jumpers that are not mentioned explicitly are assumed to be placed in their default position as per the respective X-NUCLEO-GNSS board schematic.

### Configuring the X-NUCLEO-GNSS1A1 Board
#### STM32WL5x and S2-LP Radio

The following jumpers must be repositioned for this app to work properly:
- **J3 -> J2**: UART MCU TX Line (RX Line on Teseo side)
- **J4 -> J5**: UART MCU RX Line (Teseo TX line)
- **J9 -> J7**: Teseo Wakeup

#### SX126x Radio

The following jumpers must be repositioned for this app to work properly:
- **J3 -> J2**: UART MCU TX Line (RX Line on Teseo side)
- **J4 -> J5**: UART MCU RX Line (Teseo TX line)
- **J9 -> J7**: Teseo Wakeup
- **J13 -> J10**: Teseo Reset
- **For the NUCLEO-WBA55 board only**: put a jumper between D9 and D10 lines of the NUCLEO-WBA55 board (CN4.19 and CN4.17)

### Configuring the X-NUCLEO-GNSS2A1 Board
#### STM32WL5x and S2-LP Radio

The following jumpers must be repositioned for this app to work properly:
- **J28 -> bridge pins 1 & 2**: UART MCU RX Line (Teseo TX line)
- **J26 -> bridge pins 1 & 2**: Teseo Wakeup
- **J23 -> leave open**: Teseo SYS_FWD line
- **J24 -> leave open**: Teseo SYS_WHEELTICK line
- **J29 -> leave open**: Teseo SYS_PPS line
- **J30 -> leave open**: Teseo SYS_IRQ line

#### SX126x Radio

The following jumpers must be repositioned for this app to work properly:
- **J28 -> bridge pins 1 & 2**: UART MCU RX Line (Teseo TX line)
- **J26 -> bridge pins 1 & 2**: Teseo Wakeup
- **J25 -> bridge pins 2 & 3**: Teseo Reset
- **J23 -> leave open**: Teseo SYS_FWD line
- **J24 -> leave open**: Teseo SYS_WHEELTICK line
- **J29 -> leave open**: Teseo SYS_PPS line
- **J30 -> leave open**: Teseo SYS_IRQ line
- **For the NUCLEO-WBA55 board only**: put a jumper between D9 and D10 lines of the NUCLEO-WBA55 board (CN4.19 and CN4.17)

This configuration aims to resolve GPIO conflicts between Teseo, Sidewalk radio, and other items (e.g., buttons, LEDs) present on NUCLEO-WBAxx boards.

## CLI Usage
For the list of the available CLI commands and respective parameters see [usage instructions](./teseo_cli_usage.md)