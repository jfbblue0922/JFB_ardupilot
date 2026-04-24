# JFB-200 Flight Controller

The JFB-200 flight controller of [JAE](https://www.jae.com/Motion_Sensor_Control/eVTOL/FlightController/)  
The JFB-200 consists of an octa module with a CPU and sensors—and an I/O board.  
Some of the sensors mounted on the octa module are vibration-isolated and feature built-in heaters.  
![JFB-200 overview](JFB-200_overview.png)

## Features
- Processor
  - STM32H755 microcontroller
- Sensors
  - Three IMUs: ICM45686, ASM330 and IIM42653 SPI IMU
  - Two BAROs: two BMP390 SPI barometer
  - Two Mags: builtin I2C BMM350 and IST8310 magnetometer
- Interfaces
  - microSD card slot
  - 6 UARTs plus USB (including GPS, RCIN and S.OUT)
  - 16 PWM outputs (8 PWM shares GPIO)
  - Three I2C and two CAN ports
  - Two external Buzzer (Open/Drain and 27V Out)
  - external safety Switch
  - voltage monitoring for servo rail and Vcc
  - Ethernet port
- Power
  - two dedicated power input ports for external power bricks
  - two analog battery voltage and current sensing port

## Pinout

![JFB-200 pinout](JFB-200_pinout.png)

## Wiring Diagram

## UART Mapping
- SERIAL0 -> USB
- SERIAL1 -> UART7  (Telem1)
- SERIAL2 -> UART5  (Telem2)
- SERIAL3 -> USART3 (Telem3)
- SERIAL4 -> USART1 (GPS1)
- SERIAL5 -> USART2 (GPS2)
- SERIAL6 -> UART8  (RCIN)
- SERIAL7 -> USART6 (S.OUT)
- SERIAL8 -> USB

The Telem1, Telem2 and Telem3 ports have RTS/CTS pins, the other UARTs do not
have RTS/CTS.

## RC Input

RC input is configured on the port marked RCIN/UART. This connector
supports all RC protocols. Two cables are available for this port. To
use software binding of Spektrum satellite receivers you need to use
the Spektrum satellite cable.

## PWM Output

The JFB-200 supports up to 16 PWM outputs.PWM1-8 are connected via dedicated output buffers.
PWM9-16 are connected via bidirectional buffers.

The 16 PWM outputs are in 4 groups:

- PWM  1,  2,  3 and  4 in group1 (TIM1)
- PWM  5,  6,  7 and  8 in group2 (TIM3)
- PWM  9, 10, 11 and 12 in group3 (TIM4)
- PWM 13, 14, 15 and 16 in group4 (TIM8)

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.
PWM output voltage can be changed setting BRD_PWM_COLT_SEL parameter.

## Battery Monitoring

The board has two dedicated power monitor ports on 8 pin connectors.
The correct battery setting parameters are dependent on the type of
power brick which is connected.
Recomended input voltage is 4.9 to 5.5 volt.

## Compass

The JFB-200 has two builtin compasses BMM350 and IST8310. Due to potential
interference the board is usually used with an external I2C compass as
part of a GPS/Compass combination.

## GPIOs

The 8 PWM ports (PMW9-16) can be used as GPIOs (relays, buttons, RPM etc).

The numbering of the GPIOs for PIN variables in ArduPilot is:

- PWM(9)  50
- PWM(10) 51
- PWM(11) 52
- PWM(12) 53
- PWM(13) 54
- PWM(14) 55
- PWM(15) 56
- PWM(16) 57
- FMU_CAP1 58
- FMU_CAP2 59

## Analog inputs

The JFB-200 has 9 analog inputs

- ADC Pin0  -> not used
- ADC Pin1  -> not used
- ADC Pin2  -> +3.3V Sens
- ADC Pin3  -> not used
- ADC Pin4  -> Battery Voltage 2
- ADC Pin5  -> not used
- ADC Pin6  -> Battery Current Sensor 2
- ADC Pin7  -> not used
- ADC Pin8  -> ADC SPARE 1 (6.6V)
- ADC Pin9  -> not used
- ADC Pin10 -> RSSI voltage monitoring
- ADC Pin11 -> not used
- ADC Pin12 -> not used
- ADC Pin13 -> ADC SPARE 2 (3.3V)
- ADC Pin14 -> SERVORAIL sens
- ADC Pin15 -> 5V sens
- ADC Pin16 -> Battery Voltage
- ADC Pin17 -> not used
- ADC Pin18 -> Battery Current Sensor
- ADC Pin19 -> not used

## I2C Buses

The JFB-200 has 4 I2C interfaces.
I2C 3 is for internal only.

- the internal I2C port  is bus 3 in ArduPilot (I2C3 in hardware)
- the port labelled GPS1 is bus 1 in ArduPilot (I2C1 in hardware)
- the port labelled GPS2 is bus 2 in ArduPilot (I2C2 in hardware)
- the port labelled I2C  is bus 4 in ArduPilot (I2C4 in hardware)

## CAN

The JFB-200 has two independent CAN buses with terminating resistors.

## Dedicated Signal input/output

The JFB-200 has the following dedicated discrete signals

- ARMED status signal output
- Hardware WDT Fail signal output
- HW reset signal input

## Connectors

Unless noted otherwise all connectors are JST GH 1.25mm pitch

### AUX port

| Pin | Signal | Volt | Remarks |
| --- | --- | --- | |
| 1 | ARMEDn | +3.3V | |
| 2 | WDT_FAILn | +3.3V  | |
| 3 | EXT_RESETn | +3.3V  | |
| 4 | SHEILD | GND | |

### PWM1, PWM2 ports

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | PWM(1) or (9) | +3.3V or +5V |
| 3 | PWM(2) or (10) | +3.3V or +5V |
| 4 | PWM(3) or (11) | +3.3V or +5V |
| 5 | PWM(4) or (12) | +3.3V or +5V |
| 6 | PWM(5) or (13) | +3.3V or +5V |
| 7 | PWM(6) or (14) | +3.3V or +5V |
| 8 | PWM(7) or (15) | +3.3V or +5V |
| 9 | PWM(8) or (16) | +3.3V or +5V |
| 10 | GND | |

### SPI port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | SPI5_SCK | +3.3V |
| 3 | SPI5_MISO | +3.3V |
| 4 | SPI5_MOSI | +3.3V |
| 5 | SPI5_CSn | +3.3V |
| 6 | N.C. | |
| 7 | GND | |

### CAN1, CAN2 ports

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | CAN_H | |
| 3 | CAN_L | |
| 4 | GND | |

### ADC/IO port
| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | CAP1 | +3.3V |
| 3 | CAP2 | +3.3V |
| 4 | AIN3 | +3.3V |
| 5 | AIN6 | +6.6V |
| 6 | GND | |

### TELEM1, TELEM2, TELEM3 ports

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | CTS | +3.3V |
| 5 | RTS | +3.3V |
| 6 | GND | GND |

### GPS2 port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | I2C2 SCL | +3.3V |
| 5 | I2C2 SDA | +3.3V |
| 6 | GND | |

### ETH port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | RXN | |
| 2 | RXP | |
| 3 | TXN | |
| 4 | TXP | |

### GPS1 port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | I2C1 SCL | +3.3V(pullup) |
| 5 | I2C1 SDA| +3.3V(pullup) |
| 6 | Safety Button | +3.3V |
| 7 | Safety LED | +3.3V |
| 8 | VCC3 | +3.3V |
| 9 | BUZZER | OPEN/DRAIN |
| 10 | GND |  |

### BUZZER port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | BUZZER VCC | +24V/GND |
| 2 | GND | |

### RCIN/UART port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | RSSI | +3.3V |
| 5 | PPM | +3.3V |
| 6 | TX (OUT) | +3.3V |
| 7 | RX (IN) | +3.3V |
| 8 | GND | |

### I2C port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | I2C4 SCL | +3.3V |
| 3 | I2C4 SDA | +3.3V |
| 4 | GND | |

### USB port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | SCL I2C2 | +3.3V |
| 5 | SDA I2C2 | +3.3V |
| 6 | GND | |

### RCIN port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | RCIN | +3.3V |
| 2 | VCC | +5V |
| 3 | GND | |

### S.OUT port

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | S.OUT | +3.3V |
| 2 | VCC | +5V |
| 3 | GND | |

### POWER1, POWER2 ports

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC IN | +5V |
| 2 | VCC IN | +5V |
| 3 | CURRENT | +3.3V |
| 4 | VOLTAGE | +3.3V |
| 5 | GND | |
| 6 | GND | |


### SBUS Out port

The SBUSo port is a port attached to the IO processor which can be
used to output all servo channels via SBUS. It is enabled by setting
the BRD_SBUS_OUT parameter.

The pinout below for the SBUSo port is labelled so that GND is closest
to the cube. The 5V pin on the SBUS output port is connected to the
servo rail.

When SBUS output is disabled (by setting BRD_SBUS_OUT to 0) you can
use the port for analog RSSI input from receivers. To enable for RSSI
input you need to set:

- BRD_SBUS_OUT 0
- RSSI_TYPE 1
- RSSI_PIN 103

You cannot have both SBUS output and analog RSSI input at the same time.

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | GND | GND |
| 2 | 5v(Vservo) | +5.0V |
| 3 | TX (OUT) | +3.3V |

### SPKT port

The SPKT port provides a connector for Spektrum satellite
receivers. It is needed to allow for software controlled binding of
satellite receivers.

The pinout of the SPKT port given below is given with the 3.3V power
pin closest to the cube (pin 3).

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | RX (IN) | +3.3V |
| 2 | GND | GND |
| 3 | 3.3v | +3.3V |



### I2C2
| Pin | Signal | Volt |
| --- | --- | --- |
| 1 (red) | VCC | +5V |
| 2 (blk) | SCL | +3.3 (pullups) |
| 3 (blk) | SDA | +3.3 (pullups) |
| 4 (blk) | GND | GND |

### FMU and IO SWD

When the case is removed there are two SWD connectors, one for FMU and
the other for IOMCU. The IO SWD connector is the one closer to the
servo rail. The GND pin of both connectors is the one furthest from
the servo rail.

| Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX | +3.3 |
| 3 | RX | +3.3 |
| 4 | SWDIO | +3.3 |
| 5 | SWCLK | +3.3 |
| 6 | GND | GND |


### USB

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 (red) | VCC | +5V |
| 2 (blk) | D_plus | +3.3V |
| 3 (blk) | D_minus | +3.3V |
| 4 (blk) | GND | GND |
| 5 (blk) | BUZZER | battery voltage |
| 6 (blk) | Boot/Error LED |  |

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.
The JFB-200 can be booted into DFU mode using a dedicated adapter.
