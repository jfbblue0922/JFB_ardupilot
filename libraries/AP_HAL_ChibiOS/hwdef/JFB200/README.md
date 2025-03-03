# JFB-X Flight Controller

The JFB-X flight controller is sold by [JAE](https://www.jae.com/Motion_Sensor_Control/eVTOL/FlightController/)

## Features

 - STM32H755 microcontroller
 - Three IMUs: SCHA63T and two IMUs
 - Three BAROs: MS5611 SPI barometer x3
 - builtin Two I2C IST8310 magnetometer
 - microSD card slot
 - 6 UARTs plus USB, RCIN, SBUS_OUT
 - 16 PWM outputs
 - Four I2C and two CAN ports
 - External Buzzer (Open/Drain and 27V Out)
 - external safety Switch
 - voltage monitoring for servo rail and Vcc
 - two dedicated power input ports for external power bricks

## UART Mapping

 - SERIAL0 -> USB
 - SERIAL1 -> UART7  (Telem1)
 - SERIAL2 -> UART5  (Telem2)
 - SERIAL3 -> USART1 (GPS)
 - SERIAL4 -> UART4  (GPS2, marked UART/I2CB)
 - SERIAL5 -> USART6 (RCIN)
 - SERIAL6 -> UART8  (SBUS_OUT)
 - SERIAL7 -> USART3 (debug)
 - SERIAL8 -> USB    (SLCAN)
 - SERIALx -> USART2 (Telem3)


The Telem1, Telem2 and Telem3 ports have RTS/CTS pins, the other UARTs do not
have RTS/CTS.

The USART3 connector is labelled debug, but is available as a general
purpose UART with ArduPilot.

## RC Input
 
RC input is configured on the port marked DSM/SBUS RC. This connector
supports all RC protocols. Two cables are available for this port. To
use software binding of Spektrum satellite receivers you need to use
the Spektrum satellite cable.

## PWM Output

The JFB-X supports up to 16 PWM outputs. 
These are directly attached to the STM32H755 and support all
PWM protocols.

The 16 PWM outputs are in 6 groups:
 - PWM  1,  2,  3 and  4 in group1 (TIM1)
 - PWM  5,  6,  7 and  8 in group2 (TIM3)
 - PWM  9, 10, 11 and 12 in group3 (TIM4)
 - PWM 13, 14, 15 and 16 in group4 (TIM8)

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has two dedicated power monitor ports on 8 pin
connectors. The correct battery setting parameters are dependent on
the type of power brick which is connected.
Recomended input voltage is 4.9 to 5.5 volt.

## Compass

The JFB-X has a builtin IST8310 compass. Due to potential
interference the board is usually used with an external I2C compass as
part of a GPS/Compass combination.

## GPIOs

The 16 PWM ports can be used as GPIOs (relays, buttons, RPM etc). To
use them you need to limit the number of these pins that is used for
PWM by setting the BRD_PWM_COUNT to a number less than 6. For example
if you set BRD_PWM_COUNT to 4 then PWM5 and PWM6 will be available for
use as GPIOs.

The numbering of the GPIOs for PIN variables in ArduPilot is:
 - PWM(1)  50
 - PWM(2)  51
 - PWM(3)  52
 - PWM(4)  53
 - PWM(5)  54
 - PWM(6)  55
 - PWM(7)  56
 - PWM(8)  57
 - PWM(9)  58
 - PWM(10) 59
 - PWM(11) 60
 - PWM(12) 61
 - PWM(13) 62
 - PWM(14) 63
 - PWM(15) 64
 - PWM(16) 65
 - FMU_CAP1 66
 - FMU_CAP2 67
 

## Analog inputs

The JFB-X has 9 analog inputs
 - ADC  Pin0  -> not used
 - ADC  Pin1  -> not used
 - ADC1 Pin2  -> +3.3V Sensor
 - ADC  Pin3  -> not used
 - ADC3 Pin4  -> Battery Voltage 2
 - ADC3 Pin5  -> HW_REV_SENS
 - ADC1 Pin6  -> Battery Current Sensor 2
 - ADC  Pin7  -> not used
 - ADC3 Pin8  -> ADC SPARE 1 (6.6V) Battery Voltage
 - ADC3 Pin9  -> HW_VER_SENS
 - ADC1 Pin10 -> RSSI voltage monitoring
 - ADC  Pin11 -> not used
 - ADC  Pin12 -> not used
 - ADC1 Pin13 -> ADC SPARE (3.3V)
 - ADC3 Pin14 -> SERVORAIL sens
 - ADC3 Pin15 -> 5V sens
 - ADC1 Pin16 -> Battery Voltage 2
 - ADC  Pin17 -> not used
 - ADC1 Pin18 -> Battery Current Sensor
 - ADC  Pin19 -> not used

## I2C Buses

The JFB-X has 4 I2C interfaces.
I2C 3 is for internal only.
 - the internal I2C port  is bus 3 in ArduPilot (I2C3 in hardware)
 - the port labelled I2CA is bus 4 in ArduPilot (I2C4 in hardware)
 - the port labelled I2CB is bus 2 in ArduPilot (I2C2 in hardware)
 - the port labelled GPS  is bus 1 in ArduPilot (I2C1 in hardware)

## CAN

The JFB-X has two independent CAN buses, with the following pinouts.

## Debug

The JFB-X supports SWD debugging on the debug port

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.
