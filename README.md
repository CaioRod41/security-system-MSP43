# Security System - MSP430F5529

A security system that controls an alarm that sends messages via bluetooth and LCD.

## Modules

* PIR
* HC-05
* Display LCD

## How to assemble the project
(In this project you need a protoboard to connect the modules)

1. Connect the PIR sensor to the MSP430
   - OUT - P2.2
   - GND - GND
   - VCC - 5V
2. Connect the HC-05 bluetooth module to the MSP430.
   - RX - P3.3
   - TX - P3.4
   - VCC - 5V
   - GND - GND
3. Connect the LCD display to the MSP430.
   - SDA - P3.0
   - SCL - P3.1
   - VCC - 5V
   - GND - GND
     
4. Upload the code to the MSP430.

## How to use the project

1. When user press P2.1 pin, the PIR will be armed.
2. When PIR detect motion, bluetooth and lcd will notify.
3. to stop the alarm just press P1.1.

