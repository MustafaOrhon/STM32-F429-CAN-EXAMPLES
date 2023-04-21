# CAN Bus Examples Using STM32-F429

This repository contains examples of using the STM32-F429DISC microcontroller to communicate over the Controller Area Network (CAN) bus. The examples cover both standard and extended identifier (ID) formats, filtering and multifiltering, message priority, and two projects that demonstrate the use of CAN bus communication with other peripherals.

## Requirements

To run the examples and projects in this repository, you will need:

- STM32-F429DISC microcontroller board
- CAN bus transceiver (such as the MCP2551)
- IDE with STM32CubeMX and STM32CubeIDE installed
- 2x16 LCD DISPLAY
- HCSR04 DISTANCE SENSOR
- Other peripherals used in the projects (see respective sections)

## Getting Started

1. Clone this repository to your local machine.
2. Open STM32CubeMX and import the project file for the example or project you want to run.
3. Configure the project settings as necessary, including selecting the correct board and peripherals.
4. Generate the code and open the project in STM32CubeIDE.
5. Build and flash the project onto your STM32F429 board.
6. Connect the CAN bus transceiver to the appropriate pins on the board.
7. Power on the board and the peripheral devices used in the project.
8. Enjoy experimenting with CAN bus communication!

## Examples

This repository includes two examples of CAN bus communication with the STM32-F429 board. The examples cover:



## Project 1: ADC DMA Data Transmission

This project reads analog-to-digital converter (ADC) data from one STM32-F429DISC board and sends it via the CAN bus to another board. The receiving board displays the data on an LCD. The project demonstrates the use of CAN bus communication with other peripherals.

You can find the project files in the `012_CAN_BUS_ADC_DMA_LCDDISPLAY_BOARD1/2` folders.

## Project 2: HC-SR04 Sensor Data and LED Control

This project reads distance data from an HC-SR04 ultrasonic sensor and sends it via the CAN bus to another STM32-F429DISC board. The receiving board uses the data to control of its LED based on the distance data that is received.

You can find the project files in the `013_CAN_BUS_EXTID_HCSR04_BOARD1/2` folders.

