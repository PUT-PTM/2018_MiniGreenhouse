# 2018_MiniGreenhouse

## Overview
This project is created to automate process of seed germination. You can put the plant pot in a transparent container and let the system do the work for you. All you need is to make sure that the water is refilled. When water level is low than everything is blocked and stm waits for the water to be refilled. Everything is working on a 2 seconds timer and it is for demonstrating purpose so you can adjust it to your own needs.

Parts:
* SSD1306,
* Water Level Sensor Float Switch,
* FC-28,
* GL5537,
* 2 channel relay,
* DS18B20,
* 12V white 1m LED strip,
* 12V water pump,
* 12V 2A power supply.

## Tools
* System workbench for stm32 with HAL library,
* STM32CubeMX.

## How to run
Connections:
* PB6, PB7 <-> SCL, SDA (SSD1306 - I2C display),
* PB8 <-> Water Level Sensor Float Switch (PULL_UP),
* PA2, PA3 <-> ADC, DIGITAL (FC-28),
* PA4 <-> ADC (GL5537 photoresistor),
* PA6, PA7 <-> IN1, IN2 (2 channel relay for pump and led),
* PB11 <-> OneWire (DS18B20).

## How to compile
For printing floats add flag -u _printf_float to MCC GCU Linker.

## Future improvements
* bluetooth module and android app to allow user to change parameters such as threshold for launching LED and water pump,
* multiple humidity sensors and water pumps to control more plants.

## Attributions
https://elektronika327.blogspot.com/2017/11/31-stm32f4-obsuga-czujnika-ds18b20.html

http://controllerstech.com/oled-display-using-i2c-stm32/

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**

## Credits

The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.

Contractors: [Wojciech Niedbała](https://github.com/wojtii), [Przemysław Pernak](https://github.com/perninio)

Supervisor: Adam Bondyra
