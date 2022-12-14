# AVehicle
### Autonomus vehicle project - engineering thesis. 

## Table of contents
- [Description](#description) 
  - [Capabilities](#capabilities) 
  - [MCU used](#mcu-used) 
  - [IDE and other software used](#ide-and-other-software-used)
  - [Programing language](#programing-language)
- [Gallery](#gallery)


## Description
The vehicle has been made from model parts (1:10 scale) and metal parts from the junkyard. It has two independent brush motors with reducer (1:16), controlled by STM32f411 MCU via H-bridge. The vehicle is RWD with front steering axle, implemented by 180 servo. The vehicle is equipped with numerous sensors:
- RPLIDAR A1M8
- 6 ultrasound sensors HCSR04
- 2 encoders (transoptors)
- 4 limit switches as collision sensors
- accelerometer
- magnetometer
- temperature sensor

The vehicle works like state machine with hardware interrupts.

## Capabilities

The vehicle capabilities are listed below:
- Can be radio controlled via nRF24L0+
- Can send vehicle state and sensor data to controller
- The vehicle can provide data about: (via ESP32)
  - Close obstacles within 2 - 400 cm distance (HCSR04)
  - Obstacles within 20 cm to 12 m (RPLIDAR)
  - Information about colision and it's location (limit switch)
  - Angular and linear vehicle velocity (encoders)
  - Distance traveled (encoders)
  - Current acceleration in 3 axis (accelerometer)
  - E-compass data (magnetometer)
  - Environment temperature for measurement correction 

## MCU used

- STM32F411
- ESP32
- nRF24L0+

## IDE and other software used

- STM32 Cube IDE (Eclipse)
- STM32 Cube MX
- STM Studio
- Visual Studio Code

## Programing language

C/C++, mostly C

## Gallery

### Demonstration video: 
[https://www.youtube.com/watch?v=PXvDGG3j_pY&list=PLEnQao540nqGiPTDcLybcyLzKcGvvVYyQ](https://www.youtube.com/watch?v=PXvDGG3j_pY&list=PLEnQao540nqGiPTDcLybcyLzKcGvvVYyQ)

### Photos 

 <p align = "center"> <img src = "Photos/Próby na płytce stykowej.jpg" width="576"</p>
 <p align = "center"> Prototyping </p>

 <p align = "center"> <img src = "Photos/Podwozie.jpg" width="576"</p>
 <p align = "center"> Chassis </p>

 <p align = "center"> <img src = "Photos/Warsztatowe.jpg" width="576"</p>
 <p align = "center"> Workshop photo </p>
 
 <p align = "center"> <img src = "Photos/IMG_20220125_180801.jpg" width="576"</p>
 <p align = "center"> Soldering ESP32 socket </p>
 
 <p align = "center"> <img src = "Photos/Pojazd + kontroler.jpg" width="576"</p>
 <p align = "center"> Vehicle + controller </p>
 
 
 
 

 

  
