# Introduction {#mainpage}
[TOC]

This library provides an easy way to use Dynamixel motors in a project using the OpenCM9.04 board, in either Dynamixel protocol. \n 
Its main strengths are the expansion of the Dynamixel2Arduino library from ROBOTIS, the implementation of syncWrite and bulkRead, as well as the abstraction of the hardware.

## Working concept
The communication with the motors (reading/writing) is done through this library's ```Reader``` and ```Writer``` classes.

The KMR_dxluc::BaseRobot class provides a basis to be inherited by a custom, project-specific Robot class written by the user. BaseRobot provides general-use functions such as opening the communication port, pinging the motors, enabling/disabling motors and setting the angle limits of the motors. \n
The user needs to create custom Reader and Writer objects in their child Robot class, handling the fields they need (for example writing to goal position and reading the current position), as well as their corresponding reading/writing functions. Those functions are extremely straightforward to implement, see CODE. 

This library is written in C++, and lives inside the ```KMR_dxluC``` namespace. \n 
It uses SI units, the only exception being temperature expressed in °C instead of Kelvins.


## Links

- Repository: https://github.com/KM-RoBoTa/KMR_dxluC
- About Robotis's OpenCM9.04 board: https://emanual.robotis.com/docs/en/parts/controller/opencm904/
- Using OpenCM9.04 with the Arduino IDE and Dynamixel2Arduino library : https://emanual.robotis.com/docs/en/software/arduino_ide/
- How to [setup](#setup)
- How to [use](#how-to-use)

## About

KM-RoBoTa sarl's KMR_dxluC library to facilitate Dynamixel control while using the OpenCM9.04 board from ROBOTIS.

### Authors
Library written by Katarina Lichardova: katarina.lichardova@km-robota.com


### Copyright
Copyright 2021-2023, Laura Paez Coy and Kamilo Melo. \n
This code is under MIT licence: https://opensource.org/licenses/MIT


## Currently supported motor models
- MX-64
- MX-106
- AX-12A
