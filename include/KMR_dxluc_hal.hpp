/**
 ******************************************************************************
 * @file            KMR_dxluc_hal.hpp
 * @brief           Header for the KMR_dxluc_hal.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#ifndef KMR_DXLUC_HAL_HPP
#define KMR_DXLUC_HAL_HPP

#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include "../config/KMR_dxluc_control_tables.hpp"

/**
 * @brief   Class that defines a base robot, handling the communication with Dynamixel motors.
 *          It can be used as is, or inherited by a custom class
 */
class Hal {

public:
    Hal(int protocol_version);

    ControlTable getControlTable(int modelNumber);
    Field getControlField(ControlTable motor, ControlTableItem::ControlTableItemIndex item);

private:
    ControlTable* MX_64;

};


#endif