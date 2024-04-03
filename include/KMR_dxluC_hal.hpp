/**
 ******************************************************************************
 * @file            KMR_dxluC_hal.hpp
 * @brief           Header for the KMR_dxluC_hal.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 03/2024
 * @authors kamilo.melo@km-robota.com, 03/2024
 ******************************************************************************
 */

#ifndef KMR_DXLUC_HAL_HPP
#define KMR_DXLUC_HAL_HPP

#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include "../config/KMR_dxluC_control_tables.hpp"

namespace KMR_dxluC
{

/**
 * @class    Hardware abstraction layer for Dynamixel motors
 * @note     The lowest-level element in the library. The Hal class serves primarily as
 *           an abstraction layer, providing high-level functions to get the Dynamixel control
 *           table fields (addresses, byte sizes and units).
 */
class Hal {

public:
    Hal(int protocol_version);
    void init(int* ids, int nbrMotors, int* models);
    int m_protocol;
    //USBSerial m_DEBUG_SERIAL;

    ControlTable getControlTable(int modelNumber);
    Field getControlField(ControlTable motor, ControlTableItem::ControlTableItemIndex item);
    Field getControlFieldFromModel(int modelNumber, ControlTableItem::ControlTableItemIndex item);
    int getModelNumberFromID(int id);
    float getPositionOffset(int modelNumber);

private:
    ControlTable* MX_64;
    ControlTable* MX_106;
    ControlTable* AX_12A;
    
    int* m_ids;
    int m_nbrMotors;
    int* m_models;

};

}

#endif