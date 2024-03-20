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
    void init(int* ids, int nbrMotors, int* models);

    ControlTable getControlTable(int modelNumber);
    Field getControlField(ControlTable motor, ControlTableItem::ControlTableItemIndex item);
    Field getControlFieldFromModel(int modelNumber, ControlTableItem::ControlTableItemIndex item);
    int getModelNumberFromID(int id);
    float getPositionOffset(int modelNumber);

    template <typename T>
    int32_t getParameter(T data, int id, ControlTableItem::ControlTableItemIndex item)
    {
        int modelNbr = getModelNumberFromID(id);
        float offset = 0;
        Field field;
        int32_t parameter;

        if (item == ControlTableItem::GOAL_POSITION ||
            item == ControlTableItem::PRESENT_POSITION ||
            item == ControlTableItem::MIN_POSITION_LIMIT || 
            item == ControlTableItem::MAX_POSITION_LIMIT ||
            item == ControlTableItem::HOMING_OFFSET) {
            offset = getPositionOffset(modelNbr);
            data += offset;
        }

        field = getControlFieldFromModel(modelNbr, item);
        parameter = (float)data/field.unit; 
 
        return parameter;
    }

    float getSIData(int8_t parameter, int id, ControlTableItem::ControlTableItemIndex item);

// DEBUG
//private:
    ControlTable* MX_64;

    int* m_ids;
    int m_nbrMotors;
    int* m_models;

};


#endif