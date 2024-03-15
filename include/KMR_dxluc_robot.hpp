/**
 ******************************************************************************
 * @file            KMR_dxl_robot.hpp
 * @brief           Header for the KMR_dxl_robot.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#ifndef KMR_DXLUC_ROBOT_HPP
#define KMR_DXLUC_ROBOT_HPP

#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include "../config/KMR_dxluc_structures.hpp"
#include "../include/KMR_dxluc_hal.hpp"

/**
 * @brief   Class that defines a base robot, handling the communication with Dynamixel motors.
 *          It can be used as is, or inherited by a custom class
 */
class BaseRobot {
public:
    void init(const int* ids, const int nbrMotors, const int baudrate, const int protocol_version, int* modes);
    void enableMotors();
    void disableMotors();
    void enableMotor(int id);
    void disableMotor(int id);
    void setGoalPositions(float* goal_positions);
    void setGoalPositions(int* goal_positions);
    void getCurrentPositions(float* current_positions);
    void getCurrentPositions(int* current_positions);
    void setMinAngles(float* minAngles);
    Field getControlFieldFromModel(int modelNumber, ControlTableItem::ControlTableItemIndex item);
    void readItem(ControlTableItem::ControlTableItemIndex item, float* read_data);

    template <typename T>
    void writeItem(T data, int id, ControlTableItem::ControlTableItemIndex item)
    {
        int modelNbr;
        Field field;
        int32_t parameter;
        float offset;

        modelNbr = getModelNumberFromID(id);

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

        m_dxl->writeControlTableItem(item, id, parameter);        
    }

    template <typename T>
    void writeItem(T* data, ControlTableItem::ControlTableItemIndex item)
    {
        int modelNbr, id;
        Field field;
        int32_t parameter;
        float offset;

        for (int i=0; i<m_nbrMotors; i++) {
            id = m_ids[i];
            modelNbr = getModelNumberFromID(id);

            if (item == ControlTableItem::GOAL_POSITION ||
                item == ControlTableItem::PRESENT_POSITION ||
                item == ControlTableItem::MIN_POSITION_LIMIT || 
                item == ControlTableItem::MAX_POSITION_LIMIT ||
                item == ControlTableItem::HOMING_OFFSET) {
                offset = getPositionOffset(modelNbr);
                data[i] += offset;
            }

            field = getControlFieldFromModel(modelNbr, item);
            parameter = (float)data[i]/field.unit; 

            m_dxl->writeControlTableItem(item, id, parameter);      
        }
    }

private:
    int* m_ids;
    int m_nbrMotors;
    int m_protocolVersion;
    int* m_modelNumbers;
    Dynamixel2Arduino* m_dxl;
    Hal* m_hal;

    void initMotors(const int baudrate, const int protocol_version, int* modes);
    void pingMotors();
    void initComm(const int baudrate, const int protocol_version);
    void setOperatingMode(int mode);
    void setOperatingMode(int id, int mode);
    void setOperatingModes(int* modes);
    float getPositionOffset(int modelNumber);
    int getModelNumberFromID(int id);
};


#endif