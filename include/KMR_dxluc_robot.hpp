/**
 ******************************************************************************
 * @file            KMR_dxl_robot.hpp
 * @brief           Header for the KMR_dxl_robot.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors kamilo.melo@km-robota.com, 03/2024
 * @authors katarina.lichardova@km-robota.com, 03/2024
 ******************************************************************************
 */

#ifndef KMR_DXLUC_ROBOT_HPP
#define KMR_DXLUC_ROBOT_HPP

#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include "../config/KMR_dxluc_structures.hpp"
#include "../include/KMR_dxluc_hal.hpp"
#include "../include/KMR_dxluc_writer.hpp"
#include "../include/KMR_dxluc_reader.hpp"

/**
 * @brief   Class that defines a base robot, handling the communication with Dynamixel motors.
 *          It needs to be inherited by a custom Robot class
 */
class BaseRobot {
public:
    int m_nbrMotors;
    Writer* EEPROM_writer;

    BaseRobot(const int* ids, const int nbrMotors, const int baudrate, const int protocol_version);
    void enableMotors();
    void disableMotors();
    void enableMotor(int id);
    void disableMotor(int id);
    void setMinAngles(float* minAngles);
    void setMaxAngles(float* maxAngles);
    void setMinVoltages(float* minVoltages);
    void setMaxVoltages(float* maxVoltages);
    void setMaxTorques(float* maxTorques);
    void setOperatingModes(int* modes);

protected:
    Dynamixel2Arduino* m_dxl;
    Hal* m_hal;

private:
    int* m_ids;
    int m_protocolVersion;
    int* m_modelNumbers;

    void initMotors(const int baudrate, const int protocol_version);
    void pingMotors();
    void initComm(const int baudrate, const int protocol_version);


};


#endif