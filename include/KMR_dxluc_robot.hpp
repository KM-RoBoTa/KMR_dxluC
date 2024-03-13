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



/**
 * @brief   Class that defines a base robot, handling the communication with Dynamixel motors.
 *          It can be used as is, or inherited by a custom class
 */
class BaseRobot {
public:
    void init(const int* ids, const int nbrMotors, const int baudrate, const int protocol_version, int* modes);
    void enableMotors();
    void disableMotors();
    void setGoalPositions(float* goal_positions);
    void setGoalPositions(int* goal_positions);
    void getCurrentPositions(float** current_positions);
    void getCurrentPositions(int** current_positions);

private:
    int* m_ids;
    int m_nbrMotors;
    Dynamixel2Arduino* m_dxl;

    void initMotors(const int baudrate, const int protocol_version, int* modes);
    void pingMotors();
    void initComm(const int baudrate, const int protocol_version);
    void setOperatingMode(int mode);
    void setOperatingMode(int id, int mode);
    void setOperatingModes(int* modes);


};


#endif