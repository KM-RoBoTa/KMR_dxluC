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

#ifndef KMR_DXLUC_READER_HPP
#define KMR_DXLUC_READER_HPP

#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include "KMR_dxluc_handler.hpp"
#include "../include/KMR_dxluc_hal.hpp"

/**
 * @brief   Class that defines a base robot, handling the communication with Dynamixel motors.
 */
class Reader : public Handler{
public:
    Reader(int* ids, int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl);
    void read(float* fbck);

private:
    InfoBulkReadInst_t m_readerInfo;
    int32_t* m_fbck_params;

    void bulkRead(float* fbck);
    void basicRead(float* fbck);
};


#endif