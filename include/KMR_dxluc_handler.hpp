/**
 ******************************************************************************
 * @file            KMR_dxluc_handler.hpp
 * @brief           Header for the KMR_dxluc_handler.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors kamilo.melo@km-robota.com, 03/2024
 * @authors katarina.lichardova@km-robota.com, 03/2024
 ******************************************************************************
 */

#ifndef KMR_DXLUC_HANDLER_HPP
#define KMR_DXLUC_HANDLER_HPP

#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include "../config/KMR_dxluc_structures.hpp"
#include "../include/KMR_dxluc_hal.hpp"

/**
 * @brief   Parent class, to be specialized as a Reader or Writer
 * @note	This class is not usable by itself, it is a non-specialized sketelon inherited
 * 			by the child classes Reader and Writer. \n
 * 			It contains functionalities to check the viability of sync readers/writers 
 * 			that will be defined in child classes (motor compatibility).
 */
class Handler {
public:
    Handler(int* ids, int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl);

protected:
    int* m_ids;
    int* m_models;
    int m_nbrMotors;
    ControlTableItem::ControlTableItemIndex m_item;
    Hal* m_hal;

    Dynamixel2Arduino* m_dxl;
    int m_addr, m_length;
    float* m_offsets;
    float* m_units;

    void checkMotorCompatibility();

};


#endif