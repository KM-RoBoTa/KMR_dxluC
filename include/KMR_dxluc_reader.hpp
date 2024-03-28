/**
 ******************************************************************************
 * @file            KMR_dxl_reader.hpp
 * @brief           Header for the KMR_dxl_reader.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors kamilo.melo@km-robota.com, 03/2024
 * @authors katarina.lichardova@km-robota.com, 03/2024
 ******************************************************************************
 */

#ifndef KMR_DXLUC_READER_HPP
#define KMR_DXLUC_READER_HPP

#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include "KMR_dxluc_handler.hpp"
#include "../include/KMR_dxluc_hal.hpp"

/**
 * @class   Class used for reading a control field
 */
class Reader : public Handler{
public:
    Reader(const int* ids, const int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl);
    void read(float* fbck);

private:
    InfoBulkReadInst_t m_readerInfo;
    int32_t* m_fbck_params;
    bool m_canUseBulkRead = 1;

    void bulkRead(float* fbck);
    void basicRead(float* fbck);
    void checkBulkReadAvailability();
};


#endif