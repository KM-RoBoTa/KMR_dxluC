/**
 ******************************************************************************
 * @file            KMR_dxluC_reader.hpp
 * @brief           Header for the KMR_dxluC_reader.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 03/2024
 * @authors kamilo.melo@km-robota.com, 03/2024
 ******************************************************************************
 */

#ifndef KMR_DXLUC_READER_HPP
#define KMR_DXLUC_READER_HPP

#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include "KMR_dxluC_handler.hpp"
#include "../include/KMR_dxluC_hal.hpp"

namespace KMR_dxluC
{

/**
 * @class   Class used for reading a control field
 */
class Reader : public Handler{
public:
    Reader(const int* ids, const int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl);
    ~Reader();
    int read(float* fbck);

private:
    DYNAMIXEL::InfoBulkReadInst_t m_readerInfo;
    int32_t* m_fbck_params = nullptr;
    bool m_canUseBulkRead = 1;
    int m_readError = 0;

    void bulkRead(float* fbck);
    void basicRead(float* fbck);
    void checkBulkReadAvailability();
};

}

#endif