/**
 ******************************************************************************
 * @file            KMR_dxluc_writer.hpp
 * @brief           Header for the KMR_dxluc_writer.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 03/2024
 * @authors kamilo.melo@km-robota.com, 03/2024
 ******************************************************************************
 */

#ifndef KMR_DXLUC_WRITER_HPP
#define KMR_DXLUC_WRITER_HPP

#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include "KMR_dxluc_handler.hpp"
#include "../config/KMR_dxluc_structures.hpp"
#include "../include/KMR_dxluc_hal.hpp"

#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

namespace KMR_dxluC
{

/**
 * @class   Class used for writing to a control field
 */
class Writer : public Handler{
public:
    Writer(const int* ids, const int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl);

    /**
     * @brief       Write the input data to all the motors' control field handled by this Writer
     * @tparam      data Array of values to be sent to motors, in SI units
     */
    template <typename T>
    void write(T* data)
    {
        int32_t parameter;
        ParamForSyncWriteInst_t syncWriteParams;
        Field field;

        // Prepare the parameters required by syncWrite
        syncWriteParams.addr = m_addr;
        syncWriteParams.length = m_length;
        syncWriteParams.id_count = m_nbrMotors;

        for (int i=0; i<m_nbrMotors; i++) {
            syncWriteParams.xel[i].id = m_ids[i];
            parameter = ((float)data[i] + m_offsets[i])/m_units[i];

            syncWriteParams.xel[i].data[0] = DXL_LOBYTE(DXL_LOWORD(parameter));
            syncWriteParams.xel[i].data[1] = DXL_HIBYTE(DXL_LOWORD(parameter));
            syncWriteParams.xel[i].data[2] = DXL_LOBYTE(DXL_HIWORD(parameter));
            syncWriteParams.xel[i].data[3] = DXL_HIBYTE(DXL_HIWORD(parameter));
        }

        // Send the prepared data
        m_dxl->syncWrite(syncWriteParams);
    }


private:

};

}

#endif