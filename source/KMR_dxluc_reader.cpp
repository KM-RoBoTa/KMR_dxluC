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

#define BUFFER_SIZE 128

#include "../include/KMR_dxluc_reader.hpp"


Reader::Reader(int* ids, int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl)
: Handler(ids, nbrMotors, item, hal, dxl)
{
    m_fbck_params = new int32_t[nbrMotors]();

    m_readerInfo.is_info_changed = true;
    m_readerInfo.xel_count = nbrMotors;
    m_readerInfo.packet.p_buf = new uint8_t[BUFFER_SIZE]();
    m_readerInfo.packet.buf_capacity = BUFFER_SIZE;
    m_readerInfo.packet.is_completed = false;
    m_readerInfo.p_xels = new XELInfoBulkRead_t[nbrMotors]();

    for (int i=0; i<nbrMotors; i++) {
        m_readerInfo.p_xels[i].addr = m_addr;
        m_readerInfo.p_xels[i].addr_length = m_length;
        m_readerInfo.p_xels[i].id = m_ids[i];
        m_readerInfo.p_xels[i].p_recv_buf = (uint8_t*)&m_fbck_params[i];
    }

} 

void Reader::read(float* fbck)
{
    int32_t parameter;
    uint8_t recv_count;

    recv_count = m_dxl->bulkRead(&m_readerInfo);

    // Get the feedback values and convert them to SI units
    if (recv_count > 0) {
        for (int i=0; i<m_nbrMotors; i++) {
            parameter = m_fbck_params[i];
            fbck[i] = parameter * m_units[i] - m_offsets[i];
        }
    }
    else {
        for (int i=0; i<m_nbrMotors; i++)
            fbck[i] = 9999;
        // error print
    }
}