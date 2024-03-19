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

#include "../include/KMR_dxluc_reader.hpp"


Reader::Reader(int* ids, int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl)
: Handler(ids, nbrMotors, item, hal, dxl)
{
    m_dataFromMotor = new float(nbrMotors);
} 

/*
float* Reader::read()
{
    ParamForBulkReadInst_t bulkReadParams;
    RecvInfoFromStatusInst_t bulkReadResult;

    bulkReadParams.id_count = m_nbrMotors;
    bulkReadResult.id_count = m_nbrMotors;

    for (int i=0; i<m_nbrMotors; i++) {
        bulkReadParams.xel[i].addr = m_addr;
        bulkReadParams.xel[i].length = m_length;
        bulkReadParams.xel[i].id = m_ids[i];
    }

    m_dxl->bulkRead(bulkReadParams, bulkReadResult);

    // Convert received data (parameters) into SI units
    for (int i=0; i<m_nbrMotors; i++) {

        m_hal->getSIData(bulkReadResult.xel[i].data, bulkReadResult.xel[i].id, m_item);
    }

}*/


