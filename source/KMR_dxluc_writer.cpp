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

#include "../include/KMR_dxluc_writer.hpp"


Writer::Writer(int* ids, int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl)
: Handler(ids, nbrMotors, item, hal, dxl)
{

} 



