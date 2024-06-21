/**
 ******************************************************************************
 * @file            KMR_dxluC_handler.cpp
 * @brief           Defines the Handler class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 03/2024
 * @authors kamilo.melo@km-robota.com, 03/2024
 ******************************************************************************
 */

#define DEBUG_SERIAL Serial

#include "../include/KMR_dxluC_handler.hpp"

namespace KMR_dxluC
{

/**
 * @brief       Constructor for a Handler, parent class for the Writer and Reader classes
 * @param[in]   ids List of IDs of the motors handled by this specific handler
 * @param[in]   nbrMotors Number of motors handled by this handler 
 * @param[in]   item Control field handled by this handler
 * @param[in]   hal Pointer to the previously created Hal object
 * @param[in]   dxl Pointer to the previously created Dynamixel2Arduino object
 */
Handler::Handler(const int* ids, const int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl)
{
    m_nbrMotors = nbrMotors;
    m_item = item;
    m_hal = hal;
    m_dxl = dxl;

    m_ids = new int[m_nbrMotors]; 
    m_models = new int[m_nbrMotors];
    m_units = new float[m_nbrMotors];
    m_offsets = new float [m_nbrMotors];

    // Get the motor model numbers
    for (int i=0; i<m_nbrMotors; i++) {
        m_ids[i] = ids[i];
        m_models[i] = m_hal->getModelNumberFromID(ids[i]);   
    }

    checkMotorCompatibility();
}

Handler::~Handler()
{
    // Free dynamically allocated memory to heap
    delete[] m_ids;
    delete[] m_models;
    delete[] m_units;
    delete[] m_offsets;
}

/**
 * @brief       Check if the motors are compatible (address + byte size of the handler's control field)
 * @note        Also populate the position offsets and units vectors
 */
void Handler::checkMotorCompatibility()
{
    Field field, ref_field;
    
    ref_field = m_hal->getControlFieldFromModel(m_models[0], m_item);
    m_units[0] = ref_field.unit;

    if (m_item == ControlTableItem::GOAL_POSITION       ||
        m_item == ControlTableItem::PRESENT_POSITION    ||
        m_item == ControlTableItem::MIN_POSITION_LIMIT  || 
        m_item == ControlTableItem::MAX_POSITION_LIMIT  ||
        m_item == ControlTableItem::HOMING_OFFSET       ||
        m_item == ControlTableItem::CW_ANGLE_LIMIT      || 
        m_item == ControlTableItem::CCW_ANGLE_LIMIT      ) {
        m_offsets[0] = m_hal->getPositionOffset(m_models[0]);
    }
    else
        m_offsets[0] = 0;

    for(int i=1; i<m_nbrMotors; i++) {
        field = m_hal->getControlFieldFromModel(m_models[i], m_item);

        if (field.addr != ref_field.addr || field.length != ref_field.length) {
            DEBUG_SERIAL.println("Error! Incompatible addresses or lengths");
            exit(1);
        }
        else {
            m_units[i] = field.unit;
            if (m_item == ControlTableItem::GOAL_POSITION       ||
                m_item == ControlTableItem::PRESENT_POSITION    ||
                m_item == ControlTableItem::MIN_POSITION_LIMIT  || 
                m_item == ControlTableItem::MAX_POSITION_LIMIT  ||
                m_item == ControlTableItem::HOMING_OFFSET       ||
                m_item == ControlTableItem::CW_ANGLE_LIMIT      || 
                m_item == ControlTableItem::CCW_ANGLE_LIMIT      ) {
                m_offsets[i] = m_hal->getPositionOffset(m_models[i]);
            }
            else
                m_offsets[i] = 0;
        }
    }

    m_addr = ref_field.addr;
    m_length = ref_field.length;
}

}