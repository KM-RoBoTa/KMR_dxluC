/**
 ******************************************************************************
 * @file            KMR_dxl_robot.cpp
 * @brief           Defines the Handler class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include "../include/KMR_dxluc_handler.hpp"

Handler::Handler(int* ids, int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl)
{
    m_nbrMotors = nbrMotors;
    m_item = item;
    m_hal = hal;
    m_dxl = dxl;

    m_ids = new int(m_nbrMotors);
    m_models = new int(m_nbrMotors);
    for (int i=0; i<m_nbrMotors; i++) {
        m_ids[i] = ids[i];
        m_models[i] = m_hal->getModelNumberFromID(ids[i]);
    }

    m_units = new float(m_nbrMotors);
    m_offsets = new float (m_nbrMotors);

    checkMotorCompatibility();
}
   

void Handler::checkMotorCompatibility()
{
    Field field, ref_field;
    
    ref_field = m_hal->getControlFieldFromModel(m_models[0], m_item);
    m_units[0] = ref_field.unit;

    if (m_item == ControlTableItem::GOAL_POSITION ||
        m_item == ControlTableItem::PRESENT_POSITION ||
        m_item == ControlTableItem::MIN_POSITION_LIMIT || 
        m_item == ControlTableItem::MAX_POSITION_LIMIT ||
        m_item == ControlTableItem::HOMING_OFFSET) {
        m_offsets[0] = m_hal->getPositionOffset(m_models[0]);
    }
    else
        m_offsets[0] = 0;

    for(int i=1; i<m_nbrMotors; i++) {
        field = m_hal->getControlFieldFromModel(m_models[i], m_item);
        if (field.addr != ref_field.addr || field.length != ref_field.length) {
            exit(1);
            // COUT
        }
        else {
            m_units[i] = field.unit;
            if (m_item == ControlTableItem::GOAL_POSITION ||
                m_item == ControlTableItem::PRESENT_POSITION ||
                m_item == ControlTableItem::MIN_POSITION_LIMIT || 
                m_item == ControlTableItem::MAX_POSITION_LIMIT ||
                m_item == ControlTableItem::HOMING_OFFSET) {
                m_offsets[i] = m_hal->getPositionOffset(m_models[i]);
            }
            else
                m_offsets[i] = 0;
        }
    }

    m_addr = ref_field.addr;
    m_length = ref_field.length;
}
