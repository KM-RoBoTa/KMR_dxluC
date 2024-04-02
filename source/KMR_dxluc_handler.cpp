/**
 ******************************************************************************
 * @file            KMR_dxluc_handler.cpp
 * @brief           Defines the Handler class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors kamilo.melo@km-robota.com, 03/2024
 * @authors katarina.lichardova@km-robota.com, 03/2024
 * 
 ******************************************************************************
 */

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
    #include <SoftwareSerial.h>
    SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
    #define DXL_SERIAL   Serial
    #define DEBUG_SERIAL soft_serial
    const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
    #define DXL_SERIAL   Serial
    #define DEBUG_SERIAL SerialUSB
    const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
    #define DXL_SERIAL   Serial1
    #define DEBUG_SERIAL SerialUSB
    const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
    #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
    #define DEBUG_SERIAL Serial
    const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
    #define DEFAULT_BAUDRATE 57600
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
    // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
    // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
    #define DXL_SERIAL   Serial3
    #define DEBUG_SERIAL Serial
    const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
    //OpenRB does not require the DIR control pin.
    #define DXL_SERIAL Serial1
    #define DEBUG_SERIAL Serial
    const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
    #define DXL_SERIAL   Serial1
    #define DEBUG_SERIAL Serial
    const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

#include "../include/KMR_dxluc_handler.hpp"

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