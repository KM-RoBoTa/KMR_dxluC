/**
 ******************************************************************************
 * @file            KMR_dxl_reader.hpp
 * @brief           Defines the Reader class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors kamilo.melo@km-robota.com, 03/2024
 * @authors katarina.lichardova@km-robota.com, 03/2024
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
 
#define BUFFER_SIZE 128
#define TIMEOUT 100

#include "../include/KMR_dxluc_reader.hpp"

namespace KMR_dxluC
{

/**
 * @brief       Constructor for a Reader handler, used for reading data from motors
 * @param[in]   ids List of IDs of the motors handled by this specific handler
 * @param[in]   nbrMotors Number of motors handled by this handler 
 * @param[in]   item Control field read by this handler
 * @param[in]   hal Pointer to the previously created Hal object
 * @param[in]   dxl Pointer to the previously created Dynamixel2Arduino object
 */
Reader::Reader(const int* ids, const int nbrMotors, ControlTableItem::ControlTableItemIndex item, Hal* hal, Dynamixel2Arduino* dxl)
: Handler(ids, nbrMotors, item, hal, dxl)
{
    checkBulkReadAvailability();

    // Prepare bulkRead if it can be used
    if (m_canUseBulkRead) {
        m_fbck_params = new int32_t[nbrMotors]();

        m_readerInfo.is_info_changed = true;
        m_readerInfo.xel_count = (uint8_t) nbrMotors;
        m_readerInfo.packet.p_buf = new uint8_t[BUFFER_SIZE]();
        m_readerInfo.packet.buf_capacity = BUFFER_SIZE;
        m_readerInfo.packet.is_completed = false;
        m_readerInfo.p_xels = new XELInfoBulkRead_t[nbrMotors]();

        for (int i=0; i<nbrMotors; i++) {
            m_readerInfo.p_xels[i].addr = (uint16_t) m_addr;
            m_readerInfo.p_xels[i].addr_length = (uint16_t) m_length;
            m_readerInfo.p_xels[i].id = (uint8_t) m_ids[i];
            m_readerInfo.p_xels[i].p_recv_buf = (uint8_t*)&m_fbck_params[i];
        }
    }
} 

/**
 * @brief       Read the control field of all the motors handled by this Reader
 * @param[out]  fbck Array for storing the read values, expressed in SI units
 * @retval      0 if reading successful, -1 if error
 * @note        Regardless of the control field type, the output will always be floats
 */
int Reader::read(float* fbck)
{
    m_readError = 0;

    if (m_canUseBulkRead)
        bulkRead(fbck);
    else    
        basicRead(fbck);

    return m_readError;
}

/**
 * @brief       Read the motor feedbacks using bulkRead
 * @param[out]  fbck Array for storing the read values
 * @note        Regardless of the control field type, the output will always be floats
 */
void Reader::bulkRead(float* fbck)
{
    int32_t parameter;
    uint8_t recv_count;

    recv_count = m_dxl->bulkRead(&m_readerInfo, TIMEOUT);

    // Get the feedback values and convert them to SI units
    if (recv_count > 0) {
        for (int i=0; i<m_nbrMotors; i++) {
            parameter = m_fbck_params[i];
            fbck[i] = parameter * m_units[i] - m_offsets[i];
        }
    }
    else {
        m_readError = -1;
        DEBUG_SERIAL.println("Error reading feedback"); 
        DEBUG_SERIAL.println(m_dxl->getLastLibErrCode());
    }
}

/**
 * @brief       Read the motor feedbacks using the basic read. Used when bulkRead cannot be used
 * @param[out]  fbck Array for storing the read values
 * @note        Regardless of the control field type, the output will always be floats
 */
void Reader::basicRead(float* fbck)
{
    int32_t recv_count;  // nbr of received parameters, -1 if fail
    int32_t parameter;

    for (int i=0; i<m_nbrMotors; i++) {
        recv_count = m_dxl->read( (uint8_t)m_ids[i], (uint16_t)m_addr, (uint16_t)m_length, (uint8_t*)&parameter, sizeof(parameter), TIMEOUT);

        if (recv_count == 1) {
            int8_t data = (int8_t) parameter;
            parameter = (int32_t) data;
        }
        else if (recv_count == 2) {
            int16_t data = (int16_t) parameter;
            parameter = (int32_t)data;
        }
        else {
            m_readError = -1;
            DEBUG_SERIAL.print("Error reading motor "); DEBUG_SERIAL.println(m_ids[i]); 
        }

        // Convert the read parameter into SI units
        fbck[i] = (float)parameter * m_units[i]  - m_offsets[i];
    }
}

/**
 * @brief   Check if it is possible to use bulkRead, which is faster than read
 */
void Reader::checkBulkReadAvailability()
{
    // bulkRead cannot be used in protocol 1 if single motor (theoretically)
    // https://emanual.robotis.com/docs/en/dxl/protocol1/
    // In practice, bulkRead did not work with protocol 1 at all, perhaps the old deprecated
    // version is needed, but it is not implemented

    if (m_hal->m_protocol == 1)
        m_canUseBulkRead = 0;
    else
        m_canUseBulkRead = 1;
}

}