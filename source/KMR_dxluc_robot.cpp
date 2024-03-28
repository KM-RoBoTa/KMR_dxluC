/**
 ******************************************************************************
 * @file            KMR_dxl_robot.cpp
 * @brief           Defines the BaseRobot class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors kamilo.melo@km-robota.com, 03/2024
 * @authors katarina.lichardova@km-robota.com, 03/2024
 ******************************************************************************
 */

#include "../include/KMR_dxluc_robot.hpp"

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

namespace KMR_dxluC
{

/**
 * @brief       Constructor for BaseRobot
 * @param[in]   ids List of IDs of all the motors in the robot
 * @param[in]   nbrMotors Number of motors in the robot
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   protocol_version Dynamixel protocol version (1 or 2)
 */
BaseRobot::BaseRobot(const int* ids, const int nbrMotors, const int baudrate, const int protocol_version)
{
    m_hal = new Hal(protocol_version);
    m_nbrMotors = nbrMotors;
    m_ids = new int[nbrMotors];
    m_modelNumbers = new int[nbrMotors];
    m_protocolVersion = protocol_version;

    // Populate the ids array
    for (int i=0; i<nbrMotors; i++)
        m_ids[i] = ids[i];

    initMotors(baudrate, protocol_version);

    m_hal->init(m_ids, m_nbrMotors, m_modelNumbers);
}


/**
 * @brief       Initialize the motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   protocol_version Dynamixel protocol version (1 or 2)
 */
void BaseRobot::initMotors(const int baudrate, const int protocol_version)
{
    m_dxl = new Dynamixel2Arduino(DXL_SERIAL, DXL_DIR_PIN);

    initComm(baudrate, protocol_version);
    delay(1000);
    pingMotors();
}


/**
 * @brief       Ping all motors to check the communication is working
 */
void BaseRobot::pingMotors()
{
    for (int i=0; i<m_nbrMotors; i++) {
        if(m_dxl->ping(m_ids[i]) == true) {
            m_modelNumbers[i] = m_dxl->getModelNumber(m_ids[i]);
            DEBUG_SERIAL.print("Ping succeeded for id ");
            DEBUG_SERIAL.print(m_ids[i]);
            DEBUG_SERIAL.print(", model number: ");
            DEBUG_SERIAL.println(m_modelNumbers[i]);
        }
        else {
            DEBUG_SERIAL.print("Error! Motor ");
            DEBUG_SERIAL.print(m_ids[i]);
            DEBUG_SERIAL.println(" not responding");
            exit(1);
        }
    }
}


/**
 * @brief       Initialize the communication with the motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   protocol_version Dynamixel protocol version (1 or 2)
 */
void BaseRobot::initComm(const int baudrate, const int protocol_version)
{
    // Set port baudrate
    m_dxl->begin(DEFAULT_BAUDRATE);

    if (baudrate != DEFAULT_BAUDRATE) {
        disableMotors();

        for (int i=0; i<m_nbrMotors; i++)
            m_dxl->setBaudrate(m_ids[i], baudrate);

        // Change communication to the new baudrate
        m_dxl->begin(baudrate);
    } 

    // Change the protocol version
    m_dxl->setPortProtocolVersion(protocol_version);

    // Disable motors
    disableMotors();
}


/*
******************************************************************************
 *                         Enable/disable motors
 ****************************************************************************/

/**
 * @brief   Enable all motors
 */
void BaseRobot::enableMotors()
{
    for (int i=0; i<m_nbrMotors; i++)
        m_dxl->torqueOn(m_ids[i]);
}

/**
 * @brief   Disable all motors
 */
void BaseRobot::disableMotors()
{
    for (int i=0; i<m_nbrMotors; i++)
        m_dxl->torqueOff(m_ids[i]);
}

/**
 * @brief   Enable a specific motor
 */
void BaseRobot::enableMotor(int id)
{
    m_dxl->torqueOn(id);
}

/**
 * @brief   Disable a specific motor
 */
void BaseRobot::disableMotor(int id)
{
    m_dxl->torqueOff(id);
}


/*
******************************************************************************
 *                         EEPROM writing functions
 ****************************************************************************/

/**
 * @brief       Set the minimum angle limits for all motors
 * @param[in]   minAngles Minimum angle limit for each motor [rad]
 */
void BaseRobot::setMinAngles(float* minAngles)
{
    if (m_protocolVersion == 1)
        EEPROM_writer = new Writer(m_ids, m_nbrMotors, ControlTableItem::CW_ANGLE_LIMIT, m_hal, m_dxl);
    else
        EEPROM_writer = new Writer(m_ids, m_nbrMotors, ControlTableItem::MIN_POSITION_LIMIT, m_hal, m_dxl);

    EEPROM_writer->write(minAngles);
    delay(30);
}

/**
 * @brief       Set the maximum angle limits for all motors
 * @param[in]   maxAngles Maximum angle limit for each motor [rad]
 */
void BaseRobot::setMaxAngles(float* maxAngles)
{
    if (m_protocolVersion == 1)
        EEPROM_writer = new Writer(m_ids, m_nbrMotors, ControlTableItem::CCW_ANGLE_LIMIT, m_hal, m_dxl);
    else
        EEPROM_writer = new Writer(m_ids, m_nbrMotors, ControlTableItem::MAX_POSITION_LIMIT, m_hal, m_dxl);

    EEPROM_writer->write(maxAngles);
    delay(30);
}

/**
 * @brief       Set the minimum voltage limits for all motors
 * @param[in]   minVoltages Minimum voltage limit for each motor [V]
 */
void BaseRobot::setMinVoltages(float* minVoltages)
{
    EEPROM_writer = new Writer(m_ids, m_nbrMotors, ControlTableItem::MIN_VOLTAGE_LIMIT, m_hal, m_dxl);

    EEPROM_writer->write(minVoltages);
    delay(30);
}

/**
 * @brief       Set the maximum voltage limits for all motors
 * @param[in]   maxVoltages Maximum voltage limit for each motor [V]
 */
void BaseRobot::setMaxVoltages(float* maxVoltages)
{
    EEPROM_writer = new Writer(m_ids, m_nbrMotors, ControlTableItem::MAX_VOLTAGE_LIMIT, m_hal, m_dxl);

    EEPROM_writer->write(maxVoltages);
    delay(30);
}


/**
 * @brief       Set the maximum torque limits for all motors
 * @note        !! Only supported in protocol 1 !!
 * @param[in]   maxTorques Maximum torque limit for each motor [Nm]
 */
void BaseRobot::setMaxTorques(float* maxTorques)
{
    if (m_protocolVersion == 1)  {
        EEPROM_writer = new Writer(m_ids, m_nbrMotors, ControlTableItem::MAX_TORQUE, m_hal, m_dxl);
        EEPROM_writer->write(maxTorques);
        delay(30);
    }
    else {
        DEBUG_SERIAL.print("Error! Max torque setting is not supported in protocol 2. Exiting....");
        exit(1);
    }
}

/** 
 * @brief       Set the operating modes of all motors
 * @param[in]   modes Control modes for each motor
 * @note        Modes: current = 0; velocity = 1; position = 3; extended position = 4;
 *              current-based position = 5; PWM = 16 
 */
void BaseRobot::setOperatingModes(int* modes)
{
    if (m_protocolVersion == 2) {
        for (int i=0; i<m_nbrMotors; i++) {
            m_dxl->setOperatingMode(m_ids[i], modes[i]);
            delay(30);
        }
    }
    else {
        // error message
    }
}

/**
 * @brief       Set the return times to all motors
 * @param[in]   returnTimes Return times of the motors [s]
 */
void BaseRobot::setReturnTime(float* returnTimes)
{
    EEPROM_writer = new Writer(m_ids, m_nbrMotors, ControlTableItem::RETURN_DELAY_TIME, m_hal, m_dxl);

    EEPROM_writer->write(returnTimes);
    delay(30);    
}

}



