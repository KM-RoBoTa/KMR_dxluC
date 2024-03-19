/**
 ******************************************************************************
 * @file            KMR_dxl_robot.cpp
 * @brief           Defines the BaseRobot class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
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



/**
 * @brief       Constructor for BaseRobot
 * @param[in]   all_ids List of IDs of all the motors in the robot
 * @param[in]   port_name Name of the port handling the communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   hal Previously initialized Hal object
 */

BaseRobot::BaseRobot(const int* ids, const int nbrMotors, const int baudrate, const int protocol_version, int* modes)
{
    m_hal = new Hal(protocol_version);
    m_nbrMotors = nbrMotors;
    m_ids = new int[nbrMotors];
    m_modelNumbers = new int[nbrMotors];
    m_protocolVersion = protocol_version;

    // Populate the ids array
    for (int i=0; i<nbrMotors; i++)
        m_ids[i] = ids[i];

    initMotors(baudrate, protocol_version, modes);

    // Use UART port of DYNAMIXEL Shield to debug.
    DEBUG_SERIAL.begin(115200);
    while(!DEBUG_SERIAL);

    m_hal->init(m_ids, m_nbrMotors, m_modelNumbers);
}

void BaseRobot::initMotors(const int baudrate, const int protocol_version, int* modes)
{
    m_dxl = new Dynamixel2Arduino(DXL_SERIAL, DXL_DIR_PIN);

    initComm(baudrate, protocol_version);
    setOperatingModes(modes);
    delay(1000);
    pingMotors();
}


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

void BaseRobot::setOperatingMode(int mode)
{
    for (int i=0; i<m_nbrMotors; i++)
        m_dxl->setOperatingMode(m_ids[i], mode);
}

void BaseRobot::setOperatingMode(int id, int mode)
{
    m_dxl->setOperatingMode(id, mode);
}

void BaseRobot::setOperatingModes(int* modes)
{
    for (int i=0; i<m_nbrMotors; i++)
        m_dxl->setOperatingMode(m_ids[i], modes[i]);
}

void BaseRobot::enableMotors()
{
    for (int i=0; i<m_nbrMotors; i++)
        m_dxl->torqueOn(m_ids[i]);
}

void BaseRobot::disableMotors()
{
    for (int i=0; i<m_nbrMotors; i++)
        m_dxl->torqueOff(m_ids[i]);
}

void BaseRobot::enableMotor(int id)
{
    m_dxl->torqueOn(id);
}

void BaseRobot::disableMotor(int id)
{
    m_dxl->torqueOff(id);
}

void BaseRobot::setGoalPositions(float* goal_positions)
{
    float offset;

    for (int i=0; i<m_nbrMotors; i++) {
        offset = m_hal->getPositionOffset(m_modelNumbers[i]);
        m_dxl->setGoalPosition(m_ids[i], (goal_positions[i]+offset)*180/M_PI, UNIT_DEGREE);
    }
}

void BaseRobot::setGoalPositions(int* goal_positions)
{
    for (int i=0; i<m_nbrMotors; i++)
        m_dxl->setGoalPosition(m_ids[i], goal_positions[i]);
}

void BaseRobot::getCurrentPositions(float* current_positions)
{
    float offset;

    for (int i=0; i<m_nbrMotors; i++)  {
        offset = m_hal->getPositionOffset(m_modelNumbers[i]);
        current_positions[i] = m_dxl->getPresentPosition(m_ids[i], UNIT_DEGREE)*M_PI/180 - offset;
    }
}

void BaseRobot::getCurrentPositions(int* current_positions)
{
    for (int i=0; i<m_nbrMotors; i++)
        current_positions[i] = m_dxl->getPresentPosition(m_ids[i]);
}


void BaseRobot::setMinAngles(float* minAngles)
{
    if (m_protocolVersion == 1)
        writeItem(minAngles, ControlTableItem::CW_ANGLE_LIMIT);
    else
        writeItem(minAngles, ControlTableItem::MIN_POSITION_LIMIT);
}

void BaseRobot::setMaxAngles(float* maxAngles)
{
    if (m_protocolVersion == 1)
        writeItem(maxAngles, ControlTableItem::CCW_ANGLE_LIMIT);
    else
        writeItem(maxAngles, ControlTableItem::MAX_POSITION_LIMIT);
}

void BaseRobot::setMinVoltage(float* minVoltage)
{
    writeItem(minVoltage, ControlTableItem::MIN_VOLTAGE_LIMIT);
}

void BaseRobot::setMaxVoltage(float* maxVoltage)
{
    writeItem(maxVoltage, ControlTableItem::MAX_VOLTAGE_LIMIT);
}

void BaseRobot::setMaxTorque(float* maxTorque)
{
    if (m_protocolVersion == 1)
        writeItem(maxTorque, ControlTableItem::MAX_TORQUE);
    else
        exit(1);
        // DEBUG PRINT
}

void BaseRobot::readItem(ControlTableItem::ControlTableItemIndex item, float* output)
{
    int modelNbr, id;
    Field field;
    int32_t parameter;
    float offset = 0; 

    for (int i=0; i<m_nbrMotors; i++) {
        id = m_ids[i];
        modelNbr = m_hal->getModelNumberFromID(id);

        if (item == ControlTableItem::GOAL_POSITION ||
            item == ControlTableItem::PRESENT_POSITION ||
            item == ControlTableItem::MIN_POSITION_LIMIT || 
            item == ControlTableItem::MAX_POSITION_LIMIT ||
            item == ControlTableItem::HOMING_OFFSET)
            offset = m_hal->getPositionOffset(modelNbr);
        else    
            offset = 0;

        field = m_hal->getControlFieldFromModel(modelNbr, item);
        parameter = m_dxl->readControlTableItem(item, id);
        output[i] = parameter * field.unit - offset; 
    }
}



