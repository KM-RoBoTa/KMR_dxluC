
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

#define POS_OFFSET_DEFAULT 3.14159265358979323846264338327950288 // M_PI
#define POS_OFFSET_AX_12A 2.61799387799 // 130 degrees in radians 


/**
 * @brief       Constructor for BaseRobot
 * @param[in]   all_ids List of IDs of all the motors in the robot
 * @param[in]   port_name Name of the port handling the communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   hal Previously initialized Hal object
 */
void BaseRobot::init(const int* ids, const int nbrMotors, const int baudrate, const int protocol_version, int* modes)
{
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

    // Create control tables 
    if (protocol_version == 1) {
        MX_64 = new MX_64_P1();
    }
    else {
        MX_64 = new MX_64_P2();
    }
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
        offset = getPositionOffset(m_modelNumbers[i]);
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
        offset = getPositionOffset(m_modelNumbers[i]);
        current_positions[i] = m_dxl->getPresentPosition(m_ids[i], UNIT_DEGREE)*M_PI/180 - offset;
    }
}

void BaseRobot::getCurrentPositions(int* current_positions)
{
    for (int i=0; i<m_nbrMotors; i++)
        current_positions[i] = m_dxl->getPresentPosition(m_ids[i]);
}

float BaseRobot::getPositionOffset(int modelNumber)
{
    float offset = 0;

    if (modelNumber != 12)
        offset = POS_OFFSET_DEFAULT;
    else
        offset = POS_OFFSET_AX_12A;
            
    return offset;
}

/*
void BaseRobot::setMinAngles(float* minAngles)
{
    ControlTable motor;

    if (m_protocolVersion == 1) {
        for (int i=0; i<m_nbrMotors; i++) {
            motor = getControlTable(m_modelNumbers[i]);

            m_dxl->writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, 1, 1);
        }
            
    }
}*/

Field BaseRobot::getControlFieldFromModel(int modelNumber, ControlTableItem::ControlTableItemIndex item)
{
    ControlTable motor = getControlTable(modelNumber);
    Field field = getControlField(motor, item);

    return field;
}

int BaseRobot::getModelNumberFromID(int id)
{
    int i;
    for (i=0; i<m_nbrMotors; i++) {
        if (id == m_ids[i])
            break;
    }

    return m_modelNumbers[i];
}

ControlTable BaseRobot::getControlTable(int modelNumber)
{
    ControlTable motor;

    switch (modelNumber) {
    case 310: // MX_64
        motor = *MX_64; break;
    case 311: // MX_64AR
        motor = *MX_64; break;
    //case 3:
    //    cout << "Wednesday";
    //    break;
    //case 4:
    //    cout << "Thursday";
    //    break;
    //case 5:
    //    cout << "Friday";
    //    break;
    //case 6:
    //    cout << "Saturday";
    //    break;
    //case 7:
    //    cout << "Sunday";
    //    break;
    default:
        DEBUG_SERIAL.println("Error: this model is unknown");
        exit(1);
    }

    return motor;
}

Field BaseRobot::getControlField(ControlTable motor, ControlTableItem::ControlTableItemIndex item)
{
    Field field;

    switch (item) {
    case ControlTableItem::MODEL_NUMBER:         field = motor.modelNumber;         break;
    case ControlTableItem::MODEL_INFORMATION:    field = motor.modelInfo;           break;
    case ControlTableItem::FIRMWARE_VERSION:     field = motor.firmwareVersion;     break;
    case ControlTableItem::PROTOCOL_VERSION:     field = motor.protocolVersion;     break;
    case ControlTableItem::ID:                   field = motor.id;                  break;
    case ControlTableItem::SECONDARY_ID:         field = motor.secondaryId;         break;
    case ControlTableItem::BAUD_RATE:            field = motor.baudrate;            break;
    case ControlTableItem::DRIVE_MODE:           field = motor.driveMode;           break;
    case ControlTableItem::CONTROL_MODE:         field = motor.controlMode;         break;
    case ControlTableItem::OPERATING_MODE:       field = motor.operatingMode;       break;
    case ControlTableItem::CW_ANGLE_LIMIT:       field = motor.CW_angleLimit;       break;
    case ControlTableItem::CCW_ANGLE_LIMIT:      field = motor.CCW_angleLimit;      break;
    case ControlTableItem::TEMPERATURE_LIMIT:    field = motor.temperatureLimit;    break;
    case ControlTableItem::MIN_VOLTAGE_LIMIT:    field = motor.minVoltageLimit;     break;
    case ControlTableItem::MAX_VOLTAGE_LIMIT:    field = motor.maxVoltageLimit;     break;
    case ControlTableItem::PWM_LIMIT:            field = motor.PWM_limit;           break;
    case ControlTableItem::CURRENT_LIMIT:        field = motor.currentLimit;        break;
    case ControlTableItem::VELOCITY_LIMIT:       field = motor.velocityLimit;       break;
    case ControlTableItem::MAX_POSITION_LIMIT:   field = motor.maxPositionLimit;    break;
    case ControlTableItem::MIN_POSITION_LIMIT:   field = motor.minPositionLimit;    break;
    case ControlTableItem::ACCELERATION_LIMIT:   field = motor.accelerationLimit;   break;
    case ControlTableItem::MAX_TORQUE:           field = motor.maxTorque;           break;
    case ControlTableItem::HOMING_OFFSET:        field = motor.homingOffset;        break;
    case ControlTableItem::MOVING_THRESHOLD:     field = motor.movingThreshold;     break;
    case ControlTableItem::MULTI_TURN_OFFSET:    field = motor.multiturnOffset;     break;
    case ControlTableItem::RESOLUTION_DIVIDER:   field = motor.resolutionDivider;   break;
    case ControlTableItem::EXTERNAL_PORT_MODE_1: field = motor.externalPortMode1;   break;
    case ControlTableItem::EXTERNAL_PORT_MODE_2: field = motor.externalPortMode2;   break;
    case ControlTableItem::EXTERNAL_PORT_MODE_3: field = motor.externalPortMode3;   break;
    case ControlTableItem::EXTERNAL_PORT_MODE_4: field = motor.externalPortMode4;   break;
    case ControlTableItem::STATUS_RETURN_LEVEL:  field = motor.statusReturnLevel;   break;
    case ControlTableItem::RETURN_DELAY_TIME:    field = motor.returnDelayTime;     break;
    case ControlTableItem::ALARM_LED:            field = motor.alarmLed;            break;
    case ControlTableItem::SHUTDOWN:             field = motor.shutdown;            break;

    case ControlTableItem::TORQUE_ENABLE: field = motor.torqueEnable; break;
    case ControlTableItem::LED: field = motor.LED; break;
    case ControlTableItem::LED_RED: field = motor.LED_red; break;
    case ControlTableItem::LED_GREEN: field = motor.LED_green; break;
    case ControlTableItem::LED_BLUE: field = motor.LED_blue; break;
    case ControlTableItem::REGISTERED_INSTRUCTION: field = motor.registeredInstruction; break;
    case ControlTableItem::HARDWARE_ERROR_STATUS: field = motor.hardwareErrorStatus; break;
    case ControlTableItem::VELOCITY_P_GAIN: field = motor.velocity_P_gain; break;
    case ControlTableItem::VELOCITY_I_GAIN: field = motor.velocity_I_gain; break;
    case ControlTableItem::POSITION_P_GAIN: field = motor.position_P_gain; break;
    case ControlTableItem::POSITION_I_GAIN: field = motor.position_I_gain; break;
    case ControlTableItem::POSITION_D_GAIN: field = motor.position_D_gain; break;
    case ControlTableItem::FEEDFORWARD_1ST_GAIN: field = motor.feedforward_1_gain; break;
    case ControlTableItem::FEEDFORWARD_2ND_GAIN: field = motor.feedforward_2_gain; break;
    case ControlTableItem::P_GAIN: field = motor.P_gain; break;
    case ControlTableItem::I_GAIN: field = motor.I_gain; break;
    case ControlTableItem::D_GAIN: field = motor.D_gain; break;
    case ControlTableItem::CW_COMPLIANCE_MARGIN: field = motor.CW_complianceMargin; break;
    case ControlTableItem::CCW_COMPLIANCE_MARGIN: field = motor.CCW_complianceMargin; break;
    case ControlTableItem::CW_COMPLIANCE_SLOPE: field = motor.CW_complianceSlope; break;
    case ControlTableItem::CCW_COMPLIANCE_SLOPE: field = motor.CCW_complianceSlope; break;
    case ControlTableItem::GOAL_PWM: field = motor.goalPWM; break;
    case ControlTableItem::GOAL_TORQUE: field = motor.goalTorque; break;
    case ControlTableItem::GOAL_CURRENT: field = motor.goalCurrent; break;
    case ControlTableItem::GOAL_POSITION: field = motor.goalPosition; break;
    case ControlTableItem::GOAL_VELOCITY: field = motor.goalVelocity; break;
    case ControlTableItem::GOAL_ACCELERATION: field = motor.goalAcceleration; break;
    case ControlTableItem::MOVING_SPEED: field = motor.movingSpeed; break;
    case ControlTableItem::PRESENT_PWM: field = motor.presentPWM; break;
    case ControlTableItem::PRESENT_LOAD: field = motor.presentLoad; break;
    case ControlTableItem::PRESENT_SPEED: field = motor.presentSpeed; break;
    case ControlTableItem::PRESENT_CURRENT: field = motor.presentCurrent; break;
    case ControlTableItem::PRESENT_POSITION: field = motor.presentPosition; break;
    case ControlTableItem::PRESENT_VELOCITY: field = motor.presentVelocity; break;
    case ControlTableItem::PRESENT_VOLTAGE: field = motor.presentVoltage; break;
    case ControlTableItem::PRESENT_TEMPERATURE: field = motor.presentTemperature; break;
    case ControlTableItem::TORQUE_LIMIT: field = motor.torqueLimit; break;
    case ControlTableItem::REGISTERED: field = motor.registered; break;
    case ControlTableItem::MOVING: field = motor.moving; break;
    case ControlTableItem::LOCK: field = motor.lock; break;
    case ControlTableItem::PUNCH: field = motor.punch; break;
    case ControlTableItem::CURRENT: field = motor.current; break;
    case ControlTableItem::SENSED_CURRENT: field = motor.sensedCurrent; break;
    case ControlTableItem::REALTIME_TICK: field = motor.realtimeTick; break;
    case ControlTableItem::TORQUE_CTRL_MODE_ENABLE: field = motor.torqueCtrlModeEnabled; break;
    case ControlTableItem::BUS_WATCHDOG: field = motor.busWatchdog; break;
    case ControlTableItem::PROFILE_ACCELERATION: field = motor.profileAcceleration; break;
    case ControlTableItem::PROFILE_VELOCITY: field = motor.profileVelocity; break;
    case ControlTableItem::MOVING_STATUS: field = motor.movingStatus; break;
    case ControlTableItem::VELOCITY_TRAJECTORY: field = motor.velocityTrajectory; break;
    case ControlTableItem::POSITION_TRAJECTORY: field = motor.positionTrajectory; break;
    case ControlTableItem::PRESENT_INPUT_VOLTAGE: field = motor.presentInputVoltage; break;
    case ControlTableItem::EXTERNAL_PORT_DATA_1: field = motor.externalPortData1; break;
    case ControlTableItem::EXTERNAL_PORT_DATA_2: field = motor.externalPortData2; break;
    case ControlTableItem::EXTERNAL_PORT_DATA_3: field = motor.externalPortData3; break;
    case ControlTableItem::EXTERNAL_PORT_DATA_4: field = motor.externalPortData4; break;

    default:
        DEBUG_SERIAL.println("Error: this field is unknown");
        exit(1);
    }    

    if (field.length == UNDEF) {
        DEBUG_SERIAL.println("Error: this field does not exist for this motor or protocol!");
        exit(1);
    }

    return field;
}

void BaseRobot::readItem(ControlTableItem::ControlTableItemIndex item, float* output)
{
    int modelNbr, id;
    Field field;
    int32_t parameter;
    float offset = 0; 

    for (int i=0; i<m_nbrMotors; i++) {
        id = m_ids[i];
        modelNbr = getModelNumberFromID(id);

        if (item == ControlTableItem::GOAL_POSITION ||
            item == ControlTableItem::PRESENT_POSITION ||
            item == ControlTableItem::MIN_POSITION_LIMIT || 
            item == ControlTableItem::MAX_POSITION_LIMIT ||
            item == ControlTableItem::HOMING_OFFSET)
            offset = getPositionOffset(modelNbr);
        else    
            offset = 0;

        field = getControlFieldFromModel(modelNbr, item);
        parameter = m_dxl->readControlTableItem(item, id);
        output[i] = parameter * field.unit - offset; 
    }
}


