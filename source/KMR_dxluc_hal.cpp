/**
 ******************************************************************************
 * @file            KMR_dxluc_hal.hpp
 * @brief           Header for the KMR_dxluc_hal.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#define POS_OFFSET_DEFAULT 3.14159265358979323846264338327950288 // M_PI
#define POS_OFFSET_AX_12A 2.61799387799 // 130 degrees in radians 

#define MODEL_NBR_AX_12A 12

#include "../include/KMR_dxluc_hal.hpp"

/**
 * @brief       Constructor for BaseRobot
 * @param[in]   all_ids List of IDs of all the motors in the robot
 * @param[in]   port_name Name of the port handling the communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   hal Previously initialized Hal object
 */
Hal::Hal(int protocol_version)
{
    m_protocol = protocol_version;

    // Create control tables 
    if (protocol_version == 1) {
        MX_64 = new MX_64_P1();
    }
    else {
        MX_64 = new MX_64_P2();
    }
}

void Hal::init(int* ids, int nbrMotors, int* models)
{
    m_nbrMotors = nbrMotors;
    m_ids = ids;
    m_models =  models;
}

ControlTable Hal::getControlTable(int modelNumber)
{
    ControlTable motor;

    switch (modelNumber) {
    case 310: // MX_64
        motor = *MX_64; break;
    case 311: // MX_64AR
        motor = *MX_64; break;
    default:
        //DEBUG_SERIAL.println("Error: this model is unknown");
        exit(1);
    }

    return motor;
}


Field Hal::getControlFieldFromModel(int modelNumber, ControlTableItem::ControlTableItemIndex item)
{
    ControlTable motor = getControlTable(modelNumber);
    Field field = getControlField(motor, item);

    return field;
}

int Hal::getModelNumberFromID(int id)
{
    int i;
    for (i=0; i<m_nbrMotors; i++) {
        if (id == m_ids[i])
            break;
    }

    return m_models[i];
}


Field Hal::getControlField(ControlTable motor, ControlTableItem::ControlTableItemIndex item)
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
        //DEBUG_SERIAL.println("Error: this field is unknown");
        exit(1);
    }    

    if (field.length == UNDEF) {
        //DEBUG_SERIAL.println("Error: this field does not exist for this motor or protocol!");
        exit(1);
    }

    return field;
}

float Hal::getPositionOffset(int modelNumber)
{
    float offset = 0;

    if (modelNumber != MODEL_NBR_AX_12A)
        offset = POS_OFFSET_DEFAULT;
    else
        offset = POS_OFFSET_AX_12A;
            
    return offset;
}
