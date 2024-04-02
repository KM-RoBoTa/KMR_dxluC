/**
 ******************************************************************************
 * @file            KMR_dxluc_control_tables.hpp
 * @brief           Header file containing control tables of different motors
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 03/2024
 * @authors kamilo.melo@km-robota.com, 03/2024
 ******************************************************************************
 */

#ifndef KMR_DLXUC_CONTROL_TABLES_HPP
#define KMR_DLXUC_CONTROL_TABLES_HPP

#include "KMR_dxluc_structures.hpp"

namespace KMR_dxluC
{
#define MODEL_NBR_AX_12A    12
#define MODEL_NBR_MX_64_1   310
#define MODEL_NBR_MX_64_2   311
#define MODEL_NBR_MX_106_1  320
#define MODEL_NBR_MX_106_2  321


struct MX_64_P1 : ControlTable {
    MX_64_P1 ()
    {
        modelNumber.addr = 0;
        modelNumber.length = 2;
        modelNumber.unit = 1;
        firmwareVersion.addr = 2;
        firmwareVersion.length = 1;
        firmwareVersion.unit = 1;
        id.addr = 3;
        id.length = 1;
        id.unit = 1;
        baudrate.addr = 4;
        baudrate.length = 1;
        baudrate.unit = 1;
        returnDelayTime.addr = 5;
        returnDelayTime.length = 1;
        returnDelayTime.unit = 0.000002;
        CW_angleLimit.addr = 6;
        CW_angleLimit.length = 2;
        CW_angleLimit.unit = 0.001536;
        CCW_angleLimit.addr = 8;
        CCW_angleLimit.length = 2;
        CCW_angleLimit.unit = 0.001536;
        temperatureLimit.addr = 11;    
        temperatureLimit.length = 1;
        temperatureLimit.unit = 1;
        minVoltageLimit.addr = 12;
        minVoltageLimit.length = 1;
        minVoltageLimit.unit = 0.1;
        maxVoltageLimit.addr = 13;
        maxVoltageLimit.length = 1;
        maxVoltageLimit.unit = 0.1;
        maxTorque.addr = 14;
        maxTorque.length = 2;
        maxTorque.unit = 0.1;
        statusReturnLevel.addr = 16;
        statusReturnLevel.length = 1;
        statusReturnLevel.unit = 1;
        alarmLed.addr = 17;
        alarmLed.length = 1;
        alarmLed.unit = 1;
        shutdown.addr = 18;
        shutdown.length = 1;
        shutdown.unit = 1;
        multiturnOffset.addr = 20;
        multiturnOffset.length = 2;
        multiturnOffset.unit = 1;
        resolutionDivider.addr = 22;
        resolutionDivider.length = 1;
        resolutionDivider.unit = 1;

        torqueEnable.addr = 24; 
        torqueEnable.length = 1;
        torqueEnable.unit = 1;
        LED.addr = 25;
        LED.length = 1;
        LED.unit = 1;
        D_gain.addr = 26;
        D_gain.length = 1;
        D_gain.unit = 1;
        I_gain.addr = 27;
        I_gain.length = 1;
        I_gain.unit = 1;
        P_gain.addr = 28;
        P_gain.length = 1;
        P_gain.unit = 1;
        goalPosition.addr = 30;
        goalPosition.length = 2;
        goalPosition.unit = 0.001536;
        movingSpeed.addr = 32;
        movingSpeed.length = 2;
        movingSpeed.unit =  0.0119;
        torqueLimit.addr = 34;
        torqueLimit.length = 2;
        torqueLimit.unit = 0.1;
        presentPosition.addr = 36;
        presentPosition.length = 2;
        presentPosition.unit = 0.001536;
        presentSpeed.addr = 38;
        presentSpeed.length = 2;
        presentSpeed.unit = 0.0115;
        presentLoad.addr = 40;
        presentLoad.length = 2;
        presentLoad.unit = 0.1;
        presentInputVoltage.addr = 42;
        presentInputVoltage.length = 1;
        presentInputVoltage.unit = 0.1;
        presentTemperature.addr = 43;
        presentTemperature.length = 1;
        presentTemperature.unit = 1;
        registered.addr = 44;
        registered.length = 1;
        registered.unit = 1;
        moving.addr = 46;
        moving.length = 1;
        moving.unit = 1;
        lock.addr = 47;
        lock.length = 1;
        lock.unit = 1;
        punch.addr = 48;
        punch.length = 2;
        punch.unit = 1;
        realtimeTick.addr = 50;
        realtimeTick.length = 2;
        realtimeTick.unit = 0.001;
        current.addr = 68;
        current.length = 2;
        current.unit = 0.0045;
        torqueCtrlModeEnabled.addr = 70;
        torqueCtrlModeEnabled.length = 1;
        torqueCtrlModeEnabled.unit = 1;
        goalTorque.addr = 71;
        goalTorque.length = 2;
        goalTorque.unit = 0.0045;
        goalAcceleration.addr = 73;
        goalAcceleration.length = 1;
        goalAcceleration.unit = 0.0175;
    }
};


struct MX_64_P2 : ControlTable {
    MX_64_P2 ()
    {
        modelNumber.addr = 0;
        modelNumber.length = 2;
        modelNumber.unit = 1;
        modelInfo.addr = 2;
        modelInfo.length = 4;
        modelInfo.unit = 1;
        firmwareVersion.addr = 6;
        firmwareVersion.length = 1;
        firmwareVersion.unit = 1;
        id.addr = 7;
        id.length = 1;
        id.unit = 1;
        baudrate.addr = 8;
        baudrate.length = 1;
        baudrate.unit = 1;
        returnDelayTime.addr = 9;
        returnDelayTime.length = 1;
        returnDelayTime.unit = 0.000002;
        driveMode.addr = 10;
        driveMode.length = 1;
        driveMode.unit = 1;
        operatingMode.addr = 11;
        operatingMode.length = 1;
        operatingMode.unit = 1;
        secondaryId.addr = 12;
        secondaryId.length = 1;
        secondaryId.unit = 1;
        protocolVersion.addr = 13;
        protocolVersion.length = 1;
        protocolVersion.unit = 1;
        homingOffset.addr = 20;
        homingOffset.length = 4;
        homingOffset.unit = 0.001536;
        movingThreshold.addr = 24;
        movingThreshold.length = 4;
        movingThreshold.unit = 0.0240;
        temperatureLimit.addr = 31;
        temperatureLimit.length = 1;
        temperatureLimit.unit = 1;
        maxVoltageLimit.addr = 32;
        maxVoltageLimit.length = 2;
        maxVoltageLimit.unit = 0.1;
        minVoltageLimit.addr = 34;
        minVoltageLimit.length = 2;
        minVoltageLimit.unit = 0.1;
        PWM_limit.addr = 36;
        PWM_limit.length = 2;
        PWM_limit.unit = 0.113;
        currentLimit.addr = 38;
        currentLimit.length = 2;
        currentLimit.unit = 0.00336;
        accelerationLimit.addr = 40;
        accelerationLimit.length = 4;
        accelerationLimit.unit = 0.3745;
        velocityLimit.addr = 44;
        velocityLimit.length = 4;
        velocityLimit.unit = 0.0240;
        maxPositionLimit.addr = 48;
        maxPositionLimit.length = 4;
        maxPositionLimit.unit = 0.001536;
        minPositionLimit.addr = 52;
        minPositionLimit.length = 4;
        minPositionLimit.unit = 0.001536;        
        shutdown.addr = 63;
        shutdown.length = 1;
        shutdown.unit = 1;
        torqueEnable.addr = 64;
        torqueEnable.length = 1;
        torqueEnable.unit = 1;
        LED.addr = 65;
        LED.length = 1;
        LED.unit = 1;
        statusReturnLevel.addr = 68;
        statusReturnLevel.length = 1;
        statusReturnLevel.unit = 1;
        registered.addr = 69;
        registered.length = 1;
        registered.unit = 1;
        hardwareErrorStatus.addr = 70;
        hardwareErrorStatus.length = 1;
        hardwareErrorStatus.unit = 1;
        velocity_I_gain.addr = 76;
        velocity_I_gain.length = 2;
        velocity_I_gain.unit = 1;
        velocity_P_gain.addr = 78;
        velocity_P_gain.length = 2;
        velocity_P_gain.unit = 1;
        position_D_gain.addr = 80;
        position_D_gain.length = 2;
        position_D_gain.unit = 1;
        position_I_gain.addr = 82;
        position_I_gain.length = 2;
        position_I_gain.unit = 1;
        position_P_gain.addr = 84;
        position_P_gain.length = 2;
        position_P_gain.unit = 1;
        feedforward_2_gain.addr = 88;
        feedforward_2_gain.length = 2;
        feedforward_2_gain.unit = 1;
        feedforward_1_gain.addr = 90;
        feedforward_1_gain.length = 2;
        feedforward_1_gain.unit = 1;
        busWatchdog.addr = 98;
        busWatchdog.length = 1;
        busWatchdog.unit = 1;
        goalPWM.addr = 100;
        goalPWM.length = 2;
        goalPWM.unit = 0.113;
        goalCurrent.addr = 102;
        goalCurrent.length = 2;
        goalCurrent.unit = 0.00336;
        goalVelocity.addr = 104;
        goalVelocity.length = 4;
        goalVelocity.unit = 0.0240;
        profileAcceleration.addr = 108;
        profileAcceleration.length = 4;
        profileAcceleration.unit = 214.577;
        profileVelocity.addr = 112;
        profileVelocity.length = 4;
        profileVelocity.unit = 0.0240;
        goalPosition.addr = 116;
        goalPosition.length = 4;
        goalPosition.unit = 0.001536;
        realtimeTick.addr = 120;
        realtimeTick.length = 2;
        realtimeTick.unit = 0.001;
        moving.addr = 122;
        moving.length = 1;
        moving.unit = 1;
        movingStatus.addr = 123;
        movingStatus.length = 1;
        movingStatus.unit = 1;
        presentPWM.addr = 124;
        presentPWM.length = 2;
        presentPWM.unit = 0.113;
        presentCurrent.addr = 126;
        presentCurrent.length = 2;
        presentCurrent.unit = 0.00336;
        presentVelocity.addr = 128;
        presentVelocity.length = 4;
        presentVelocity.unit = 0.0240;
        presentPosition.addr = 132;
        presentPosition.length = 4;
        presentPosition.unit = 0.001536;
        velocityTrajectory.addr = 136;    
        velocityTrajectory.length = 4;
        velocityTrajectory.unit = 1;
        positionTrajectory.addr = 140;
        positionTrajectory.length = 4;
        positionTrajectory.unit = 1;
        presentInputVoltage.addr = 144;
        presentInputVoltage.length = 2;
        presentInputVoltage.unit = 0.1;
        presentTemperature.addr = 146;
        presentTemperature.length = 1;
        presentTemperature.unit = 1; 
    }
};

struct MX_106_P1 : ControlTable {
    MX_106_P1 ()
    {
        modelNumber.addr = 0;
        modelNumber.length = 2;
        modelNumber.unit = 1;
        firmwareVersion.addr = 2;
        firmwareVersion.length = 1;
        firmwareVersion.unit = 1;
        id.addr = 3;
        id.length = 1;
        id.unit = 1;
        baudrate.addr = 4;
        baudrate.length = 1;
        baudrate.unit = 1;
        returnDelayTime.addr = 5;
        returnDelayTime.length = 1;
        returnDelayTime.unit = 0.000002;
        CW_angleLimit.addr = 6;
        CW_angleLimit.length = 2;
        CW_angleLimit.unit = 0.001536;
        CCW_angleLimit.addr = 8;
        CCW_angleLimit.length = 2;
        CCW_angleLimit.unit = 0.001536;
        temperatureLimit.addr = 11;    
        temperatureLimit.length = 1;
        temperatureLimit.unit = 1;
        minVoltageLimit.addr = 12;
        minVoltageLimit.length = 1;
        minVoltageLimit.unit = 0.1;
        maxVoltageLimit.addr = 13;
        maxVoltageLimit.length = 1;
        maxVoltageLimit.unit = 0.1;
        maxTorque.addr = 14;
        maxTorque.length = 2;
        maxTorque.unit = 0.1;
        statusReturnLevel.addr = 16;
        statusReturnLevel.length = 1;
        statusReturnLevel.unit = 1;
        alarmLed.addr = 17;
        alarmLed.length = 1;
        alarmLed.unit = 1;
        shutdown.addr = 18;
        shutdown.length = 1;
        shutdown.unit = 1;
        multiturnOffset.addr = 20;
        multiturnOffset.length = 2;
        multiturnOffset.unit = 1;
        resolutionDivider.addr = 22;
        resolutionDivider.length = 1;
        resolutionDivider.unit = 1;

        torqueEnable.addr = 24; 
        torqueEnable.length = 1;
        torqueEnable.unit = 1;
        LED.addr = 25;
        LED.length = 1;
        LED.unit = 1;
        D_gain.addr = 26;
        D_gain.length = 1;
        D_gain.unit = 1;
        I_gain.addr = 27;
        I_gain.length = 1;
        I_gain.unit = 1;
        P_gain.addr = 28;
        P_gain.length = 1;
        P_gain.unit = 1;
        goalPosition.addr = 30;
        goalPosition.length = 2;
        goalPosition.unit = 0.001536;
        movingSpeed.addr = 32;
        movingSpeed.length = 2;
        movingSpeed.unit =  0.0119;
        torqueLimit.addr = 34;
        torqueLimit.length = 2;
        torqueLimit.unit = 0.1;
        presentPosition.addr = 36;
        presentPosition.length = 2;
        presentPosition.unit = 0.001536;
        presentSpeed.addr = 38;
        presentSpeed.length = 2;
        presentSpeed.unit = 0.0115;
        presentLoad.addr = 40;
        presentLoad.length = 2;
        presentLoad.unit = 0.1;
        presentInputVoltage.addr = 42;
        presentInputVoltage.length = 1;
        presentInputVoltage.unit = 0.1;
        presentTemperature.addr = 43;
        presentTemperature.length = 1;
        presentTemperature.unit = 1;
        registered.addr = 44;
        registered.length = 1;
        registered.unit = 1;
        moving.addr = 46;
        moving.length = 1;
        moving.unit = 1;
        lock.addr = 47;
        lock.length = 1;
        lock.unit = 1;
        punch.addr = 48;
        punch.length = 2;
        punch.unit = 1;
        realtimeTick.addr = 50;
        realtimeTick.length = 2;
        realtimeTick.unit = 0.001;
        current.addr = 68;
        current.length = 2;
        current.unit = 0.0045;
        torqueCtrlModeEnabled.addr = 70;
        torqueCtrlModeEnabled.length = 1;
        torqueCtrlModeEnabled.unit = 1;
        goalTorque.addr = 71;
        goalTorque.length = 2;
        goalTorque.unit = 0.0045;
        goalAcceleration.addr = 73;
        goalAcceleration.length = 1;
        goalAcceleration.unit = 0.0175;
    }
};



struct MX_106_P2 : ControlTable {
    MX_106_P2 ()
    {
        modelNumber.addr = 0;
        modelNumber.length = 2;
        modelNumber.unit = 1;
        modelInfo.addr = 2;
        modelInfo.length = 4;
        modelInfo.unit = 1;
        firmwareVersion.addr = 6;
        firmwareVersion.length = 1;
        firmwareVersion.unit = 1;
        id.addr = 7;
        id.length = 1;
        id.unit = 1;
        baudrate.addr = 8;
        baudrate.length = 1;
        baudrate.unit = 1;
        returnDelayTime.addr = 9;
        returnDelayTime.length = 1;
        returnDelayTime.unit = 0.000002;
        driveMode.addr = 10;
        driveMode.length = 1;
        driveMode.unit = 1;
        operatingMode.addr = 11;
        operatingMode.length = 1;
        operatingMode.unit = 1;
        secondaryId.addr = 12;
        secondaryId.length = 1;
        secondaryId.unit = 1;
        protocolVersion.addr = 13;
        protocolVersion.length = 1;
        protocolVersion.unit = 1;
        homingOffset.addr = 20;
        homingOffset.length = 4;
        homingOffset.unit = 0.001536;
        movingThreshold.addr = 24;
        movingThreshold.length = 4;
        movingThreshold.unit = 0.0240;
        temperatureLimit.addr = 31;
        temperatureLimit.length = 1;
        temperatureLimit.unit = 1;
        maxVoltageLimit.addr = 32;
        maxVoltageLimit.length = 2;
        maxVoltageLimit.unit = 0.1;
        minVoltageLimit.addr = 34;
        minVoltageLimit.length = 2;
        minVoltageLimit.unit = 0.1;
        PWM_limit.addr = 36;
        PWM_limit.length = 2;
        PWM_limit.unit = 0.113;
        currentLimit.addr = 38;
        currentLimit.length = 2;
        currentLimit.unit = 0.00336;
        accelerationLimit.addr = 40;
        accelerationLimit.length = 4;
        accelerationLimit.unit = 0.3745;
        velocityLimit.addr = 44;
        velocityLimit.length = 4;
        velocityLimit.unit = 0.0240;
        maxPositionLimit.addr = 48;
        maxPositionLimit.length = 4;
        maxPositionLimit.unit = 0.001536;
        minPositionLimit.addr = 52;
        minPositionLimit.length = 4;
        minPositionLimit.unit = 0.001536;        
        shutdown.addr = 63;
        shutdown.length = 1;
        shutdown.unit = 1;
        torqueEnable.addr = 64;
        torqueEnable.length = 1;
        torqueEnable.unit = 1;
        LED.addr = 65;
        LED.length = 1;
        LED.unit = 1;
        statusReturnLevel.addr = 68;
        statusReturnLevel.length = 1;
        statusReturnLevel.unit = 1;
        registered.addr = 69;
        registered.length = 1;
        registered.unit = 1;
        hardwareErrorStatus.addr = 70;
        hardwareErrorStatus.length = 1;
        hardwareErrorStatus.unit = 1;
        velocity_I_gain.addr = 76;
        velocity_I_gain.length = 2;
        velocity_I_gain.unit = 1;
        velocity_P_gain.addr = 78;
        velocity_P_gain.length = 2;
        velocity_P_gain.unit = 1;
        position_D_gain.addr = 80;
        position_D_gain.length = 2;
        position_D_gain.unit = 1;
        position_I_gain.addr = 82;
        position_I_gain.length = 2;
        position_I_gain.unit = 1;
        position_P_gain.addr = 84;
        position_P_gain.length = 2;
        position_P_gain.unit = 1;
        feedforward_2_gain.addr = 88;
        feedforward_2_gain.length = 2;
        feedforward_2_gain.unit = 1;
        feedforward_1_gain.addr = 90;
        feedforward_1_gain.length = 2;
        feedforward_1_gain.unit = 1;
        busWatchdog.addr = 98;
        busWatchdog.length = 1;
        busWatchdog.unit = 1;
        goalPWM.addr = 100;
        goalPWM.length = 2;
        goalPWM.unit = 0.113;
        goalCurrent.addr = 102;
        goalCurrent.length = 2;
        goalCurrent.unit = 0.00336;
        goalVelocity.addr = 104;
        goalVelocity.length = 4;
        goalVelocity.unit = 0.0240;
        profileAcceleration.addr = 108;
        profileAcceleration.length = 4;
        profileAcceleration.unit = 214.577;
        profileVelocity.addr = 112;
        profileVelocity.length = 4;
        profileVelocity.unit = 0.0240;
        goalPosition.addr = 116;
        goalPosition.length = 4;
        goalPosition.unit = 0.001536;
        realtimeTick.addr = 120;
        realtimeTick.length = 2;
        realtimeTick.unit = 0.001;
        moving.addr = 122;
        moving.length = 1;
        moving.unit = 1;
        movingStatus.addr = 123;
        movingStatus.length = 1;
        movingStatus.unit = 1;
        presentPWM.addr = 124;
        presentPWM.length = 2;
        presentPWM.unit = 0.113;
        presentCurrent.addr = 126;
        presentCurrent.length = 2;
        presentCurrent.unit = 0.00336;
        presentVelocity.addr = 128;
        presentVelocity.length = 4;
        presentVelocity.unit = 0.0240;
        presentPosition.addr = 132;
        presentPosition.length = 4;
        presentPosition.unit = 0.001536;
        velocityTrajectory.addr = 136;    
        velocityTrajectory.length = 4;
        velocityTrajectory.unit = 1;
        positionTrajectory.addr = 140;
        positionTrajectory.length = 4;
        positionTrajectory.unit = 1;
        presentInputVoltage.addr = 144;
        presentInputVoltage.length = 2;
        presentInputVoltage.unit = 0.1;
        presentTemperature.addr = 146;
        presentTemperature.length = 1;
        presentTemperature.unit = 1; 
    }
};


struct AX_12A_P1 : ControlTable {
    AX_12A_P1 ()
    {
        modelNumber.addr = 0;
        modelNumber.length = 2;
        modelNumber.unit = 1;
        firmwareVersion.addr = 2;
        firmwareVersion.length = 1;
        firmwareVersion.unit = 1;
        id.addr = 3;
        id.length = 1;
        id.unit = 1;
        baudrate.addr = 4;
        baudrate.length = 1;
        baudrate.unit = 1;
        returnDelayTime.addr = 5;
        returnDelayTime.length = 1;
        returnDelayTime.unit = 0.000002;
        CW_angleLimit.addr = 6;
        CW_angleLimit.length = 2;
        CW_angleLimit.unit = 0.005061;
        CCW_angleLimit.addr = 8;
        CCW_angleLimit.length = 2;
        CCW_angleLimit.unit = 0.005061;
        temperatureLimit.addr = 11;    
        temperatureLimit.length = 1;
        temperatureLimit.unit = 1;
        minVoltageLimit.addr = 12;
        minVoltageLimit.length = 1;
        minVoltageLimit.unit = 0.1;
        maxVoltageLimit.addr = 13;
        maxVoltageLimit.length = 1;
        maxVoltageLimit.unit = 0.1;
        maxTorque.addr = 14;
        maxTorque.length = 2;
        maxTorque.unit = 0.1;
        statusReturnLevel.addr = 16;
        statusReturnLevel.length = 1;
        statusReturnLevel.unit = 1;
        alarmLed.addr = 17;
        alarmLed.length = 1;
        alarmLed.unit = 1;
        shutdown.addr = 18;
        shutdown.length = 1;
        shutdown.unit = 1;

        torqueEnable.addr = 24; 
        torqueEnable.length = 1;
        torqueEnable.unit = 1;
        LED.addr = 25;
        LED.length = 1;
        LED.unit = 1;
        goalPosition.addr = 30;
        goalPosition.length = 2;
        goalPosition.unit = 0.005061;
        movingSpeed.addr = 32;
        movingSpeed.length = 2;
        movingSpeed.unit =  0.0116;
        torqueLimit.addr = 34;
        torqueLimit.length = 2;
        torqueLimit.unit = 0.1;
        presentPosition.addr = 36;
        presentPosition.length = 2;
        presentPosition.unit = 0.005061;
        presentSpeed.addr = 38;
        presentSpeed.length = 2;
        presentSpeed.unit = 0.0116;
        presentLoad.addr = 40;
        presentLoad.length = 2;
        presentLoad.unit = 0.1;
        presentInputVoltage.addr = 42;
        presentInputVoltage.length = 1;
        presentInputVoltage.unit = 0.1;
        presentTemperature.addr = 43;
        presentTemperature.length = 1;
        presentTemperature.unit = 1;
        registered.addr = 44;
        registered.length = 1;
        registered.unit = 1;
        moving.addr = 46;
        moving.length = 1;
        moving.unit = 1;
        lock.addr = 47;
        lock.length = 1;
        lock.unit = 1;
        punch.addr = 48;
        punch.length = 2;
        punch.unit = 1;
    }
};
















}
#endif