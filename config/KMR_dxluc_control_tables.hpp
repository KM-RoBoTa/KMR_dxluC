#ifndef KMR_DLXUC_CONTROL_TABLES_HPP
#define KMR_DLXUC_CONTROL_TABLES_HPP

#include "KMR_dxluc_structures.hpp"

struct MX_64_P1 : ControlTable {
    MX_64_P1 ()
    {
        modelNumber.length = 2;
        modelNumber.unit = 1;
        firmwareVersion.length = 1;
        firmwareVersion.unit = 1;
        id.length = 1;
        id.unit = 1;
        baudrate.length = 4;
        baudrate.unit = 1;
        returnDelayTime.length = 1;
        returnDelayTime.unit = 0.000002;
        CW_angleLimit.length = 2;
        CW_angleLimit.unit = 0.001536;
        CCW_angleLimit.length = 2;
        CCW_angleLimit.unit = 0.001536;    
        temperatureLimit.length = 1;
        temperatureLimit.unit = 1;
        minVoltageLimit.length = 1;
        minVoltageLimit.unit = 0.1;
        maxVoltageLimit.length = 1;
        maxVoltageLimit.unit = 0.1;
        maxTorque.length = 2;
        maxTorque.unit = 0.1;
        statusReturnLevel.length = 2;
        statusReturnLevel.unit = 0.1;
        alarmLed.length = 1;
        alarmLed.unit = 1;
        shutdown.length = 1;
        shutdown.unit = 1;
        multiturnOffset.length = 2;
        multiturnOffset.unit = 1;
        resolutionDivider.length = 1;
        resolutionDivider.unit = 1;

        torqueEnable.length = 1;
        torqueEnable.unit = 1;
        LED.length = 1;
        LED.unit = 1;
        D_gain.length = 1;
        D_gain.unit = 1;
        I_gain.length = 1;
        I_gain.unit = 1;
        P_gain.length = 1;
        P_gain.unit = 1;
        goalPosition.length = 2;
        goalPosition.unit = 0.001536;
        movingSpeed.length = 2;
        movingSpeed.unit =  0.0119;
        torqueLimit.length = 2;
        torqueLimit.unit = 0.1;
        presentPosition.length = 2;
        presentPosition.unit = 0.001536;
        presentSpeed.length = 2;
        presentSpeed.unit = 0.0115;
        presentLoad.length = 2;
        presentLoad.unit = 0.1;
        presentInputVoltage.length = 1;
        presentInputVoltage.unit = 0.1;
        presentTemperature.length = 1;
        presentTemperature.unit = 1;
        registered.length = 1;
        registered.unit = 1;
        moving.length = 1;
        moving.unit = 1;
        lock.length = 1;
        lock.unit = 1;
        punch.length = 2;
        punch.unit = 1;
        realtimeTick.length = 2;
        realtimeTick.unit = 0.001;
        current.length = 2;
        current.unit = 0.0045;
        torqueCtrlModeEnabled.length = 1;
        torqueCtrlModeEnabled.unit = 1;
        goalTorque.length = 2;
        goalTorque.unit = 0.0045;
        goalAcceleration.length = 1;
        goalAcceleration.unit = 0.0175;
    }
};


struct MX_64_P2 : ControlTable {
    MX_64_P2 ()
    {
        modelNumber.length = 2;
        modelNumber.unit = 1;
        modelInfo.length = 4;
        modelInfo.unit = 1;
        firmwareVersion.length = 1;
        firmwareVersion.unit = 1;
        id.length = 1;
        id.unit = 1;
        baudrate.length = 1;
        baudrate.unit = 1;
        returnDelayTime.length = 1;
        returnDelayTime.unit = 0.000002;
        driveMode.length = 1;
        driveMode.unit = 1;
        operatingMode.length = 1;
        operatingMode.unit = 1;
        protocolVersion.length = 1;
        protocolVersion.unit = 1;
        homingOffset.length = 4;
        homingOffset.unit = 0.001536;
        movingThreshold.length = 4;
        movingThreshold.unit = 0.0240;
        temperatureLimit.length = 1;
        temperatureLimit.unit = 1;
        minVoltageLimit.length = 2;
        minVoltageLimit.unit = 0.1;
        maxVoltageLimit.length = 2;
        maxVoltageLimit.unit = 0.1;
        PWM_limit.length = 2;
        PWM_limit.unit = 0.113;
        currentLimit.length = 2;
        currentLimit.unit = 0.00336;
        accelerationLimit.length = 4;
        accelerationLimit.unit = 0.3745;
        velocityLimit.length = 4;
        velocityLimit.unit = 0.0240;
        maxPositionLimit.length = 4;
        maxPositionLimit.unit = 0.001536;
        minPositionLimit.length = 4;
        minPositionLimit.unit = 0.001536;        
        shutdown.length = 1;
        shutdown.unit = 1;

        torqueEnable.length = 1;
        torqueEnable.unit = 1;
        LED.length = 1;
        LED.unit = 1;
        statusReturnLevel.length = 1;
        statusReturnLevel.unit = 1;
        registered.length = 1;
        registered.unit = 1;
        hardwareErrorStatus.length = 1;
        hardwareErrorStatus.unit = 1;
        velocity_I_gain.length = 2;
        velocity_I_gain.unit = 1;
        velocity_P_gain.length = 2;
        velocity_P_gain.unit = 1;
        position_P_gain.length = 2;
        position_P_gain.unit = 1;
        position_I_gain.length = 2;
        position_I_gain.unit = 1;
        position_D_gain.length = 2;
        position_D_gain.unit = 1;
        feedforward_1_gain.length = 2;
        feedforward_1_gain.unit = 1;
        feedforward_2_gain.length = 2;
        feedforward_2_gain.unit = 1;
        busWatchdog.length = 1;
        busWatchdog.unit = 1;
        goalPWM.length = 2;
        goalPWM.unit = 0.113;
        goalCurrent.length = 2;
        goalCurrent.unit = 0.00336;
        goalVelocity.length = 4;
        goalVelocity.unit = 0.0240;
        profileAcceleration.length = 4;
        profileAcceleration.unit = 214.577;
        profileVelocity.length = 4;
        profileVelocity.unit = 0.0240;
        goalPosition.length = 4;
        goalPosition.unit = 0.001536;
        realtimeTick.length = 2;
        realtimeTick.unit = 0.001;
        moving.length = 1;
        moving.unit = 1;
        movingStatus.length = 1;
        movingStatus.unit = 1;
        presentPWM.length = 2;
        presentPWM.unit = 0.113;
        presentCurrent.length = 2;
        presentCurrent.unit = 0.00336;
        presentVelocity.length = 4;
        presentVelocity.unit = 0.0240;
        presentPosition.length = 4;
        presentPosition.unit = 0.001536;    
        velocityTrajectory.length = 4;
        velocityTrajectory.unit = 1;
        positionTrajectory.length = 4;
        positionTrajectory.unit = 1;
        presentInputVoltage.length = 2;
        presentInputVoltage.unit = 0.1;
        presentTemperature.length = 1;
        presentTemperature.unit = 1;
    }
};


#endif