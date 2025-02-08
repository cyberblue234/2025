#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <frc/DigitalInput.h>

#include "Constants.h"

using namespace ElevatorConstants;
using namespace ctre::phoenix6;

class Elevator
{
public:
    Elevator();

    void SetMotors(double power);

    enum Positions
    {
        L1, L2, L3, L4, Pickup, Processor, Barge, Floor
    };
    void GoToPosition(Positions pos);

    void UpdateElevator();
    const units::turn_t GetElevatorEncoder();
    void ResetElevatorEncoders();

    units::turn_t GetTurnsToPosition(Positions pos);

    bool GetBottomLimitSwitch() { return bottomLimitSwitch.Get(); };

private:

    hardware::TalonFX motor1{RobotMap::Elevator::kMotor1ID, "rio"};
    hardware::TalonFX motor2{RobotMap::Elevator::kMotor2ID, "rio"};

    frc::DigitalInput bottomLimitSwitch{RobotMap::Elevator::kBottomLimitSwitchID};

    bool isElevatorRegistered = false;

    bool followerInverted = true;
    controls::Follower follower{RobotMap::Elevator::kMotor1ID, followerInverted};

    controls::PositionVoltage positionOut{0_tr};
};
