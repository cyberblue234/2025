#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

#include "Constants.h"

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

    units::turn_t GetTurnsToPosition(Positions pos);

private:

    hardware::TalonFX motor1{RobotMap::Elevator::kMotor1ID, "rio"};
    hardware::TalonFX motor2{RobotMap::Elevator::kMotor2ID, "rio"};

    bool followerInverted = true;
    controls::Follower follower{RobotMap::Elevator::kMotor1ID, followerInverted};

    controls::PositionVoltage positionOut{0_tr};
};
