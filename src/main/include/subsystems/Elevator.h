#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include "Constants.h"

using namespace ctre::phoenix6;

class Elevator
{
public:
    Elevator();

private:

    hardware::TalonFX motor1{RobotMap::kMotor1ID, "rio"};
    hardware::TalonFX motor2{RobotMap::kMotor2ID, "rio"};

};
