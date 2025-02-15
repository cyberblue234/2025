#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include "Constants.h"

using namespace ClimberConstants;
using namespace ctre::phoenix6;

class Climber
{
public:
    Climber();

    void SetPower(double power);
private:
    hardware::TalonFX climbMotor{RobotMap::Climber::kClimbMotorID, "rio"};
};