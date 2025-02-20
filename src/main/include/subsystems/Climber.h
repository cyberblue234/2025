#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include "Constants.h"

using namespace ClimberConstants;
using namespace ctre::phoenix6;

class Climber
{
public:
    /// @brief Constructs the climber
    Climber();

    /// @brief Manually sets the power of the climber motor
    /// @param power Power to set the motor to
    void SetPower(double power);
private:
    // Creates the climber motor
    hardware::TalonFX climbMotor{RobotMap::Climber::kClimbMotorID, "rio"};
};