#pragma once

#include "rev/SparkFlex.h"


#include "Constants.h"

using namespace ClimberConstants;
using namespace rev::spark;

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
    SparkFlex climbMotor{RobotMap::Climber::kClimbMotorID, SparkLowLevel::MotorType::kBrushless};
};