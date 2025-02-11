#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include "rev/SparkFlex.h"

#include "Constants.h"

using namespace ClawConstants;
using namespace ctre::phoenix6;
using namespace rev::spark;

class Claw
{
public:
    Claw();

    void SetWristPower(double power);
    void GoToAngle(units::degree_t angle);
    void GoToPosition(Positions pos);
    units::degree_t GetAngleToPosition(Positions pos);

    void SetIntakePower(double power);

private:
    hardware::TalonFX wristMotor{RobotMap::Claw::kWristMotorID, "rio"};
    SparkFlex intakeMotor{RobotMap::Claw::kIntakeMotorID, SparkFlex::MotorType::kBrushless};
    hardware::CANcoder canCoderWrist{RobotMap::Claw::kCanCoderID,"rio"};

    controls::PositionVoltage angleOut{0_tr};
};