#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include "rev/SparkFlex.h"

#include <frc/smartdashboard/SmartDashboard.h>

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

    units::degree_t GetCurrentAngle() { return canCoderWrist.GetAbsolutePosition().GetValue(); };

    void SetIntakePower(double power);
    void IntakeCoral();
    void OutputCoral(Positions pos);

    bool IsCoralInClaw() { return proxSensor.GetIsDetected().GetValue(); };
    units::meter_t GetDistance() { return proxSensor.GetDistance().GetValue(); };

    void UpdateTelemetry();

    void SimMode();

private:
    hardware::TalonFX wristMotor{RobotMap::Claw::kWristMotorID, "rio"};
    SparkFlex intakeMotor{RobotMap::Claw::kIntakeMotorID, SparkFlex::MotorType::kBrushless};
    hardware::CANcoder canCoderWrist{RobotMap::Claw::kCanCoderID,"rio"};

    hardware::CANrange proxSensor{RobotMap::Claw::kCanRangeID, "rio"};

    controls::PositionVoltage angleOut{0_tr};
};