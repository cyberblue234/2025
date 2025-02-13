#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/RobotController.h>
#include <frc/RobotBase.h>

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
    bool GoToAngle(units::degree_t angle);
    bool GoToPosition(Positions pos);
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

    frc::sim::SingleJointedArmSim clawSim
    {
        frc::DCMotor::KrakenX60(1),
        kWristGearRatio.value(),
        frc::sim::SingleJointedArmSim::EstimateMOI(0.56_m, 5_kg),
        0.56_m,
        -std::numbers::pi * 1_rad,
        std::numbers::pi * 1_rad,
        false,
        0_rad
    };
};