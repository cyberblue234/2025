#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/LinearSystemId.h>

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
    bool GoToAngle(units::degree_t angle);
    bool GoToPosition(Positions pos);
    units::degree_t GetAngleToPosition(Positions pos);

    units::degree_t GetCurrentAngle() { return canCoderWrist.GetAbsolutePosition().GetValue(); };

    void SetIntakePower(double power);
    void Intake(Positions pos);
    void Output(Positions pos);

    bool IsCoralInClaw() { return proxSensor.GetIsDetected().GetValue(); };
    units::meter_t GetDistance() { return proxSensor.GetDistance().GetValue(); };

    void UpdateTelemetry();

    void SimMode();

private:
    bool IsPositionForCoralOutput(Positions pos)
    {
        return pos == Positions::L1 || pos == Positions::L2 || pos == Positions::L3 || pos == Positions::L4;
    }
    bool IsPositionForAlgaeIntake(Positions pos)
    {
        return pos == Positions::AlgaeLow || pos == Positions::AlgaeHigh;
    }
    bool IsPositionForAlgaeOutput(Positions pos)
    {
        return pos == Positions::Processor || pos == Positions::Barge;
    }


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
        std::numbers::pi * 1_rad - 0.01_rad,
        false,
        0_rad
    };
};