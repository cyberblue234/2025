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

/// @brief Class representation of the claw. Includes both the wrist and intake/output (IO) motor
class Claw
{
public:
    /// @brief Constructs the claw
    Claw();

    /// @brief Manually sets the power to the wrist motor
    /// @param power Power to set to the motor
    void SetWristPower(double power);
    /// @brief Uses PID control to go to a desired angle
    /// @param angle Desired angle to travel to
    /// @return True if the current angle of the claw is within the deadzone of the desired angle
    bool GoToAngle(units::degree_t angle);
    /// @brief Sets the angle of the claw to a position of the Position enum
    /// @param pos Value of the Position enum
    /// @return True if the current angle of the claw is within the deadzone of the desired angle
    bool GoToPosition(Positions pos);
    /// @brief Returns the angle that corresponds to the desired Position
    /// @param pos Value of the Position enum
    /// @return Angle that corresponds
    const units::degree_t GetAngleToPosition(Positions pos);

    /// @brief Gets the current angle of the claw
    /// @return The absolute value of the CANcoder in degrees
    const units::degree_t GetCurrentAngle() { return canCoderWrist.GetAbsolutePosition().GetValue(); };

    /// @brief Sets the power of the IO (intake/output) motor
    /// @param power Power to set to the motor
    void SetIOPower(double power);
    /// @brief Uses the Position to determine what to do during intaking. If the pos is at the coral station, it knows it will be intaking a coral, etc.
    /// @param pos Value of the Position enum
    void Intake(Positions pos);
    /// @brief Uses the Position to determine what to do during outputting. If the position is a reef level, it will output based on the level, etc.
    /// @param pos Value of the Position enum
    void Output(Positions pos);

    /// @brief Gets whether the proximity sensor detects a coral
    /// @return True if the proximity sensor detects a coral, false if not
    bool IsCoralInClaw() { return proxSensor.GetIsDetected().GetValue(); };
    /// @brief Gets the distance to the closest object from the proximity sensor
    /// @return Distance from the closest object
    units::meter_t GetDistance() { return proxSensor.GetDistance().GetValue(); };

    /// @brief Updates all the values of the SmartDashboard
    void UpdateTelemetry();

    /// @brief Simulation periodic
    void SimMode();

private:
    /// @brief Determines if the given Position is one of the levels of the reef
    /// @param pos Value of the Position enum
    /// @return True if pos is L1-L4
    bool IsPositionForCoralOutput(Positions pos)
    {
        return pos == Positions::L1 || pos == Positions::L2 || pos == Positions::L3 || pos == Positions::L4;
    }
    /// @brief Determines if the given Position is AlgaeLow or AlgaeHigh
    /// @param pos Value of the Position enum
    /// @return True if pos is AlgaeLow or AlgaeHigh
    bool IsPositionForAlgaeIntake(Positions pos)
    {
        return pos == Positions::AlgaeLow || pos == Positions::AlgaeHigh;
    }
    /// @brief Determines if the given Position is Processor or Barge
    /// @param pos Value of the Position enum
    /// @return True if pos is Processor or Barge
    bool IsPositionForAlgaeOutput(Positions pos)
    {
        return pos == Positions::Processor || pos == Positions::Barge;
    }

    // Creates the motors, CANcoder, and proximity sensor
    hardware::TalonFX wristMotor{RobotMap::Claw::kWristMotorID, "rio"};
    SparkFlex ioMotor{RobotMap::Claw::kIOMotorID, SparkFlex::MotorType::kBrushless};
    hardware::CANcoder canCoderWrist{RobotMap::Claw::kCanCoderID,"rio"};
    hardware::CANrange proxSensor{RobotMap::Claw::kCanRangeID, "rio"};

    /*
     * CTRE uses classes from the controls namespace to control the motors in more complex manners.
     * PositionVoltage allows us to run the wrist motor to a position using voltages.
     */
    controls::PositionVoltage angleOut{0_tr};

    // Creates a simulation tool for the wrist part of the claw
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