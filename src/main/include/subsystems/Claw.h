#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Constants.h"

using namespace ClawConstants;
using namespace ctre::phoenix6;

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
    /// @brief Sets the angle of the claw to a Position
    /// @param pos Position object
    /// @return True if the current angle of the claw is within the deadzone of the desired angle
    bool GoToPosition(const Position &pos);

    /// @brief Gets the current angle of the claw
    /// @return The absolute value of the CANcoder in degrees
    const units::degree_t GetCurrentAngle() { return canCoderWrist.GetAbsolutePosition().GetValue(); }

    /// @brief Sets the power of the IO (intake/output) motor
    /// @param power Power to set to the motor
    void SetIOPower(double power);

    /// @brief Gets whether the proximity sensor detects a coral
    /// @return True if the proximity sensor detects a coral, false if not
    bool IsCoralInClaw() { return proxSensor.GetIsDetected().GetValue(); };
    /// @brief Gets the distance to the closest object from the proximity sensor
    /// @return Distance from the closest object
    units::meter_t GetDistance() { return proxSensor.GetDistance().GetValue(); }

    /// @brief Updates all the values of the SmartDashboard
    void UpdateTelemetry();

    void ResetMotionController() { controller.Reset(GetCurrentAngle()); }

    bool IsAtPosition() { return controller.AtGoal(); }

    /// @brief Simulation periodic
    void SimMode();

private:
    // Creates the motors, CANcoder, and proximity sensor
    hardware::TalonFX wristMotor{RobotMap::Claw::kWristMotorID, "rio"};
    hardware::TalonFX ioMotor{RobotMap::Claw::kIOMotorID, "rio"};
    hardware::CANcoder canCoderWrist{RobotMap::Claw::kCanCoderID,"rio"};
    hardware::CANrange proxSensor{RobotMap::Claw::kCanRangeID, "rio"};

    /*
     * CTRE uses classes from the controls namespace to control the motors in more complex manners.
     * VoltageOut allows us to set the voltage of the motors.
     * The benefit of using VoltageOut instead of just motor.SetVoltage() is that we can set forward and reverse
     * limits and we can more accurately determine what we want to do
     */
    controls::VoltageOut voltageOut{0_V};

    frc::ProfiledPIDController<units::degrees> controller{ClawConstants::kP, ClawConstants::kI, ClawConstants::kD, ClawConstants::kTrapezoidProfileContraints};
    frc::ArmFeedforward feedforward{ClawConstants::kS, ClawConstants::kG, ClawConstants::kV, ClawConstants::kA};

    // Creates a simulation tool for the wrist part of the claw
    frc::sim::SingleJointedArmSim clawSim
    {
        frc::DCMotor::KrakenX60(1),
        kWristGearRatio.value(),
        frc::sim::SingleJointedArmSim::EstimateMOI(0.56_m, 5_kg),
        0.56_m,
        -std::numbers::pi / 2 * 1_rad,
        std::numbers::pi / 2 * 1_rad - 0.01_rad,
        false,
        0_rad
    };
};