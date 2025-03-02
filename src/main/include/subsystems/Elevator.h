#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/DigitalInput.h>

#include <frc/Timer.h>

#include "Constants.h"

using namespace ElevatorConstants;
using namespace ctre::phoenix6;

class Elevator
{
public:
    /// @brief Constructs the elevator
    Elevator();

    /// @brief Manually sets the power of the elevator motors
    /// @param power Power to set the motors to
    void SetMotors(double power);
    /// @brief Uses PID control to run the elevator motors to a height
    /// @param desiredHeight Height to go to
    /// @return True if the current encoder value is within the deadzone of the desired encoder value
    bool GoToHeight(const units::meter_t desiredHeight);
    /// @brief Uses PID control to run the elevator motors to a Position
    /// @param pos Position object
    /// @return True if the current encoder value is within the deadzone of the desired encoder value
    bool GoToPosition(const Position &pos);

    /// @brief Runs every cycle, checks to see if the elevator has hit the limit switch
    void UpdateElevator();
    
    /// @brief Gets the greater of the two motors' encoder values
    /// @return The greater encoder value
    const units::turn_t GetEncoder();
    /// @brief Gets the physical height of the elevator based on encoder values
    /// @return The height of the elevator
    const units::meter_t GetHeight() { return GetEncoder() * kMetersPerMotorTurn + kHeightOffset; }
    /// @brief Sets both encoders to 0
    void ResetEncoders();

    /// @brief Updates all the values of the SmartDashboard
    void UpdateTelemtry();

    /// @brief Gets the state of the bottom limit switch
    /// @retval true if the limit switch is closed (pressed)
    /// @retval false if the limit switch is open
    bool IsBottomLimitSwitchClosed() { return !bottomLimitSwitch.Get() || simLimSwitch; }

    void ResetMotionController() { controller.Reset(GetHeight()); }

    /// @brief Simulation periodic
    void SimMode();

private:
    // Creates the motor objects
    hardware::TalonFX motor1{RobotMap::Elevator::kMotor1ID, "rio"};
    hardware::TalonFX motor2{RobotMap::Elevator::kMotor2ID, "rio"};

    // Creates the limit switch - it is a digital (true or false) input
    frc::DigitalInput bottomLimitSwitch{RobotMap::Elevator::kBottomLimitSwitchID};
    // Simulated representation of the limit switch
    bool simLimSwitch = false;

    // If the elevator is not registered, the elevator encoders have not been reset. 
    // In that case, we will not allow the elevator to use PID control
    bool isElevatorRegistered = false;

    // Whether or not to output the negative input to the follower motor
    bool followerInverted = false;
    // Will use this to set motor2 to a follower motor of motor1
    // This means every input to motor1 will be sent to motor2
    // If followerInverted is true, the input will be the opposite (5 volts -> -5 volts)
    controls::Follower follower{RobotMap::Elevator::kMotor1ID, followerInverted};

    /*
     * CTRE uses classes from the controls namespace to control the motors in more complex manners.
     * VoltageOut allows us to set the voltage of the motors.
     * The benefit of using VoltageOut instead of just motor.SetVoltage() is that we can set forward and reverse
     * limits and we can more accurately determine what we want to do
     */
    controls::VoltageOut voltageOut{0_V};

    frc::ProfiledPIDController<units::meters> controller{kP, kI, kD, kTrapezoidProfileContraints};
    frc::ElevatorFeedforward feedforward{kS, kG, kV, kA};

    frc::sim::ElevatorSim elevatorSim
    {
        frc::LinearSystemId::ElevatorSystem
        (
            frc::DCMotor::KrakenX60(2),
            9.07_kg,
            kSprocketPitchDiameter / 2,
            kMotorGearing.value()
        ),
        frc::DCMotor::KrakenX60(2),
        kHeightOffset,
        kMaxElevatorHeight,
        true,
        kHeightOffset
    };
};
