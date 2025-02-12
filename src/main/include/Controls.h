#pragma once

#include <frc/XboxController.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Limelight.h"
#include "subsystems/Claw.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

class Controls
{
public:
    /// @brief Constructs the Controls object to control the provided subsystems 
    /// @param swerve pointer to the Drivetrain object
    /// @param elevator pointer to the Elevator object
    /// @param limelightHigh pointer to one of the Limelight objects
    /// @param limelightLow pointer to one of the Limelight objects
    Controls(Drivetrain *swerve, Elevator *elevator, Claw *claw, Limelight *limelightHigh, Limelight *limelightLow);
    /// @brief Runs all of the subsystems controls every cycle
    void Periodic();
    /// @brief Drivetain controls
    void DriveControls();
    /// @brief Elevator controls
    void ElevatorControls();
    /// @brief Claw controls
    void ClawControls();

    /// @brief Sets current elevator position
    void SetElevatorPosition(Positions pos) { elevatorPosition = pos; };
    /// @brief Gets current elevator position
    /// @retval @ref Position of the elevator
    /// @retval @ref Positions::Null if the elevator is not at any preset position
    Positions GetElevatorPosition() { return elevatorPosition; };
    /// @brief Sets current claw position
    void SetClawPosition(Positions pos) { clawPosition = pos; };
    /// @brief Gets current claw position
    /// @retval Position of the claw
    /// @retval Positions::Null if the claw is not at any preset position
    Positions GetClawPosition() { return clawPosition; };
    /// @brief Returns the position of the elevator-claw system
    /// @retval Position of the system
    /// @retval Positions::Null if the two subsystems are not at the same position
    Positions GetCurrentPosition()
    {
        if (GetElevatorPosition() == GetClawPosition()) return GetElevatorPosition();
        return Positions::Null;
    }

    /// @brief Sets desired position
    void SetDesiredPosition();
    /// @brief Gets desired position
    Positions GetDesiredPosition() { return desiredPosition; };


    /// @brief Applies a deadband around zero. Zone depends on deadband value. 
    /// @param value Value to apply the deadband to
    /// @param deadband Value of the deadband
    /// @retval value if within deadband
    /// @retval 0 if not within deadband
    double ApplyDeadband(double value, double deadband)
    {
        if (std::abs(value) < deadband) return 0;
        return value;
    }

    frc::XboxController gamepad{0};
    frc::XboxController gamepad2{1};

private:
    Drivetrain *swerve;
    Elevator *elevator;
    Claw *claw;
    Limelight *limelightHigh;
    Limelight *limelightLow; 

    Positions elevatorPosition = Positions::Null;
    Positions clawPosition = Positions::Null;
    Positions desiredPosition = Positions::Null; 

    int branch = 0;
    std::optional<frc2::CommandPtr> path;

    bool fieldRelative = true;
};