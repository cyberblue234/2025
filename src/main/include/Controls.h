#pragma once

#include <frc/XboxController.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Limelight.h"
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
    Controls(Drivetrain *swerve, Elevator *elevator, Limelight *limelightHigh, Limelight *limelightLow);
    /// @brief Runs all of the subsystems controls every cycle
    void Periodic();
    /// @brief Drivetain controls
    void DriveControls();
    /// @brief Elevator controls
    void ElevatorControls();

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

private:
    Drivetrain *swerve;
    Elevator *elevator;
    Limelight *limelightHigh;
    Limelight *limelightLow;

    int branch = 0;
    std::optional<frc2::CommandPtr> path;

    bool fieldRelative = true;
};