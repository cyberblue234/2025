#pragma once

#include <frc/XboxController.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/KitBotOutput.h"
#include "subsystems/Limelight.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

class Controls
{
public:
    /// @brief Constructs the Controls object to control the provided subsystems 
    /// @param swerve pointer to the Drivetrain object
    /// @param kitBotOutput pointer to the KitBotOutput object
    Controls(Drivetrain *swerve, KitBotOutput *kitBotOutput, Limelight *limelight3);
    /// @brief Runs all of the subsystems controls every cycle
    /// @param period
    void Periodic(units::second_t period);
    /// @brief Drivetrain controls
    /// @param period
  
    void DriveControls(units::second_t period);
    /// @brief KitBotOutput controls
    void KitBotControls();

    /// @brief Applies a deadband around zero. Zone depends on deadband value. 
    /// @param value Value to apply the deadband to
    /// @param deadband Value around zero
    /// @return value or 0
    double ApplyDeadband(double value, double deadband)
    {
        if (std::abs(value) < deadband) return 0;
        return value;
    }

    frc::XboxController gamepad{0};

private:
    Drivetrain *swerve;
    KitBotOutput *kitBotOutput;
    Limelight *limelight3;

    bool fieldRelative = true;
};