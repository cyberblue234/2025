#pragma once

#include <frc/XboxController.h>
#include <frc/Joystick.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Claw.h"
#include "subsystems/Climber.h"
#include "subsystems/Pneumatics.h"
#include "subsystems/Limelight.h"

using namespace ControlsConstants;

class Controls
{
public:
    /// @brief Constructs the Controls object to control the provided subsystems 
    /// @param swerve pointer to the Drivetrain object
    /// @param elevator pointer to the Elevator object
    /// @param claw pointer to the Claw object
    /// @param climber pointer to the Climber object
    /// @param pneumatics pointer to the Pneumatics object
    /// @param limelightHigh pointer to one of the Limelight objects
    /// @param limelightLow pointer to one of the Limelight objects
    Controls(Drivetrain *swerve, Elevator *elevator, Claw *claw, Climber *climber, Pneumatics *pneumatics,  Limelight *limelightHigh, Limelight *limelightLow);
    /// @brief Runs all of the subsystems controls every cycle
    void Periodic();
    /// @brief Drivetain controls
    void DriveControls();
    /// @brief Elevator controls
    void ElevatorControls();
    /// @brief Claw controls
    void ClawControls();
    /// @brief Climber controls
    void ClimberControls();
    /// @brief Pneumatics controls
    void PneumaticsControls();

    frc2::CommandPtr GetBargeCommand();

    /// @brief Sets current elevator position
    void SetElevatorPosition(std::optional<Position> pos) { elevatorPosition = pos; }
    /// @brief Gets current elevator position
    /// @retval Position of the elevator
    /// @retval Positions::Null if the elevator is not at any preset position
    std::optional<Position> GetElevatorPosition() { return elevatorPosition; }
    /// @brief Sets current claw position
    void SetWristPosition(std::optional<Position> pos) { wristPosition = pos; }
    /// @brief Gets current claw position
    /// @retval Position of the claw
    /// @retval Positions::Null if the claw is not at any preset position
    std::optional<Position> GetWristPosition() { return wristPosition; }
    /// @brief Returns the position of the elevator-claw system
    /// @retval Position of the system
    /// @retval Positions::Null if the two subsystems are not at the same position
    std::optional<Position> GetCurrentPosition()
    {
        // Short circuits if either elevator or claw position does not exist
        // Equals comparison checks if the height, angle, IO power, and coral output check are the same
        if (GetElevatorPosition() && GetWristPosition() && GetElevatorPosition().value().operator==(GetWristPosition().value())) return GetElevatorPosition();
        return std::nullopt;
    }

    /// @brief Sets desired position
    void SetDesiredPosition();
    /// @brief Gets desired position
    std::optional<Position> GetDesiredPosition() { return desiredPosition; }

    void UpdateTelemetry();

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
    frc::Joystick controlBoard{1};

private:
    Drivetrain *swerve;
    Elevator *elevator;
    Claw *claw;
    Climber *climber;
    Pneumatics *pneumatics;
    Limelight *limelightHigh;
    Limelight *limelightLow; 

    std::optional<Position> elevatorPosition;
    std::optional<Position> wristPosition;
    std::optional<Position> desiredPosition; 

    std::optional<frc2::CommandPtr> path;

    frc2::CommandPtr barge = GetBargeCommand();

    bool fieldRelative = true;
};