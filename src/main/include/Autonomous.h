#pragma once

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc/smartdashboard/SendableChooser.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Claw.h"
#include "subsystems/Climber.h"
#include "subsystems/Pneumatics.h"
#include "subsystems/Limelight.h"
#include "Constants.h"

using namespace pathplanner;

class Autonomous
{
public:
    Autonomous(Drivetrain *swerve, Elevator *elevator, Claw *claw, Climber *climber, Pneumatics *pneumatics,  Limelight *limelightHigh, Limelight *limelightLow);
    std::optional<frc2::CommandPtr> GetAutoCommand();

    frc2::CommandPtr IO();

    frc2::CommandPtr GoToL1();
    frc2::CommandPtr GoToL2();
    frc2::CommandPtr GoToL3();
    frc2::CommandPtr GoToL4();
    frc2::CommandPtr GoToCoralStation();
    frc2::CommandPtr GoToCoralHome();

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

    /// @brief Gets desired position
    std::optional<Position> GetDesiredPosition() { return desiredPosition; }

    void UpdateTelemetry();

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

    frc::SendableChooser<std::string> autoChooser;

    bool simCoralInClaw = false;
};