#include "Autonomous.h"

Autonomous::Autonomous(Drivetrain *swerve)
{
    this->swerve = swerve;
}

frc2::CommandPtr Autonomous::GetAutoCommand()
{
    return pathplanner::PathPlannerAuto("Right Auto").ToPtr();
}
