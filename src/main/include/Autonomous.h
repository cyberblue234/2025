#pragma once

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "subsystems/Drivetrain.h"
#include "Constants.h"

using namespace pathplanner;

class Autonomous
{
public:
    Autonomous(Drivetrain *swerve);
    frc2::CommandPtr GetAutoCommand();
private:
    Drivetrain *swerve;
};