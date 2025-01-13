#pragma once


#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/KitBotOutput.h"
#include "Constants.h"

using namespace pathplanner;

class Autonomous
{
public:
    Autonomous(Drivetrain *swerve, KitBotOutput *kitBotOutput);
    frc2::CommandPtr GetAutoCommand();
	frc2::CommandPtr GetKitBotOutputCommand();
private:
    Drivetrain *swerve;
    KitBotOutput *kitBotOutput;
};