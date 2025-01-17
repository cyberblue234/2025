#include "Autonomous.h"

Autonomous::Autonomous(Drivetrain *swerve, KitBotOutput *kitBotOutput)
{
    this->swerve = swerve;
    this->kitBotOutput = kitBotOutput;
    NamedCommands::registerCommand("KitBotOutput", GetKitBotOutputCommand());
}

frc2::CommandPtr Autonomous::GetAutoCommand()
{
    return pathplanner::PathPlannerAuto("Right Auto").ToPtr();
}

frc2::CommandPtr Autonomous::GetKitBotOutputCommand()
{
    return frc2::RunCommand
    (
        [this]
        {
            this->kitBotOutput->SetMotor(-0.375);
        }
    ).WithDeadline
    (
        frc2::WaitCommand(0.35_s).ToPtr()
    ).AndThen
    (
        [this]
        {
            this->kitBotOutput->SetMotor(0.0);
        }
    );
}