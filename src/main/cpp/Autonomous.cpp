#include "Autonomous.h"

Autonomous::Autonomous(Drivetrain *swerve, Elevator *elevator, Claw *claw, Climber *climber, Pneumatics *pneumatics,  Limelight *limelightHigh, Limelight *limelightLow)
{
    this->swerve = swerve;
    this->elevator = elevator;
    this->claw = claw;
    this->climber = climber;
    this->pneumatics = pneumatics;
    this->limelightHigh = limelightHigh;
    this->limelightLow = limelightLow;

    NamedCommands::registerCommand("Score", IO());
    NamedCommands::registerCommand("Intake", IO());
    NamedCommands::registerCommand("L1", GoToL1());
	NamedCommands::registerCommand("L2", GoToL2());
    NamedCommands::registerCommand("L3", GoToL3());
	NamedCommands::registerCommand("L4", GoToL4());
    NamedCommands::registerCommand("CoralStation", GoToCoralStation());

    frc::SmartDashboard::PutBoolean("Simulated Coral in Claw", this->simCoralInClaw);

    std::vector<std::string> autos = pathplanner::AutoBuilder::getAllAutoNames();
    autoChooser.SetDefaultOption(autos[0], autos[0]);
	for (auto i = autos.begin(); i != autos.end(); ++i)
	{
		autoChooser.AddOption(*i, *i);
	}

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

frc2::CommandPtr Autonomous::GetAutoCommand()
{
    return pathplanner::PathPlannerAuto(autoChooser.GetSelected()).ToPtr();
}


frc2::CommandPtr Autonomous::IO()
{
    return frc2::RunCommand
    (
        [this]
        {
            if (this->GetCurrentPosition().has_value())
            {
                this->claw->SetIOPower(this->GetCurrentPosition().value().ioMotorPower);
            }
            else
            {
                this->claw->SetIOPower(0.0);
            }
        }
    ).Until
    (
        [this]
        {
            if (frc::RobotBase::IsReal()) 
            {
                if (this->GetCurrentPosition().has_value() == false) return false;
                return this->claw->IsCoralInClaw() == this->GetCurrentPosition().value().isForCoralIntake;
            }
            else 
            {
                if (this->GetCurrentPosition().has_value() == false) return false;
                return frc::SmartDashboard::GetBoolean("Simulated Coral in Claw", this->simCoralInClaw) == this->GetCurrentPosition().value().isForCoralIntake;
            }
        }
    );
}

void Autonomous::GoToPosition(Position pos)
{
    frc::SmartDashboard::PutString("Desired Position", pos.to_string());

    bool isElevatorAtPos = elevator->GoToPosition(pos);
    if (isElevatorAtPos == true) SetElevatorPosition(pos);
    else SetElevatorPosition(std::nullopt);

    bool isWristAtPosition = claw->GoToPosition(pos);
    if (isWristAtPosition == true) SetWristPosition(pos);
    else SetWristPosition(std::nullopt); 
}

frc2::CommandPtr Autonomous::GoToL1()
{
    return frc2::RunCommand
    (
        [this]
        {
            this->GoToPosition(Positions::L1);
        }
    ).Until
    (
        [this]
        {
            return this->GetCurrentPosition().has_value();
        }
    );
}

frc2::CommandPtr Autonomous::GoToL2()
{
    return frc2::RunCommand
    (
        [this]
        {
            this->GoToPosition(Positions::L2);
        }
    ).Until
    (
        [this]
        {
            return this->GetCurrentPosition().has_value();
        }
    );
}

frc2::CommandPtr Autonomous::GoToL3()
{
    return frc2::RunCommand
    (
        [this]
        {
            this->GoToPosition(Positions::L3);
        }
    ).Until
    (
        [this]
        {
            return this->GetCurrentPosition().has_value();
        }
    );
}

frc2::CommandPtr Autonomous::GoToL4()
{
    return frc2::RunCommand
    (
        [this]
        {
            this->GoToPosition(Positions::L4);
        }
    ).Until
    (
        [this]
        {
            return this->GetCurrentPosition().has_value();
        }
    );
}

frc2::CommandPtr Autonomous::GoToCoralStation()
{
    return frc2::RunCommand
    (
        [this]
        {
            this->GoToPosition(Positions::CoralStation);
        }
    ).Until
    (
        [this]
        {
            return this->GetCurrentPosition().has_value();
        }
    );
}

void Autonomous::UpdateTelemetry()
{
    frc::SmartDashboard::PutString("Wrist Position", GetWristPosition() ? GetWristPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Elevator Position", GetElevatorPosition() ? GetElevatorPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Current Position", GetCurrentPosition() ? GetCurrentPosition().value().to_string() : "None");
}