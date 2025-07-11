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
    NamedCommands::registerCommand("Home", GoToCoralHome());
    NamedCommands::registerCommand("AlgaeLow", GoToAlgaeLow());
    NamedCommands::registerCommand("AlgaeHome", GoToAlgaeHome());
    

    frc::SmartDashboard::PutBoolean("Simulated Coral in Claw", this->simCoralInClaw);

    std::vector<std::string> autos = pathplanner::AutoBuilder::getAllAutoNames();
    autoChooser.SetDefaultOption("Nothing", "Nothing");
	for (auto i = autos.begin(); i != autos.end(); ++i)
	{
		autoChooser.AddOption(*i, *i);
	}

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

frc2::CommandPtr Autonomous::GetBargeCommand()
{
    return frc2::cmd::Parallel
    (
        frc2::cmd::Run
        (
            [this]
            {
                bool isElevatorAtPos = elevator->GoToPosition(Positions::Barge);
                if (isElevatorAtPos == true) SetElevatorPosition(Positions::Barge);
                else SetElevatorPosition(std::nullopt);
            }
        ),
        frc2::cmd::RunOnce
        (
            [this]
            {
                claw->SetWristPower(0.0);
            }
        ).Repeatedly().Until([this] { return elevator->GetHeight() > 2.5_ft; }).AndThen
        (
            frc2::cmd::Run
            (
                [this]
                {
                    bool isClawAtPos = claw->GoToPosition(Positions::Barge);
                    if (isClawAtPos == true) SetWristPosition(Positions::Barge);
                    else SetWristPosition(std::nullopt);
                }
            )
        ),
        frc2::cmd::Run
        (
            [this]
            {
                claw->SetIOPower(ClawConstants::kManualIOPower);
            }
        ).Until([this] { return GetElevatorPosition().has_value() && GetWristPosition().has_value(); })
    );
}

std::optional<frc2::CommandPtr> Autonomous::GetAutoCommand()
{
    std::string auton = autoChooser.GetSelected();
    if (auton == "Nothing") return {};
    return pathplanner::PathPlannerAuto(auton).FinallyDo(
        [this]
        {
            swerve->SetDriveMotorsCoastOut();
        }
    );
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
                if (this->GetCurrentPosition().value() == Positions::CoralHome) return false;
                if (this->GetCurrentPosition().value() == Positions::Barge) return false;
                if (this->GetCurrentPosition().value() == Positions::AlgaeLow) return false;
                return this->claw->IsCoralInClaw() == this->GetCurrentPosition().value().isForCoralIntake;
            }
            else 
            {
                if (this->GetCurrentPosition().has_value() == false) return false;
                return frc::SmartDashboard::GetBoolean("Simulated Coral in Claw", this->simCoralInClaw) == this->GetCurrentPosition().value().isForCoralIntake;
            }
        }
    ).AndThen
    (
        [this]
        {
            claw->SetIOPower(0.0);
        }
    );
}


frc2::CommandPtr Autonomous::GoToL1()
{
    return frc2::RunCommand
    (
        [this]
        {
            bool isElevatorAtPos = elevator->GoToPosition(Positions::L1);
            if (isElevatorAtPos == true) SetElevatorPosition(Positions::L1);
            else SetElevatorPosition(std::nullopt);

            bool isWristAtPosition = claw->GoToPosition(Positions::L1);
            if (isWristAtPosition == true) SetWristPosition(Positions::L1);
            else SetWristPosition(std::nullopt);
        }
    ).FinallyDo
    (
        [this]
        {
            elevator->SetMotors(0.0);
            claw->SetWristPower(0.0);
            elevator->ResetMotionController();
            claw->ResetMotionController();
        }
    );
}

frc2::CommandPtr Autonomous::GoToL2()
{
    return frc2::RunCommand
    (
        [this]
        {
            bool isElevatorAtPos = elevator->GoToPosition(Positions::L2);
            if (isElevatorAtPos == true) SetElevatorPosition(Positions::L2);
            else SetElevatorPosition(std::nullopt);

            bool isWristAtPosition = claw->GoToPosition(Positions::L2);
            if (isWristAtPosition == true) SetWristPosition(Positions::L2);
            else SetWristPosition(std::nullopt);
        }
    ).FinallyDo
    (
        [this]
        {
            elevator->SetMotors(0.0);
            claw->SetWristPower(0.0);
            elevator->ResetMotionController();
            claw->ResetMotionController();
        }
    );
}

frc2::CommandPtr Autonomous::GoToL3()
{
    return frc2::RunCommand
    (
        [this]
        {
            bool isElevatorAtPos = elevator->GoToPosition(Positions::L3);
            if (isElevatorAtPos == true) SetElevatorPosition(Positions::L3);
            else SetElevatorPosition(std::nullopt);

            bool isWristAtPosition = claw->GoToPosition(Positions::L3);
            if (isWristAtPosition == true) SetWristPosition(Positions::L3);
            else SetWristPosition(std::nullopt);
        }
    ).FinallyDo
    (
        [this]
        {
            elevator->SetMotors(0.0);
            claw->SetWristPower(0.0);
            elevator->ResetMotionController();
            claw->ResetMotionController();
        }
    );
}

frc2::CommandPtr Autonomous::GoToL4()
{
    return frc2::RunCommand
    (
        [this]
        {
            bool isElevatorAtPos = elevator->GoToPosition(Positions::L4);
            if (isElevatorAtPos == true) SetElevatorPosition(Positions::L4);
            else SetElevatorPosition(std::nullopt);

            bool isWristAtPosition = claw->GoToPosition(Positions::L4);
            if (isWristAtPosition == true) SetWristPosition(Positions::L4);
            else SetWristPosition(std::nullopt);
        }
    ).FinallyDo
    (
        [this]
        {
            elevator->SetMotors(0.0);
            claw->SetWristPower(0.0);
            elevator->ResetMotionController();
            claw->ResetMotionController();
        }
    );
}

frc2::CommandPtr Autonomous::GoToCoralStation()
{
    return frc2::RunCommand
    (
        [this]
        {
            bool isElevatorAtPos = elevator->GoToPosition(Positions::CoralStation);
            if (isElevatorAtPos == true) SetElevatorPosition(Positions::CoralStation);
            else SetElevatorPosition(std::nullopt);

            bool isWristAtPosition = claw->GoToPosition(Positions::CoralStation);
            if (isWristAtPosition == true) SetWristPosition(Positions::CoralStation);
            else SetWristPosition(std::nullopt);
        }
    ).FinallyDo
    (
        [this]
        {
            elevator->SetMotors(0.0);
            claw->SetWristPower(0.0);
            elevator->ResetMotionController();
            claw->ResetMotionController();
        }
    );
}

frc2::CommandPtr Autonomous::GoToCoralHome()
{
    return frc2::RunCommand
    (
        [this]
        {
            bool isElevatorAtPos = elevator->GoToPosition(Positions::CoralHome);
            if (isElevatorAtPos == true) SetElevatorPosition(Positions::CoralHome);
            else SetElevatorPosition(std::nullopt);

            bool isWristAtPosition = claw->GoToPosition(Positions::CoralHome);
            if (isWristAtPosition == true) SetWristPosition(Positions::CoralHome);
            else SetWristPosition(std::nullopt);
        }
    ).FinallyDo
    (
        [this]
        {
            elevator->SetMotors(0.0);
            claw->SetWristPower(0.0);
            elevator->ResetMotionController();
            claw->ResetMotionController();
        }
    );
}

frc2::CommandPtr Autonomous::GoToAlgaeLow()
{
    return frc2::RunCommand
    (
        [this]
        {
            bool isElevatorAtPos = elevator->GoToPosition(Positions::AlgaeLow);
            if (isElevatorAtPos == true) SetElevatorPosition(Positions::AlgaeLow);
            else SetElevatorPosition(std::nullopt);

            bool isWristAtPosition = claw->GoToPosition(Positions::AlgaeLow);
            if (isWristAtPosition == true) SetWristPosition(Positions::AlgaeLow);
            else SetWristPosition(std::nullopt);
        }
    ).FinallyDo
    (
        [this]
        {
            elevator->SetMotors(0.0);
            claw->SetWristPower(0.0);
            elevator->ResetMotionController();
            claw->ResetMotionController();
        }
    );
}

frc2::CommandPtr Autonomous::GoToAlgaeHome()
{
    return frc2::RunCommand
    (
        [this]
        {
            bool isElevatorAtPos = elevator->GoToPosition(Positions::AlgaeHome);
            if (isElevatorAtPos == true) SetElevatorPosition(Positions::AlgaeHome);
            else SetElevatorPosition(std::nullopt);

            bool isWristAtPosition = claw->GoToPosition(Positions::AlgaeHome);
            if (isWristAtPosition == true) SetWristPosition(Positions::AlgaeHome);
            else SetWristPosition(std::nullopt);
        }
    ).FinallyDo
    (
        [this]
        {
            elevator->SetMotors(0.0);
            claw->SetWristPower(0.0);
            elevator->ResetMotionController();
            claw->ResetMotionController();
        }
    );
}



void Autonomous::UpdateTelemetry()
{
    frc::SmartDashboard::PutString("Wrist Position", GetWristPosition() ? GetWristPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Elevator Position", GetElevatorPosition() ? GetElevatorPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Current Position", GetCurrentPosition() ? GetCurrentPosition().value().to_string() : "None");
}