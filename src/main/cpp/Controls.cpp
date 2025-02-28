#include "Controls.h"

Controls::Controls(Drivetrain *swerve, Elevator *elevator, Claw *claw, Climber *climber, Pneumatics *pneumatics, Limelight *limelightHigh, Limelight *limelightLow)
{
    // Sets all of the class pointers to the arguments for later use
    this->swerve = swerve;
    this->elevator = elevator;
    this->claw = claw;
    this->climber = climber;
    this->pneumatics = pneumatics;
    this->limelightHigh = limelightHigh;
    this->limelightLow = limelightLow;
}

void Controls::Periodic()
{
    // Runs the different controls
    DriveControls();
    // Must call before elevator and claw controls, otherwise they would be behind 20ms
    SetDesiredPosition();
    ElevatorControls();
    ClawControls();
    ClimberControls();
    PneumaticsControls();
}

void Controls::DriveControls()
{
    if (gamepad.GetYButton()) swerve->ResetDrivingGyro();
    /// @deprecated will be removed once the control board is finalized
    if (gamepad.GetLeftBumperButtonPressed()) 
    {
        if (branch == 11) branch = 0;
        else branch += 1;
    }
    else if (gamepad.GetRightBumperButtonPressed()) 
    {
        if (branch == 0) branch = 11;
        else branch -= 1;
    }
    frc::SmartDashboard::PutNumber("Selected Branch", branch);
    if (gamepad.GetAButtonPressed()) 
    { 
        path = swerve->PathfindToBranch(Drivetrain::ReefBranches(branch), gamepad.GetLeftTriggerAxis() > 0.5);
        if (path) path->Schedule();
    }
    else if (gamepad.GetAButtonReleased()) if (path) path->Cancel();
    
    if (gamepad.GetBButtonPressed()) 
    { 
        path = swerve->PathfindToCoralStation(Drivetrain::CoralStations::Right, gamepad.GetLeftTriggerAxis() > 0.5);
        if (path) path->Schedule();
    }
    else if (gamepad.GetBButtonReleased()) if (path) path->Cancel();

    if (gamepad.GetXButtonPressed()) 
    { 
        path = swerve->PathfindToProcessor(gamepad.GetLeftTriggerAxis() > 0.5);
        if (path) path->Schedule();
    }
    else if (gamepad.GetXButtonReleased()) if (path) path->Cancel();

    // Ensures driver control is disabled while pathfinding is occuring
    if (path)
    {
        if (path->IsScheduled()) return;
    }

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const double x = -gamepad.GetLeftY();
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const double y = -gamepad.GetLeftX();
    // The scalar helps to smooth out driving while preserving full control over the speed
    const double scalar = x * x + y * y;
    // Adds a speed adjusmtment based on the right trigger - the more it is pressed, the slower the bot will travel for a maximum reduction of -80%
    const double speedAdjust = 1 - 0.8 * gamepad.GetRightTriggerAxis(); 

    const units::meters_per_second_t xSpeed = ApplyDeadband(x * scalar, 0.015) * DrivetrainConstants::kMaxSpeed * speedAdjust;
    const units::meters_per_second_t ySpeed = ApplyDeadband(y * scalar, 0.015) * DrivetrainConstants::kMaxSpeed * speedAdjust;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const double omega = -gamepad.GetRightX();
    const units::radians_per_second_t omegaSpeed = ApplyDeadband(pow(omega, 3), 0.05) *
                     DrivetrainConstants::kMaxAngularSpeed * pow(speedAdjust, 0.5);

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.value());
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.value());
    frc::SmartDashboard::PutNumber("omegaSpeed", omegaSpeed.value());

    frc::ChassisSpeeds setSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, omegaSpeed};
    swerve->Drive(setSpeeds, fieldRelative);
}

void Controls::ElevatorControls() 
{
    // Take the elevator down if the robot is tilting
    if (units::math::abs(swerve->GetRoll()) > 2_deg || units::math::abs(swerve->GetPitch()) > 2_deg)
    {
        elevator->GoToHeight(kMaxElevatorHeight);
    }
    // If there is a desired position
    if (GetDesiredPosition().has_value())
    {
        // Runs the elevator to the position
        bool isElevatorAtPos = elevator->GoToPosition(GetDesiredPosition().value());
        // Set the elevator current position if the height is within the deadzone of the setpoint
        if (isElevatorAtPos == true) SetElevatorPosition(GetDesiredPosition());
        // If the elevator height is not within the deadzone, set the current elevator pos to null
        else SetElevatorPosition(std::nullopt);
    }
    else if (controlBoard.GetRawAxis(kManualElevatorAxis) < -0.5) 
    {
        // Manual control up
        elevator->SetMotors(0.4);
        SetElevatorPosition(std::nullopt);
    }
    else if (controlBoard.GetRawAxis(kManualElevatorAxis) > 0.5)
    {
        // Manual control down
        elevator->SetMotors(-0.4);
        SetElevatorPosition(std::nullopt);
    }
    else
    {
        // Stop the motors
        elevator->SetMotors(0);
    }
}

void Controls::ClawControls()
{
    if (GetDesiredPosition().has_value())
    {
        bool isClawAtPosition = claw->GoToPosition(GetDesiredPosition().value());
        if (isClawAtPosition == true) SetClawPosition(GetDesiredPosition());
        else SetClawPosition(std::nullopt);
    }
    else if (controlBoard.GetRawAxis(kManualWristAxis) < -0.5)
    {
        claw->SetWristPower(kWristPower);
        SetClawPosition(std::nullopt);
    }
    else if (controlBoard.GetRawAxis(kManualWristAxis) > 0.5)
    {
        claw->SetWristPower(-kWristPower);
        SetClawPosition(std::nullopt);
    }
    else
    {
        claw->SetWristPower(0.0);
    }
    
    
    if (controlBoard.GetRawButton(kOutputButton) || controlBoard.GetRawButton(kIntakeButton))
    {
        if (GetCurrentPosition().has_value())
        {
            // If we are intaking a coral and there the proximity sensor detects a coral, stop the IO motor
            if (GetCurrentPosition().value().isForCoralIntake == true && claw->IsCoralInClaw() == true)
            {
                claw->SetIOPower(0.0);
            }
            // Otherwise, set the IO motor to the constant at the current Position
            else claw->SetIOPower(GetCurrentPosition().value().ioMotorPower);
        }
    }
    // else if (controlBoard.GetRawAxis(kManualIntakeAxis) < -0.5 || gamepad.GetLeftTriggerAxis() >= 0.5) 
    // {
    //     claw->SetIOPower(kManualIOPower);
    // }
    // else if (controlBoard.GetRawAxis(kManualIntakeAxis) > 0.5 || gamepad.GetRightTriggerAxis() >= 0.5)
    // {
    //     claw->SetIOPower(-kManualIOPower);
    // }
    else if (gamepad.GetLeftTriggerAxis() >= 0.5) 
    {
        claw->SetIOPower(kManualIOPower);
    }
    else if (gamepad.GetRightTriggerAxis() >= 0.5)
    {
        claw->SetIOPower(-kManualIOPower);
    }
    else
    {
        claw->SetIOPower(0);
    }
}

void Controls::ClimberControls()
{
    if (controlBoard.GetRawAxis(kClimberAxis) < -0.5)
    {
        climber->SetPower(kClimberPower);
    }
    else if (controlBoard.GetRawAxis(kClimberAxis) > 0.5)
    {
        climber->SetPower(-kClimberPower);
    }
    else
    {
        climber->SetPower(0.0);
    }
}

void Controls::PneumaticsControls()
{
    pneumatics->SetStopper(controlBoard.GetRawButton(kStopperButton));
}

void Controls::SetDesiredPosition()
{
    if (controlBoard.GetRawButton(kL1Button))
    {
        desiredPosition = Positions::L1;
    }
    else if (controlBoard.GetRawButton(kL2Button))
    {
        desiredPosition = Positions::L2;
    }
    else if (controlBoard.GetRawButton(kL3Button))
    {
        desiredPosition = Positions::L3;
    }
    else if (controlBoard.GetRawButton(kL4Button))
    {
        desiredPosition = Positions::L4;
    }
    else if (controlBoard.GetRawButton(kAlgaeLowButton))
    {
        desiredPosition = Positions::AlgaeLow;
    }
    else if (controlBoard.GetRawButton(kAlgaeHighButton))
    {
        desiredPosition = Positions::AlgaeHigh;
    }
    else if (controlBoard.GetRawButton(kCoralStationButton))
    {
        desiredPosition = Positions::CoralStation;
    }
    else if (controlBoard.GetRawButton(kProcessorButton))
    {
        desiredPosition = Positions::Processor;
    }
    else
    {
        desiredPosition = std::nullopt;
    }
}

void Controls::UpdateTelemetry()
{
    frc::SmartDashboard::PutString("Claw Position", GetClawPosition() ? GetClawPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Elevator Position", GetElevatorPosition() ? GetElevatorPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Current Position", GetCurrentPosition() ? GetCurrentPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Desired Position", GetDesiredPosition() ? GetDesiredPosition().value().to_string() : "None");
}