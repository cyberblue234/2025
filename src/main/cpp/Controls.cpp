#include "Controls.h"

Controls::Controls(Drivetrain *swerve, Elevator *elevator, Limelight *limelightHigh, Limelight *limelightLow)
{
    // Sets all of the class pointers to the arguments for later use
    this->swerve = swerve;
    this->elevator = elevator;
    this->limelightHigh = limelightHigh;
    this->limelightLow = limelightLow;
}

void Controls::Periodic()
{
    // Runs the different controls
    DriveControls();
    ElevatorControls();
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
    // The scalar helps to smooth out driving while preserving full control over the joystick
    const double scalar = x * x + y * y;
    // Adds a speed adjusmtment based on the right trigger - the more it is pressed, the slower the bot will travel for a maximum reduction of 80%
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
    int dPadControl = gamepad.GetPOV();
    if (dPadControl == 0) 
    {
        elevator->SetMotors(0.4);
    }
    else if (dPadControl == 180)
    {
        elevator->SetMotors(-0.4);
    }
    else
    {
        elevator->SetMotors(0);
    }
}