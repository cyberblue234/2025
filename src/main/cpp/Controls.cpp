#include "Controls.h"

Controls::Controls(Drivetrain *swerve, KitBotOutput *kitBotOutput, Limelight *limelight3)
{
    this->swerve = swerve;
    this->kitBotOutput = kitBotOutput;
    this->limelight3 = limelight3;
    frc::SmartDashboard::PutNumber("Kitbot Output Speed", 0.4);
}

void Controls::Periodic(units::second_t period)
{
    DriveControls(period);
    KitBotControls();
}

void Controls::DriveControls(units::second_t period)
{
    if (gamepad.GetYButton()) swerve->ResetGyro();
    if (gamepad.GetLeftBumperButtonPressed()) 
    {
        if (branch == 11) branch = 0;
        else branch += 1;
    }
    frc::SmartDashboard::PutNumber("Selected Branch", branch);
    if (gamepad.GetAButton()) 
    { 
        path = swerve->PathfindToBranch(branch);
        path->Schedule();
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
    const units::radians_per_second_t rot = -ApplyDeadband(pow(gamepad.GetRightX(), 3), 0.05) *
                     DrivetrainConstants::kMaxAngularSpeed * pow(speedAdjust, 0.5);

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.value());
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.value());
    frc::SmartDashboard::PutNumber("rot", rot.value());

    frc::ChassisSpeeds setSpeeds = frc::ChassisSpeeds::Discretize(frc::ChassisSpeeds{xSpeed, ySpeed, rot}, period);
    if (path)
    {
        if (path->IsScheduled())
        {
            if (setSpeeds.vx > 0.0_mps || setSpeeds.vy > 0.0_mps || setSpeeds.omega > 0.0_rad_per_s) path->Cancel();
            else return;
        }
    }
    swerve->Drive(setSpeeds, fieldRelative);
}

void Controls::KitBotControls() 
{
    if (gamepad.GetLeftTriggerAxis() > 0.5) kitBotOutput->SetMotor(-frc::SmartDashboard::GetNumber("Kitbot Output Speed", 0.4));
    else kitBotOutput->SetMotor(0.0);
}