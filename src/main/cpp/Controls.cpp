#include "Controls.h"

Controls::Controls(Drivetrain *swerve, KitBotOutput *kitBotOutput)
{
    this->swerve = swerve;
    this->kitBotOutput = kitBotOutput;
}

void Controls::Periodic(units::time::second_t period)
{
    // if (gamepad.GetYButton()) frc::SmartDashboard::PutNumber("Drive kS", swerve->FindDrive_kS(0_V).value());
    // if (gamepad.GetXButton()) swerve->ResetDriveDistances();
    DriveControls(period);
}

void Controls::DriveControls(units::time::second_t period)
{
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double x = -gamepad.GetLeftY();
    double y = -gamepad.GetLeftX();
    double scalar = x * x + y * y;
    const units::meters_per_second_t xSpeed = ApplyDeadband(x * DrivetrainConstants::kMaxSpeed.value() * scalar, 0.1) * 1_mps;
    
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const units::meters_per_second_t ySpeed = ApplyDeadband(y * DrivetrainConstants::kMaxSpeed.value() * scalar, 0.1) * 1_mps;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const units::radians_per_second_t rot = -ApplyDeadband(pow(gamepad.GetRightX(), 3), 0.1) *
                     DrivetrainConstants::kMaxAngularSpeed;


    if (swerve) swerve->Drive(xSpeed, ySpeed, rot, true, period);
}