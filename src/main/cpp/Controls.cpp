#include "Controls.h"

Controls::Controls(Drivetrain *swerve, KitBotOutput *kitBotOutput)
{
    this->swerve = swerve;
    this->kitBotOutput = kitBotOutput;
}

void Controls::Periodic(time::second_t period)
{
    DriveControls(period);
}

void Controls::DriveControls(time::second_t period)
{
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

    const meters_per_second_t xSpeed = ApplyDeadband(x * scalar, 0.075) * DrivetrainConstants::kMaxSpeed * speedAdjust;
    const meters_per_second_t ySpeed = ApplyDeadband(y * scalar, 0.075) * DrivetrainConstants::kMaxSpeed * speedAdjust;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const radians_per_second_t rot = -ApplyDeadband(pow(gamepad.GetRightX(), 3), 0.05) *
                     DrivetrainConstants::kMaxAngularSpeed;

    if (swerve) swerve->Drive(xSpeed, ySpeed, rot, true, period);
}