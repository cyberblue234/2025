#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain()
{
    gyro.Reset();
}

void Drivetrain::Drive(meters_per_second_t xSpeed,
                       meters_per_second_t ySpeed,
                       radians_per_second_t rot, bool fieldRelative,
                       second_t period)
{
    SmartDashboard::PutNumber("xSpeed", xSpeed.value());
    SmartDashboard::PutNumber("ySpeed", ySpeed.value());
    SmartDashboard::PutNumber("rot", rot.value());
    
    wpi::array<SwerveModuleState, 4U> states = kinematics.ToSwerveModuleStates(ChassisSpeeds::Discretize(
            fieldRelative ? ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.GetRotation2d())
                          : ChassisSpeeds{xSpeed, ySpeed, rot}, period));
    // auto states =
    //     kinematics.ToSwerveModuleStates(
    //         fieldRelative ? ChassisSpeeds::FromFieldRelativeSpeeds(
    //                             xSpeed, ySpeed, rot, gyro.GetRotation2d())
    //                       : ChassisSpeeds{xSpeed, ySpeed, rot}
    //         );

    kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    frontLeft.SetDesiredState(fl);
    frontRight.SetDesiredState(fr);
    backLeft.SetDesiredState(bl);
    backRight.SetDesiredState(br);
    
    SmartDashboard::PutData("Field", &field);
}

void Drivetrain::UpdateOdometry()
{
    odometry.Update(gyro.GetRotation2d(),
                    {frontLeft.GetPosition(), frontRight.GetPosition(),
                     backLeft.GetPosition(), backRight.GetPosition()});
    field.SetRobotPose(odometry.GetEstimatedPosition());
}

void Drivetrain::UpdateTelemetry()
{
    frontLeft.UpdateTelemetry();
    frontRight.UpdateTelemetry();
    backLeft.UpdateTelemetry();
    backRight.UpdateTelemetry();

    SmartDashboard::PutNumber("X Acceleration", GetXAcceleration().value());
    SmartDashboard::PutNumber("Y Acceleration", GetYAcceleration().value());
    SmartDashboard::PutNumber("Gyro Yaw", gyro.GetRotation2d().Degrees().value());
}