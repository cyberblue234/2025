#include "subsystems/Drivetrain.h"

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period)
{
    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.value());
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.value());
    frc::SmartDashboard::PutNumber("rot", rot.value());
    
    auto states =
        kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
            fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rot, gyro.GetRotation2d())
                          : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
            period));
    // auto states =
    //     kinematics.ToSwerveModuleStates(
    //         fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    //                             xSpeed, ySpeed, rot, gyro.GetRotation2d())
    //                       : frc::ChassisSpeeds{xSpeed, ySpeed, rot}
    //         );

    kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    frontLeft.SetDesiredState(fl);
    frontRight.SetDesiredState(fr);
    backLeft.SetDesiredState(bl);
    backRight.SetDesiredState(br);
    
    frc::SmartDashboard::PutData("Field", &field);
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

    frc::SmartDashboard::PutNumber("X Acceleration", GetXAcceleration().value());
    frc::SmartDashboard::PutNumber("Y Acceleration", GetYAcceleration().value());
    frc::SmartDashboard::PutNumber("Gyro Yaw", gyro.GetRotation2d().Degrees().value());
}
