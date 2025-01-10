#include "subsystems/Drivetrain.h"


Drivetrain::Drivetrain()
{
    gyro.Reset();

    RobotConfig config = RobotConfig::fromGUISettings();
    // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ Drive(speeds, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(kDriveP, kDriveI, kDriveD), // Translation PID constants
            PIDConstants(kTurnP, kTurnP, kTurnP) // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}


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

void Drivetrain::Drive(frc::ChassisSpeeds speeds, bool fieldRelative)
{
    SetRobotRelativeSpeeds(speeds); // Sets ChassisSpeeds before field relative translation for PPLib
    if (fieldRelative) speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, gyro.GetRotation2d());
    wpi::array<frc::SwerveModuleState, 4U> states = kinematics.ToSwerveModuleStates(speeds);
    
    kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    frontLeft.SetDesiredState(fl);
    frontRight.SetDesiredState(fr);
    backLeft.SetDesiredState(bl);
    backRight.SetDesiredState(br);
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
