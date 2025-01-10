#pragma once

#include <frc/XboxController.h>
#include <frc/DriverStation.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include "studica/AHRS.h"

#include "subsystems/SwerveModule.h"
#include "Constants.h"

using namespace DrivetrainConstants;
using namespace pathplanner;

class Drivetrain : frc2::SubsystemBase
{
public:
    /// @brief Constructs the swerve drivetrain 
    Drivetrain();
    
    /// @brief Calculates the desired SwerveModuleStates for all of the SwerveModules
    /// @param xSpeed Desired speed in the x-direction
    /// @param ySpeed Desired speed in the y-direction
    /// @param rot Desired rotational speed
    /// @param fieldRelative True for field relative, false for robot relative
    /// @param period RoboRio cycle period - typically 20ms
    void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::second_t period);

    /// @brief Calculates the desired SwerveModuleStates for all of the Swerve Modules
    /// @param speeds The created ChassisSpeeds to run the bot - must be robot relative
    /// @param fieldRelative When set to true, the ChassisSpeeds will be updated to fieldRelative
    void Drive(ChassisSpeeds speeds, bool fieldRelative);

    /// @brief Sets current robot pose
    /// @warning might be an issue - before it was .ResetPosition(...)
    void ResetPose(frc::Pose2d pose) { odometry.ResetPose(pose); };
    /// @brief Returns current robot pose
    /// @return Pose2d of current robot pose
    frc::Pose2d GetPose() { return odometry.GetEstimatedPosition(); };
    /// @brief Sets the current chassis speeds
    void SetRobotRelativeSpeeds(frc::ChassisSpeeds newSpeeds) { robotRelativeSpeeds = newSpeeds; };
    /// @brief Returns the current chassis speeds
    /// @return ChassisSpeeds of the current speeds
    frc::ChassisSpeeds GetRobotRelativeSpeeds() { return robotRelativeSpeeds; };

    /// @brief Calls odometry update
    void UpdateOdometry();
    /// @brief Updates SmartDashboard values
    void UpdateTelemetry();
    /// @brief Resets the gyro yaw
    void ResetGyro() { gyro.Reset(); }
    /// @brief Resets drive encoders to 0
    void ResetDriveDistances() 
    {
        frontLeft.SetEncoder(0_tr);
        frontRight.SetEncoder(0_tr);
        backLeft.SetEncoder(0_tr);
        backRight.SetEncoder(0_tr);
    };

    /// @brief Returns the acceleration in the x-direction
    /// @return Acceleration in meters per second squared
    units::meters_per_second_squared_t GetXAcceleration() { return gyro.GetWorldLinearAccelX() * 9.80665_mps_sq; };
    /// @brief Returns the acceleration in the y-direction
    /// @return Acceleration in meters per second squared
    units::meters_per_second_squared_t GetYAcceleration() { return gyro.GetWorldLinearAccelY() * 9.80665_mps_sq; };

private:
    SwerveModule frontLeft{"Front Left", RobotMap::kFrontLeftDriveID, RobotMap::kFrontLeftTurnID, RobotMap::kFrontLeftCanCoderID, kFrontLeftMagnetOffset};
    SwerveModule frontRight{"Front Right", RobotMap::kFrontRightDriveID, RobotMap::kFrontRightTurnID, RobotMap::kFrontRightCanCoderID, kFrontRightMagnetOffset};
    SwerveModule backLeft{"Back Left", RobotMap::kBackLeftDriveID, RobotMap::kBackLeftTurnID, RobotMap::kBackLeftCanCoderID, kBackLeftMagnetOffset};
    SwerveModule backRight{"Back Right", RobotMap::kBackRightDriveID, RobotMap::kBackRightTurnID, RobotMap::kBackRightCanCoderID, kBackRightMagnetOffset};

    studica::AHRS gyro{studica::AHRS::NavXComType::kMXP_SPI};

    frc::Field2d field{};

    frc::ChassisSpeeds robotRelativeSpeeds;

    frc::SwerveDriveKinematics<4> kinematics{
        kFrontLeftLocation,
        kFrontRightLocation,
        kBackLeftLocation,
        kBackRightLocation};

    frc::SwerveDrivePoseEstimator<4> odometry{
        kinematics,
        gyro.GetRotation2d(),
        {frontLeft.GetPosition(), frontRight.GetPosition(),
         backLeft.GetPosition(), backRight.GetPosition()},
        frc::Pose2d()};
};