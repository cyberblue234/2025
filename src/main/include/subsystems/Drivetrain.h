#pragma once

#include <frc/XboxController.h>
#include <frc/DriverStation.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>

#include <frc2/command/SubsystemBase.h>

#include "studica/AHRS.h"
#include <frc/simulation/SimDeviceSim.h>

#include "subsystems/SwerveModule.h"
#include "subsystems/Limelight.h"
#include "Constants.h"

using namespace DrivetrainConstants;
using namespace PathPlannerConstants;
using namespace pathplanner;

class Drivetrain : frc2::SubsystemBase
{
public:
    /// @brief Constructs the swerve drivetrain 
    Drivetrain(Limelight *limelight);

    /// @brief Calculates the desired SwerveModuleStates for all of the Swerve Modules
    /// @param speeds The created ChassisSpeeds to run the bot - must be robot relative
    /// @param fieldRelative When set to true, the ChassisSpeeds will be updated to fieldRelative
    void Drive(frc::ChassisSpeeds speeds, bool fieldRelative);

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
    /// @brief Gets the gyro angle
    /// @return Rotation2d of the current gyro angle
    frc::Rotation2d GetGyroAngle() { return frc::RobotBase::IsReal() ?  gyro.GetRotation2d() : simYaw.RotateBy(simOffset); };
    
    /// @brief Resets the gyro yaw
    void ResetGyro() { if (frc::RobotBase::IsReal()) gyro.Reset(); else simOffset = -simYaw.Degrees(); }
    /// @brief Sets the gyro adjustment
    void SetGyroAdjustment(double angle) { if (frc::RobotBase::IsReal()) gyro.SetAngleAdjustment(angle); else simOffset = units::degree_t(angle); };

    /// @brief Resets drive encoders to 0
    void ResetDriveDistances() 
    {
        frontLeft.SetEncoder(0_tr);
        frontRight.SetEncoder(0_tr);
        backLeft.SetEncoder(0_tr);
        backRight.SetEncoder(0_tr);
    };

    void Sim() 
    {
        frontLeft.SimMode();
        frontRight.SimMode();
        backLeft.SimMode();
        backRight.SimMode();
    }

    /// @brief Returns the acceleration in the x-direction
    /// @return Acceleration in meters per second squared
    const units::meters_per_second_squared_t GetXAcceleration() { return gyro.GetWorldLinearAccelX() * 9.80665_mps_sq; };
    /// @brief Returns the acceleration in the y-direction
    /// @return Acceleration in meters per second squared
    const units::meters_per_second_squared_t GetYAcceleration() { return gyro.GetWorldLinearAccelY() * 9.80665_mps_sq; };

private:
    SwerveModule frontLeft{"Front Left", RobotMap::kFrontLeftDriveID, RobotMap::kFrontLeftTurnID, RobotMap::kFrontLeftCanCoderID, kFrontLeftMagnetOffset};
    SwerveModule frontRight{"Front Right", RobotMap::kFrontRightDriveID, RobotMap::kFrontRightTurnID, RobotMap::kFrontRightCanCoderID, kFrontRightMagnetOffset};
    SwerveModule backLeft{"Back Left", RobotMap::kBackLeftDriveID, RobotMap::kBackLeftTurnID, RobotMap::kBackLeftCanCoderID, kBackLeftMagnetOffset};
    SwerveModule backRight{"Back Right", RobotMap::kBackRightDriveID, RobotMap::kBackRightTurnID, RobotMap::kBackRightCanCoderID, kBackRightMagnetOffset};

    studica::AHRS gyro{studica::AHRS::NavXComType::kMXP_SPI};
    frc::Rotation2d simYaw{0_deg};
    units::degree_t simOffset;

    Limelight *limelight;

    frc::Field2d field{};

    frc::ChassisSpeeds robotRelativeSpeeds;
    PIDConstants translationPIDs{kTranslationP, kTranslationI, kTranslationD};
    PIDConstants rotationPIDs{kRotationP, kRotationI, kRotationD};


    frc::SwerveDriveKinematics<4> kinematics{
        kFrontLeftLocation,
        kFrontRightLocation,
        kBackLeftLocation,
        kBackRightLocation};

    frc::SwerveDrivePoseEstimator<4> odometry{
        kinematics,
        GetGyroAngle(),
        {frontLeft.GetPosition(), frontRight.GetPosition(),
         backLeft.GetPosition(), backRight.GetPosition()},
        frc::Pose2d()};

};