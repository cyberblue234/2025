#pragma once

#include <frc/XboxController.h>
#include <frc/DriverStation.h>

#include <frc/TimedRobot.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>

#include "subsystems/SwerveModule.h"
#include "subsystems/Limelight.h"
#include "Constants.h"

// This file uses Drivetrain and PathPlanner a lot
using namespace DrivetrainConstants;
using namespace PathPlannerConstants;
using namespace pathplanner;
using namespace ctre::phoenix6;

class Drivetrain : frc2::SubsystemBase
{
public:
    /// @brief Constructs the swerve drivetrain
    /// @param limlightHigh Pointer to the higher limelight object
    /// @param limlightLow Pointer to the lower limelight object
    Drivetrain(Limelight *limelightHigh, Limelight *limelightLow);

    /// @brief Calculates the desired SwerveModuleStates for all of the Swerve Modules
    /// @param speeds The created ChassisSpeeds to run the bot - must be robot relative
    /// @param fieldRelative When set to true, the ChassisSpeeds will be updated to fieldRelative
    void Drive(frc::ChassisSpeeds speeds, bool fieldRelative);

    /// @brief Sets current robot pose
    /// @param pose Pose to reset the pose to
    /// @warning Might be an issue - before it was .ResetPosition(...)
    void ResetPose(frc::Pose2d pose) { odometry.ResetPose(pose); };
    /// @brief Returns current robot pose
    /// @return Pose2d of current robot pose
    frc::Pose2d GetPose() { return odometry.GetEstimatedPosition(); };
    /// @brief Sets the current chassis speeds
    /// @param newSpeeds The new robot relative chassis speeds
    void SetRobotRelativeSpeeds(frc::ChassisSpeeds newSpeeds) { robotRelativeSpeeds = newSpeeds; };
    /// @brief Returns the current chassis speeds
    /// @return ChassisSpeeds of the current speeds
    frc::ChassisSpeeds GetRobotRelativeSpeeds() { return robotRelativeSpeeds; };

    /// @brief List of the branches of the reef. A is the left branch at the face closets to the driver station wall
    enum ReefBranches
    {
        A, B, C, D, E, F, G, H, I, J, K, L
    };
    /// @brief Pathfind to thealliance-specific specified branch
    /// @param branch Branch to pathfind to
    /// @param usePPLibPathFinding Set true to use PPLib pathfinding, false for internal pathfinding (temp)
    /// @return CommandPtr of the path to run - std::nullopt if the path can not exist
    std::optional<frc2::CommandPtr> PathfindToBranch(ReefBranches branch, bool usePPLibPathfinding);

    /// @brief Lists the left and right coral stations from the perspective of the driver station
    enum CoralStations
    {
        Left, Right
    };
    /// @brief Pathfind to the alliance-specific specified coral loading station
    /// @param station Station to pathfind to
    /// @param usePPLibPathFinding Set true to use PPLib pathfinding, false for internal pathfinding (temp)
    /// @return CommandPtr of the path to run - std::nullopt if the path can not exist
    std::optional<frc2::CommandPtr> PathfindToCoralStation(CoralStations station, bool usePPLibPathfinding);

    /// @brief Pathfind to alliance-specific processor
    /// @param usePPLibPathFinding Set true to use PPLib pathfinding, false for internal pathfinding (temp)
    /// @return CommandPtr of the path to run - std::nullopt if the path can not exist
    std::optional<frc2::CommandPtr> PathfindToProcessor(bool usePPLibPathfinding);

    /// @brief Internal pathfinding
    /// @param pose Pose to pathfind to
    /// @param endHeading The heading to drive at of the end pose. Note, this is NOT the same as the end goal state - that should be included in pose
    /// @return CommandPtr of the path to run - std::nullopt if the path can not exist
    std::optional<frc2::CommandPtr> PathfindToPose(frc::Pose2d pose, frc::Rotation2d endHeading, bool preventFlipping);

    /// @brief Calls odometry update
    void UpdateOdometry();
    /// @brief Updates SmartDashboard values
    void UpdateTelemetry();
    /// @brief Gets the gyro angle from the robot perspective
    /// @return Rotation2d of the current gyro angle
    frc::Rotation2d GetRobotGyroAngle() { return gyro.GetRotation2d(); };
    /// @brief Gets the gyro angle from the driver's perspective
    /// @return Rotation2d of the current gyro angle
    frc::Rotation2d GetDriverGyroAngle() { return GetRobotGyroAngle().RotateBy(drivingOffset); };
    /// @brief Gets the gyro angle from the perspective of a blue origin
    /// @return Rotation2d of the current gyro angle
    frc::Rotation2d GetBlueOriginGyroAngle() { return GetRobotGyroAngle().RotateBy(blueOriginOffset); };
    /// @brief Gets the rate of the gyro yaw
    /// @return Rate of gyro yaw in degrees per second
    units::degrees_per_second_t GetYawRate() { return units::degrees_per_second_t{-gyro.GetRate()}; };
    
    /// @brief Resets the gyro angles
    void ResetGyro() { gyro.Reset(); };
    /// @brief Sets the driving offset to the negated current angle
    void ResetDrivingGyro() { drivingOffset = -GetRobotGyroAngle().Degrees(); };

    void ConfigureBlueOriginOffset() 
    {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
            if (alliance.value() == frc::DriverStation::Alliance::kBlue) blueOriginOffset = 180_deg;
        }
    };

    /// @brief Resets drive encoders to 0
    void ResetDriveDistances() 
    {
        frontLeft.SetEncoder(0_tr);
        frontRight.SetEncoder(0_tr);
        backLeft.SetEncoder(0_tr);
        backRight.SetEncoder(0_tr);
    };

    /// @brief Simulation periodic
    void Sim() 
    {
        frontLeft.SimMode();
        frontRight.SimMode();
        backLeft.SimMode();
        backRight.SimMode();
    }

    /// @brief Returns the acceleration in the x-direction
    /// @return Acceleration in meters per second squared
    const units::meters_per_second_squared_t GetXAcceleration() { return gyro.GetAccelerationX().GetValue(); };
    /// @brief Returns the acceleration in the y-direction
    /// @return Acceleration in meters per second squared
    const units::meters_per_second_squared_t GetYAcceleration() { return gyro.GetAccelerationY().GetValue(); };

private:
    // Creates the four swerve modules - see SwerveModule.h
    SwerveModule frontLeft{"Front Left", RobotMap::Drivetrain::kFrontLeftDriveID, RobotMap::Drivetrain::kFrontLeftTurnID, RobotMap::Drivetrain::kFrontLeftCanCoderID, kFrontLeftMagnetOffset};
    SwerveModule frontRight{"Front Right", RobotMap::Drivetrain::kFrontRightDriveID, RobotMap::Drivetrain::kFrontRightTurnID, RobotMap::Drivetrain::kFrontRightCanCoderID, kFrontRightMagnetOffset};
    SwerveModule backLeft{"Back Left", RobotMap::Drivetrain::kBackLeftDriveID, RobotMap::Drivetrain::kBackLeftTurnID, RobotMap::Drivetrain::kBackLeftCanCoderID, kBackLeftMagnetOffset};
    SwerveModule backRight{"Back Right", RobotMap::Drivetrain::kBackRightDriveID, RobotMap::Drivetrain::kBackRightTurnID, RobotMap::Drivetrain::kBackRightCanCoderID, kBackRightMagnetOffset};

    // Creates the gyro object
    hardware::Pigeon2 gyro{RobotMap::Drivetrain::kGyroID, "rio"};
    units::degree_t drivingOffset = 180_deg;
    units::degree_t blueOriginOffset = 0_deg;

    // Creates the two limelight objects, one is higher on the robot and one is lower
    Limelight *limelightHigh;
    Limelight *limelightLow;
    
    // Creates a field object for use of odometry and PathPlanner debugging
    frc::Field2d field{};

    // Container for the current relative robot speeds
    frc::ChassisSpeeds robotRelativeSpeeds;
    // Creates the translation and rotation PIDs for PathPlanner
    PIDConstants translationPIDs{kTranslationP, kTranslationI, kTranslationD};
    PIDConstants rotationPIDs{kRotationP, kRotationI, kRotationD};

    // FRC kinematics to determine individual wheel speeds based on the desired chassis speeds
    frc::SwerveDriveKinematics<4> kinematics
    {
        kFrontLeftLocation,
        kFrontRightLocation,
        kBackLeftLocation,
        kBackRightLocation
    };

    // Odometry object that allows for vision input with a standard deviation
    frc::SwerveDrivePoseEstimator<4> odometry
    {
        kinematics,
        GetRobotGyroAngle(),
        {
            frontLeft.GetPosition(), frontRight.GetPosition(),
            backLeft.GetPosition(), backRight.GetPosition()
        },
        frc::Pose2d()
    };
    
    /// @brief Gets the pose of the different branches - will flip the pose if on the blue alliance
    /// @param branch Branch to get the pose of
    /// @return Pose2d of the branch pose
    static std::optional<frc::Pose2d> FormatBranch(ReefBranches branch)
    {
        frc::Pose2d pose;
        switch(branch)
        {
            case 0:  pose = frc::Pose2d(14.40_m, 3.87_m, frc::Rotation2d(180_deg)); break;
            case 1:  pose = frc::Pose2d(14.40_m, 4.16_m, frc::Rotation2d(180_deg)); break;
            case 6:  pose = frc::Pose2d(11.74_m, 4.16_m, frc::Rotation2d(0_deg)); break;
            case 7:  pose = frc::Pose2d(11.74_m, 3.87_m, frc::Rotation2d(0_deg)); break;

            case 2:  pose = frc::Pose2d(13.85_m, 5.05_m, frc::Rotation2d(-120_deg)); break;
            case 3:  pose = frc::Pose2d(13.59_m, 5.22_m, frc::Rotation2d(-120_deg)); break;
            case 4:  pose = frc::Pose2d(12.58_m, 5.22_m, frc::Rotation2d(-60_deg)); break;
            case 5:  pose = frc::Pose2d(12.28_m, 5.06_m, frc::Rotation2d(-60_deg)); break;
            case 8:  pose = frc::Pose2d(12.28_m, 2.97_m, frc::Rotation2d(60_deg)); break;
            case 9:  pose = frc::Pose2d(12.58_m, 2.84_m, frc::Rotation2d(60_deg)); break;
            case 10: pose = frc::Pose2d(13.58_m, 2.84_m, frc::Rotation2d(120_deg)); break;
            case 11: pose = frc::Pose2d(13.85_m, 2.97_m, frc::Rotation2d(120_deg)); break;

            default: return std::nullopt; break;
        }
        return FlipPose(pose);
    }

    /// @brief Gets the pose of the left or right station - will flip the pose if on the blue alliance
    /// @param station Station to get the pose of
    /// @return Pose2d of the station
    static std::optional<frc::Pose2d> FormatStation(CoralStations station)
    {
        frc::Pose2d pose;
        switch(station)
        {
            case CoralStations::Left: pose = frc::Pose2d(16.15_m, 1.29_m, frc::Rotation2d(126.0_deg)); break;
            case CoralStations::Right: pose = frc::Pose2d(16.15_m, 6.82_m, frc::Rotation2d(-126_deg)); break;

            default: return std::nullopt; break;
        }
        return FlipPose(pose);
    }

    /// @brief Gets the pose of the processor - will flip the pose if on the blue alliance
    /// @return Pose2d of the processor
    static std::optional<frc::Pose2d> FormatProcessor()
    {
        frc::Pose2d pose = frc::Pose2d(11.52_m, 7.59_m, frc::Rotation2d(90_deg));
        return FlipPose(pose);
    }

    /// @brief Flips the given pose if the alliance is blue
    /// @param pose Pose to flip based on conditional
    /// @return Pose2d of the possibly flipped pose - std::nullopt if the alliance doesn't exist
    static std::optional<frc::Pose2d> FlipPose(frc::Pose2d pose)
    {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) 
        {
            if (alliance.value() == frc::DriverStation::Alliance::kBlue)
            {
                return pathplanner::FlippingUtil::flipFieldPose(pose);
            }
            return pose;
        }
        else return std::nullopt;
    }
};