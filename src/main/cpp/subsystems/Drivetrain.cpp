#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain(Limelight *limelightHigh, Limelight *limelightLow)
{
    this->limelightHigh = limelightHigh;
    this->limelightLow = limelightLow;

    ResetGyro();

    RobotConfig config = RobotConfig::fromGUISettings();
    // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ Drive(speeds, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>(translationPIDs, rotationPIDs),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the blue alliance
            // This will flip the path being followed to the blue side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kBlue;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    pathplanner::PathPlannerLogging::setLogActivePathCallback([this](auto poses) {
        this->field.GetObject("path")->SetPoses(poses);
    });

    frc::SmartDashboard::PutData("Field", &field);
}

void Drivetrain::Drive(frc::ChassisSpeeds speeds, bool fieldRelative)
{
    SetRobotRelativeSpeeds(speeds); // Sets ChassisSpeeds before field relative translation for PPLib
    if (fieldRelative) speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetGyroAngle());
    wpi::array<frc::SwerveModuleState, 4U> states = kinematics.ToSwerveModuleStates(speeds);
    
    kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    if (frc::RobotBase::IsSimulation()) simYaw = simYaw + frc::Rotation2d(speeds.omega * 0.02_s);

    auto [fl, fr, bl, br] = states;

    frontLeft.SetDesiredState(fl);
    frontRight.SetDesiredState(fr);
    backLeft.SetDesiredState(bl);
    backRight.SetDesiredState(br);
}

std::optional<frc2::CommandPtr> Drivetrain::PathfindToBranch(ReefBranches branch, bool usePPLibPathfinding)
{
    auto tmpPose = FormatBranch(branch);
    if (!tmpPose) return std::nullopt;
    frc::Pose2d pose = tmpPose.value();
    if (usePPLibPathfinding) return AutoBuilder::pathfindToPose(pose, PathConstraints(1_mps, 1_mps_sq, 720_deg_per_s, 720_deg_per_s_sq));
    return PathfindToPose(pose, pose.Rotation(), true);
}

std::optional<frc2::CommandPtr> Drivetrain::PathfindToCoralStation(CoralStations station, bool usePPLibPathfinding)
{
    auto tmpPose = FormatStation(station);
    if (!tmpPose) return std::nullopt;
    frc::Pose2d pose = tmpPose.value();
    if (usePPLibPathfinding) return AutoBuilder::pathfindToPose(pose, PathConstraints(1_mps, 1_mps_sq, 720_deg_per_s, 720_deg_per_s_sq));
    return PathfindToPose(pose, pose.Rotation().RotateBy(180_deg), true);
}

std::optional<frc2::CommandPtr> Drivetrain::PathfindToProcessor(bool usePPLibPathfinding)
{
    auto tmpPose = FormatProcessor();
    if (!tmpPose) return std::nullopt;
    frc::Pose2d pose = tmpPose.value();
    if (usePPLibPathfinding) return AutoBuilder::pathfindToPose(pose, PathConstraints(1_mps, 1_mps_sq, 720_deg_per_s, 720_deg_per_s_sq));
    return PathfindToPose(pose, pose.Rotation(), true);
}

std::optional<frc2::CommandPtr> Drivetrain::PathfindToPose(frc::Pose2d pose, frc::Rotation2d endHeading, bool preventFlipping)
{
    double xDiff = pose.X().value() - GetPose().X().value();
    double yDiff = pose.Y().value() - GetPose().Y().value();
    units::radian_t heading = units::radian_t(sgn(yDiff) * acos((xDiff) / (pow(pow(xDiff, 2) + pow(yDiff, 2), 0.5))));
    std::vector<frc::Pose2d> poses 
    {
        frc::Pose2d(GetPose().X(), GetPose().Y(), frc::Rotation2d(heading)), 
        frc::Pose2d(pose.X(), pose.Y(), endHeading)
    };
    auto path = std::make_shared<PathPlannerPath>(
        PathPlannerPath::waypointsFromPoses(poses),
        PathConstraints(1_mps, 1_mps_sq, 720_deg_per_s, 720_deg_per_s_sq),
        std::nullopt,
        GoalEndState(0.0_mps, pose.Rotation())
    );
    path->preventFlipping = preventFlipping;
    return AutoBuilder::followPath(path);
}

void Drivetrain::UpdateOdometry()
{
    odometry.Update(frc::RobotBase::IsReal() ? GetGyroAngle() : simYaw,
                    {frontLeft.GetPosition(), frontRight.GetPosition(),
                     backLeft.GetPosition(), backRight.GetPosition()});

    PoseEstimate visionHigh = limelightHigh->GetBotPoseBlue(GetYaw(), GetYawRate());
    if ((abs(GetYawRate().value()) > 720 || visionHigh.tagCount == 0) == false)
    {
        odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.7, 0.7, 9999999.0});
        odometry.AddVisionMeasurement(
            visionHigh.pose,
            frc::Timer::GetFPGATimestamp()
    );
    }
    PoseEstimate visionLow = limelightLow->GetBotPoseBlue(GetYaw(), GetYawRate());
    if ((abs(GetYawRate().value()) > 720 || visionLow.tagCount == 0) == false)
    {
        odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.7, 0.7, 9999999.0});
        odometry.AddVisionMeasurement(
            visionLow.pose,
            frc::Timer::GetFPGATimestamp()
    );
    }
    // units::second_t timeDif = accelTimer.Get();
    // accelTimer.Reset();
    // odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.7, 0.7, 0.0});
    // odometry.AddVisionMeasurement(GetPose().TransformBy(frc::Transform2d{GetXAcceleration() * (timeDif * timeDif), GetYAcceleration() * (timeDif * timeDif), frc::Rotation2d()}), frc::Timer::GetFPGATimestamp());

    
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
    frc::SmartDashboard::PutNumber("Gyro Yaw", GetGyroAngle().Degrees().value());
    frc::SmartDashboard::PutNumber("Odom X", odometry.GetEstimatedPosition().X().value());
    frc::SmartDashboard::PutNumber("Odom Y", odometry.GetEstimatedPosition().Y().value());
    frc::SmartDashboard::PutNumber("Odom Rot", odometry.GetEstimatedPosition().Rotation().Degrees().value());
}