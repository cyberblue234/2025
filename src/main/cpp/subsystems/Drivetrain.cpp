#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain(Limelight *limelightHigh, Limelight *limelightLow)
{
    // Sets the class variables to the given inputs
    this->limelightHigh = limelightHigh;
    this->limelightLow = limelightLow;

    // Resets gyro to ensure proper initialization
    gyro.GetConfigurator().Apply(configs::Pigeon2Configuration{});
    ResetGyro();

    // Gets the settings from PathPlanner GUI
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

    // The field will draw the current path
    pathplanner::PathPlannerLogging::setLogActivePathCallback([this](auto poses) {
        this->field.GetObject("path")->SetPoses(poses);
    });

    // Gives a reference to the field object, which allows it to update without a periodic call
    frc::SmartDashboard::PutData("Field", &field);

    moduleStatesPublisher.Set(wpi::array<frc::SwerveModuleState, 4U>{frontLeft.GetState(), frontRight.GetState(), backLeft.GetState(), backRight.GetState()});
}

void Drivetrain::Drive(frc::ChassisSpeeds speeds, bool fieldRelative)
{
    // Sets ChassisSpeeds before field relative translation for PPLib
    SetRobotRelativeSpeeds(speeds);
    // Converts to a field relative drive system if fieldRelative is true
    if (fieldRelative) speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetDriverGyroAngle());
    // Turns the ChassisSpeeds to four states to set the SwerveModules to
    wpi::array<frc::SwerveModuleState, 4U> states = kinematics.ToSwerveModuleStates(speeds);
    // Ensures no module has a set velocity above the theoretically maximum velocity the modules can achieve
    kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    // Changes the simulation gyro tools
    if (frc::RobotBase::IsSimulation())
    {
        gyroSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
        gyroSim.AddYaw(speeds.omega * 0.02_s);
    }
    
    // Splits the vector into four individual states
    auto [fl, fr, bl, br] = states;
    // Calls the funcitons to set the states
    frontLeft.SetDesiredState(fl);
    frontRight.SetDesiredState(fr);
    backLeft.SetDesiredState(bl);
    backRight.SetDesiredState(br);

    moduleStatesPublisher.Set(states);
}

std::optional<frc2::CommandPtr> Drivetrain::PathfindToBranch(Sides side, units::meter_t offset, bool usePPLibPathfinding)
{
    frc::Pose2d aprilTagPose = GetClosestBranchTagPose();

    units::degree_t theta = aprilTagPose.Rotation().Degrees() + 90_deg;
    units::meter_t deltaX1 = units::math::cos(theta) * kDeltaReefAprilTagToBranch;
    units::meter_t deltaY1 = units::math::sin(theta) * kDeltaReefAprilTagToBranch;
    if (side == Sides::Left)
    {
        deltaX1 = -deltaX1;
        deltaY1 = -deltaY1;
    }
    units::meter_t deltaX2 = units::math::sin(theta) * offset;
    units::meter_t deltaY2 = -units::math::cos(theta) * offset;

    frc::Pose2d pose{aprilTagPose.X() + deltaX1 + deltaX2, aprilTagPose.Y() + deltaY1 + deltaY2, aprilTagPose.Rotation().Degrees() + 180_deg};
    // Uses PPLib pathfinding with given constraints
    if (usePPLibPathfinding) return AutoBuilder::pathfindToPose(pose, pathfindingConstraints);
    // Uses internal pathfinding
    return PathfindToPose(pose, pose.Rotation(), true);
}

std::optional<frc2::CommandPtr> Drivetrain::PathfindToCoralStation(Sides station, bool usePPLibPathfinding)
{
    frc::Pose2d pose = FormatStation(station);
    // Uses PPLib pathfinding with given constraints
    if (usePPLibPathfinding) return AutoBuilder::pathfindToPose(pose, pathfindingConstraints);
    // Uses internal pathfinding
    return PathfindToPose(pose, pose.Rotation().RotateBy(180_deg), true);
}

std::optional<frc2::CommandPtr> Drivetrain::PathfindToProcessor(bool usePPLibPathfinding)
{
    frc::Pose2d pose = FormatProcessor();
    // Uses PPLib pathfinding with given constraints
    if (usePPLibPathfinding) return AutoBuilder::pathfindToPose(pose, pathfindingConstraints);
    // Uses internal pathfinding
    return PathfindToPose(pose, pose.Rotation(), true);
}

std::optional<frc2::CommandPtr> Drivetrain::PathfindToPose(frc::Pose2d pose, frc::Rotation2d endHeading, bool preventFlipping)
{
    // Finds the difference of the two x and the two y values
    double xDiff = pose.X().value() - GetPose().X().value();
    double yDiff = pose.Y().value() - GetPose().Y().value();
    // cos(x) is equal to the lengh of the adjacent side divided by the length of the hypotenuse
    // The inverse of cos will give you the angle to drive at, in quadrants one and two
    // If you multiply that by the sign of the yDiff, you will get the final hedaing in radians
    units::radian_t heading = units::radian_t(sgn(yDiff) * acos((xDiff) / (pow(pow(xDiff, 2) + pow(yDiff, 2), 0.5))));
    // Creates a vector of two poses with the rotation being the heading to drive at
    // The first pose is the current pose, and the second is the pose to drive to
    std::vector<frc::Pose2d> poses 
    {
        frc::Pose2d(GetPose().X(), GetPose().Y(), frc::Rotation2d(heading)), 
        frc::Pose2d(pose.X(), pose.Y(), endHeading)
    };
    // Creates a path based on the vector of poses and PathPlanner constraints
    auto path = std::make_shared<PathPlannerPath>(
        PathPlannerPath::waypointsFromPoses(poses),
        PathConstraints(1_mps, 1_mps_sq, 720_deg_per_s, 720_deg_per_s_sq),
        std::nullopt,
        GoalEndState(0.0_mps, pose.Rotation())
    );
    // If preventFlipping is true, it stops the path from flipping automatically 
    // because we have already flipped the desired poses
    path->preventFlipping = preventFlipping;
    // Creates and returns the command to follow the path
    return AutoBuilder::followPath(path);
}

void Drivetrain::UpdateOdometry()
{
    // Updates the odometry with the gyro angle and the wheel positions (drive distance and turn angle)
    odometry.Update(GetRobotGyroAngle(),
                    {frontLeft.GetPosition(), frontRight.GetPosition(),
                     backLeft.GetPosition(), backRight.GetPosition()});
    
    field.SetRobotPose(odometry.GetEstimatedPosition());
    odometryPublisher.Set(GetPose());
}

void Drivetrain::UpdateLimelights()
{
    // Gets the estimated pose from the limelight
    // Uses MegaTag 2 which utilizes current gyro rotation in order to ensure better quality estimations
    PoseEstimate visionHigh = limelightHigh->GetPose(GetBlueOriginGyroAngle().Degrees(), GetYawRate());
    // Rejects the estimation if the rotation rate is too great or if the limelight doesn't see any tags
    if ((abs(GetYawRate().value()) > 720 || visionHigh.tagCount == 0) == false)
    {
        odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.7, 0.7, 9999999.0});
        odometry.AddVisionMeasurement(visionHigh.pose, frc::Timer::GetFPGATimestamp());
    }
    PoseEstimate visionLow = limelightLow->GetPose(GetBlueOriginGyroAngle().Degrees(), GetYawRate());
    if ((abs(GetYawRate().value()) > 720 || visionLow.tagCount == 0) == false)
    {
        odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>{0.7, 0.7, 9999999.0});
        odometry.AddVisionMeasurement(visionLow.pose, frc::Timer::GetFPGATimestamp());
    }
}

void Drivetrain::UpdateTelemetry()
{
    frontLeft.UpdateTelemetry();
    frontRight.UpdateTelemetry();
    backLeft.UpdateTelemetry();
    backRight.UpdateTelemetry();

    frc::SmartDashboard::PutNumber("X Acceleration", GetXAcceleration().value());
    frc::SmartDashboard::PutNumber("Y Acceleration", GetYAcceleration().value());
    frc::SmartDashboard::PutNumber("Gyro Robot Yaw", GetRobotGyroAngle().Degrees().value());
    frc::SmartDashboard::PutNumber("Gyro Driver Yaw", GetDriverGyroAngle().Degrees().value());
    frc::SmartDashboard::PutNumber("Gyro Blue Origin Yaw", GetBlueOriginGyroAngle().Degrees().value());
    frc::SmartDashboard::PutNumber("Odom X", odometry.GetEstimatedPosition().X().value());
    frc::SmartDashboard::PutNumber("Odom Y", odometry.GetEstimatedPosition().Y().value());
    frc::SmartDashboard::PutNumber("Odom Rot", odometry.GetEstimatedPosition().Rotation().Degrees().value());
}