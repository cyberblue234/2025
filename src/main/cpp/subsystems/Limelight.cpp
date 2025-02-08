#include "subsystems/Limelight.h"

Limelight::Limelight(std::string name)
{
    this->name = name;
}

Limelight::Limelight()
{
    this->name = "";
}

PoseEstimate Limelight::GetBotPoseBlue(units::degree_t yaw, units::degrees_per_second_t yawRate)
{
    // Sets the robot angle and angular velocity
    SetRobotOrientation(name, yaw.value(), yawRate.value(), 0, 0, 0, 0);
    // Gets the estimated position
    return getBotPoseEstimate_wpiBlue_MegaTag2(name);
}

units::meter_t Limelight::GetDistanceFromTarget()
{
    std::vector<double> targetPoseRobotSpace = getTargetPose_RobotSpace();
    if (targetPoseRobotSpace.empty() == true) return -1_m;
    // Grabs the distance to target on the x and z planes (forward/back, left/right)
    double xDist = targetPoseRobotSpace.at(0);
    double zDist = targetPoseRobotSpace.at(2);

    // Find hypotenuse (total distance) of x and z planes
    double distance = sqrt((xDist * xDist) + (zDist * zDist));

    // Return the total distance
    return units::meter_t(distance);
}

void Limelight::UpdateTelemetry()
{
    TelemetryHelperNumber("Distance from Apriltag", GetDistanceFromTarget().value());
}