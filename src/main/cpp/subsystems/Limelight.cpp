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
    SetRobotOrientation(name, yaw.value(), yawRate.value(), 0, 0, 0, 0);
    return getBotPoseEstimate_wpiBlue_MegaTag2(name);
}

double Limelight::GetDistanceFromTarget()
{
    // Grabs the distance to target on the x and z planes (forward/back, left/right)
    double xDist = getTargetPose_RobotSpace().at(0);
    double zDist = getTargetPose_RobotSpace().at(2);

    // Find hypotenuse (total distance) of x and z planes
    double distance = sqrt((xDist * xDist) + (zDist * zDist));

    // Return the total distance
    return distance;
}