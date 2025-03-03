#include "subsystems/Limelight.h"

Limelight::Limelight(std::string name)
{
    this->name = SanitizeName(name);
    SetupPortForwarding();

    tv = GetIntegerSubscriber(name, "tv"); 
    tx = GetDoubleSubscriber(name, "tx");
    ty = GetDoubleSubscriber(name, "ty");
    txnc = GetDoubleSubscriber(name, "txnc");
    tync = GetDoubleSubscriber(name, "tync");
    ta = GetDoubleSubscriber(name, "ta");
    tl = GetDoubleSubscriber(name, "tl");
    cl = GetDoubleSubscriber(name, "cl");
    t2d = GetDoubleArraySubscriber(name, "t2d");
    getpipe = GetIntegerSubscriber(name, "getpipe");
    getpipetype = GetStringSubscriber(name, "getpipetype"); 
    json = GetStringSubscriber(name, "json");
    tclass = GetStringSubscriber(name, "tclass"); 
    tc = GetDoubleArraySubscriber(name, "tc");
    hb = GetDoubleSubscriber(name, "hb");
    hw = GetDoubleArraySubscriber(name, "hw");
    crosshairs = GetDoubleArraySubscriber(name, "crosshair");
    tcclass = GetStringSubscriber(name, "tcclass");
    tdclass = GetStringSubscriber(name, "tdclass");
    botpose = GetDoubleArraySubscriber(name, "botpose"); 
    botpose_wpiblue = GetDoubleArraySubscriber(name, "botpose_wpiblue");
    botpose_wpired = GetDoubleArraySubscriber(name, "botpose_wpired");
    botpose_orb = GetDoubleArraySubscriber(name, "botpose_orb");
    botpose_orb_wpiblue = GetDoubleArraySubscriber(name, "botpose_orb_wpiblue"); 
    botpose_orb_wpired = GetDoubleArraySubscriber(name, "botpose_orb_wpired");
    camerapose_targetspace = GetDoubleArraySubscriber(name, "camerapose_targetspace");
    targetpose_cameraspace = GetDoubleArraySubscriber(name, "targetpose_cameraspace");
    targetpose_robotspace = GetDoubleArraySubscriber(name, "targetpose_robotspace");
    botpose_targetspace = GetDoubleArraySubscriber(name, "botpose_targetspace");
    camerapose_robotspace = GetDoubleArraySubscriber(name, "camerapose_robotspace");
    tid = GetIntegerSubscriber(name, "tid");
    stddevs = GetDoubleArraySubscriber(name, "stddevs");
    camerapose_robotspace_set = GetDoubleArrayPublisher(name, "camerapose_robotspace_set");
    priorityid = GetIntegerPublisher(name, "priorityid");
    robot_orientation_set = GetDoubleArrayPublisher(name, "robot_orientation_set");
    fiducial_id_filters_set = GetIntegerArrayPublisher(name, "fiducial_id_filters_set");
    fiducial_offset_set = GetDoubleArrayPublisher(name, "fiducial_offset_set"); 
    ledMode = GetIntegerPublisher(name, "ledMode");
    pipeline = GetIntegerPublisher(name, "pipeline");
    stream = GetIntegerPublisher(name, "stream");
    crop = GetDoubleArrayPublisher(name, "crop");
    rawfiducials = GetDoubleArraySubscriber(name, "rawfiducials");
    rawdetections = GetDoubleArraySubscriber(name, "rawdetections");
    distanceFromTarget = GetDoublePublisher(name, "distance_from_target");
}

Limelight::Limelight()
{
    Limelight("");
}

PoseEstimate Limelight::GetPose(units::degree_t yaw, units::degrees_per_second_t yawRate)
{
    // Sets the robot angle and angular velocity
    SetRobotOrientation(yaw, yawRate);
    // Gets the estimated position
    return GetBotPoseMegatag2Blue();
}

units::meter_t Limelight::GetDistanceFromTarget()
{
    std::vector<double> targetPoseRobotSpace = GetTargetPoseRobotSpace();
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
    distanceFromTarget.Set(GetDistanceFromTarget().value());
}