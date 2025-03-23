#include "subsystems/Limelight.h"

Limelight::Limelight(std::string name)
{
    this->name = name;
    SetupPortForwarding();

    tv = GetIntegerSubscriber("tv"); 
    tx = GetDoubleSubscriber("tx");
    ty = GetDoubleSubscriber("ty");
    txnc = GetDoubleSubscriber("txnc");
    tync = GetDoubleSubscriber("tync");
    ta = GetDoubleSubscriber("ta");
    tl = GetDoubleSubscriber("tl");
    cl = GetDoubleSubscriber("cl");
    t2d = GetDoubleArraySubscriber("t2d");
    getpipe = GetIntegerSubscriber("getpipe");
    getpipetype = GetStringSubscriber("getpipetype"); 
    json = GetStringSubscriber("json");
    tclass = GetStringSubscriber("tclass"); 
    tc = GetDoubleArraySubscriber("tc");
    hb = GetDoubleSubscriber("hb");
    hw = GetDoubleArraySubscriber("hw");
    crosshairs = GetDoubleArraySubscriber("crosshair");
    tcclass = GetStringSubscriber("tcclass");
    tdclass = GetStringSubscriber("tdclass");
    botpose = GetDoubleArraySubscriber("botpose"); 
    botpose_wpiblue = GetDoubleArraySubscriber("botpose_wpiblue");
    botpose_wpired = GetDoubleArraySubscriber("botpose_wpired");
    botpose_orb = GetDoubleArraySubscriber("botpose_orb");
    botpose_orb_wpiblue = GetDoubleArraySubscriber("botpose_orb_wpiblue"); 
    botpose_orb_wpired = GetDoubleArraySubscriber("botpose_orb_wpired");
    camerapose_targetspace = GetDoubleArraySubscriber("camerapose_targetspace");
    targetpose_cameraspace = GetDoubleArraySubscriber("targetpose_cameraspace");
    targetpose_robotspace = GetDoubleArraySubscriber("targetpose_robotspace");
    botpose_targetspace = GetDoubleArraySubscriber("botpose_targetspace");
    camerapose_robotspace = GetDoubleArraySubscriber("camerapose_robotspace");
    tid = GetIntegerSubscriber("tid");
    stddevs = GetDoubleArraySubscriber("stddevs");
    camerapose_robotspace_set = GetDoubleArrayPublisher("camerapose_robotspace_set");
    priorityid = GetIntegerPublisher("priorityid");
    robot_orientation_set = GetDoubleArrayPublisher("robot_orientation_set");
    fiducial_id_filters_set = GetIntegerArrayPublisher("fiducial_id_filters_set");
    fiducial_offset_set = GetDoubleArrayPublisher("fiducial_offset_set"); 
    ledMode = GetIntegerPublisher("ledMode");
    pipeline = GetIntegerPublisher("pipeline");
    stream = GetIntegerPublisher("stream");
    crop = GetDoubleArrayPublisher("crop");
    rawfiducials = GetDoubleArraySubscriber("rawfiducials");
    rawdetections = GetDoubleArraySubscriber("rawdetections");
}

Limelight::Limelight()
{
    Limelight("");
}

PoseEstimate Limelight::GetPose(units::degree_t yaw, units::degrees_per_second_t yawRate)
{
    // Sets the robot angle and angular velocity
    SetRobotOrientation(yaw, yawRate);
    // auto alliance = frc::DriverStation::GetAlliance();
    // if (alliance) {
    //     if (alliance == frc::DriverStation::Alliance::kBlue) return GetBotPoseMegatag2Red();
    // }
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
    TelemetryHelperNumber("Distance from Apriltag", GetDistanceFromTarget().value());
}