#pragma once

#include "LimelightHelpers.h"
#include "Constants.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

using namespace LimelightHelpers;

class Limelight
{
public:
    // Constructors
    Limelight(std::string name);
    Limelight();
    void SetAprilTagPipeline(int id) { setPipelineIndex(name, id); };
    double GetTargetID() { return getFiducialID(); };
    PoseEstimate GetBotPoseBlue(units::degree_t yaw, units::degrees_per_second_t yawRate);
    double GetDistanceFromTarget();

private:
    std::string name;
};