#pragma once

#ifndef _LIMELIGHT_H
#define _LIMELIGHT_H

#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <numbers>

class Limelight
{
public:
    // Constructors
    Limelight(std::string name);
    Limelight();

    enum LEDMode
    {
        kLEDPipline,
        kLEDOff,
        kLEDBlink,
        kLEDOn
    };
    enum CamMode
    {
        kVisonProcessor,
        kDriverCamera
    };
    enum PipelineID
    {
        kAprilTag,
        kSpeakerDetection,
        kAmpDetection,
        kStageDetection,
        kNoteDetection
    };

    void UpdateLimelightTracking();
    // double GetTargetX() { return target_x; };
    // double GetTargetY() { return target_y; };
    // double GetTargetArea() { return target_area; };
    double GetAprilTagID() { return april_tag_id; };
    // double GetTargetValid() { return target_valid; };   // 0 for no, 1 for yes
    // double GetActivePipeline() { return active_pipe; }; // Returns the active pipeline id
    // double GetTotalLatency() { return pipeline_latency + capture_latency; }
    // void SetLEDMode(LEDMode);
    // void SetCamMode(CamMode);
    // void SetPipelineID(PipelineID);
    double GetDistanceFromTarget();
    // double GetAprilTagOffset();
    frc::Pose2d GetRobotPose();
    void UpdateTelemetry();

private:
    std::string limelight_name;
    double target_x;
    double target_y;
    double target_area;
    double target_valid;
    double active_pipe;
    double april_tag_id;
    double pipeline_latency;
    double capture_latency;

    std::vector<double> botpose;
    std::vector<double> botpose_blue;
    std::vector<double> botpose_red;

    std::vector<double> targetpose_robotspace;
};

#endif