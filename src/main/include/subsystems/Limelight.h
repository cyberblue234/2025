#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include "LimelightHelpers.h"

#include "Constants.h"

using namespace LimelightHelpers;

class Limelight
{
public:
    /// @brief Constructor for the limelight object that sets the name of the limelight to the parameter
    /// @param name Name of the limelight
    Limelight(std::string name);
    /// @brief Constructor for the limelight object that sets the name of the limelight to ""
    Limelight();
    /// @brief Sets the pipeline for the limelight
    /// @param id Id of the pipeline
    void SetAprilTagPipeline(int id) { setPipelineIndex(name, id); };
    double GetTargetID() { return getFiducialID(); };
    PoseEstimate GetBotPoseBlue(units::degree_t yaw, units::degrees_per_second_t yawRate);
    double GetDistanceFromTarget();
    void UpdateTelemetry();
    /// @brief Quick SmartDashboard helper tool for printing diagnostics
    /// @param valueName Description of the value
    /// @param value Value to be printed
    void TelemetryHelperNumber(std::string valueName, double value) { frc::SmartDashboard::PutNumber(valueName + " " + name, value); }

    

private:
    std::string name;
};