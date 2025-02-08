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
    /// @param id ID of the pipeline
    void SetAprilTagPipeline(int id) { setPipelineIndex(name, id); };
    /// @brief Gets the ID of the currently targeted april tag
    /// @return ID of the april tag
    double GetTargetID() { return getFiducialID(); };
    /// @brief Gets the estimated position of the robot with the blue origin as is FRC standard
    /// @param yaw Yaw of the robot in degrees
    /// @param yawRate Angular velocity in the yaw direction in degrees per second
    /// @return PoseEstimate based on MegaTag2 calcluation
    PoseEstimate GetBotPoseBlue(units::degree_t yaw, units::degrees_per_second_t yawRate);
    /// @brief Gets the distance from the targeted april tag
    /// @retval distance in meters
    /// @retval -1 if no april tags are in sight
    units::meter_t GetDistanceFromTarget();
    /// @brief Updates telemtry for the limelight
    void UpdateTelemetry();
    /// @brief Quick SmartDashboard helper tool for printing diagnostics
    /// @param valueName Description of the value
    /// @param value Value to be printed
    void TelemetryHelperNumber(std::string valueName, double value) { frc::SmartDashboard::PutNumber(valueName + " " + name, value); }

    

private:
    std::string name;
};