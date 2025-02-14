#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StringTopic.h>
#include "networktables/NetworkTableValue.h"
#include <wpinet/PortForwarder.h>
#include "wpi/json.h"
#include <chrono>
#include <iostream>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <cstring>
#include <fcntl.h>

#include "Constants.h"

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

    inline std::string SanitizeName(const std::string &name)
    {
        if (name == "")
        {
            return "limelight";
        }
        return name;
    }

    inline frc::Pose3d ToPose3D(const std::vector<double>& inData)
    {
        if(inData.size() < 6)
        {
            return frc::Pose3d();
        }
        return frc::Pose3d(
            frc::Translation3d(units::length::meter_t(inData[0]), units::length::meter_t(inData[1]), units::length::meter_t(inData[2])),
            frc::Rotation3d(units::angle::degree_t(inData[3]), units::angle::degree_t(inData[4]),
                   units::angle::degree_t(inData[5])));
    }

    inline frc::Pose2d ToPose2D(const std::vector<double>& inData)
    {
        if(inData.size() < 6)
        {
            return frc::Pose2d();
        }
        return frc::Pose2d(
            frc::Translation2d(units::length::meter_t(inData[0]), units::length::meter_t(inData[1])), 
            frc::Rotation2d(units::angle::degree_t(inData[5])));
    }

protected:
    inline std::shared_ptr<nt::NetworkTable> GetTable(const std::string &tableName)
    {
        return nt::NetworkTableInstance::GetDefault().GetTable(sanitizeName(tableName));
    }

    inline nt::DoubleSubscriber GetDoubleSub(const std::string &tableName, const std::string &entryName)
    {
        return GetTable(tableName)->GetDoubleTopic(entryName).Subscribe(0.0);
    }

    inline nt::DoubleArraySubscriber GetDoubleArraySub(const std::string &tableName, const std::string &entryName)
    {
        return GetTable(tableName)->GetDoubleArrayTopic(entryName).Subscribe(std::span<double>{});
    }

    inline nt::StringSubscriber GetStringSub(const std::string &tableName, const std::string &entryName)
    {
        return GetTable(tableName)->GetStringTopic(entryName).Subscribe("");
    }

    inline nt::DoublePublisher GetDoublePub(const std::string &tableName, const std::string &entryName)
    {
        return GetTable(tableName)->GetDoubleTopic(entryName).Publish();
    }
    
    inline nt::DoubleArrayPublisher GetDoublePub(const std::string &entryName)
    {
        return GetTable(sanitizeName(name))->GetDoubleArrayTopic(entryName).Publish();
    }

    inline void SetupPortForwarding() 
    {
        auto& portForwarder = wpi::PortForwarder::GetInstance();
        portForwarder.Add(5800, sanitizeName(name), 5800);
        portForwarder.Add(5801, sanitizeName(name), 5801);
        portForwarder.Add(5802, sanitizeName(name), 5802);
        portForwarder.Add(5803, sanitizeName(name), 5803);
        portForwarder.Add(5804, sanitizeName(name), 5804);
        portForwarder.Add(5805, sanitizeName(name), 5805);
        portForwarder.Add(5806, sanitizeName(name), 5806);
        portForwarder.Add(5807, sanitizeName(name), 5807);
        portForwarder.Add(5808, sanitizeName(name), 5808);
        portForwarder.Add(5809, sanitizeName(name), 5809);
    }

private:
    std::string name;
    nt::DoubleSubscriber tx;
};