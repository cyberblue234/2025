#pragma once

#include <wpinet/PortForwarder.h>
#include "wpi/json.h"
#include <chrono>
#include <iostream>
#include <cstring>
#include <fcntl.h>

#include "Constants.h"


class RawFiducial 
{
public:
    int id{0};
    double txnc{0.0};
    double tync{0.0};
    double ta{0.0};
    double distToCamera{0.0};
    double distToRobot{0.0};
    double ambiguity{0.0};

    RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity)
        : id(id), txnc(txnc), tync(tync), ta(ta), distToCamera(distToCamera), distToRobot(distToRobot), ambiguity(ambiguity) {}
};

class RawDetection 
{
public:
    int classId{-1};
    double txnc{0.0};
    double tync{0.0};
    double ta{0.0};
    double corner0_X{0.0};
    double corner0_Y{0.0};
    double corner1_X{0.0};
    double corner1_Y{0.0};
    double corner2_X{0.0};
    double corner2_Y{0.0};
    double corner3_X{0.0};
    double corner3_Y{0.0};

    RawDetection(int classId, double txnc, double tync, double ta, 
                double corner0_X, double corner0_Y, 
                double corner1_X, double corner1_Y, 
                double corner2_X, double corner2_Y, 
                double corner3_X, double corner3_Y)
        : classId(classId), txnc(txnc), tync(tync), ta(ta), 
        corner0_X(corner0_X), corner0_Y(corner0_Y), 
        corner1_X(corner1_X), corner1_Y(corner1_Y), 
        corner2_X(corner2_X), corner2_Y(corner2_Y), 
        corner3_X(corner3_X), corner3_Y(corner3_Y) {}
};

class PoseEstimate
{
public:
    frc::Pose2d pose;
    units::time::second_t timestampSeconds{0.0};
    double latency{0.0};
    int tagCount{0};
    double tagSpan{0.0};
    double avgTagDist{0.0};
    double avgTagArea{0.0};
    std::vector<RawFiducial> rawFiducials;

    PoseEstimate() = default;

    PoseEstimate(const frc::Pose2d& pose, units::time::second_t timestampSeconds, 
    double latency, int tagCount, double tagSpan, double avgTagDist, double avgTagArea,
    const std::vector<RawFiducial>& rawFiducials)
    : pose(pose), timestampSeconds(timestampSeconds), 
        latency(latency), tagCount(tagCount), tagSpan(tagSpan), 
        avgTagDist(avgTagDist), avgTagArea(avgTagArea), rawFiducials(rawFiducials) {}
};

class Limelight
{
public:
    /// @brief Constructor for the limelight object that sets the name of the limelight to the parameter
    /// @param name Name of the limelight
    Limelight(std::string name);
    /// @brief Constructor for the limelight object that sets the name of the limelight to ""
    Limelight();
    /// @brief Gets the estimated position of the robot with the blue origin as is FRC standard
    /// @param yaw Yaw of the robot in degrees
    /// @param yawRate Angular velocity in the yaw direction in degrees per second
    /// @return PoseEstimate based on MegaTag2 calcluation
    PoseEstimate GetPose(units::degree_t yaw, units::degrees_per_second_t yawRate);
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

    int GetTV() { return tv.Get(); }
    double GetTX() { return tx.Get(); }
    double GetTY() { return ty.Get(); }
    double GetTXNC() { return txnc.Get(); }
    double GetTYNC() { return tync.Get(); }
    double GetTA() { return ta.Get(); }
    double GetTL() { return tl.Get(); }
    double GetCL() { return cl.Get(); }
    std::vector<double> GetT2D() { return t2d.Get(); }
    int GetPipe() { return getpipe.Get(); }
    std::string GetPipeType() { return getpipetype.Get(); }
    std::string GetJSON() { return json.Get(); }
    std::string GetTClass() { return tclass.Get(); }
    std::vector<double> GetTC() { return tc.Get(); }
    double GetHB() { return hb.Get(); }
    std::vector<double> GetHW() { return hw.Get(); }
    std::vector<double> GetCrosshairs() { return crosshairs.Get(); }
    std::string GetTCClass() { return tcclass.Get(); }
    std::string GetTDClass() { return tdclass.Get(); }
    std::vector<double> GetCameraPoseTargetSpace() { return camerapose_targetspace.Get(); }
    std::vector<double> GetTargetPoseCameraSpace() { return targetpose_cameraspace.Get(); }
    std::vector<double> GetTargetPoseRobotSpace() { return targetpose_robotspace.Get(); }
    std::vector<double> GetBotPoseTargetSpace() { return botpose_targetspace.Get(); }
    std::vector<double> GetCameraPoseRobotSpace() { return camerapose_robotspace.Get(); }
    int GetTID() { return tid.Get(); }
    std::vector<double> GetStdDevs() { return stddevs.Get(); }

    PoseEstimate GetBotPose() { return GetBotPoseEstimate(botpose); }
    PoseEstimate GetBotPoseBlue() { return GetBotPoseEstimate(botpose_wpiblue); }
    PoseEstimate GetBotPoseRed() { return GetBotPoseEstimate(botpose_wpired); }
    PoseEstimate GetBotPoseMegatag2() { return GetBotPoseEstimate(botpose_orb); }
    PoseEstimate GetBotPoseMegatag2Blue() { return GetBotPoseEstimate(botpose_orb_wpiblue); }
    PoseEstimate GetBotPoseMegatag2Red() { return GetBotPoseEstimate(botpose_orb_wpired); }

    void SetCameraPoseRobotSpace(double forward, double side, double up, double roll, double pitch, double yaw)
    {
        std::vector<double> entries = {forward, side, up, roll, pitch, yaw};
        camerapose_robotspace_set.Set(entries);
    }
    void SetPriorityID(int id)
    {
        priorityid.Set(id);
    }
    void SetRobotOrientation(
        units::degree_t yaw, units::degrees_per_second_t yawRate, 
        units::degree_t pitch, units::degrees_per_second_t pitchRate, 
        units::degree_t roll, units::degrees_per_second_t rollRate)
    {
        std::vector<double> entries = {yaw.value(), yawRate.value(), pitch.value(), pitchRate.value(), roll.value(), rollRate.value()};
        robot_orientation_set.Set(entries);
    }
    void SetRobotOrientation(units::degree_t yaw, units::degrees_per_second_t yawRate)
    {
        SetRobotOrientation(yaw, yawRate, units::degree_t{0}, units::degrees_per_second_t{0}, units::degree_t{0}, units::degrees_per_second_t{0});
    }
    void SetFiducialIDFilters(const std::vector<int64_t>& validIDs)
    {
        fiducial_id_filters_set.Set(validIDs);
    }
    void SetFiducialOffset(double x, double y, double z)
    {
        std::vector<double> entries = {x, y, z};
        fiducial_offset_set.Set(entries);
    }
    enum LedMode { kPipeline = 0, kForceOff = 1, kForceBlink = 2, kForceOn = 3 };
    void SetLEDMode(LedMode m)
    {
        ledMode.Set(m);
    }
    enum Pipeline { kApriltag = 0 };
    void SetPipeline(Pipeline p)
    {
        pipeline.Set(p);
    }
    enum Stream { kStandard = 0, kPiPMain = 1, kPiPSecondary = 2 };
    void SetStream(Stream s)
    {
        stream.Set(s);
    }
    void SetCropWindow(double cropXMin, double cropXMax, double cropYMin, double cropYMax)
    {
        std::vector<double> entries = {cropXMin, cropXMax, cropYMin, cropYMax};
        crop.Set(entries);
    }

    std::string SanitizeName(const std::string &name)
    {
        if (name == "")
        {
            return "limelight";
        }
        return name;
    }

    frc::Pose3d ToPose3D(const std::vector<double>& inData)
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

    frc::Pose2d ToPose2D(const std::vector<double>& inData)
    {
        if(inData.size() < 6)
        {
            return frc::Pose2d();
        }
        return frc::Pose2d(
            frc::Translation2d(units::length::meter_t(inData[0]), units::length::meter_t(inData[1])), 
            frc::Rotation2d(units::angle::degree_t(inData[5])));
    }

    std::shared_ptr<nt::NetworkTable> GetTable(const std::string &tableName)
    {
        return nt::NetworkTableInstance::GetDefault().GetTable(SanitizeName(tableName));
    }
    nt::IntegerSubscriber GetIntegerSubscriber(const std::string &entryName)
    {
        return GetTable(SanitizeName(name))->GetIntegerTopic(entryName).Subscribe(0);
    }
    nt::IntegerArraySubscriber GetIntegerArraySubscriber(const std::string &entryName)
    {
        return GetTable(SanitizeName(name))->GetIntegerArrayTopic(entryName).Subscribe(std::span<int64_t>{});
    }
    nt::DoubleSubscriber GetDoubleSubscriber(const std::string &entryName)
    {
        return GetTable(SanitizeName(name))->GetDoubleTopic(entryName).Subscribe(0.0);
    }
    nt::DoubleArraySubscriber GetDoubleArraySubscriber(const std::string &entryName)
    {
        return GetTable(SanitizeName(name))->GetDoubleArrayTopic(entryName).Subscribe(std::span<double>{});
    }
    nt::StringSubscriber GetStringSubscriber(const std::string &entryName)
    {
        return GetTable(SanitizeName(name))->GetStringTopic(entryName).Subscribe("");
    }
    nt::IntegerPublisher GetIntegerPublisher(const std::string &entryName)
    {
        return GetTable(SanitizeName(name))->GetIntegerTopic(entryName).Publish();
    }
    nt::IntegerArrayPublisher GetIntegerArrayPublisher(const std::string &entryName)
    {
        return GetTable(SanitizeName(name))->GetIntegerArrayTopic(entryName).Publish();
    }
    nt::DoublePublisher GetDoublePublisher(const std::string &entryName)
    {
        return GetTable(SanitizeName(name))->GetDoubleTopic(entryName).Publish();
    }
    nt::DoubleArrayPublisher GetDoubleArrayPublisher(const std::string &entryName)
    {
        return GetTable(SanitizeName(name))->GetDoubleArrayTopic(entryName).Publish();
    }

    void SetupPortForwarding() 
    {
        auto& portForwarder = wpi::PortForwarder::GetInstance();
        portForwarder.Add(5800, SanitizeName(name), 5800);
        portForwarder.Add(5801, SanitizeName(name), 5801);
        portForwarder.Add(5802, SanitizeName(name), 5802);
        portForwarder.Add(5803, SanitizeName(name), 5803);
        portForwarder.Add(5804, SanitizeName(name), 5804);
        portForwarder.Add(5805, SanitizeName(name), 5805);
        portForwarder.Add(5806, SanitizeName(name), 5806);
        portForwarder.Add(5807, SanitizeName(name), 5807);
        portForwarder.Add(5808, SanitizeName(name), 5808);
        portForwarder.Add(5809, SanitizeName(name), 5809);
    }

    std::vector<RawFiducial> GetRawFiducials() 
    {
        std::vector<double> rawFiducialArray = rawfiducials.Get();
        int valsPerEntry = 7;
        if (rawFiducialArray.size() % valsPerEntry != 0) {
            return {};
        }

        int numFiducials = rawFiducialArray.size() / valsPerEntry;
        std::vector<RawFiducial> rawFiducials;

        for (int i = 0; i < numFiducials; ++i) {
            int baseIndex = i * valsPerEntry;
            int id = static_cast<int>(ExtractArrayEntry(rawFiducialArray, baseIndex));
            double txnc = ExtractArrayEntry(rawFiducialArray, baseIndex + 1);
            double tync = ExtractArrayEntry(rawFiducialArray, baseIndex + 2);
            double ta = ExtractArrayEntry(rawFiducialArray, baseIndex + 3);
            double distToCamera = ExtractArrayEntry(rawFiducialArray, baseIndex + 4);
            double distToRobot = ExtractArrayEntry(rawFiducialArray, baseIndex + 5);
            double ambiguity = ExtractArrayEntry(rawFiducialArray, baseIndex + 6);

            rawFiducials.emplace_back(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
        }

        return rawFiducials;
    }

    std::vector<RawDetection> GetRawDetections() 
    {
        std::vector<double> rawDetectionArray = rawdetections.Get();
        int valsPerEntry = 11;

        if (rawDetectionArray.size() % valsPerEntry != 0) {
            return {};
        }

        int numDetections = rawDetectionArray.size() / valsPerEntry;
        std::vector<RawDetection> rawDetections;

        for (int i = 0; i < numDetections; ++i) {
            int baseIndex = i * valsPerEntry;
            int classId = static_cast<int>(ExtractArrayEntry(rawDetectionArray, baseIndex));
            double txnc = ExtractArrayEntry(rawDetectionArray, baseIndex + 1);
            double tync = ExtractArrayEntry(rawDetectionArray, baseIndex + 2);
            double ta = ExtractArrayEntry(rawDetectionArray, baseIndex + 3);
            double corner0_X = ExtractArrayEntry(rawDetectionArray, baseIndex + 4);
            double corner0_Y = ExtractArrayEntry(rawDetectionArray, baseIndex + 5);
            double corner1_X = ExtractArrayEntry(rawDetectionArray, baseIndex + 6);
            double corner1_Y = ExtractArrayEntry(rawDetectionArray, baseIndex + 7);
            double corner2_X = ExtractArrayEntry(rawDetectionArray, baseIndex + 8);
            double corner2_Y = ExtractArrayEntry(rawDetectionArray, baseIndex + 9);
            double corner3_X = ExtractArrayEntry(rawDetectionArray, baseIndex + 10);
            double corner3_Y = ExtractArrayEntry(rawDetectionArray, baseIndex + 11);

            rawDetections.emplace_back(classId, txnc, tync, ta, corner0_X, corner0_Y, corner1_X, corner1_Y, corner2_X, corner2_Y, corner3_X, corner3_Y);
        }

        return rawDetections;
    }

    inline PoseEstimate GetBotPoseEstimate(const nt::DoubleArraySubscriber &sub) {
        std::vector<double> poseArray = sub.Get();
        frc::Pose2d pose = ToPose2D(poseArray);

        double latency = ExtractArrayEntry(poseArray, 6);
        int tagCount = static_cast<int>(ExtractArrayEntry(poseArray, 7));
        double tagSpan = ExtractArrayEntry(poseArray, 8);
        double tagDist = ExtractArrayEntry(poseArray, 9);
        double tagArea = ExtractArrayEntry(poseArray, 10);

        // getLastChange: microseconds; latency: milliseconds
        units::time::second_t timestamp = units::time::second_t((sub.GetLastChange() / 1000000.0) - (latency / 1000.0));

        std::vector<RawFiducial> rawFiducials;
        int valsPerFiducial = 7;
        unsigned int expectedTotalVals = 11 + valsPerFiducial * tagCount;
        
        if (poseArray.size() == expectedTotalVals) 
        {
            for (int i = 0; i < tagCount; i++) 
            {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = static_cast<int>(ExtractArrayEntry(poseArray, baseIndex));
                double txnc = ExtractArrayEntry(poseArray, baseIndex + 1);
                double tync = ExtractArrayEntry(poseArray, baseIndex + 2);
                double ta = ExtractArrayEntry(poseArray, baseIndex + 3);
                double distToCamera = ExtractArrayEntry(poseArray, baseIndex + 4);
                double distToRobot = ExtractArrayEntry(poseArray, baseIndex + 5);
                double ambiguity = ExtractArrayEntry(poseArray, baseIndex + 6);
                rawFiducials.emplace_back(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials);
    }

private:
    std::string name;


    // Basic Targeting Data https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#basic-targeting-data

    nt::IntegerSubscriber tv; // 1 if valid target exists. 0 if no valid targets exist
    nt::DoubleSubscriber tx; // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
    nt::DoubleSubscriber ty; // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
    nt::DoubleSubscriber txnc; // Horizontal Offset From Principal Pixel To Target (degrees)
    nt::DoubleSubscriber tync; // Vertical Offset From Principal Pixel To Target (degrees)
    nt::DoubleSubscriber ta; // Target Area (0% of image to 100% of image)
    nt::DoubleSubscriber tl; // The pipeline's latency contribution (ms). Add to "cl" to get total latency.
    nt::DoubleSubscriber cl; // Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
    nt::DoubleArraySubscriber t2d; // Array containing several values for matched-timestamp statistics: [targetValid, targetCount, targetLatency, captureLatency, tx, ty, txnc, tync, ta, tid, targetClassIndexDetector , targetClassIndexClassifier, targetLongSidePixels, targetShortSidePixels, targetHorizontalExtentPixels, targetVerticalExtentPixels, targetSkewDegrees]
    nt::IntegerSubscriber getpipe; // True active pipeline index of the camera (0 .. 9)
    nt::StringSubscriber getpipetype; // Pipeline Type e.g. "pipe_color"
    nt::StringSubscriber json; // Full JSON dump of targeting results. Must be enabled per-pipeline in the 'output' tab
    nt::StringSubscriber tclass; // Class name of primary neural detector result or neural classifier result
    nt::DoubleArraySubscriber tc; // Get the average HSV color underneath the crosshair region (3x3 pixel region) as a NumberArray
    nt::DoubleSubscriber hb; // Heartbeat value. Increases once per frame, resets at 2 billion
    nt::DoubleArraySubscriber hw; // HW metrics [fps, cpu temp, ram usage, temp]
    nt::DoubleArraySubscriber crosshairs; //2D Crosshairs [cx0, cy0, cx1, cy1]
    nt::StringSubscriber tcclass; // Name of classifier pipeline's computed class
    nt::StringSubscriber tdclass; // Name of detector pipeline's primary detection


    // AprilTag and 3D Data https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data

    nt::DoubleArraySubscriber botpose; // Robot transform in field-space. Translation (X,Y,Z) in meters Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
    nt::DoubleArraySubscriber botpose_wpiblue; // Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) in meters Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
    nt::DoubleArraySubscriber botpose_wpired; // Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) in meters, Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
    nt::DoubleArraySubscriber botpose_orb; // Robot transform in field-space (Megatag2). Translation (X,Y,Z) in meters Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
    nt::DoubleArraySubscriber botpose_orb_wpiblue; // Robot transform in field-space (Megatag2) (blue driverstation WPILIB origin). Translation (X,Y,Z) in meters Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
    nt::DoubleArraySubscriber botpose_orb_wpired; // Robot transform in field-space (Megatag2) (red driverstation WPILIB origin). Translation (X,Y,Z) in meters, Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
    nt::DoubleArraySubscriber camerapose_targetspace; // 3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
    nt::DoubleArraySubscriber targetpose_cameraspace; // 3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
    nt::DoubleArraySubscriber targetpose_robotspace; // 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
    nt::DoubleArraySubscriber botpose_targetspace; // 3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
    nt::DoubleArraySubscriber camerapose_robotspace; // 3D transform of the camera in the coordinate system of the robot (array (6))
    nt::IntegerSubscriber tid; // ID of the primary in-view AprilTag
    nt::DoubleArraySubscriber stddevs; // MegaTag Standard Deviations [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]

    nt::DoubleArrayPublisher camerapose_robotspace_set; // SET the camera's pose in the coordinate system of the robot.
    nt::IntegerPublisher priorityid; // SET the required ID for tx/ty targeting. Ignore other targets. Does not affect localization
    nt::DoubleArrayPublisher robot_orientation_set; // SET Robot Orientation and angular velocities in degrees and degrees per second[yaw,yawrate,pitch,pitchrate,roll,rollrate]
    nt::IntegerArrayPublisher fiducial_id_filters_set; // SET Override valid fiducial ids for localization (array)
    nt::DoubleArrayPublisher fiducial_offset_set; // SET the 3D Point of Interest Offset [x,y,z]


    // Camera Controls https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#camera-controls

    /* Sets limelight's LED state
    *  [0] use the LED Mode set in the current pipeline
    *  [1] force off
    *  [2] force blink
    *  [3] force on
    */
    nt::IntegerPublisher ledMode;
    /* Sets limelight's current pipeline
    *  [0..9] Select pipeline [0..9]
    */
    nt::IntegerPublisher pipeline;
    /* Sets limelight's streaming mode
    *  [0] Standard - Side-by-side streams if a webcam is attached to Limelight
    *  [1] PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
    *  [2] PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
    */
    nt::IntegerPublisher stream;
    /* Sets the crop rectangle. The pipeline must utilize the default crop rectangle in the web interface. The array must have exactly 4 entries.
    *  [0] X0 - Min or Max X value of crop rectangle (-1 to 1)
    *  [1] X1 - Min or Max X value of crop rectangle (-1 to 1)
    *  [2] Y0 - Min or Max Y value of crop rectangle (-1 to 1)
    *  [3] Y1 - Min or Max Y value of crop rectangle (-1 to 1)
    */
    nt::DoubleArrayPublisher crop;


    // Raw Data https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#raw-data

    /* Raw Fiducials:
    *  Get all valid (unfiltered) fiducials
    *  [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity, id2.....]
    */
    nt::DoubleArraySubscriber rawfiducials;
    /* Raw Detections:
    *  Get all valid (unfiltered) nerual detection results
    *  [id, txnc, tync, ta, corner0x, corner0y, corner1x, corner1y, corner2x, corner2y, corner3x, corner3y, id2.....]
    */
    nt::DoubleArraySubscriber rawdetections;
};