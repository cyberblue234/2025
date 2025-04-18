#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/area.h>
#include <units/capacitance.h>
#include <units/charge.h>
#include <units/concentration.h>
#include <units/conductance.h>
#include <units/current.h>
#include <units/curvature.h>
#include <units/data.h>
#include <units/data_transfer_rate.h>
#include <units/density.h>
#include <units/dimensionless.h>
#include <units/energy.h>
#include <units/force.h>
#include <units/frequency.h>
#include <units/illuminance.h>
#include <units/impedance.h>
#include <units/inductance.h>
#include <units/length.h>
#include <units/luminous_flux.h>
#include <units/luminous_intensity.h>
#include <units/magnetic_field_strength.h>
#include <units/magnetic_flux.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/power.h>
#include <units/pressure.h>
#include <units/radiation.h>
#include <units/solid_angle.h>
#include <units/substance.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/volume.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/config/ModuleConfig.h>
#include <pathplanner/lib/config/RobotConfig.h>

#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/IntegerTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <ctre/phoenix6/swerve/SwerveDrivetrainConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveModuleConstants.hpp>

#include <numbers>
#include <string>
#include <array>
#include <vector>
#include <math.h>

/// @brief Contains the IDs of all the CAN devices. They all have device types to make sure no conflicts of IDs
namespace RobotMap
{
    /// @brief The Drivetrain IDs
    namespace Drivetrain
    {
        // TalonFX
        const int kFrontLeftDriveID = 1;
        // TalonFX
        const int kFrontLeftTurnID = 2;
        // CANCoder
        const int kFrontLeftCanCoderID = 1;

        // TalonFX
        const int kFrontRightDriveID = 3;
        // TalonFX
        const int kFrontRightTurnID = 4;
        // CANCoder
        const int kFrontRightCanCoderID = 2;

        // TalonFX
        const int kBackLeftDriveID = 5;
        // TalonFX
        const int kBackLeftTurnID = 6;
        // CANCoder
        const int kBackLeftCanCoderID = 3;

        // TalonFX
        const int kBackRightDriveID = 7;
        // TalonFX
        const int kBackRightTurnID = 8;
        // CANCoder
        const int kBackRightCanCoderID = 4;

        // Pigeon2
        const int kGyroID = 1;
    }

    /// @brief The Elevator IDs
    namespace Elevator
    {
        //TalonFX
        const int kMotor1ID = 9;
        //TalonFX
        const int kMotor2ID = 10;
        //DIO
        const int kBottomLimitSwitchID = 2;
        //DIO
        const int kTopLimitSwitchID = 3;
    }

    namespace Claw
    {
        //TalonFX
        const int kWristMotorID = 11;
        //SparkFlex
        const int kIOMotorID = 12;
        //CANcoder
        const int kCanCoderID = 5;

        //CANrange
        const int kCanRangeID = 1;
    }

    namespace Climber
    {
        //SparkMax
        const int kClimbMotorID = 2;
        //DIO
        const int kLimitSwitch = 1;
    }
    namespace Pneumatics
    {
        //RevPH
        const int kPneumaticHubID = 1;
        //Pneumatic Hub Slot
        const int kStopperSlot = 0;
    }
}

namespace ControlsConstants
{
    // Analog inputs
    const int kManualWristAxis = 2;
    const int kManualElevatorAxis = 1;
    const int kClimberAxis = 3;
    const int kManualIntakeAxis = 0;

    const int kL1Button = 5;
    const int kL2Button = 3;
    const int kL3Button = 1;
    const int kL4Button = 6;
    const int kAlgaeHighButton = 8;
    const int kAlgaeLowButton = 11;
    const int kCoralStationButton = 4;
    const int kProcessorButton = 7;
    const int kBargeButton = 2;
    const int kCoralHomeButton = 9;
    const int kAlgaeHomeButton = 12;

    const int kIOButton = 10;
}

/// @brief Personal add-on to the WPILib units library. See https://docs.wpilib.org/en/stable/docs/software/basic-programming/cpp-units.html for details on it.
namespace units
{
    using meters_per_turn = compound_unit<meter, inverse<turn>>;
    using meters_per_turn_t = unit_t<meters_per_turn>;
    using radians_per_turn = compound_unit<radian, inverse<turn>>;
    using radians_per_turn_t = unit_t<radians_per_turn>;
    using degrees_per_turn = compound_unit<degree, inverse<turn>>;
    using degrees_per_turn_t = unit_t<degrees_per_turn>;

    using kv_meters_t = unit_t<frc::SimpleMotorFeedforward<meters>::kv_unit>;
    using ka_meters_t = unit_t<frc::SimpleMotorFeedforward<meters>::ka_unit>;

    using kv_degrees_t = unit_t<frc::SimpleMotorFeedforward<degrees>::kv_unit>;
    using ka_degrees_t = unit_t<frc::SimpleMotorFeedforward<degrees>::ka_unit>;
}

namespace RobotConstants
{
    const units::meter_t kRobotWidth = 28_in;
    const units::meter_t kRobotLength = 31_in;
    const units::meter_t kBumperWidth = 3_in;
    const units::meter_t kRobotWidthWithBumper = kRobotWidth + kBumperWidth * 2;
    const units::meter_t kRobotLengthWithBumper = kRobotLength + kBumperWidth * 2;

    const units::kilogram_t kBatteryMass = 12.89_lb;
    const units::kilogram_t kBumperMass = 16.6_lb;
    const units::kilogram_t kMass = 115_lb + kBumperMass + kBatteryMass; 
    const units::kilogram_square_meter_t kMOI = (1 / 12) * kMass * (kRobotWidthWithBumper * kRobotWidthWithBumper + kRobotLengthWithBumper * kRobotLengthWithBumper);
}

/// @brief Constants for the Drivetrain class
namespace DrivetrainConstants
{
    // The locations of the swerve modules in reference to the center of the robot
    // A common error is to use an incorrect coordinate system where the positive Y axis points forward on the robot. 
    // The correct coordinate system has the positive X axis pointing forward.
    const frc::Translation2d kFrontLeftLocation{+12_in, +10.375_in};
    const frc::Translation2d kFrontRightLocation{+12_in, -10.375_in};
    const frc::Translation2d kBackLeftLocation{-12_in, +10.375_in};
    const frc::Translation2d kBackRightLocation{-12_in, -10.375_in};

    // Maximum desired speed of the robot. Does not have to be maximum theoretical speed if that is too high for desired driving speed
    const units::meters_per_second_t kMaxSpeed = 4.74_mps;
    // Maximum desired angular speed of the robot. Does not have to be maximum theoretical speed if that is too high for desired driving speed
    const units::radians_per_second_t kMaxAngularSpeed = std::numbers::pi * 4_rad_per_s;

    // Offsets for the CANcoders to ensure 0 is pointing forward
    const units::turn_t kFrontLeftMagnetOffset  = -0.69067_tr;
    const units::turn_t kFrontRightMagnetOffset = 0.68213_tr;
    const units::turn_t kBackLeftMagnetOffset   = -0.052002_tr;
    const units::turn_t kBackRightMagnetOffset  = -0.12573_tr;

    const units::meter_t kDeltaReefAprilTagToBranch = 6.5_in;
}

/// @brief Constants for the SwerveModule class
namespace SwerveModuleConstants
{   
    // Gearing between the drive motor and wheel in turns - how many turns of the drive motor does it take to drive the wheel one full revolution
    const units::turn_t kDriveGearRatio = 6.39_tr;
    // Gearing between the turn motor and wheel in turns - how many turns of the turn motor does it take to turn the wheel one full revolution
    const units::turn_t kTurnGearRatio = 12.1_tr;
    // Radius of the wheel
    const units::meter_t kWheelRadius = 2_in;
    // Width of the wheel
    const units::meter_t kWheelWidth = 2_in;
    // MOI of the drive motor - 1/2 * m * r^2
    const units::kilogram_square_meter_t kDriveMOI{0.01};
    // MOI of the turn motor - 1/12 * m * w^2
    const units::kilogram_square_meter_t kTurnMOI{0.025};
    // The amount of meters the robot drives per turn of the drive motor. The circumference of the wheel is the distance the robot drives for one full revolution of the wheel. Dividing by the gear ratio gets you to meters per one turn of the drive motor
    const units::meters_per_turn_t kDriveDistanceRatio = kWheelRadius * 2 * std::numbers::pi / kDriveGearRatio;
    // Because we use the CANcoder to determine turning distance, the ratio is just one revolution of the wheel (360° or 2π) for every turn
    const units::radians_per_turn_t kTurnDistanceRatio = 2_rad * std::numbers::pi / 1_tr;

    const double kWheelCOF = 1.0;

    // PIDs and feedforward constants of the drive motor
    namespace Drive
    {
        const double kP = 0.0;
        const double kI = 0.0;
        const double kD = 0.0;
        const units::volt_t kS{0.24};
        const units::kv_meters_t kV{2.3 * kDriveDistanceRatio.value()}; // values from https://www.reca.lc/drive?appliedVoltageRamp=%7B%22s%22%3A1200%2C%22u%22%3A%22V%2Fs%22%7D&batteryAmpHours=%7B%22s%22%3A18%2C%22u%22%3A%22A%2Ah%22%7D&batteryResistance=%7B%22s%22%3A0.015%2C%22u%22%3A%22Ohm%22%7D&batteryVoltageAtRest=%7B%22s%22%3A12.5%2C%22u%22%3A%22V%22%7D&efficiency=97&filtering=1&gearRatioMax=%7B%22magnitude%22%3A15%2C%22ratioType%22%3A%22Reduction%22%7D&gearRatioMin=%7B%22magnitude%22%3A3%2C%22ratioType%22%3A%22Reduction%22%7D&maxSimulationTime=%7B%22s%22%3A4%2C%22u%22%3A%22s%22%7D&maxSpeedAccelerationThreshold=%7B%22s%22%3A0.15%2C%22u%22%3A%22ft%2Fs2%22%7D&motor=%7B%22quantity%22%3A4%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&motorCurrentLimit=%7B%22s%22%3A120%2C%22u%22%3A%22A%22%7D&numCyclesPerMatch=24&peakBatteryDischarge=20&ratio=%7B%22magnitude%22%3A6.54%2C%22ratioType%22%3A%22Reduction%22%7D&sprintDistance=%7B%22s%22%3A21%2C%22u%22%3A%22ft%22%7D&swerve=1&targetTimeToGoal=%7B%22s%22%3A2%2C%22u%22%3A%22s%22%7D&throttleResponseMax=0.99&throttleResponseMin=0.5&weightAuxilliary=%7B%22s%22%3A23%2C%22u%22%3A%22lbs%22%7D&weightDistributionFrontBack=0.5&weightDistributionLeftRight=0.5&weightInspected=%7B%22s%22%3A105%2C%22u%22%3A%22lbs%22%7D&wheelBaseLength=%7B%22s%22%3A27%2C%22u%22%3A%22in%22%7D&wheelBaseWidth=%7B%22s%22%3A20%2C%22u%22%3A%22in%22%7D&wheelCOFDynamic=0.9&wheelCOFLateral=1.1&wheelCOFStatic=1&wheelDiameter=%7B%22s%22%3A4%2C%22u%22%3A%22in%22%7D
        const units::ka_meters_t kA{0.20 * kDriveDistanceRatio.value()};

        const units::ampere_t kCurrentLimit = 100_A;
    }
    // PIDs of the turn motor
    namespace Turn
    {
        const double kP = 30.0;
        const double kI = 0.0;
        const double kD = 0.5;

        const units::ampere_t kCurrentLimit = 30_A;
    }
    
    inline pathplanner::ModuleConfig moduleConfigs{kWheelRadius, DrivetrainConstants::kMaxSpeed, kWheelCOF, frc::DCMotor::KrakenX60(1), kDriveGearRatio.value(), Drive::kCurrentLimit, 1};
}

/// @brief Constants for PathPlanner
namespace PathPlannerConstants
{
    // PIDs for the translation component of PathPlanner
    namespace Translation
    {
        const double kP = 8.0;
        const double kI = 0.0;
        const double kD = 0.1;
    }
    // PIDs for the rotation component of PathPlanner
    namespace Rotation
    {
        const double kP = 5.0;
        const double kI = 0.0;
        const double kD = 0.1;
    }
    
    inline pathplanner::RobotConfig kConfig{RobotConstants::kMass, RobotConstants::kMOI, SwerveModuleConstants::moduleConfigs, std::vector<frc::Translation2d>{DrivetrainConstants::kFrontLeftLocation, DrivetrainConstants::kFrontRightLocation, DrivetrainConstants::kBackLeftLocation, DrivetrainConstants::kBackRightLocation}};
    
    const pathplanner::PathConstraints kReefPathfindingConstraints{0.75_mps, 0.75_mps_sq, 240_deg_per_s, 240_deg_per_s_sq};
    const pathplanner::PathConstraints kCoralStationPathfindingConstraints{1_mps, 0.5_mps_sq, 360_deg_per_s, 360_deg_per_s_sq};
    const pathplanner::PathConstraints kProcessorPathfindingConstraints{1_mps, 0.5_mps_sq, 360_deg_per_s, 360_deg_per_s_sq};
}

/// @brief Constants for the Elevator Class
namespace ElevatorConstants
{
    // PIDs and feedforward values
    const double kP = 40.0;
    const double kI = 0.0;
    const double kD = 0.0;
    const units::volt_t kS{0.0};
    const units::volt_t kG{0.1};
    const units::kv_meters_t kV{9.0};
    const units::ka_meters_t kA{0.3};
    const frc::TrapezoidProfile<units::meters>::Constraints kUpTrapezoidProfileContraints{2.7_mps, 2.7_mps_sq};
    const frc::TrapezoidProfile<units::meters>::Constraints kDownTrapezoidProfileContraints{2.5_mps, 2.5_mps_sq};
    
    // The gearing between the motor and the sprocket
    const units::turn_t kMotorGearing = 20_tr;
    // Theoretical diameter of the sprocket connected to the chain which raises the first stage
    const units::meter_t kSprocketPitchDiameter = 1.751_in;
    // How many meters the carriage travels per rotation of the motors.
    // Note, carriage raises at a rate of 2:1 compared to the first stage
    const units::meters_per_turn_t kMetersPerMotorTurn = 2 * (kSprocketPitchDiameter * std::numbers::pi) / kMotorGearing;

    // The distance from the ground to the bottom of the carriage
    const units::meter_t kHeightOffset      = 6.5_in;
    // Should be equal to kMaxEncoderValue * kMetersPerMotorTurn + kHeightOffset
    const units::meter_t kMaxElevatorHeight = 4.5_ft + kHeightOffset;
    // Maximum encoder count - should be slightly lower than the maximum possible encoder count
    const units::turn_t kMaxEncoderValue    = (kMaxElevatorHeight - kHeightOffset) / kMetersPerMotorTurn;

    const units::meter_t kTolerance = 0.5_in;

    const double kElevatorPower = 0.25;
}

namespace ClawConstants
{
    const double kP = 0.18;
    const double kI = 0.0;
    const double kD = 0.0;
    const units::volt_t kS{0.1595};
    const units::volt_t kG{0.0};
    const units::kv_degrees_t kV{0.07};
    const units::ka_degrees_t kA{0.0};
    const frc::TrapezoidProfile<units::degrees>::Constraints kTrapezoidProfileContraints{200_deg_per_s, 180_deg_per_s_sq};

    // Intake and output powers for coral and algae
    const double kCoralIntakePower = -0.35;
    const double kAlgaeIntakePower = 0.5;
    // Outputting should be negative compared to intaking
    const double kCoralOutputPower = 0.5;
    const double kProcessorPower = -0.3;
    const double kBargePower = -1.0;
    const double kManualIOPower = 0.2;

    const units::turn_t kCanCoderMagnetOffset = 0.1582_tr;

    const units::degree_t kTolerance = 2.0_deg;

    const units::degree_t kLowLimit = -10.0_deg;
    const units::degree_t kHighLimit = 180_deg;

    const units::turn_t kWristGearRatio = 233.45_tr;
    const units::degrees_per_turn_t kDegreesPerMotorTurn = 360_deg / kWristGearRatio;

    const double kWristPower = 0.2;
}

namespace ClimberConstants
{
    const double kClimberPower = 0.4;
}

namespace LimelightConstants
{
    // Forward, Right, Up, Roll, Pitch, Yaw
    const frc::Pose3d kHighOffset{6.75_in, 12_in, 38.125_in, frc::Rotation3d{0_deg, 0_deg, -2.29_deg}};
    const frc::Pose3d kLowOffset{11.25_in, -0.125_in, 7.625_in, frc::Rotation3d{0_deg, 20_deg, 0_deg}};

    const wpi::array<double, 3> autonStdDevs{1.8, 1.8, 1.8};
    const wpi::array<double, 3> teleopStdDevs{0.9, 0.9, 0.9};
}

/// @brief Struct for the different possible positions
struct Position
{

    /// @brief The height of the elevator
    const units::meter_t height;
    /// @brief The angle of the claw
    const units::degree_t angle;
    /// @brief What power to set to the IO motor
    const double ioMotorPower;
    /// @brief Set to true when intaking coral - will be used to stop the IO motor when we have a coral in the claw
    const bool isForCoralIntake = false;

    const Position operator=(const Position &rhs)
    {
        return {rhs.height, rhs.angle, rhs.ioMotorPower, rhs.isForCoralIntake};
    }
    bool operator==(const Position &rhs)
    {
        return this->height == rhs.height && this->angle == rhs.angle && this->ioMotorPower == rhs.ioMotorPower && this->isForCoralIntake == rhs.isForCoralIntake;
    }
    std::string to_string()
    {
        return "Height: " + units::to_string(height.convert<units::feet>()) + "; Angle: " + units::to_string(angle) + "; IO Power: " + std::to_string(ioMotorPower);
    }
};
namespace Positions
{    
    const Position L1           = Position(1.6_ft, 150.0_deg,  ClawConstants::kCoralOutputPower, false);
    const Position L2           = Position(1.1_ft, 13.5_deg,  -ClawConstants::kCoralOutputPower, false);
    const Position L3           = Position(2.5_ft, 13.5_deg,  -ClawConstants::kCoralOutputPower, false);
    const Position L4           = Position(4.55_ft, 13.5_deg, -ClawConstants::kCoralOutputPower, false);
    const Position AlgaeLow     = Position(2.2_ft, 170.0_deg,  ClawConstants::kAlgaeIntakePower, false);
    const Position AlgaeHigh    = Position(3.4_ft, 170.0_deg,  ClawConstants::kAlgaeIntakePower, false);
    const Position CoralStation = Position(1.915_ft, 110.0_deg,  ClawConstants::kCoralIntakePower, true);
    const Position Processor    = Position(ElevatorConstants::kHeightOffset, 160.0_deg,  ClawConstants::kProcessorPower, false);
    const Position Barge        = Position(ElevatorConstants::kMaxElevatorHeight, 63.0_deg,  ClawConstants::kBargePower, false);
    const Position CoralHome    = Position(ElevatorConstants::kHeightOffset, 15.0_deg,  ClawConstants::kCoralIntakePower, false);
    const Position AlgaeHome    = Position(ElevatorConstants::kHeightOffset, 75.0_deg,  0.0, false);
}

/// @brief Clamps the input to a specifed range
/// @param val Value to clamp
/// @param low Lower bound
/// @param high Higher bound
/// @retval val if within range
/// @retval low if below range
/// @retval high if above range
template <typename T>
static T clamp(T val, T low, T high) 
{
    return val > low && val < high ? val : val <= low ? low : high; 
}

/// @brief Returns the sign of the input
/// @param val Input to determine sign of
/// @retval 0 if val == 0
/// @retval -1 if val < 0
/// @retval 1 if val > 0
template <typename T>
inline static T sgn(T val)
{
    return val == T{0} ? T{0} : val > T{0} ? T{1} : T{-1};
}

/// @brief Returns the value at the index
/// @param inData Array
/// @param position Index
/// @retval Default value of the type if position is not within bounds of the array
/// @retval Value at position
template <typename T>
inline static T ExtractArrayEntry(const std::vector<T>& inData, int position) 
{
    if (inData.size() < static_cast<size_t>(position + 1)) {
        return T{};
    }
    return inData[position];
}

inline static units::meter_t DistanceBetweenPoses(const frc::Pose2d& pose1, const frc::Pose2d& pose2)
{
    // Distance formula: d = sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
    return units::meter_t{std::pow(std::pow(pose1.X().value() - pose2.X().value(), 2) + std::pow(pose1.Y().value() - pose2.Y().value(), 2), 0.5)};
}