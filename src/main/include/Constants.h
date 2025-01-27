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

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <numbers>
#include <string>
#include <array>
#include <vector>
#include <math.h>

namespace RobotMap
{
    // TalonFX
    constexpr int kFrontLeftDriveID = 8;
    // TalonFX
    constexpr int kFrontLeftTurnID = 9;
    // CANCoder
    constexpr int kFrontLeftCanCoderID = 21;

    // TalonFX
    constexpr int kFrontRightDriveID = 2;
    // TalonFX
    constexpr int kFrontRightTurnID = 3;
    // CANCoder
    constexpr int kFrontRightCanCoderID = 22;

    // TalonFX
    constexpr int kBackLeftDriveID = 7;
    // TalonFX
    constexpr int kBackLeftTurnID = 6;
    // CANCoder
    constexpr int kBackLeftCanCoderID = 23;

    // TalonFX
    constexpr int kBackRightDriveID = 4;
    // TalonFX
    constexpr int kBackRightTurnID = 5;
    // CANCoder
    constexpr int kBackRightCanCoderID = 24;
}

namespace units
{
    using meters_per_turn = compound_unit<meter, inverse<turn>>;
    using meters_per_turn_t = unit_t<meters_per_turn>;
    using radians_per_turn = compound_unit<radian, inverse<turn>>;
    using radians_per_turn_t = unit_t<radians_per_turn>;

    using volts_per_tps_t = unit_t<frc::SimpleMotorFeedforward<turns>::kv_unit>;
    using volts_per_tps_sq_t = unit_t<frc::SimpleMotorFeedforward<turns>::ka_unit>;
}

namespace SwerveModuleConstants
{   
    inline constexpr units::turn_t kDriveGearRatio = 6.54_tr;
    inline constexpr units::turn_t kTurnGearRatio = 11.31_tr;
    inline constexpr units::meter_t kWheelRadius = 0.0491_m;
    inline constexpr units::meters_per_turn_t kDriveDistanceRatio = kWheelRadius * 2 * std::numbers::pi / kDriveGearRatio;
    inline constexpr units::radians_per_turn_t kTurnDistanceRatio = 2_rad * std::numbers::pi / 1_tr;

    inline constexpr double kDriveP = 0.0;
    inline constexpr double kDriveI = 0.0;
    inline constexpr double kDriveD = 0.0;
    inline constexpr units::volt_t kDrive_kS = 0.24_V;
    inline constexpr units::volts_per_tps_t kDrive_kV = 2.46_V / 1_mps * kDriveDistanceRatio; // values from https://www.reca.lc/drive?appliedVoltageRamp=%7B%22s%22%3A1200%2C%22u%22%3A%22V%2Fs%22%7D&batteryAmpHours=%7B%22s%22%3A18%2C%22u%22%3A%22A%2Ah%22%7D&batteryResistance=%7B%22s%22%3A0.015%2C%22u%22%3A%22Ohm%22%7D&batteryVoltageAtRest=%7B%22s%22%3A12.5%2C%22u%22%3A%22V%22%7D&efficiency=97&filtering=1&gearRatioMax=%7B%22magnitude%22%3A15%2C%22ratioType%22%3A%22Reduction%22%7D&gearRatioMin=%7B%22magnitude%22%3A3%2C%22ratioType%22%3A%22Reduction%22%7D&maxSimulationTime=%7B%22s%22%3A4%2C%22u%22%3A%22s%22%7D&maxSpeedAccelerationThreshold=%7B%22s%22%3A0.15%2C%22u%22%3A%22ft%2Fs2%22%7D&motor=%7B%22quantity%22%3A4%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&motorCurrentLimit=%7B%22s%22%3A120%2C%22u%22%3A%22A%22%7D&numCyclesPerMatch=24&peakBatteryDischarge=20&ratio=%7B%22magnitude%22%3A6.54%2C%22ratioType%22%3A%22Reduction%22%7D&sprintDistance=%7B%22s%22%3A21%2C%22u%22%3A%22ft%22%7D&swerve=1&targetTimeToGoal=%7B%22s%22%3A2%2C%22u%22%3A%22s%22%7D&throttleResponseMax=0.99&throttleResponseMin=0.5&weightAuxilliary=%7B%22s%22%3A23%2C%22u%22%3A%22lbs%22%7D&weightDistributionFrontBack=0.5&weightDistributionLeftRight=0.5&weightInspected=%7B%22s%22%3A105%2C%22u%22%3A%22lbs%22%7D&wheelBaseLength=%7B%22s%22%3A27%2C%22u%22%3A%22in%22%7D&wheelBaseWidth=%7B%22s%22%3A20%2C%22u%22%3A%22in%22%7D&wheelCOFDynamic=0.9&wheelCOFLateral=1.1&wheelCOFStatic=1&wheelDiameter=%7B%22s%22%3A4%2C%22u%22%3A%22in%22%7D
    inline constexpr units::volts_per_tps_sq_t kDrive_kA = 0.20_V / 1_mps_sq * kDriveDistanceRatio;

    inline constexpr double kTurnP = 15.0;
    inline constexpr double kTurnI = 0.0;
    inline constexpr double kTurnD = 0.5;
}

namespace DrivetrainConstants
{
    inline constexpr frc::Translation2d kFrontLeftLocation{+0.2254_m, +0.2699_m};
    inline constexpr frc::Translation2d kFrontRightLocation{+0.2254_m, -0.2699_m};
    inline constexpr frc::Translation2d kBackLeftLocation{-0.3016_m, +0.2699_m};
    inline constexpr frc::Translation2d kBackRightLocation{-0.3016_m, -0.2699_m};

    inline constexpr units::meters_per_second_t kMaxSpeed = 4.74_mps;
    inline constexpr units::radians_per_second_t kMaxAngularSpeed = std::numbers::pi * 4_rad_per_s;

    // R1 Chassis
    inline constexpr units::turn_t kFrontLeftMagnetOffset  = -0.547_tr;
    inline constexpr units::turn_t kFrontRightMagnetOffset = -0.846_tr;
    inline constexpr units::turn_t kBackLeftMagnetOffset   = -0.023_tr;
    inline constexpr units::turn_t kBackRightMagnetOffset  = -0.245_tr;
}

namespace PathPlannerConstants
{
    inline constexpr double kTranslationP = 5;
    inline constexpr double kTranslationI = 0.0;
    inline constexpr double kTranslationD = 0.1;
    inline constexpr double kRotationP = 5;
    inline constexpr double kRotationI = 0.0;
    inline constexpr double kRotationD = 0.1;
}

template <typename T>
constexpr T clamp(T val, T low, T high) 
{
    // if (val > low && val < high)
    // {
    //     return val;
    // }
    // else if (val <= low)
    // {
    //     return low;
    // }
    // else return high;
    return val > low && val < high ? val : val <= low ? low : high; 
};