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
#include <numbers>
#include <string>
#include <array>
#include <vector>

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

namespace SwerveModuleConstants
{
    inline constexpr double kDriveP = 0.50;
    inline constexpr double kDriveI = 0.0;
    inline constexpr double kDriveD = 0.1;
    inline constexpr auto kDrive_kS = 1_V;
    inline constexpr auto kDrive_kV = 0.5_V / 1_tps;

    inline constexpr double kTurnP = 15.0;
    inline constexpr double kTurnI = 0.0;
    inline constexpr double kTurnD = 0.5;

    inline constexpr double kDriveGearRatio = 6.54;
    inline constexpr double kTurnGearRatio = 11.31;
    inline constexpr double kWheelRadius = 0.0491;
    inline constexpr double kDriveDistanceRatio = kWheelRadius * 2 * std::numbers::pi / kDriveGearRatio;
    inline constexpr double kTurnDistanceRatio = 2 * std::numbers::pi;

    inline constexpr units::radians_per_second_t kModuleMaxAngularVelocity = std::numbers::pi * 4_rad_per_s;
    inline constexpr units::radians_per_second_squared_t kModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s;

    inline constexpr units::volt_t kVoltageComp = 11.5_V;
}

namespace DrivetrainConstants
{
    inline constexpr frc::Translation2d kFrontLeftLocation{+0.2254_m, +0.2699_m};
    inline constexpr frc::Translation2d kFrontRightLocation{+0.2254_m, -0.2699_m};
    inline constexpr frc::Translation2d kBackLeftLocation{-0.3016_m, +0.2699_m};
    inline constexpr frc::Translation2d kBackRightLocation{-0.3016_m, -0.2699_m};

    inline constexpr units::meters_per_second_t kMaxSpeed = 4.0_mps;
    inline constexpr units::radians_per_second_t kMaxAngularSpeed = std::numbers::pi * 2_rad_per_s;

    inline constexpr units::turn_t kFrontLeftMagnetOffset  = -0.547_tr; //-0.666016;
    inline constexpr units::turn_t kFrontRightMagnetOffset = -0.846_tr; //-0.78125;
    inline constexpr units::turn_t kBackLeftMagnetOffset   = -0.023_tr; //0.8852546;
    inline constexpr units::turn_t kBackRightMagnetOffset  = -0.245_tr; //-0.251953;

    inline constexpr double kDriveSlowAdjustment = 0.20;
}

template <typename T>
constexpr T clamp(T val, T low, T high) 
{
    return val > low && val < high ? val : val <= low ? low : high; 
};