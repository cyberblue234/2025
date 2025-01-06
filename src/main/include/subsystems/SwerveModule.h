#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <frc/geometry/Rotation2d.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/SendableRegistry.h>

#include <numbers>
#include <string>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/EncoderSim.h>

#include "Constants.h"

using namespace ctre::phoenix6;
using namespace SwerveModuleConstants;

class SwerveModule
{
public:
    /// @brief Constructs the swerve module
    /// @param name Swerve module name 
    /// @param driveMotorID Can bus ID for the drive motor
    /// @param turnMotorID Can bus ID for the turn motor
    /// @param canCoderID Can bus ID for the CANcoder
    /// @param canCoderMagnetOffset Magnet offset for the CANcoder; ensures CANcoder is at 0 when facing straight ahead
    SwerveModule(std::string name, int driveMotorID, int turnMotorID, int canCoderID, units::turn_t canCoderMagnetOffset);

    /// @brief Returns the state of the module. State is drive velocity and turn angle
    /// @return SwerveModuleState
    frc::SwerveModuleState GetState() { return {GetVelocity(), GetAngle()}; };
    /// @brief Returns the position of the module. Position is drive distance and turn angle
    /// @return SwerveModulePosition
    frc::SwerveModulePosition GetPosition() { return {GetDistance(), GetAngle()}; };
    /// @brief Sets the desired states of the module. State is drive velocity and turn angle
    /// @param state Reference to a new SwerveModuleState
    void SetDesiredState(const frc::SwerveModuleState &state);

    /// @brief Updates the SmartDashboard with the module information
    void UpdateTelemetry();
    void TelemetryHelperNumber(std::string valueName, double value) { frc::SmartDashboard::PutNumber(valueName + " " + name, value); }
    /// @brief Simulates the module
    /// @param angle Angle to set the CANcoder at for the simulation
    void SimMode();

    /// @brief Returns the distance of the drive motor
    /// @return Distance in meters
    units::meter_t GetDistance() { return units::meter_t(driveMotor.GetPosition().GetValueAsDouble() * kDriveDistanceRatio); };
    /// @brief Returns the velocity of the drive motor
    /// @return Velocity in meters per second
    units::meters_per_second_t GetVelocity() { return units::meters_per_second_t(driveMotor.GetRotorVelocity().GetValueAsDouble() * kDriveDistanceRatio); };
    /// @brief Returns the angle of the module
    /// @return Rotation2d of the angle; domain: [0, 2π), [0°, 360°)
    frc::Rotation2d GetAngle() { return frc::Rotation2d{units::radian_t(canCoder.GetAbsolutePosition().GetValueAsDouble() * kTurnDistanceRatio)}; };
    /// @brief Returns the position of the CANcoder
    /// @return units::turn_t CANcoder position 
    units::angle::turn_t GetCANcoderPosition() { return canCoder.GetPosition().GetValue(); };
    /// @brief Returns the absolute position of the CANcoder
    /// @return units::turn_t CANcoder position 
    units::angle::turn_t GetAbsoluteCANcoderPosition() { return canCoder.GetAbsolutePosition().GetValue(); };

    /// @brief Returns the supply voltage of the drive motor
    /// @return Supply voltage
    units::voltage::volt_t GetDriveSupplyVoltage() { return driveMotor.GetSupplyVoltage().GetValue(); };
    /// @brief Returns the supply voltage of the turn motor
    /// @return Supply voltage
    units::voltage::volt_t GetTurnSupplyVoltage() { return turnMotor.GetSupplyVoltage().GetValue(); };
    /// @brief Returns the output voltage of the drive motor
    /// @return Output (applied) voltage
    units::voltage::volt_t GetDriveOutputVoltage() { return driveMotor.GetMotorVoltage().GetValue(); };
    /// @brief Returns the output voltage of the turn motor
    /// @return Output (applied) voltage
    units::voltage::volt_t GetTurnOutputVoltage() { return turnMotor.GetMotorVoltage().GetValue(); };
    /// @brief Returns the torque current of the drive motor
    /// @return Torque current
    units::current::ampere_t GetDriveTorqueCurrent() { return driveMotor.GetTorqueCurrent().GetValue(); };
    /// @brief Returns the toque current of the turn motor
    /// @return Torque current
    units::current::ampere_t GetTurnTorqueCurrent() { return turnMotor.GetTorqueCurrent().GetValue(); };
    /// @brief Returns the stator current of the drive motor
    /// @return Stator current
    units::current::ampere_t GetDriveStatorCurrent() { return driveMotor.GetStatorCurrent().GetValue(); };
    /// @brief Returns the stator current of the turn motor
    /// @return Stator current
    units::current::ampere_t GetTurnStatorCurrent() { return turnMotor.GetStatorCurrent().GetValue(); };
    /// @brief Returns the supply current of the drive motor
    /// @return Supply current
    units::current::ampere_t GetDriveSupplyCurrent() { return driveMotor.GetSupplyCurrent().GetValue(); };
    /// @brief Returns the supply current of the turn motor
    /// @return Supply current
    units::current::ampere_t GetTurnSupplyCurrent() { return turnMotor.GetSupplyCurrent().GetValue(); };
    /// @brief Returns the temperature of the drive motor
    /// @return Temperature (°C)
    units::temperature::celsius_t GetDriveTemp() { return driveMotor.GetDeviceTemp().GetValue(); };
    /// @brief Returns the temperature of the turn motor
    /// @return Temperature (°C)
    units::temperature::celsius_t GetTurnTemp() { return turnMotor.GetDeviceTemp().GetValue(); };
    /// @brief Returns the temperature of the drive motor controller
    /// @return Temperature (°C)
    units::temperature::celsius_t GetDriveProcessorTemp() { return driveMotor.GetProcessorTemp().GetValue(); };
    /// @brief Returns the temperature of the turn motor controller
    /// @return Temperature (°C)
    units::temperature::celsius_t GetTurnProcessorTemp() { return turnMotor.GetProcessorTemp().GetValue(); };

    /// @brief Returns the drive motor object
    /// @return Pointer to TalonFX
    hardware::TalonFX *GetDriveMotor() { return &driveMotor; };
    /// @brief Returns the turn motor object
    /// @return Pointer to TalonFX
    hardware::TalonFX *GetTurnMotor() { return &turnMotor; };
    /// @brief Returns the canCoder object
    /// @return Pointer to CANcoder
    hardware::CANcoder *GetCANcoder() { return &canCoder; };
    
    /// @brief Sets the raw encoder position of the drive motor
    /// @param value new raw position
    void SetEncoder(double value) { driveMotor.SetPosition(units::angle::turn_t(value)); };
    /// @brief Sets the raw encoder position of the CANcoder
    /// @param value new raw position
    void SetCanCoder(double value) { canCoder.SetPosition(units::angle::turn_t(value)); }

private:
    std::string name;

    hardware::TalonFX driveMotor;
    hardware::TalonFX turnMotor;
    hardware::CANcoder canCoder;

    controls::PositionVoltage turnPositionOut{0_tr};
};