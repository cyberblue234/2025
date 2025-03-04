#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

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
    const frc::SwerveModuleState GetState() { return {GetVelocity(), GetAngle()}; }
    /// @brief Returns the position of the module. Position is drive distance and turn angle
    /// @return SwerveModulePosition
    const frc::SwerveModulePosition GetPosition() { return {GetDistance(), GetAngle()}; }
    /// @brief Sets the desired states of the module. State is drive velocity and turn angle
    /// @param state Reference to a new SwerveModuleState
    void SetDesiredState(frc::SwerveModuleState &state);

    /// @brief Updates the SmartDashboard with the module information
    void UpdateTelemetry();
    /// @brief Quick SmartDashboard helper tool for printing diagnostics
    /// @param valueName Description of the value
    /// @param value Value to be printed
    void TelemetryHelperNumber(std::string valueName, double value) { frc::SmartDashboard::PutNumber(valueName + " " + name, value); }
    double TelemetryHelperGetNumber(std::string valueName, double defaultVal) { return frc::SmartDashboard::GetNumber(valueName + " " + name, defaultVal); }

    /// @brief Runs a simulation of the swerve module based on the inputs of SetDesiredState()
    void SimMode();

    /// @brief Returns the distance of the drive motor
    /// @return Distance in meters
    const units::meter_t GetDistance() { return driveMotor.GetPosition().GetValue() * kDriveDistanceRatio; }
    /// @brief Returns the velocity of the drive motor
    /// @return Velocity in meters per second
    const units::meters_per_second_t GetVelocity() { return driveMotor.GetRotorVelocity().GetValue() * kDriveDistanceRatio; }
    /// @brief Returns the angle of the module
    /// @return Rotation2d of the angle; domain: [0, 2π), [0°, 360°)
    const frc::Rotation2d GetAngle() { return frc::Rotation2d{canCoder.GetAbsolutePosition().GetValue() * kTurnDistanceRatio}; }
    /// @brief Returns the position of the CANcoder
    /// @return units::turn_t CANcoder position 
    const units::turn_t GetCANcoderPosition() { return canCoder.GetPosition().GetValue(); }
    /// @brief Returns the absolute position of the CANcoder
    /// @return units::turn_t CANcoder position 
    const units::turn_t GetAbsoluteCANcoderPosition() { return canCoder.GetAbsolutePosition().GetValue(); }

    /// @brief Returns the supply voltage of the drive motor
    /// @return Supply voltage
    const units::volt_t GetDriveSupplyVoltage() { return driveMotor.GetSupplyVoltage().GetValue(); }
    /// @brief Returns the supply voltage of the turn motor
    /// @return Supply voltage
    const units::volt_t GetTurnSupplyVoltage() { return turnMotor.GetSupplyVoltage().GetValue(); }
    /// @brief Returns the output voltage of the drive motor
    /// @return Output (applied) voltage
    const units::volt_t GetDriveOutputVoltage() { return driveMotor.GetMotorVoltage().GetValue(); }
    /// @brief Returns the output voltage of the turn motor
    /// @return Output (applied) voltage
    const units::volt_t GetTurnOutputVoltage() { return turnMotor.GetMotorVoltage().GetValue(); }
    /// @brief Returns the torque current of the drive motor
    /// @return Torque current
    const units::ampere_t GetDriveTorqueCurrent() { return driveMotor.GetTorqueCurrent().GetValue(); }
    /// @brief Returns the toque current of the turn motor
    /// @return Torque current
    const units::ampere_t GetTurnTorqueCurrent() { return turnMotor.GetTorqueCurrent().GetValue(); }
    /// @brief Returns the stator current of the drive motor
    /// @return Stator current
    const units::ampere_t GetDriveStatorCurrent() { return driveMotor.GetStatorCurrent().GetValue(); }
    /// @brief Returns the stator current of the turn motor
    /// @return Stator current
    const units::ampere_t GetTurnStatorCurrent() { return turnMotor.GetStatorCurrent().GetValue(); }
    /// @brief Returns the supply current of the drive motor
    /// @return Supply current
    const units::ampere_t GetDriveSupplyCurrent() { return driveMotor.GetSupplyCurrent().GetValue(); }
    /// @brief Returns the supply current of the turn motor
    /// @return Supply current
    const units::ampere_t GetTurnSupplyCurrent() { return turnMotor.GetSupplyCurrent().GetValue(); }
    /// @brief Returns the temperature of the drive motor
    /// @return Temperature (°C)
    const units::celsius_t GetDriveTemp() { return driveMotor.GetDeviceTemp().GetValue(); }
    /// @brief Returns the temperature of the turn motor
    /// @return Temperature (°C)
    const units::celsius_t GetTurnTemp() { return turnMotor.GetDeviceTemp().GetValue(); }
    /// @brief Returns the temperature of the drive motor controller
    /// @return Temperature (°C)
    const units::celsius_t GetDriveProcessorTemp() { return driveMotor.GetProcessorTemp().GetValue(); }
    /// @brief Returns the temperature of the turn motor controller
    /// @return Temperature (°C)
    const units::celsius_t GetTurnProcessorTemp() { return turnMotor.GetProcessorTemp().GetValue(); }

    /// @brief Returns the drive motor object
    /// @return Pointer to TalonFX
    const hardware::TalonFX *GetDriveMotor() { return &driveMotor; }
    /// @brief Returns the turn motor object
    /// @return Pointer to TalonFX
    const hardware::TalonFX *GetTurnMotor() { return &turnMotor; }
    /// @brief Returns the canCoder object
    /// @return Pointer to CANcoder
    const hardware::CANcoder *GetCANcoder() { return &canCoder; }
    
    /// @brief Sets the raw encoder position of the drive motor
    /// @param value new raw position
    void SetEncoder(units::turn_t value) { driveMotor.SetPosition(value); }
    /// @brief Sets the raw encoder position of the CANcoder
    /// @param value new raw position
    void SetCanCoder(units::turn_t value) { canCoder.SetPosition(value); }

private:
    // The name of the swerve module, helps with telemetry
    std::string name;

    // Creates the drive and turn motor and the cancoder objects
    hardware::TalonFX driveMotor;
    hardware::TalonFX turnMotor;
    hardware::CANcoder canCoder;

    /*
     * CTRE uses classes from the controls namespace to control the motors in more complex manners.
     * In this case, we are using PositionVoltage and VelocityVoltage for the 
     * turn/angle motor and the drive motor respectfully.
     * PositionVoltage allows us to run the turn/angle motor to a position using voltages.
     * VelocityVoltage allows us to run the drive motor at a velocity using voltages.
     */
    controls::VelocityVoltage driveVelocityOut{0_tps};

    frc::ProfiledPIDController<units::degrees> turnController{Turn::kP, Turn::kI, Turn::kD, Turn::kTrapezoidProfileContraints};
    frc::SimpleMotorFeedforward<units::degrees> turnFeedforward{Turn::kS, Turn::kV, Turn::kA};

    // Wheel mass is about 3.6lbs
    // Drive MOI is 1/2mr^2 = 7.2 lbs in^2
    // Turn MOI is 1/12ml^2 = 1.2 lbs in^2
    // Creates a simluation tool for the drive motor
    /// @todo Think about doing kV and kA
    frc::sim::DCMotorSim driveMotorSimModel{
        frc::LinearSystemId::DCMotorSystem(
            frc::DCMotor::KrakenX60(1),
            1 / 2 * kWheelMass * kWheelRadius * kWheelRadius,
            kDriveGearRatio.value()
        ),
        frc::DCMotor::KrakenX60(1)
    };
    frc::sim::DCMotorSim turnMotorSimModel{
        frc::LinearSystemId::DCMotorSystem(
            frc::DCMotor::KrakenX60(1),
            1 / 12 * kWheelMass * kWheelWidth * kWheelWidth,
            kTurnGearRatio.value()
        ),
        frc::DCMotor::KrakenX60(1)
    };
};

/// @todo Look into StatusSignal objects. Must call .Refresh() every cycle to use
/*
/// @brief Returns the distance of the drive motor
    /// @return Distance in meters
    const units::meter_t GetDistance() { return driveMotorPosition.GetValue() * kDriveDistanceRatio; };
    /// @brief Returns the velocity of the drive motor
    /// @return Velocity in meters per second
    const units::meters_per_second_t GetVelocity() { return driveMotorVelocity.GetValue() * kDriveDistanceRatio; };
    /// @brief Returns the angle of the module
    /// @return Rotation2d of the angle; domain: [0, 2π), [0°, 360°)
    const frc::Rotation2d GetAngle() { return frc::Rotation2d{canCoderAbsolutePosition.GetValue() * kTurnDistanceRatio}; };
    /// @brief Returns the position of the CANcoder
    /// @return units::turn_t CANcoder position 
    const units::turn_t GetCANcoderPosition() { return canCoderPosition.GetValue(); };
    /// @brief Returns the absolute position of the CANcoder
    /// @return units::turn_t CANcoder position 
    const units::turn_t GetAbsoluteCANcoderPosition() { return canCoderAbsolutePosition.GetValue(); };

    /// @brief Returns the supply voltage of the drive motor
    /// @return Supply voltage
    const units::volt_t GetDriveSupplyVoltage() { return driveMotorSupplyVoltage.GetValue(); };
    /// @brief Returns the supply voltage of the turn motor
    /// @return Supply voltage
    const units::volt_t GetTurnSupplyVoltage() { return turnMotorSupplyVoltage.GetValue(); };
    /// @brief Returns the output voltage of the drive motor
    /// @return Output (applied) voltage
    const units::volt_t GetDriveOutputVoltage() { return driveMotorOutputVoltage.GetValue(); };
    /// @brief Returns the output voltage of the turn motor
    /// @return Output (applied) voltage
    const units::volt_t GetTurnOutputVoltage() { return turnMotorOutputVoltage.GetValue(); };
    /// @brief Returns the torque current of the drive motor
    /// @return Torque current
    const units::ampere_t GetDriveTorqueCurrent() { return driveMotorTorqueCurrent.GetValue(); };
    /// @brief Returns the toque current of the turn motor
    /// @return Torque current
    const units::ampere_t GetTurnTorqueCurrent() { return turnMotorTorqueCurrent.GetValue(); };
    /// @brief Returns the stator current of the drive motor
    /// @return Stator current
    const units::ampere_t GetDriveStatorCurrent() { return driveMotorStatorCurrent.GetValue(); };
    /// @brief Returns the stator current of the turn motor
    /// @return Stator current
    const units::ampere_t GetTurnStatorCurrent() { return turnMotorStatorCurrent.GetValue(); };
    /// @brief Returns the supply current of the drive motor
    /// @return Supply current
    const units::ampere_t GetDriveSupplyCurrent() { return driveMotorSupplyCurrent.GetValue(); };
    /// @brief Returns the supply current of the turn motor
    /// @return Supply current
    const units::ampere_t GetTurnSupplyCurrent() { return turnMotorSupplyCurrent.GetValue(); };
    /// @brief Returns the temperature of the drive motor
    /// @return Temperature (°C)
    const units::celsius_t GetDriveTemp() { return driveMotorTemp.GetValue(); };
    /// @brief Returns the temperature of the turn motor
    /// @return Temperature (°C)
    const units::celsius_t GetTurnTemp() { return turnMotorTemp.GetValue(); };
    /// @brief Returns the temperature of the drive motor controller
    /// @return Temperature (°C)
    const units::celsius_t GetDriveProcessorTemp() { return driveMotorProcessorTemp.GetValue(); };
    /// @brief Returns the temperature of the turn motor controller
    /// @return Temperature (°C)
    const units::celsius_t GetTurnProcessorTemp() { return turnMotorProcessorTemp.GetValue(); };

    /// @brief Returns the drive motor object
    /// @return Pointer to TalonFX
    const hardware::TalonFX *GetDriveMotor() { return &driveMotor; };
    /// @brief Returns the turn motor object
    /// @return Pointer to TalonFX
    const hardware::TalonFX *GetTurnMotor() { return &turnMotor; };
    /// @brief Returns the canCoder object
    /// @return Pointer to CANcoder
    const hardware::CANcoder *GetCANcoder() { return &canCoder; };
    
    /// @brief Sets the raw encoder position of the drive motor
    /// @param value new raw position
    void SetEncoder(units::turn_t value) { driveMotor.SetPosition(value); };
    /// @brief Sets the raw encoder position of the CANcoder
    /// @param value new raw position
    void SetCanCoder(units::turn_t value) { canCoder.SetPosition(value); }

private:
    // The name of the swerve module, helps with telemetry
    std::string name;

    // Creates the drive and turn motor and the cancoder objects and all of the StatusSignal objects that correspond
    hardware::TalonFX driveMotor;
    StatusSignal<units::turn_t>             driveMotorPosition = driveMotor.GetPosition();
    StatusSignal<units::turns_per_second_t> driveMotorVelocity = driveMotor.GetRotorVelocity();
    StatusSignal<units::volt_t>    driveMotorSupplyVoltage = driveMotor.GetSupplyVoltage();
    StatusSignal<units::volt_t>    driveMotorOutputVoltage = driveMotor.GetMotorVoltage();
    StatusSignal<units::ampere_t>  driveMotorTorqueCurrent = driveMotor.GetTorqueCurrent();
    StatusSignal<units::ampere_t>  driveMotorStatorCurrent = driveMotor.GetStatorCurrent();
    StatusSignal<units::ampere_t>  driveMotorSupplyCurrent = driveMotor.GetSupplyCurrent();
    StatusSignal<units::celsius_t> driveMotorTemp          = driveMotor.GetDeviceTemp();
    StatusSignal<units::celsius_t> driveMotorProcessorTemp = driveMotor.GetProcessorTemp();

    hardware::TalonFX turnMotor;
    StatusSignal<units::volt_t>    turnMotorSupplyVoltage = turnMotor.GetSupplyVoltage();
    StatusSignal<units::volt_t>    turnMotorOutputVoltage = turnMotor.GetMotorVoltage();
    StatusSignal<units::ampere_t>  turnMotorTorqueCurrent = turnMotor.GetTorqueCurrent();
    StatusSignal<units::ampere_t>  turnMotorStatorCurrent = turnMotor.GetStatorCurrent();
    StatusSignal<units::ampere_t>  turnMotorSupplyCurrent = turnMotor.GetSupplyCurrent();
    StatusSignal<units::celsius_t> turnMotorTemp          = turnMotor.GetDeviceTemp();
    StatusSignal<units::celsius_t> turnMotorProcessorTemp = turnMotor.GetProcessorTemp();

    hardware::CANcoder canCoder;
    StatusSignal<units::turn_t> canCoderAbsolutePosition  = canCoder.GetAbsolutePosition();
    StatusSignal<units::turn_t> canCoderPosition          = canCoder.GetPosition();
*/