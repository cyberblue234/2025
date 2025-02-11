#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <frc/DigitalInput.h>

#include "Constants.h"

using namespace ElevatorConstants;
using namespace ctre::phoenix6;

class Elevator
{
public:
    Elevator();

    void SetMotors(double power);
    void GoToTurns(units::turn_t);
    void GoToPosition(Positions pos);

    void UpdateElevator();
    const units::turn_t GetElevatorEncoder();
    void ResetElevatorEncoders();

    units::turn_t GetTurnsToPosition(Positions pos);

    void UpdateTelemtry();

    /// @brief Gets the state of the bottom limit switch
    /// @retval true if the limit switch is closed (pressed)
    /// @retval false if the limit switch is open
    bool IsBottomLimitSwitchClosed() { return !bottomLimitSwitch.Get() || simLimSwitch; };

    void SimMode();

private:

    hardware::TalonFX motor1{RobotMap::Elevator::kMotor1ID, "rio"};
    hardware::TalonFX motor2{RobotMap::Elevator::kMotor2ID, "rio"};

    frc::DigitalInput bottomLimitSwitch{RobotMap::Elevator::kBottomLimitSwitchID};
    bool simLimSwitch = false;

    bool isElevatorRegistered = false;

    bool followerInverted = true;
    controls::Follower follower{RobotMap::Elevator::kMotor1ID, followerInverted};

    controls::PositionVoltage positionOut{0_tr};

    frc::sim::ElevatorSim elevatorSim
    {
        frc::LinearSystemId::ElevatorSystem
        (
            frc::DCMotor::KrakenX60(2),
            9.07_kg,
            kSpoolRadius,
            kMotorGearing.value()
        ),
        frc::DCMotor::KrakenX60(2),
        0.051_m,
        1.7_m,
        false,
        0.051_m
    };
};
