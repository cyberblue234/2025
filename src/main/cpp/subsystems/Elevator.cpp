#include "subsystems/Elevator.h"


Elevator::Elevator()
{
    motor1.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor1Config{};

    motor1Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    motor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor1Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

    motor1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15_s;
    motor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15_s;
    motor1Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.15_s;

    motor1Config.Slot0.kP = ElevatorConstants::kP;
    motor1Config.Slot0.kI = ElevatorConstants::kI;
    motor1Config.Slot0.kD = ElevatorConstants::kD;
    motor1Config.Slot0.kS = ElevatorConstants::kS.value();
    motor1Config.Slot0.kV = ElevatorConstants::kV.value();

    motor1.GetConfigurator().Apply(motor1Config);

    motor2.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor2Config{};

    motor2Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    motor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor2Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

    motor2Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15_s;
    motor2Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15_s;
    motor2Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.15_s;

    motor2.GetConfigurator().Apply(motor2Config);

    motor2.SetControl(follower);
}

void Elevator::SetMotors(double power) 
{
    motor1.Set(power);
}

void Elevator::GoToPosition(Positions pos)
{
    units::turn_t turns = GetTurnsToPosition(pos);
    controls::PositionVoltage& turnPos = positionOut.WithPosition(turns);
    motor1.SetControl(turnPos);
}

units::turn_t Elevator::GetTurnsToPosition(Positions pos)
{
    switch (pos)
    {
    case Positions::L1: return ElevatorConstants::PositionL1; break;
    case Positions::L2: return ElevatorConstants::PositionL2; break;
    case Positions::L3: return ElevatorConstants::PositionL3; break;
    case Positions::L4: return ElevatorConstants::PositionL4; break;
    case Positions::Pickup: return ElevatorConstants::PositionPickup; break;
    case Positions::Processor: return ElevatorConstants::PositionProcessor; break;
    case Positions::Barge: return ElevatorConstants::PositionBarge; break;
    case Positions::Floor: return ElevatorConstants::PositionFloor; break;

    default: return 0_tr; break;
    }
}