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

    motor1Config.Slot0.kP = kP;
    motor1Config.Slot0.kI = kI;
    motor1Config.Slot0.kD = kD;
    motor1Config.Slot0.kS = kS.value();
    motor1Config.Slot0.kV = kV.value();

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
    if(isElevatorRegistered == false && power > 0)
    {
        power = 0;
    }
    motor1.Set(power);
}

void Elevator::GoToPosition(Positions pos)
{
    if (GetBottomLimitSwitch() == true || GetElevatorEncoder() >= kMaxEncoderValue || isElevatorRegistered == false)
    {
        SetMotors(0);
    }
    else
    {
        units::turn_t turns = GetTurnsToPosition(pos);
        controls::PositionVoltage &turnPos = positionOut.WithPosition(turns);
        motor1.SetControl(turnPos);
    }
}

void Elevator::UpdateElevator()
{
    if (GetBottomLimitSwitch() == true && (GetElevatorEncoder() > 0.025_tr || GetElevatorEncoder() < -0.025_tr))
    {
        ResetElevatorEncoders();
        isElevatorRegistered = true;
    }
}

const units::turn_t Elevator::GetElevatorEncoder()
{
    units::turn_t motor1RotorPos = motor1.GetRotorPosition().GetValue();
    units::turn_t motor2RotorPos = motor2.GetRotorPosition().GetValue();
    if (motor1RotorPos >= motor2RotorPos)
    {
        return motor1RotorPos;
    }
    return motor2RotorPos;
}

void Elevator::ResetElevatorEncoders()
{
    motor1.SetPosition(0_tr);
    motor2.SetPosition(0_tr);
}

units::turn_t Elevator::GetTurnsToPosition(Positions pos)
{
    switch (pos)
    {
    case Positions::L1:
        return kPositionL1;
        break;
    case Positions::L2:
        return kPositionL2;
        break;
    case Positions::L3:
        return kPositionL3;
        break;
    case Positions::L4:
        return kPositionL4;
        break;
    case Positions::Pickup:
        return kPositionPickup;
        break;
    case Positions::Processor:
        return kPositionProcessor;
        break;
    case Positions::Barge:
        return kPositionBarge;
        break;
    case Positions::Floor:
        return kPositionFloor;
        break;

    default:
        return 0_tr;
        break;
    }
}