#include "subsystems/Elevator.h"

Elevator::Elevator()
{
    motor1.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor1Config{};

    motor1Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    motor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor1Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

    motor1Config.Slot0.kP = kP;
    motor1Config.Slot0.kI = kI;
    motor1Config.Slot0.kD = kD;

    motor1.GetConfigurator().Apply(motor1Config);

    motor2.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor2Config{};

    motor2Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    motor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor2Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

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
    units::turn_t setTurns = GetTurnsToPosition(pos);
    units::turn_t deltaTurns = setTurns - GetElevatorEncoder();
    if ((GetBottomLimitSwitch() == true && deltaTurns < 0_tr) || (GetElevatorEncoder() >= kMaxEncoderValue && deltaTurns > 0_tr) || isElevatorRegistered == false)
    {
        SetMotors(0);
    }
    else
    {
        frc::SmartDashboard::PutNumber("Elevator Set Turns", setTurns.value());
        controls::PositionVoltage &turnPos = positionOut.WithPosition(setTurns);
        motor1.SetControl(turnPos);
    }
}

void Elevator::UpdateElevator()
{
    if (GetBottomLimitSwitch() == true && ((GetElevatorEncoder() > 0.025_tr || GetElevatorEncoder() < -0.025_tr) || isElevatorRegistered == false))
    {
        ResetElevatorEncoders();
        isElevatorRegistered = true;
    }
}

const units::turn_t Elevator::GetElevatorEncoder()
{
    units::turn_t motor1RotorPos = motor1.GetRotorPosition().GetValue();
    units::turn_t motor2RotorPos = -motor2.GetRotorPosition().GetValue();
    if (motor1RotorPos >= motor2RotorPos)
    {
        return motor1RotorPos;
    }
    return motor2RotorPos;
}

void Elevator::ResetElevatorEncoders()
{
    if (frc::RobotBase::IsSimulation()) return;
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

    default:
        return 0_tr;
        break;
    }
}

void Elevator::UpdateTelemtry()
{
    frc::SmartDashboard::PutNumber("Elevator Encoder", GetElevatorEncoder().value());
    frc::SmartDashboard::PutBoolean("Elevator Bottom Limit Switch", GetBottomLimitSwitch());
    frc::SmartDashboard::PutBoolean("Elevator Is Registered", isElevatorRegistered);
    frc::SmartDashboard::PutNumber("Elevator M1 Pos", motor1.GetRotorPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Elevator M2 Pos", motor2.GetRotorPosition().GetValueAsDouble());
}

void Elevator::SimMode()
{
    ctre::phoenix6::sim::TalonFXSimState& motor1Sim = motor1.GetSimState();
    ctre::phoenix6::sim::TalonFXSimState& motor2Sim = motor2.GetSimState();

    // set the supply voltage of the TalonFX
    motor1Sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    motor2Sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    // get the motor voltage of the TalonFX
    units::volt_t motorVoltage = motor1Sim.GetMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    elevatorSim.SetInputVoltage(motorVoltage);
    elevatorSim.Update(20_ms); // assume 20 ms loop time

    simLimSwitch = elevatorSim.HasHitLowerLimit();

    motor1Sim.SetRawRotorPosition((elevatorSim.GetPosition() - 0.051_m) / kMetersPerMotorTurn);
    motor2Sim.SetRawRotorPosition((elevatorSim.GetPosition() - 0.051_m) / kMetersPerMotorTurn);
    
    frc::SmartDashboard::PutBoolean("Simulated Elevator Has Hit Lower Limit", elevatorSim.HasHitLowerLimit());
    frc::SmartDashboard::PutNumber("Simulated Elevator Height", elevatorSim.GetPosition().value());
    
}