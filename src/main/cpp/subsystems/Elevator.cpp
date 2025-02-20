#include "subsystems/Elevator.h"

Elevator::Elevator()
{
    // Starts the configuration process for the first elevator motor
    // This line resets any previous configurations to ensure a clean slate 
    motor1.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor1Config{};

    // Sets the motor to brake mode - this is so the elevator stays at the position we tell it to stay at
    motor1Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    // Stator limit makes sure we don't burn up our motors if they get jammed
    motor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor1Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

    // Sets the PID values
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

    // Makes the second elevator motor a follower to the first elevator motor
    motor2.SetControl(follower);
}

void Elevator::SetMotors(double power)
{
    // Only allow the elevator to go down if it is not registered
    if(isElevatorRegistered == false && power > 0)
    {
        power = 0;
    }
    motor1.Set(power);
}

bool Elevator::GoToTurns(units::turn_t turns)
{
    // If delta turns is postitive, the elevator is going up - vice versa for negative delta turns
    units::turn_t deltaTurns = turns - GetEncoder();
    // Conditions to kill motors
    if ((IsBottomLimitSwitchClosed() == true && deltaTurns < 0_tr) || (GetEncoder() >= kMaxEncoderValue && deltaTurns > 0_tr) || isElevatorRegistered == false)
    {
        SetMotors(0);
    }
    else
    {
        controls::PositionVoltage &turnPos = positionOut.WithPosition(turns);
        motor1.SetControl(turnPos);
    }
    // Returns true if the change in position is less than the deadzone
    return units::math::abs(deltaTurns) < (kDeadzone / kMetersPerMotorTurn);
}

bool Elevator::GoToPosition(Positions pos)
{
    units::turn_t setTurns = GetTurnsToPosition(pos);
    return GoToTurns(setTurns);
}


void Elevator::UpdateElevator()
{
    // If the limit switch is pressed AND the encoders are close to 0 or the elevator isn't registered
    if (IsBottomLimitSwitchClosed() == true && ((GetEncoder() > 0.025_tr || GetEncoder() < -0.025_tr) || isElevatorRegistered == false))
    {
        ResetEncoders();
        isElevatorRegistered = true;
    }
}


const units::turn_t Elevator::GetTurnsToPosition(Positions pos)
{
    units::meter_t height = 0_m;
    switch (pos)
    {
    case Positions::L1:
        height = kPositionL1;
        break;
    case Positions::L2:
        height = kPositionL2;
        break;
    case Positions::L3:
        height = kPositionL3;
        break;
    case Positions::L4:
        height = kPositionL4;
        break;
    case Positions::AlgaeLow:
        height = kPositionAlgaeLow;
        break;
    case Positions::AlgaeHigh:
        height = kPositionAlgaeHigh;
        break;
    case Positions::CoralStation:
        height = kPositionCoralStation;
        break;
    case Positions::Processor:
        height = kPositionProcessor;
        break;
    case Positions::Barge:
        height = kPositionBarge;
        break;

    default:
        height = GetHeight();
        break;
    }

    return height / kMetersPerMotorTurn;
}


const units::turn_t Elevator::GetEncoder()
{
    /// @todo Look into turning this to GetPosition()
    units::turn_t motor1RotorPos = motor1.GetRotorPosition().GetValue();
    units::turn_t motor2RotorPos = -motor2.GetRotorPosition().GetValue();
    if (motor1RotorPos >= motor2RotorPos)
    {
        return motor1RotorPos;
    }
    return motor2RotorPos;
}

void Elevator::ResetEncoders()
{
    if (frc::RobotBase::IsSimulation()) return;
    motor1.SetPosition(0_tr);
    motor2.SetPosition(0_tr);
}


void Elevator::UpdateTelemtry()
{
    frc::SmartDashboard::PutNumber("Elevator Encoder", GetEncoder().value());
    frc::SmartDashboard::PutBoolean("Elevator Bottom Limit Switch", IsBottomLimitSwitchClosed());
    frc::SmartDashboard::PutBoolean("Elevator Is Registered", isElevatorRegistered);
    frc::SmartDashboard::PutNumber("Elevator M1 Pos", motor1.GetRotorPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Elevator M2 Pos", motor2.GetRotorPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Elevator Height", GetHeight().convert<units::feet>().value());
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

    frc::SmartDashboard::PutNumber("Elevator M1 Sim Voltage", motorVoltage.value());

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    elevatorSim.SetInputVoltage(motorVoltage);
    elevatorSim.Update(20_ms); // assume 20 ms loop time

    simLimSwitch = elevatorSim.HasHitLowerLimit();

    motor1Sim.SetRawRotorPosition((elevatorSim.GetPosition() - 0.051_m) / kMetersPerMotorTurn);
    motor2Sim.SetRawRotorPosition((elevatorSim.GetPosition() - 0.051_m) / kMetersPerMotorTurn);
    motor1Sim.SetRotorVelocity(elevatorSim.GetVelocity() / kMetersPerMotorTurn);
    motor2Sim.SetRotorVelocity(elevatorSim.GetVelocity() / kMetersPerMotorTurn);
    
    frc::SmartDashboard::PutBoolean("Simulated Elevator Has Hit Lower Limit", elevatorSim.HasHitLowerLimit());
    frc::SmartDashboard::PutNumber("Simulated Elevator Height", elevatorSim.GetPosition().value());
}