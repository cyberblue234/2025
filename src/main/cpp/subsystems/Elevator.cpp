#include "subsystems/Elevator.h"

Elevator::Elevator()
{
    // Starts the configuration process for the first elevator motor
    // This line resets any previous configurations to ensure a clean slate 
    motor1.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor1Config{};

    // Sets the motor to brake mode - this is so the elevator stays at the position we tell it to stay at
    motor1Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    // Ensures positive input makes the elevator go up and negative input makes the elevator go down
    motor1Config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    // Stator limit makes sure we don't burn up our motors if they get jammed
    motor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor1Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

    motor1.GetConfigurator().Apply(motor1Config);

    motor2.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor2Config{};

    motor2Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    motor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor2Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

    motor2.GetConfigurator().Apply(motor2Config);

    // Makes the second elevator motor a follower to the first elevator motor
    motor2.SetControl(follower);
    
    frc::SmartDashboard::PutData("Elevator PID", &controller);
    frc::SmartDashboard::PutNumber("Elevator kS", kS.value());
    frc::SmartDashboard::PutNumber("Elevator kG", kG.value());
    frc::SmartDashboard::PutNumber("Elevator kV", kV.value());
    frc::SmartDashboard::PutNumber("Elevator kA", kA.value());

    frc::SmartDashboard::PutBoolean("Elevator Disable Motion Profiling", false);

    controller.SetTolerance(kTolerance);
}

void Elevator::SetMotors(double power)
{
    // Only allow the elevator to go down if it is not registered
    if(isElevatorRegistered == false && power > 0)
    {
        power = 0;
    }
    motor1.SetControl(controls::DutyCycleOut{power}
                    .WithLimitForwardMotion(GetEncoder() > kMaxEncoderValue || IsTopLimitSwitchClosed())
                    .WithLimitReverseMotion(IsBottomLimitSwitchClosed()));
}

bool Elevator::GoToHeight(const units::meter_t desiredHeight)
{
    // Conditions to kill motors
    if (isElevatorRegistered == false)
    {
        SetMotors(0);
    }
    else
    {
        controller.SetGoal(desiredHeight);
        if (desiredHeight - GetHeight() > 0_m) 
        {
            if (controller.GetConstraints().maxVelocity != upTrapezoidProfileContraints.maxVelocity
                || controller.GetConstraints().maxAcceleration != upTrapezoidProfileContraints.maxAcceleration)
                controller.SetConstraints(upTrapezoidProfileContraints);
        }
        else
        {
            if (controller.GetConstraints().maxVelocity != downTrapezoidProfileContraints.maxVelocity
                || controller.GetConstraints().maxAcceleration != downTrapezoidProfileContraints.maxAcceleration)
                controller.SetConstraints(downTrapezoidProfileContraints);
        }
        units::volt_t pidSet{controller.Calculate(GetHeight())};
        units::volt_t feedforwardSet = feedforward.Calculate(controller.GetSetpoint().velocity);
        if (frc::SmartDashboard::GetBoolean("Elevator Disable Motion Profiling", false) == false)
        {
            motor1.SetControl(voltageOut.WithOutput(pidSet + feedforwardSet)
                        .WithLimitForwardMotion(GetEncoder() > kMaxEncoderValue || IsTopLimitSwitchClosed())
                        .WithLimitReverseMotion(IsBottomLimitSwitchClosed()));
        }
    }
    // Returns true if the position is within the deadzone
    return IsAtPosition();
}

bool Elevator::GoToPosition(const Position &pos)
{
    return GoToHeight(pos.height);
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


const units::turn_t Elevator::GetEncoder()
{
    units::turn_t motor1RotorPos = motor1.GetPosition().GetValue();
    units::turn_t motor2RotorPos = motor2.GetPosition().GetValue();
    if (motor1RotorPos >= motor2RotorPos)
    {
        return motor1RotorPos;
    }
    return motor2RotorPos;
}

void Elevator::ResetEncoders()
{
    motor1.SetPosition(0_tr);
    motor2.SetPosition(0_tr);
}


void Elevator::UpdateTelemtry()
{
    frc::SmartDashboard::PutNumber("Elevator Encoder", GetEncoder().value());
    frc::SmartDashboard::PutBoolean("Elevator Bottom Limit Switch", IsBottomLimitSwitchClosed());
    frc::SmartDashboard::PutBoolean("Elevator Top Limit Switch", IsTopLimitSwitchClosed());
    frc::SmartDashboard::PutBoolean("Elevator Top Bypass", bypassTopLimit);
    frc::SmartDashboard::PutBoolean("Elevator Is Registered", isElevatorRegistered);
    frc::SmartDashboard::PutNumber("Elevator M1 Pos", motor1.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Elevator M2 Pos", motor2.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Elevator Height", GetHeight().convert<units::feet>().value());
    frc::SmartDashboard::PutNumber("Elevator Setpoint", controller.GetSetpoint().position.convert<units::feet>().value());
    frc::SmartDashboard::PutBoolean("Elevator IsAtPosition", IsAtPosition());

    double newKs = frc::SmartDashboard::GetNumber("Elevator kS", kS.value());
    if (newKs != feedforward.GetKs().value()) feedforward.SetKs(units::volt_t{newKs});
    double newKg = frc::SmartDashboard::GetNumber("Elevator kG", kG.value());
    if (newKg != feedforward.GetKg().value()) feedforward.SetKg(units::volt_t{newKg});
    double newKv = frc::SmartDashboard::GetNumber("Elevator kV", kV.value());
    if (newKv != feedforward.GetKv().value()) feedforward.SetKv(units::kv_meters_t{newKv});
    double newKa = frc::SmartDashboard::GetNumber("Elevator kA", kA.value());
    if (newKa != feedforward.GetKa().value()) feedforward.SetKa(units::ka_meters_t{newKa});

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

    motor1Sim.SetRawRotorPosition((elevatorSim.GetPosition() - kHeightOffset) / kMetersPerMotorTurn);
    motor2Sim.SetRawRotorPosition((elevatorSim.GetPosition() - kHeightOffset) / kMetersPerMotorTurn);
    motor1Sim.SetRotorVelocity(elevatorSim.GetVelocity() / kMetersPerMotorTurn);
    motor2Sim.SetRotorVelocity(elevatorSim.GetVelocity() / kMetersPerMotorTurn);
}