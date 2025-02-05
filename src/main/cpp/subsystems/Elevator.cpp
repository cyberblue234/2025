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

    motor2.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor2Config{};
}