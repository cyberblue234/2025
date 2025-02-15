#include "subsystems/Climber.h"

Climber::Climber()
{
    climbMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration climbMotorConfig{};

    climbMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    climbMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    climbMotor.GetConfigurator().Apply(climbMotorConfig);
}

void Climber::SetPower(double power)
{
    climbMotor.Set(power);
}