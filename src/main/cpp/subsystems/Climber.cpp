#include "subsystems/Climber.h"

Climber::Climber()
{
    // Starts the configuration process for the climber motor
    // This line resets any previous configurations to ensure a clean slate 
    climbMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration climbMotorConfig{};

    // Sets the motor to brake mode - this is so we hopefully don't fall down when the match ends
    climbMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    // // Stator limit makes sure we don't burn up our motors if they get jammed
    climbMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    // Applies the configuration
    climbMotor.GetConfigurator().Apply(climbMotorConfig);
}

void Climber::SetPower(double power)
{
    // Sets the duty of the climb motor
    climbMotor.Set(power);
}