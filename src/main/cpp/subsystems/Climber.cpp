#include "subsystems/Climber.h"

Climber::Climber()
{
    // Creates the configurator for the IO motor - it's REV, so there are differences in configuration
    SparkBaseConfig config;
    // Brake mode
    config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
    // Limits the current so we don't burn out the motor
    config.SmartCurrentLimit(60);
    // Applies the configuration, resetting the parameters that it can, and persisting avaiable parameters
    climbMotor.Configure(config, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
}

void Climber::SetPower(double power)
{
    // Sets the duty cycle of the climb motor
    climbMotor.Set(power);
}