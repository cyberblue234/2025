#include "subsystems/KitBotOutput.h"

void KitBotOutput::SetMotor(double power)
{
    motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power);
};
