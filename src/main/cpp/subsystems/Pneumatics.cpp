#include "subsystems/Pneumatics.h"

void Pneumatics::ExtendStopper()
{
    stopperSolenoid.Set(frc::DoubleSolenoid::kForward);
}

void Pneumatics::RetractStopper()
{
    stopperSolenoid.Set(frc::DoubleSolenoid::kReverse);
}