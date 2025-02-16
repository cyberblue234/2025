#pragma once

#include <frc/DoubleSolenoid.h>

#include "Constants.h"

class Pneumatics
{
public:
    void ExtendStopper();
    void RetractStopper();
private:
    frc::DoubleSolenoid stopperSolenoid
    {
        RobotMap::Pneumatics::kPneumaticHubID, 
        frc::PneumaticsModuleType::REVPH, 
        RobotMap::Pneumatics::kStopperForwardSlot, 
        RobotMap::Pneumatics::kStopperReverseSlot
    };
};