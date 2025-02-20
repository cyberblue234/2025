#pragma once

#include <frc/Solenoid.h>

#include "Constants.h"

class Pneumatics
{
public:
    /// @brief Sets the stopper solenoid to extend or retract the stopper
    /// @param extended True for extended, false for retracted
    void SetStopper(bool extended);

    /// @brief Returns a pointer to the stopperSolenoid object
    /// @return frc::Solenoid* of the stopper solenoid
    frc::Solenoid *GetStopperSolenoid() { return &stopperSolenoid; };
private:
    frc::Solenoid stopperSolenoid
    {
        RobotMap::Pneumatics::kPneumaticHubID, 
        frc::PneumaticsModuleType::REVPH, 
        RobotMap::Pneumatics::kStopperSlot, 
    };
};