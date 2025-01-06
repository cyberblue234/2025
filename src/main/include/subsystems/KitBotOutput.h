#pragma once

#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

class KitBotOutput
{
public:
    void SetMotor(double power);
private:
    ctre::phoenix::motorcontrol::can::VictorSPX motor{0};
};