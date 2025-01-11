#pragma once

#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

using namespace ctre::phoenix;

class KitBotOutput
{
public:
    void SetMotor(double power);
private:
    motorcontrol::can::VictorSPX motor{0};
};