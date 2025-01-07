#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/KitBotOutput.h"
#include "Controls.h"
#include "Constants.h"

class Robot : public TimedRobot
{

public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    Drivetrain swerve;
    KitBotOutput kitBotOutput;

	PowerDistribution pdp{1, PowerDistribution::ModuleType::kRev};

	Controls controls{&swerve, &kitBotOutput};
};