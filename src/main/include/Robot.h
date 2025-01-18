#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/KitBotOutput.h"
#include "subsystems/Limelight.h"
#include "Controls.h"
#include "Constants.h"
#include "Autonomous.h"

class Robot : public frc::TimedRobot
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

    void OdometryInit()
	{
		limelight3.UpdateLimelightTracking();
		limelight3.UpdateTelemetry();
        
	};

    Drivetrain *GetSwerve() { return &swerve; }; 
    KitBotOutput *GetKitBotOutput() { return &kitBotOutput; };

private:
    Drivetrain swerve;
    KitBotOutput kitBotOutput;
    Limelight limelight3;

	frc::PowerDistribution pdp{1, frc::PowerDistribution::ModuleType::kRev};

    Limelight *GetLimelight3() { return &limelight3; };
	Controls controls{GetSwerve(), GetKitBotOutput(), GetLimelight3()};
    Autonomous autonomous{GetSwerve(), GetKitBotOutput()};

    std::optional<frc2::CommandPtr> autoCmd;
};