#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Claw.h"
#include "subsystems/Climber.h"
#include "subsystems/Pneumatics.h"
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

    Drivetrain *GetSwerve() { return &swerve; }
    Elevator *GetElevator() { return &elevator; }
    Claw *GetClaw() { return &claw; }
    Climber *GetClimber() { return &climber; }
    Pneumatics *GetPneumatics() { return &pneumatics; }
    Limelight *GetLimelightHigh() { return &limelightHigh; }
    Limelight *GetLimelightLow() { return &limelightLow; }

    void UpdateTelemetry();


private:
    Drivetrain swerve{GetLimelightHigh(), GetLimelightLow()};
    Elevator elevator;
    Claw claw;
    Climber climber;
    Pneumatics pneumatics;

    Limelight limelightHigh{"limelight-high"};
    Limelight limelightLow{"limelight-low"};

	frc::PowerDistribution pdp{1, frc::PowerDistribution::ModuleType::kRev};
    
	Controls controls{GetSwerve(), GetElevator(), GetClaw(), GetClimber(), GetPneumatics(), GetLimelightHigh(), GetLimelightLow()};
    Autonomous autonomous{GetSwerve()};

    std::optional<frc2::CommandPtr> autoCmd;
};