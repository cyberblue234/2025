#pragma once

#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

#include "Controls.h"
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

    frc::Mechanism2d mech{3, 3};
    frc::MechanismRoot2d *root = mech.GetRoot("elevator-claw", 0.6, 0.2);
    frc::MechanismLigament2d* elevatorMech =
      root->Append<frc::MechanismLigament2d>("elevator", 1, 90_deg);
    frc::MechanismLigament2d* clawMech =
        elevatorMech->Append<frc::MechanismLigament2d>(
            "claw", 0.5, 0_deg, 6, frc::Color8Bit{frc::Color::kPurple});

    Limelight limelightHigh{"limelight-high"};
    Limelight limelightLow{"limelight-low"};

	frc::PowerDistribution pdp{1, frc::PowerDistribution::ModuleType::kRev};
    
	Controls controls{GetSwerve(), GetElevator(), GetClaw(), GetClimber(), GetPneumatics(), GetLimelightHigh(), GetLimelightLow()};
    Autonomous autonomous{GetSwerve(), GetElevator(), GetClaw(), GetClimber(), GetPneumatics(), GetLimelightHigh(), GetLimelightLow()};

    std::optional<frc2::CommandPtr> autoCmd;
};