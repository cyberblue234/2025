#pragma once

#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"

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

    Limelight limelightHigh{"limelight-high"};
    Limelight limelightLow{"limelight-low"};

    nt::StructPublisher<frc::Pose3d> stage1Publisher = nt::NetworkTableInstance::GetDefault().GetTable("datatable")->GetStructTopic<frc::Pose3d>("stage 1").Publish();
    nt::StructPublisher<frc::Pose3d> carriagePublisher = nt::NetworkTableInstance::GetDefault().GetTable("datatable")->GetStructTopic<frc::Pose3d>("carriage").Publish();
    nt::StructPublisher<frc::Pose3d> clawPublisher = nt::NetworkTableInstance::GetDefault().GetTable("datatable")->GetStructTopic<frc::Pose3d>("claw").Publish();

	frc::PowerDistribution pdp{1, frc::PowerDistribution::ModuleType::kRev};
    
	Controls controls{GetSwerve(), GetElevator(), GetClaw(), GetClimber(), GetPneumatics(), GetLimelightHigh(), GetLimelightLow()};
    Autonomous autonomous{GetSwerve(), GetElevator(), GetClaw(), GetClimber(), GetPneumatics(), GetLimelightHigh(), GetLimelightLow()};

    std::optional<frc2::CommandPtr> autoCmd;
};