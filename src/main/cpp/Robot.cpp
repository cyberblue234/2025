#include "Robot.h"

void Robot::RobotInit() 
{
	EnableLiveWindowInTest(true);

	limelightHigh.SetCameraPoseRobotSpace(LimelightConstants::kHighOffset);
	limelightLow.SetCameraPoseRobotSpace(LimelightConstants::kLowOffset);
	limelightHigh.SetupPortForwarding();
	limelightLow.SetupPortForwarding();

	swerve.ResetOdometryRotation();

	// Starts recording to data log
	frc::DataLogManager::Start();
	// Record both DS control and joystick data
	frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
}

void Robot::RobotPeriodic() 
{
	UpdateTelemetry();
	swerve.UpdateOdometry();
	swerve.ConfigureBlueOriginOffset();
	swerve.UpdateLimelights();
	elevator.UpdateElevator();

	frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{
	std::string auton = autonomous.GetAutoChooser();
	if (auton != "Nothing")
	{
		auto paths = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(auton);
		std::vector<frc::Pose2d> allPathsPoses{};
		for (std::shared_ptr<pathplanner::PathPlannerPath> path : paths)
		{
			for (frc::Pose2d pose : path->getPathPoses())
			{
				allPathsPoses.push_back(pose);
			}
		}
		swerve.SetFieldPath(allPathsPoses);
	}
	else
	{
		swerve.SetFieldPath(std::vector<frc::Pose2d>{swerve.GetPose()});
	}
}

void Robot::AutonomousInit() 
{
	autoCmd = autonomous.GetAutoCommand();
	if (autoCmd) 
	{
		autoCmd->Schedule();
	}

	swerve.SetStdDevs(LimelightConstants::autonStdDevs);
}

void Robot::AutonomousPeriodic() 
{
	autonomous.UpdateTelemetry();
}

void Robot::TeleopInit() 
{
	swerve.SetStdDevs(LimelightConstants::teleopStdDevs);
}

void Robot::TeleopPeriodic()
{
	controls.Periodic();
	controls.UpdateTelemetry();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() 
{
	swerve.Sim();
	elevator.SimMode();
	claw.SimMode();
}

void Robot::UpdateTelemetry()
{
	frc::SmartDashboard::PutNumber("Battery Voltage", frc::RobotController::GetBatteryVoltage().value());
	swerve.UpdateTelemetry();
	elevator.UpdateTelemtry();
	claw.UpdateTelemetry();
	climber.UpdateTelemetry();
	limelightHigh.UpdateTelemetry();
	limelightLow.UpdateTelemetry();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif