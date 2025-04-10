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
				allPathsPoses.push_back(swerve.FlipPose(pose));
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

void Robot::SimulationInit() 
{
	frc::sim::DIOSim bottom{RobotMap::Elevator::kBottomLimitSwitchID};
	bottom.SetValue(false);
	frc::sim::DIOSim top{RobotMap::Elevator::kTopLimitSwitchID};
	top.SetValue(false);
	frc::sim::DIOSim climber{RobotMap::Climber::kLimitSwitch};
	climber.SetValue(false);
}

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

    stage1Publisher.Set(frc::Pose3d{0_m, 0_m, (elevator.GetHeight() - ElevatorConstants::kHeightOffset) / 2, frc::Rotation3d{0_deg, 0_deg, 0_deg}});
    carriagePublisher.Set(frc::Pose3d{0_m, 0_m, elevator.GetHeight() - ElevatorConstants::kHeightOffset, frc::Rotation3d{0_deg, 0_deg, 0_deg}});
	units::meter_t x = 0.265_m;
	units::meter_t y = 0.4354_m;
	clawPublisher.Set(frc::Pose3d{x, 0_m, elevator.GetHeight() - ElevatorConstants::kHeightOffset + y, frc::Rotation3d{0_deg, claw.GetCurrentAngle(), 0_deg}});
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif