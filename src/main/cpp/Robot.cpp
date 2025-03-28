#include "Robot.h"

void Robot::RobotInit() 
{
	EnableLiveWindowInTest(true);

	frc::SmartDashboard::PutData("Mech2d", &mech);
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
	swerve.UpdateLimelights();
	elevator.UpdateElevator();

	frc2::CommandScheduler::GetInstance().Run();

	elevatorMech->SetLength(elevator.GetHeight().value());
	clawMech->SetAngle(claw.GetCurrentAngle() - 90_deg);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{
	swerve.ConfigureBlueOriginOffset();
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