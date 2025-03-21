#include "Robot.h"

void Robot::RobotInit() 
{
	EnableLiveWindowInTest(true);

	frc::SmartDashboard::PutData("Mech2d", &mech);
	limelightHigh.SetCameraPoseRobotSpace(LimelightConstants::kHighOffset);
	limelightLow.SetCameraPoseRobotSpace(LimelightConstants::kLowOffset);
	limelightHigh.SetupPortForwarding();
	limelightLow.SetupPortForwarding();
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

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() 
{
	autoCmd = autonomous.GetAutoCommand();
	if (autoCmd) autoCmd->Schedule();
}

void Robot::AutonomousPeriodic() 
{
	autonomous.UpdateTelemetry();
}

void Robot::TeleopInit() {}

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