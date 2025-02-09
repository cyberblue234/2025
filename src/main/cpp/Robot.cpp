#include "Robot.h"

void Robot::RobotInit() 
{
	EnableLiveWindowInTest(true);
}

void Robot::RobotPeriodic() 
{
	UpdateTelemetry();
	swerve.UpdateOdometry();
	elevator.UpdateElevator();

	frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() 
{
	swerve.ResetGyro();
}

void Robot::DisabledPeriodic() 
{
	swerve.ConfigureBlueOriginOffset();
}

void Robot::AutonomousInit() 
{
	autoCmd = autonomous.GetAutoCommand();
	if (autoCmd) autoCmd->Schedule();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
	controls.Periodic();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() 
{
	swerve.Sim();
	elevator.SimMode();
}

void Robot::UpdateTelemetry()
{
	frc::SmartDashboard::PutNumber("Battery Voltage", frc::RobotController::GetBatteryVoltage().value());

	swerve.UpdateTelemetry();
	elevator.UpdateTelemtry();
	limelightHigh.UpdateTelemetry();
	limelightLow.UpdateTelemetry();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
