#include "Robot.h"

void Robot::RobotInit() 
{
	EnableLiveWindowInTest(true);
}

void Robot::RobotPeriodic() 
{
	swerve.UpdateTelemetry();
	swerve.UpdateOdometry();
	limelightHigh.UpdateTelemetry();
	limelightLow.UpdateTelemetry();

	frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

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
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
