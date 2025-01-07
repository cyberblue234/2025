#include "Robot.h"

void Robot::RobotInit() 
{
	EnableLiveWindowInTest(true);
}

void Robot::RobotPeriodic() 
{
	swerve.UpdateTelemetry();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
	swerve.ResetGyro();
}

void Robot::TeleopPeriodic()
{
	controls.Periodic(GetPeriod());
	swerve.UpdateOdometry();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() 
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return StartRobot<Robot>();
}
#endif
