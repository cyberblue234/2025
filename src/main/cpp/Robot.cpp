#include "Robot.h"

void Robot::RobotInit() 
{
	EnableLiveWindowInTest(true);

	OdometryInit();
}

void Robot::RobotPeriodic() 
{
	swerve.UpdateTelemetry();
	swerve.UpdateOdometry();
	OdometryInit();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() 
{
	autoCmd = autonomous.GetAutoCommand();
	if (autoCmd) autoCmd->Schedule();
	OdometryInit();
}

void Robot::AutonomousPeriodic() 
{
	frc2::CommandScheduler::GetInstance().Run();
}

void Robot::TeleopInit() 
{
	OdometryInit();
}

void Robot::TeleopPeriodic()
{
	controls.Periodic(GetPeriod());
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
