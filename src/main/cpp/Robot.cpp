// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//test

#include "Robot.h"
#include "math.h"

using namespace std;

void Robot::RobotInit()
{
	intakeShooter.init();
	swerve.init();
	trajectory = trajectoryMaker::MakeTrajectory(frc::filesystem::GetDeployDirectory() + "/test.traj");
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	autonomousTimer.Restart();
}
void Robot::AutonomousPeriodic() {
	while (trajectory[i].timestamp < double(autonomousTimer.Get()) && i < trajectory.size()) {
		swerve.SetPose(trajectory[i]);
		i++;
	}
	if (i >= trajectory.size()) {
		swerve.set(complex<double>(0,0), 0);
	}
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic(){
	swerve.set(complex<double>(-controller.GetLeftY(), -controller.GetLeftX())*4.0, -controller.GetRightX()*4);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
