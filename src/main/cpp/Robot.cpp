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
	// Creates UsbCamera and MjpegServer [1] and connects them
	//frc::CameraServer::StartAutomaticCapture();
	//frc::CameraServer::PutVideo("Blur", 640, 480);
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
	frc::SmartDashboard::PutNumber("posx", swerve.GetPositionChange().real());
	frc::SmartDashboard::PutNumber("posy", swerve.GetPositionChange().imag());
	frc::SmartDashboard::PutNumber("motorPos1", swerve.GetModuleMotorPosition(0));
	frc::SmartDashboard::PutNumber("motorPos2", swerve.GetModuleMotorPosition(1));
	frc::SmartDashboard::PutNumber("motorPos3", swerve.GetModuleMotorPosition(2));
	frc::SmartDashboard::PutNumber("motorPos4", swerve.GetModuleMotorPosition(3));
	frc::SmartDashboard::PutNumber("motorPosChg1", swerve.GetMotorPosChange(0));
	frc::SmartDashboard::PutNumber("motorPosChg2", swerve.GetMotorPosChange(1));
	frc::SmartDashboard::PutNumber("motorPosChg3", swerve.GetMotorPosChange(2));
	frc::SmartDashboard::PutNumber("motorPosChg4", swerve.GetMotorPosChange(3));
	frc::SmartDashboard::PutNumber("modulePosChg1x", swerve.GetModulePosChange(0).real());
	frc::SmartDashboard::PutNumber("modulePosChg1y", swerve.GetModulePosChange(0).imag());
	frc::SmartDashboard::PutNumber("modulePosChg2x", swerve.GetModulePosChange(1).real());
	frc::SmartDashboard::PutNumber("modulePosChg2y", swerve.GetModulePosChange(1).imag());
	frc::SmartDashboard::PutNumber("modulePosChg3x", swerve.GetModulePosChange(2).real());
	frc::SmartDashboard::PutNumber("modulePosChg3y", swerve.GetModulePosChange(2).imag());
	frc::SmartDashboard::PutNumber("modulePosChg4x", swerve.GetModulePosChange(3).real());
	frc::SmartDashboard::PutNumber("modulePosChg4y", swerve.GetModulePosChange(3).imag());
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic(){
	swerve.set(complex<double>(-controller.GetLeftY(), -controller.GetLeftX())*4.0, -controller.GetRightX()*4);
	frc::SmartDashboard::PutNumber("motorPos1", swerve.GetModuleMotorPosition(0));
	frc::SmartDashboard::PutNumber("motorPos2", swerve.GetModuleMotorPosition(1));
	frc::SmartDashboard::PutNumber("motorPos3", swerve.GetModuleMotorPosition(2));
	frc::SmartDashboard::PutNumber("motorPos4", swerve.GetModuleMotorPosition(3));
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
