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
	// Creates UsbCamera and MjpegServer [1] and connects them
	//frc::CameraServer::StartAutomaticCapture();
	//frc::CameraServer::PutVideo("Blur", 640, 480);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {
	frc::SmartDashboard::PutNumber("angle", intakeShooter.GetAngle());
	frc::SmartDashboard::PutBoolean("note detected", intakeShooter.eye2.Get());
	frc::SmartDashboard::PutNumber("posx", swerve.position.real());
	frc::SmartDashboard::PutNumber("posy", swerve.position.imag());
	frc::SmartDashboard::PutNumber("mod0Changex", swerve.GetModulePosChange(0).real());
	frc::SmartDashboard::PutNumber("mod0Changey", swerve.GetModulePosChange(0).imag());
	frc::SmartDashboard::PutNumber("mod1Changex", swerve.GetModulePosChange(1).real());
	frc::SmartDashboard::PutNumber("mod1Changey", swerve.GetModulePosChange(1).imag());
	frc::SmartDashboard::PutNumber("mod2Changex", swerve.GetModulePosChange(2).real());
	frc::SmartDashboard::PutNumber("mod2Changey", swerve.GetModulePosChange(2).imag());
	frc::SmartDashboard::PutNumber("mod3Changex", swerve.GetModulePosChange(3).real());
	frc::SmartDashboard::PutNumber("mod3Changey", swerve.GetModulePosChange(3).imag());
	frc::SmartDashboard::PutNumber("motor0Change", swerve.GetMotorPosChange(0));
	frc::SmartDashboard::PutNumber("motor1Change", swerve.GetMotorPosChange(1));
	frc::SmartDashboard::PutNumber("motor2Change", swerve.GetMotorPosChange(2));
	frc::SmartDashboard::PutNumber("motor3Change", swerve.GetMotorPosChange(3));
	frc::SmartDashboard::PutNumber("motor0Pos", swerve.GetMotorPos(0));
	frc::SmartDashboard::PutNumber("motor1Pos", swerve.GetMotorPos(1));
	frc::SmartDashboard::PutNumber("motor2Pos", swerve.GetMotorPos(2));
	frc::SmartDashboard::PutNumber("motor3Pos", swerve.GetMotorPos(3));
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic(){}

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
