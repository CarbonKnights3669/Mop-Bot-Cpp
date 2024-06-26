#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <complex.h>
#include <frc/Timer.h>
#include <frc/Filesystem.h>
#include <string.h>
#include "cameraserver/CameraServer.h"

#include "subsystems/Swerve.h"
#include "subsystems/IntakeShooter.h"
#include "subsystems/Limelight.h"
using namespace std;

unsigned int i = 0; // trajectory sample index

frc::XboxController controller{0};

frc::Timer autonomousTimer;


class Robot : public frc::TimedRobot{
public:
	vector<trajectoryMaker::Sample> trajectory;

	void RobotInit() override;
	void RobotPeriodic() override;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	void TeleopInit() override;
	void TeleopPeriodic() override;

	void DisabledInit() override;
	void DisabledPeriodic() override;

	void TestInit() override;
	void TestPeriodic() override;

	void SimulationInit() override;
	void SimulationPeriodic() override;

private:
	
};