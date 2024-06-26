#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/CANSparkMax.h>
#include <complex.h>
#include <string>
#include "angleMath.h"
#include "constants.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace std;
using namespace ctre::phoenix6;

class Module{
public:
    Module(int modID, complex<double> pos){
        turn_vector = pos*complex<double>(0, 1)/abs(pos);
        m_drive = new hardware::TalonFX(modID+10, "CTREdevices");
        encoder = new hardware::CANcoder(modID+20, "CTREdevices");
        m_steering = new rev::CANSparkMax(modID+30, rev::CANSparkMax::MotorType::kBrushless);
    }

    void init() {
        m_drive->SetNeutralMode(signals::NeutralModeValue::Brake);
        configs::TalonFXConfiguration configs{};
        configs::Slot0Configs& slot0Configs = configs.Slot0;
		/* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
		slot0Configs.kP = 5; // An error of 1 rotation per second results in 5 amps output
		slot0Configs.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
		slot0Configs.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
		configs.TorqueCurrent.PeakForwardTorqueCurrent = 45;
		configs.TorqueCurrent.PeakReverseTorqueCurrent = -45;
        m_drive->GetConfigurator().Apply(configs, 50_ms);
    }

    void set(complex<double> robot_velocity, double turn_rate){
        velocity = getVelocity(robot_velocity, turn_rate);
        wheel_speed = abs(velocity);
        angle = encoder->GetAbsolutePosition().GetValue().value()*(M_PI*2);
        error = arg(velocity) - angle;
        am::wrap(error);
        if (wheel_speed < 0.001) {
            error = 0;
        }
        if (abs(error) > (M_PI/2)){
            error += M_PI;
            am::wrap(error);
            wheel_speed *= -1;
        }
        m_steering->Set(error/M_PI);
        auto friction_torque = (wheel_speed > 0) ? 1_A : -1_A; // To account for friction, we add this to the arbitrary feed forward
        /* Use torque velocity */
        m_drive->SetControl(m_velocity.WithVelocity(wheel_speed*constants::rotations_per_meter*1_tps).WithFeedForward(friction_torque));
        calculateOdometry();
    }

    complex<double> getPositionChange() {
        return position_change;
    }

    void calculateOdometry() {
        motor_position = m_drive->GetPosition().GetValueAsDouble();
        motor_position_change = motor_position - motor_position_old;
        motor_position_old = motor_position;
        position_change = polar<double>(motor_position_change / constants::rotations_per_meter, angle);
        frc::SmartDashboard::PutNumber("poschgx", position_change.real());
        frc::SmartDashboard::PutNumber("poschgy", position_change.imag());
    }

    void resetEncoders() {
        motor_position_old = 0;
        m_drive->SetPosition(0_tr);
    }

    double getMotorPos() {
        return motor_position;
    }

    double getMotorPosChange() {
        return motor_position_change;
    }

    complex<double> getVelocity(complex<double> robot_velocity, double turn_rate){
        return robot_velocity + turn_vector * turn_rate;
    }

private:
    hardware::TalonFX *m_drive;
    hardware::CANcoder *encoder;
	controls::VelocityTorqueCurrentFOC m_velocity{0_tps, 0_tr_per_s_sq, 0_A, 0, false};
    rev::CANSparkMax *m_steering;
    complex<double> turn_vector;
    double angle;
    complex<double> velocity;
    double wheel_speed;
    double error;
    double motor_position;
    double motor_position_old = 0;
    double motor_position_change = 0;
    complex<double> position_change;
};
/*

# # # # # # # # # # # # # # # # # # #
# C:11                         C:13 #
# E:21                         E:23 #
# N:31                         N:33 #
#                                   #
#                ^                  #
#                | x                #
#          y <-- +                  #
#                                   #
#                                   #
#                                   #
# C:12                         C:14 #
# E:22                         E:24 #
# N:32                         N:34 #
# # # # # # # # # # # # # # # # # # #

*/
