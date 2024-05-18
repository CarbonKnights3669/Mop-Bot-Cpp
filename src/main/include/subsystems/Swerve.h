#pragma once

#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SwerveModule.h"
#include "constants.h"

using namespace std;

class Swerve{
public:
    complex<float> position = complex<float>(0, 0); // current position of the robot
    float position_P = 0.02;         // position proportional response rate
    float heading_P = 0.3;           // heading proportional response rate
    complex<float> current_velocity; // current velocity the swerve is set to in teleop
    float current_turn_rate = 0;     // current turn rate of the swerve in teleop
    
    // drives robot at given speed during teleop
    void set(complex<float> velocity, float turn_rate){
        float heading = -gyro.GetYaw()*(M_PI/180);
        complex<float> target_velocity = velocity;
        // robot orient the velocity
        velocity *= polar<float>(1, -heading);
        // find fastest module speed
        float fastest = 1;
        for (Module module : modules){
            float speed = abs(module.getVelocity(velocity, turn_rate));
            if (speed > fastest){
                fastest = speed;
            }
        }
        // move current velocity toward target
        target_velocity /= fastest;
        turn_rate /= fastest;
        complex<float> velocity_error = target_velocity-current_velocity;
        float turn_rate_error = turn_rate - current_turn_rate;
        if (abs(velocity_error) > constants::slew_rate) {
            velocity_error *= constants::slew_rate/abs(velocity_error);
        }
        if (abs(turn_rate_error) > constants::slew_rate) {
            turn_rate_error *= constants::slew_rate/abs(turn_rate_error);
        }
        current_velocity += velocity_error;
        current_turn_rate += turn_rate_error;
        // robot orient velocity
        target_velocity = current_velocity * polar<float>(1, -heading);
        // calculate odometry and drive the modules
        complex<float> position_change = complex<float>(0, 0);
        for (Module module : modules) {
            module.set(target_velocity, current_turn_rate);
            position_change += module.getPositionChange();
        }
        position += position_change * polar<float>(0.25, heading);
    }

    // drive toward the position setpoint with feedforward
    void SetPose(complex<float> position_setpoint, float heading_setpoint, complex<float> velocity, float angular_velocity) {
        // calculate proporional response
        float heading = -gyro.GetYaw()*(M_PI/180);
        complex<float> position_error = position_setpoint - position;
        float heading_error = heading_setpoint - heading;
        am::wrap(heading_error);
        velocity += position_P * position_error;
        angular_velocity += heading_P * heading_error;
        //robot orient the output
        velocity *= polar<float>(1, -heading);
        // calculate odometry and drive modules
        complex<float> position_change = complex<float>(0, 0);
        for (Module module : modules){
            module.set(velocity, angular_velocity);
            position_change += module.getPositionChange();
        }
        position += position_change * polar<float>(0.25, heading);
    }

    void init(){
        for (Module module : modules){
            module.init();
        }
    }

    void resetPos(complex<float> new_position = complex<float>(0,0)) {
        for (Module module : modules) {
            module.resetEncoders();
        }
        position = new_position;
    }

    complex<float> GetModulePosChange(int i) {
        return modules[i].getPositionChange();
    }

    float GetMotorPosChange(int i) {
        return modules[i].getMotorPosChange();
    }

    float GetMotorPos(int i) {
        return modules[i].getMotorPos();
    }

    float GetHeading() {
        return gyro.GetYaw();
    }

private:
    AHRS gyro{frc::SPI::Port::kMXP};
    Module modules[4] = {
        Module{1, complex<float>(1, 1)},
        Module{2, complex<float>(-1, 1)},
        Module{3, complex<float>(1, -1)},
        Module{4, complex<float>(-1, -1)}
    };

} swerve;
