#pragma once

#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SwerveModule.h"
#include "trajectoryMaker.h"

using namespace std;

class Swerve{
public:
    
    // drives robot at given speed during teleop
    void set(complex<float> velocity, float turn_rate){
        heading = -gyro.GetYaw()*(M_PI/180);
        target_velocity = velocity;
        // robot orient the velocity
        velocity *= polar<float>(1, -heading);
        // find fastest module speed
        fastest = 4;
        for (Module module : modules){
            module_speed = abs(module.getVelocity(velocity, turn_rate));
            if (module_speed > fastest)
                fastest = module_speed;
        }
        // move current velocity toward target
        target_velocity *= 4 / fastest;
        turn_rate /= fastest;
        velocity_error = target_velocity-current_velocity;
        turn_rate_error = turn_rate - current_turn_rate;
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
        position_change = complex<float>(0,0);
        for (Module module : modules) {
            module.set(target_velocity, current_turn_rate);
            position_change += module.getPositionChange();
        }
        position += position_change * polar<float>(0.25F, heading);
    }

    // drive toward the position setpoint with feedforward
    void SetPose(trajectoryMaker::Sample sample) {
        // calculate proporional response
        heading = -gyro.GetYaw()*(M_PI/180);
        position_error = sample.position - position;
        heading_error = sample.heading - heading;
        am::wrap(heading_error);
        sample.velocity += position_P * position_error;
        sample.angular_velocity += heading_P * heading_error;
        //robot orient the output
        sample.velocity *= polar<float>(1, -heading);
        // calculate odometry and drive modules
        position_change = complex<float>(0,0);
        for (Module module : modules){
            module.set(sample.velocity, sample.angular_velocity);
            position_change += module.getPositionChange();
        }
        position += position_change * polar<float>(0.25F, heading);
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

    complex<float> GetPosition() {
        return position;
    }

    complex<float> GetPositionChange() {
        return position_change;
    }

private:
    AHRS gyro{frc::SPI::Port::kMXP};
    Module modules[4] = {
        Module{1, complex<float>(1, 1)},
        Module{2, complex<float>(-1, 1)},
        Module{3, complex<float>(1, -1)},
        Module{4, complex<float>(-1, -1)}
    };

      // heading proportional response rate
    complex<float> current_velocity; // current velocity the swerve is set to in teleop
    float current_turn_rate = 0;     // current turn rate of the swerve in teleop
    float heading;
    complex<float> target_velocity;
    complex<float> velocity_error;
    float turn_rate_error;
    float module_speed;
    float fastest;


    complex<float> position = complex<float>(0, 0); // current position of the robot
    complex<float> position_change;
    float position_P = 0.04;         // position proportional response rate
    float heading_P = 3;
    complex<float> position_error;
    float heading_error;
} swerve;
