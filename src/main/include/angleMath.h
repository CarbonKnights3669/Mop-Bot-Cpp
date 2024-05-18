#pragma once

#include <math.h>
namespace am{
    // wrap input to between -pi and pi
    void wrap(float &angle){
        while (angle > M_PI){
            angle -= M_PI*2;
        }
        while (angle < -M_PI){
            angle += M_PI*2;
        }
    }

    // wrap input to between -180 and 180
    void wrapDeg(float &angle) {
        while (angle > 180){
            angle -= 360;
        }
        while (angle < -180){
            angle += 360;
        }
    }
}