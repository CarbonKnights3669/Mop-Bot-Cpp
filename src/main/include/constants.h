#pragma once

#include <math.h>
namespace constants{
    const double slew_rate = 0.15;    // how quickly to change the swerve velocity
    const double module_gear_ratio = 6.12;
    const double rotations_per_meter = module_gear_ratio / (0.09906 * M_PI);
}