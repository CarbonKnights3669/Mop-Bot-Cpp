#pragma once

#include <math.h>
namespace constants{
    const float slew_rate = 0.04;    // how quickly to change the swerve velocity
    const float module_gear_ratio = 6.12;
    const float rotations_per_meter = module_gear_ratio / (0.09906 * M_PI);
}