#pragma once

#include "math.h"
#include <iostream>
#include <vector>
#include <fstream>
#include "complex.h"
#include "json.hpp"

using namespace std;
namespace trajectoryMaker
{
    struct Sample {
        double timestamp;
        complex<float> position;
        float heading;
        complex<float> velocity;
        float angular_velocity;
    };

    // Function to load JSON data into a vector of Sample structures
    vector<Sample> MakeTrajectory(const std::string& filename) {
        ifstream file(filename);
        nlohmann::json jsonData;
        file >> jsonData;

        vector<Sample> trajectory;

        for (const auto& item : jsonData["samples"]) {
            Sample sample;
            sample.timestamp = item["timestamp"].get<double>();
            sample.position = complex<float>(item["x"].get<float>(), item["y"].get<float>());
            sample.heading = item["heading"].get<float>();
            sample.velocity = complex<float>(item["velocityX"].get<float>(), item["velocityY"].get<float>());
            sample.angular_velocity = item["angularVelocity"].get<double>();
            trajectory.push_back(sample);
        }

        return trajectory;
    }
}