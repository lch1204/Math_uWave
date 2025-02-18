#ifndef SENSORPRESSURE_H
#define SENSORPRESSURE_H

#pragma once
#include <random>

class PressureSensor {
public:
    PressureSensor(double noise_std = 0.05) :
        gen(rd()),
        dist(0.0, noise_std) {}

    double getDepth(double true_depth) {
        return true_depth + dist(gen);
    }

private:
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<> dist;
};

#endif // SENSORPRESSURE_H
