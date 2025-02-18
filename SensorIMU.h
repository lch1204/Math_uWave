#ifndef SENSORIMU_H
#define SENSORIMU_H

#pragma once
#include <random>
#include <Eigen/Dense>

class IMUSensor {
public:
    IMUSensor(double gyro_noise = 0.001, double bias_std = 0.0001) :
        gyro_noise_dist(0.0, gyro_noise),
        bias_dist(0.0, bias_std) {}

    void update(double dt, const Eigen::Vector3d& true_rates) {
        // Накопление ошибки смещения
        bias += Eigen::Vector3d(bias_dist(gen),
                                bias_dist(gen),
                                bias_dist(gen)) * dt;

        // Зашумленные измерения
        measured_rates = true_rates +
                         Eigen::Vector3d(gyro_noise_dist(gen),
                                         gyro_noise_dist(gen),
                                         gyro_noise_dist(gen)) +
                         bias;
    }

    Eigen::Vector3d getAngularRates() const { return measured_rates; }

private:
    Eigen::Vector3d measured_rates;
    Eigen::Vector3d bias{0,0,0};

    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<> gyro_noise_dist;
    std::normal_distribution<> bias_dist;
};

#endif // SENSORIMU_H
