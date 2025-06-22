#pragma once
#include "qvector.h"
#include <Eigen/Dense>

extern double X[2000][2];
extern QVector<double> K;

/**
 * @class NavigationEKF
 * @brief EKF для подводной навигации с коррекцией только положения
 *
 * Состояние: [x, y, z, vx, vy, vz, psi]
 * Ориентация (psi) используется из IMU без коррекции
 */
class NavigationEKF {
public:
    explicit NavigationEKF(const Eigen::Vector3d& beacon_position);

    void predict(double dt,
                 const Eigen::Vector3d& linear_acc,
                 double angular_vel_z, double psi_imu);

    bool correct(double measured_distance, double delta_t, double max_speed = 3);
    void correctDepth(double measured_z);
    void setState(double x, double y, double z, double psi);

    Eigen::VectorXd getState() const { return state_; }
    Eigen::Vector3d getPosition() const { return state_.segment<3>(0); }
    double getYaw() const { return state_(6); }

private:
    static const int STATE_DIM = 7; // x,y,z,vx,vy,vz,psi
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_distance_;
    Eigen::MatrixXd R_depth_;
    Eigen::Vector3d beacon_pos_;

    Eigen::Matrix3d yawToRotation(double psi) const;
    Eigen::MatrixXd computeProcessJacobian(double dt, double psi) const;

    // Нормализация угла в [-π, π]
    double normalizeAngle(double angle) {
        angle = std::fmod(angle + M_PI, 2 * M_PI);
        if (angle < 0)
            angle += 2 * M_PI;
        return angle - M_PI;
    }
};
