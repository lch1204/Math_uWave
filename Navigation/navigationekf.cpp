#include "navigationekf.h"
#include <iostream>

NavigationEKF::NavigationEKF(const Eigen::Vector3d& beacon_position)
    : beacon_pos_(beacon_position) {
    state_.resize(STATE_DIM);
    state_.setZero();

    covariance_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

    // Настройка шумов (примерные значения)
    Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    Q_.diagonal() << 0.1, 0.1, 0.1,   // Позиция (x,y,z)
        0.5, 0.5, 0.5,    // Скорости в связанной СК (vx,vy,vz)
        0.01;             // Курс (psi)

    R_distance_ = Eigen::MatrixXd::Identity(1,1) * 0.5;
    R_depth_ = Eigen::MatrixXd::Identity(1,1) * 0.1;
}

void NavigationEKF::predict(double dt,
                            const Eigen::Vector3d& linear_acc,
                            double angular_vel_z) {
    // 1. Обновление курса (в радианах)
    double psi = state_(6) + angular_vel_z * dt;
    // state_(6) = normalizeAngle(psi);
    state_(6) = (psi);
    X[150][0] = state_(6);
    X[152][0] = psi;
    state_(6) = angular_vel_z*M_PI/180;

    // 2. Преобразование локальных скоростей в глобальные
    Eigen::Matrix3d R = yawToRotation(state_(6));
    Eigen::Vector3d vel_global = R * state_.segment<3>(3);

    // 3. Обновление позиции в глобальной СК
    state_.segment<3>(0) += vel_global * dt;

    // 4. Обновление скоростей в связанной СК (ускорение от IMU)
    state_.segment<3>(3) += linear_acc * dt;

    // 5. Линеаризация и обновление ковариации
    Eigen::MatrixXd F = computeProcessJacobian(dt, state_(6));
    covariance_ = F * covariance_ * F.transpose() + Q_;
}

void NavigationEKF::correct(double measured_distance) {
    Eigen::Vector3d delta = state_.segment<3>(0) - beacon_pos_;
    double predicted_distance = delta.norm();

    if(predicted_distance < 1e-6) return;

    // Матрица Якоби измерений (только глобальные координаты)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
    H << delta.x()/predicted_distance,
        delta.y()/predicted_distance,
        delta.z()/predicted_distance,
        0, 0, 0, 0;

    // Коэффициент Калмана
    Eigen::MatrixXd S = H * covariance_ * H.transpose() + R_distance_;
    Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

    // Коррекция только глобальных координат
    state_.head<3>() += K.block<3,1>(0,0) * (measured_distance - predicted_distance);

    // Обновление ковариации
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    covariance_ = (I - K * H) * covariance_;
}

void NavigationEKF::correctDepth(double measured_z) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
    H(0,2) = 1; // Только координата Z

    Eigen::MatrixXd S = H * covariance_ * H.transpose() + R_depth_;
    Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

    // Коррекция только Z
    state_(2) += K(2,0) * (measured_z - state_(2));

    // Обновление ковариации
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    covariance_ = (I - K * H) * covariance_;
}

void NavigationEKF::setState(double x, double y, double z, double psi)
{
    state_(0) = x;
    state_(1) = y;
    state_(2) = z;
}

Eigen::Matrix3d NavigationEKF::yawToRotation(double psi) const {
    Eigen::Matrix3d R;
    R << cos(psi), -sin(psi), 0,
        sin(psi),  cos(psi), 0,
        0,         0,        1;
    return R;
}

Eigen::MatrixXd NavigationEKF::computeProcessJacobian(double dt, double psi) const {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

    // Производные позиции по скоростям
    Eigen::Matrix3d R = yawToRotation(psi);
    F.block<3,3>(0,3) = R * dt;

    // Производные скоростей по курсу
    Eigen::Vector3d vel_local = state_.segment<3>(3);
    F.block<3,1>(3,6) << -vel_local.y() * dt,
        vel_local.x() * dt,
        0;

    return F;
}
