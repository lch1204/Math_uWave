#include "navigationekf.h"
#include <iostream>

NavigationEKF::NavigationEKF(const Eigen::Vector3d& beacon_position)
    : beacon_pos_(beacon_position) {
    state_.resize(STATE_DIM);
    state_.setZero();

    covariance_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.1;

    // Настройка шумов (примерные значения)
    Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    Q_.diagonal() <<
        /* Позиция (x,y,z) */ 6.7e-7, 3.4e-6, 2.0e-8,
        /* Скорости (vx,vy,vz) */ 1.34e-4, 3.4e-4, 2.0e-5,
        /* Курс (ψ) */ 1.0e-4;

    // Для глубины:
    R_depth_ = Eigen::MatrixXd::Identity(1,1) * 0.1;

    // Для расстояния до маяка:
    R_distance_ = Eigen::MatrixXd::Identity(1,1) * 0.1;
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

bool NavigationEKF::correct(double measured_distance, double delta_t, double max_speed) {
    // 1. Вычисление разности между положением аппарата и маяком
    Eigen::Vector3d delta = state_.segment<3>(0) - beacon_pos_;
    double predicted_distance = delta.norm(); //вычисляем длину
    if(predicted_distance < 1e-6)
        return false;

    // 2. Вычисление инновации
    double innovation = measured_distance - predicted_distance;

    // 3. Вычисление максимально допустимого перемещения
    double max_innovation = max_speed * delta_t;

    // 4. Ограничение (гейтеринг) инновации
    if(std::fabs(innovation) > max_innovation) {
        // Можно либо просто обрезать значение инновации:
        innovation = (innovation > 0) ? max_innovation : -max_innovation;
        // Либо можно отвергнуть измерение, если инновация слишком велика.
        // В данном примере мы просто ограничиваем поправку.
    }

    // 5. Вычисление матрицы Якоби H (учитывается только влияние глобальных координат)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
    H << delta.x()/predicted_distance,
        delta.y()/predicted_distance,
        delta.z()/predicted_distance,
        0, 0, 0, 0;

    // 6. Расчет S и коэффициента Калмана K
    Eigen::MatrixXd S = H * covariance_ * H.transpose() + R_distance_;
    Eigen::MatrixXd K_gain = covariance_ * H.transpose() * S.inverse();

    // 7. Коррекция состояния с ограниченной инновацией
    state_.head<3>() += K_gain.block<3,1>(0,0) * innovation;

    // 8. Обновление ковариационной матрицы
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    covariance_ = (I - K_gain * H) * covariance_;
    return true;
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
