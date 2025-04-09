#include "navigationekf.h"
#include <iostream>
#include "../stucturs.h"
#include "qdebug.h"

NavigationEKF::NavigationEKF(const Eigen::Vector3d& beacon_position)
    : beacon_pos_(beacon_position) {
    state_.resize(7); // x,y,z,vx,vy,vz,psi
    covariance_ = Eigen::MatrixXd::Identity(7,7) * 0.1; // Инициализация всех компонент

    Q_.resize(7,7);
    Q_.diagonal() << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.01; // Шумы для всех состояний
        // /* Позиция (x,y,z) */ 1, 1, 1,
        // /* Скорости (vx,vy,vz) */ 1, 1, 1,
        // /* Курс (ψ) */ 1;
        // // /* Позиция (x,y,z) */ 0.067, 0.034, 0.020,
        // // /* Скорости (vx,vy,vz) */ 0.0134, 0.034, 0.02,
        // // /* Курс (ψ) */ 0.010;

    // Для глубины:
    R_depth_ = Eigen::MatrixXd::Identity(1,1) * 1;

    // Для расстояния до маяка:
    R_distance_ = Eigen::MatrixXd::Identity(1,1) * 1;
}

void NavigationEKF::predict(double dt,
                            const Eigen::Vector3d& linear_acc,
                            double angular_vel_z) {
    // Обновление курса с учетом угловой скорости
    state_(6) += angular_vel_z * dt; // psi += wz * dt

    // Преобразование локальных скоростей в глобальные
    Eigen::Matrix3d R = yawToRotation(state_(6));
    Eigen::Vector3d global_vel = R * state_.segment<3>(3);

    // Обновление позиции
    state_.segment<3>(0) += global_vel * dt;

    // Обновление скоростей
    state_.segment<3>(3) += linear_acc * dt;

    // Обновление ковариации
    Eigen::MatrixXd F = computeProcessJacobian(dt, state_(6));
    covariance_ = F * covariance_ * F.transpose() + Q_;
    // F_.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;

    // // Прогнозирование состояния
    // state_ = F_ * state_;
    // covariance_ = F_ * covariance_ * F_.transpose() + Q_;
    // // 1. Обновление курса (в радианах)
    // double psi = state_(6) + angular_vel_z * dt;
    // // state_(6) = normalizeAngle(psi);
    // state_(6) = (psi);
    // X[150][0] = state_(6);
    // X[152][0] = psi;
    // state_(6) = angular_vel_z*M_PI/180;

    // // 2. Преобразование локальных скоростей в глобальные
    // Eigen::Matrix3d R = yawToRotation(state_(6));
    // Eigen::Vector3d vel_global = R * state_.segment<3>(3);

    // // 3. Обновление позиции в глобальной СК
    // state_.segment<3>(0) += vel_global * dt;

    // // 4. Обновление скоростей в связанной СК (ускорение от IMU)
    // state_.segment<3>(3) += linear_acc * dt;

    // // 5. Линеаризация и обновление ковариации
    // Eigen::MatrixXd F = computeProcessJacobian(dt, state_(6));
    // covariance_ = F * covariance_ * F.transpose() + Q_;
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
        return false;
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

void NavigationEKF::setState(const Eigen::VectorXd& state) {
    if(state.size() != STATE_DIM) {
        qWarning() << "Invalid state dimension!";
        return;
    }
    state_ = state;
}

void NavigationEKF::processBufferedMeasurements(double current_time) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    while(!measurement_buffer_.empty() &&
           measurement_buffer_.front().receive_time <= current_time) {
        const auto& meas = measurement_buffer_.front();

        // Рассчет интервалов прогноза
        double dt_prev = current_time - last_update_time_;
        double dt_meas = meas.measurement_time - last_update_time_;

        // Шаг 1: Откат состояния до момента измерения
        predictWithDelay(dt_prev, -dt_meas); // Обратный прогноз

        // Шаг 2: Применение коррекции
        applyCorrection(meas);

        // Шаг 3: Прогноз до текущего времени
        predictWithDelay(dt_meas, dt_prev);

        last_update_time_ = current_time;
        measurement_buffer_.pop_front();
    }
}

Eigen::Matrix3d NavigationEKF::yawToRotation(double psi) const {
    Eigen::Matrix3d R;
    R << cos(psi), -sin(psi), 0,
        sin(psi),  cos(psi), 0,
        0,         0,        1;
    return R;
}

//Eigen::MatrixXd NavigationEKF::computeProcessJacobian(double dt, double psi) const {
//    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(7,7);

//    Eigen::Matrix3d R = yawToRotation(psi);
//    F.block<3,3>(0,3) = R * dt;

//    // Учет влияния курса на скорости
//    F(3,6) = -state_(4) * dt; // dvx/dpsi
//    F(4,6) = state_(3) * dt;  // dvy/dpsi

//    return F;
//}


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

void NavigationEKF::applyCorrection(const TimedMeasurement &meas)
{
    // switch(meas.sensor_type) {
    // case SensorType::IMU:
    //     correctIMU(meas.data);
    //     break;
    // case SensorType::DEPTH:
    //     correctDepth(meas.data[0]);
    //     break;
    // case SensorType::HYDRO:
    //     correctHydroacoustic(meas.data[0]);
    //     break;
    // case SensorType::GPS:
    //     break;
    // }
}

Eigen::MatrixXd NavigationEKF::computeFMatrix(double dt) const {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    F.block<3,3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    return F;
}

void NavigationEKF::predictWithDelay(double dt_prev, double dt_current)
{
    Eigen::MatrixXd F_prev = computeFMatrix(dt_prev);
    Eigen::MatrixXd F_current = computeFMatrix(dt_current);

    // Коррекция ковариации
    Eigen::MatrixXd G = computeFMatrix(dt_current);
    covariance_ = F_current * covariance_ * F_current.transpose() + G * Q_ * G.transpose();

    // Прогноз состояния
    state_ = F_current * state_;
}

void NavigationEKF::bufferMeasurement(const Eigen::VectorXd &data, double timestamp, SensorType type)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    TimedMeasurement meas;
    meas.data = data;
    meas.measurement_time = timestamp;
    meas.receive_time = timestamp + sensor_delays_[type];
    meas.sensor_type = type;

    // Вставка с сортировкой по receive_time
    auto it = std::lower_bound(
        measurement_buffer_.begin(),
        measurement_buffer_.end(),
        meas,
        [](const auto& a, const auto& b) {
            return a.receive_time < b.receive_time;
        }
        );
    measurement_buffer_.insert(it, meas);
}

