#pragma once
#include "qvector.h"
#include "stucturs.h"
#include <Eigen/Dense>
#include <deque>
#include <mutex>

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
                 double angular_vel_z);

    bool correct(double measured_distance, double delta_t, double max_speed = 3);
    void correctDepth(double measured_z);
    void setState(const Eigen::VectorXd& state);

    void processBufferedMeasurements(double current_time);

    Eigen::VectorXd getState() const { return state_; }
    Eigen::Vector3d getPosition() const { return state_.segment<3>(0); }
    double getYaw() const { return state_(6); }
    double last_update_time_;

    Eigen::MatrixXd computeFMatrix(double dt) const;
private:
    struct TimedMeasurement {
        Eigen::VectorXd data;    // Данные измерения
        double measurement_time; // Момент получения данных датчиком
        double receive_time;     // Момент поступления в EKF
        SensorType sensor_type;         // Тип датчика (ENUM)
    };
    // 4. Обратное преобразование состояния
    void revertState(double dt) {
        Eigen::MatrixXd F_inv = F_.inverse();
        state_ = F_inv * state_;
        covariance_ = F_inv * covariance_ * F_inv.transpose() + Q_;
    }
    std::deque<TimedMeasurement> measurement_buffer_;
    Eigen::VectorXd state_buffer_; // Состояние на момент последнего измерения
    double last_measurement_time_;
    std::mutex buffer_mutex_;

    // Параметры задержек для каждого типа датчиков
    std::unordered_map<SensorType, double> sensor_delays_ = {
        {SensorType::IMU, 0.12},    // 120 мс задержка IMU
        {SensorType::DEPTH, 0.25},  // 250 мс датчик давления
        {SensorType::HYDRO, 0.8}    // 800 мс гидроакустика
    };


    static const int STATE_DIM = 7; // x,y,z,vx,vy,vz,psi
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd R_distance_;
    Eigen::MatrixXd R_depth_;
    Eigen::Vector3d beacon_pos_;

    Eigen::Matrix3d yawToRotation(double psi) const;
    Eigen::MatrixXd computeProcessJacobian(double dt, double psi) const;

    // Нормализация угла в [-π, π]
    double normalizeAngle(double angle) {
        return std::fmod(angle + M_PI, 2*M_PI) - M_PI;
    }

    void applyCorrection(const TimedMeasurement& meas);

public:

    void bufferMeasurement(const Eigen::VectorXd& data,
                           double timestamp,
                           SensorType type);
    void predictWithDelay(double dt_prev, double dt_current);
};
