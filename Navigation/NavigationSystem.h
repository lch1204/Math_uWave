#ifndef NAVIGATIONSYSTEM_H
#define NAVIGATIONSYSTEM_H

#include "NavigationFilter.h"
#include "../SensorPressure.h"
#include "../SensorIMU.h"
#include "../sensormove.h"
#include <Eigen/Dense>

/**
 * @class NavigationSystem
 * @brief Система навигации, использующая расширенный фильтр Калмана (EKF) для оценки положения
 *
 * Объединяет данные от IMU, гидроакустического датчика и других сенсоров для оценки положения аппарата
 */
class NavigationSystem {
public:
    /**
     * @brief Конструктор системы навигации
     * @param sensorMove Референс на объект управления сенсорами
     * @param beacon_pos Известные координаты маяка-ответчика в глобальной системе координат
     */
    NavigationSystem(SensorMove& sensorMove,
                     const Eigen::Vector3d& beacon_pos)
        : sensorMove(sensorMove),
        beacon_position(beacon_pos),
        last_update_time(0.0),
        measurement_interval(5.0),
        process_noise(Eigen::MatrixXd::Identity(6,6)*0.1),
        measurement_noise(0.1)
    {
        initEKF();
    }

    /**
     * @brief Основной метод обновления состояния навигационной системы
     * @param current_time Текущее время в секундах
     * @param angular_rates Измеренные угловые скорости по осям (рад/с)
     */
    void update(double current_time, const Eigen::Vector3d& angular_rates) {
        // Вычисление временного шага
        double dt = current_time - last_update_time;

        // 1. Прогноз состояния на основе модели движения
        predict(dt, angular_rates);

        // 2. Коррекция по данным гидроакустики
        if(current_time - last_measurement_time >= measurement_interval) {
            if(auto dist = sensorMove.getCurrentDistance(); dist.has_value()) {
                updateWithDistanceMeasurement(*dist);
                last_measurement_time = current_time;
            }
        }

        last_update_time = current_time;
    }

    /**
     * @brief Получение текущей оцененной позиции
     * @return Вектор с координатами (x, y, z) в глобальной системе
     */
    Eigen::Vector3d getEstimatedPosition() const {
        return ekf_state.position;
    }

private:
    /// @brief Состояние EKF (позиция, скорость и ковариация)
    struct EKFState {
        Eigen::Vector3d position;    ///< 3D позиция в глобальной системе координат
        Eigen::Vector3d velocity;    ///< 3D скорость в глобальной системе
        Eigen::MatrixXd covariance; ///< Матрица ковариации состояния 6x6

        EKFState() : covariance(6,6) {}
    } ekf_state;

    SensorMove& sensorMove;          ///< Референс на систему управления сенсорами
    Eigen::Vector3d beacon_position; ///< Известная позиция маяка-ответчика
    double last_update_time;         ///< Время последнего обновления
    double last_measurement_time;    ///< Время последнего измерения
    double measurement_interval;     ///< Минимальный интервал между измерениями (сек)

    Eigen::MatrixXd process_noise;   ///< Матрица шумов процесса Q
    double measurement_noise;        ///< Дисперсия шума измерений R

    /**
     * @brief Инициализация параметров EKF
     */
    void initEKF() {
        ekf_state.position.setZero();
        ekf_state.velocity.setZero();
        ekf_state.covariance = Eigen::MatrixXd::Identity(6,6) * 10.0;
    }

    /**
     * @brief Этап предсказания EKF
     * @param dt Временной шаг (сек)
     * @param angular_rates Измеренные угловые скорости (рад/с)
     */
    void predict(double dt, const Eigen::Vector3d& angular_rates) {
        // Матрица перехода F
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6,6);
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;

        // Обновление состояния
        ekf_state.position += ekf_state.velocity * dt;
        ekf_state.velocity = computeRotationMatrix(angular_rates, dt) * ekf_state.velocity;

        // Обновление ковариации
        ekf_state.covariance = F * ekf_state.covariance * F.transpose() + process_noise;
    }

    /**
     * @brief Этап коррекции EKF по данным гидроакустики
     * @param measured_distance Измеренная дистанция до маяка
     */
    void updateWithDistanceMeasurement(double measured_distance) {
        // Разность позиций
        Eigen::Vector3d delta = ekf_state.position - beacon_position;
        double expected_distance = delta.norm();

        // Матрица Якобиана H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1,6);
        H.block<1,3>(0,0) = delta.transpose() / expected_distance;

        // Коэффициент усиления Калмана
        Eigen::MatrixXd K = ekf_state.covariance * H.transpose()
                            / ( (H * ekf_state.covariance * H.transpose()).value() + measurement_noise );

        // Коррекция состояния
        ekf_state.position -= K.topRows<3>() * (expected_distance - measured_distance);
        ekf_state.velocity -= K.bottomRows<3>() * (expected_distance - measured_distance);

        // Коррекция ковариации (упрощенная формула Джозефа)
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
        ekf_state.covariance = (I - K*H) * ekf_state.covariance;
    }

    /**
     * @brief Вычисление матрицы поворота на основе гироскопических данных
     * @param rates Угловые скорости (рад/с)
     * @param dt Временной шаг (сек)
     * @return Матрица поворота 3x3
     */
    Eigen::Matrix3d computeRotationMatrix(const Eigen::Vector3d& rates, double dt) {
        // Приближение для малых углов
        Eigen::AngleAxisd rollAngle(rates.x() * dt, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(rates.y() * dt, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rates.z() * dt, Eigen::Vector3d::UnitZ());

        return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
    }
};

#endif // NAVIGATIONSYSTEM_H
