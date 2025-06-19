#ifndef SENSORIMU_H
#define SENSORIMU_H

#include "qvector.h"
#pragma once
#include <random>
#include <Eigen/Dense>
#include <Eigen/Geometry> // Для AngleAxisd и преобразований
#include <cmath>

extern double X[2000][2];
extern QVector<double> K;

// Помогательный класс второго порядка фильтра, реализующий разностное уравнение:
//   y[k] = -a1*y[k-1] - a2*y[k-2] + b0*u[k] + b1*u[k-1] + b2*u[k-2]
class SecondOrderFilter {
public:
    SecondOrderFilter(double a1, double a2, double b0, double b1, double b2)
        : a1(a1), a2(a2), b0(b0), b1(b1), b2(b2),
        y1(0.0), y2(0.0), u1(0.0), u2(0.0) {}

    double filter(double input) {
        double output = -a1 * y1 - a2 * y2 + b0 * input + b1 * u1 + b2 * u2;
        // сдвиг состояний
        u2 = u1;
        u1 = input;
        y2 = y1;
        y1 = output;
        return output;
    }
private:
    double a1, a2, b0, b1, b2;
    double y1, y2;
    double u1, u2;
};

// Структура для хранения результатов измерений IMU
struct IMUOutput {
    Eigen::Vector3d orientation;        // roll,pitch,yaw (рад)
    Eigen::Vector3d angularRates;       // рад/с
    Eigen::Vector3d accelerationGlobal; // м/с² в глобальном frame
    Eigen::Vector3d speedGlobal;        // м/с в глобальном frame
    Eigen::Vector3d coordinateGlobal;   // м в глобальном frame

    // --- добавляем для локальной (body) системы ---
    Eigen::Vector3d accelerationLocal;  // м/с² в body frame (до преобразования R)
    Eigen::Vector3d speedLocal;         // м/с в body frame
    Eigen::Vector3d coordinateLocal;    // м в body frame (интеграция speedLocal)
};

//
// Класс IMUSensor – модель бесплатформенной системы ориентации.
// В данной реализации для каждого канала применяется динамика датчика,
// добавляются смещение и шум (с нормальным распределением) и применяется квантование.
// Передаточная функция датчика: T(s)=1/(0.00002*s^2+0.3*s+1)
// (коэффициенты дискретизации получены методом билинейного преобразования при dt=0.01 с)
//
class IMUSensor {
public:
    // dt – шаг дискретизации (0.01 сек при 100 Гц), resolution – квантование (0.01)
    IMUSensor(double resolution = 0.000001)
        : resolution(resolution), gen(rd()),
        // для измерений углов ориентации задаём шум по СКО (крен: 0.27, дифферент: 0.2, курс: 0.27)
        noise_dist_orientation_roll(0.0, 0.0),
        noise_dist_orientation_pitch(0.0, 0.0),
        noise_dist_orientation_yaw(0.0, 0.0),
        // для гироскопа можно задать относительно малый шум (например, 0.01)
        noise_dist_gyro(0.0, 0.00),
        // для акселерометра (СКО по оx: 0.0134, по oy: 0.034, по oz: 0.998)
        noise_dist_acc_x(0.0, 0.00),
        noise_dist_acc_y(0.0, 0.0),
        noise_dist_acc_z(0.0, 0.00),
        // инициализируем фильтры для измерения ориентации
        roll_filter(0.006472, -0.9411, 0.01618, 0.03236, 0.01618),
        pitch_filter(0.006472, -0.9411, 0.01618, 0.03236, 0.01618),
        yaw_filter(0.006472, -0.9411, 0.01618, 0.03236, 0.01618)
    {
        linear_acc.setZero();
        // для каждого канала гироскопа и акселерометра создаём свой фильтр
        for (int i = 0; i < 3; i++) {
            gyro_filters[i] = new SecondOrderFilter(0.006472, -0.9411, 0.01618, 0.03236, 0.01618);
            acc_filters[i]  = new SecondOrderFilter(0.006472, -0.9411, 0.01618, 0.03236, 0.01618);
        }
        first_update = true;
        prev_linear_vel.setZero();
        // Смещения для измерения ориентации (мат. ожидание)
        // крен = 0.1, дифферент = 0.08, курс считаем смещённым на 0
        orientation_bias << 0.0, 0.0, 0.0;
        // Для гироскопа смещение можно принять равным 0
        gyro_bias.setZero();
        // Смещения акселерометра (мат. ожидание по осям)
        acc_bias << 0.0, 0.0, 0.0;
    }

    ~IMUSensor() {
        for (int i = 0; i < 3; i++) {
            delete gyro_filters[i];
            delete acc_filters[i];
        }
    }
    // Метод update вызывается с реальными значениями:
    //   true_roll, true_pitch, true_yaw – истинные углы (в радианах)
    //   true_angular_rates – истинные угловые скорости (рад/с)
    //   true_linear_vel – истинные линейные скорости (м/с)
    void update(double true_roll, double true_pitch, double true_yaw,
                const Eigen::Vector3d& true_angular_rates,
                const Eigen::Vector3d& true_linear_vel,
                double real_dt)
    {
        // 1. Обработка измерения ориентации (например, от инклинометров)
        double filt_roll  = roll_filter.filter(true_roll);
        double filt_pitch = pitch_filter.filter(true_pitch);
        double filt_yaw   = yaw_filter.filter(true_yaw);

        double meas_roll  = quantize(filt_roll + orientation_bias(0) + noise(noise_dist_orientation_roll), resolution);
        double meas_pitch = quantize(filt_pitch + orientation_bias(1) + noise(noise_dist_orientation_pitch), resolution);
        double meas_yaw   = quantize(filt_yaw + orientation_bias(2) + noise(noise_dist_orientation_yaw), resolution);

        imu_output.orientation << meas_roll, meas_pitch, meas_yaw;

        X[110][0] = meas_roll;
        X[111][0] = meas_pitch;
        X[112][0] = meas_yaw;

        // 2. Измерение угловых скоростей (гироскоп)
        Eigen::Vector3d meas_gyro;
        for (int i = 0; i < 3; i++) {
//            double filt_rate = gyro_filters[i]->filter(true_angular_rates(i));
             double filt_rate = (true_angular_rates(i));
//            double rate = quantize(filt_rate + gyro_bias(i) + noise(noise_dist_gyro), resolution);
            double rate = filt_rate;
             meas_gyro(i) = rate;
        }
        imu_output.angularRates = meas_gyro;

        X[113][0] = meas_gyro(0);
        X[114][0] = meas_gyro(1);
        X[115][0] = meas_gyro(2);

        // 3. Измерение ускорения (акселерометр)
        // Вычисляем линейное ускорение по разности скоростей

        linear_acc = (true_linear_vel - prev_linear_vel) / real_dt; // Используем real_dt

//        Eigen::Vector3d coriolis = true_angular_rates.cross(true_linear_vel);
//        linear_acc = (true_linear_vel - prev_linear_vel) / real_dt + coriolis;
        Eigen::Vector3d true_acc = linear_acc;
//        true_acc(2) += 9.80665; // Добавляем гравитацию

        prev_linear_vel = true_linear_vel;
        // Добавляем гравитацию (предполагаем, что ось z направлена вдоль силы тяжести)
        // В состоянии покоя акселерометр должен выдавать ≈1 g по oz
        // true_acc(2) += 1.0; // g

        Eigen::Vector3d meas_acc;
        for (int i = 0; i < 3; i++) {
//            double filt_acc = acc_filters[i]->filter(true_acc(i));
            double filt_acc = true_acc(i);
            double acc_val = quantize(filt_acc + acc_bias(i) + noiseAcc(i), 0.0001);
            meas_acc(i) = acc_val;
        }
        imu_output.accelerationLocal = meas_acc;
        imu_output.speedLocal = true_linear_vel;

        X[116][0] = meas_acc(0);
        X[117][0] = meas_acc(1);
        X[118][0] = meas_acc(2);

        double roll  = imu_output.orientation(0);
        double pitch = imu_output.orientation(1);
        double yaw   = imu_output.orientation(2);

        // Построим матрицу поворота из локальной системы (body) в глобальную (NED)
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())  // Крен (X)
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())  // Тангаж (Y)
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());   // Рыскание (Z)

        // Преобразуем измеренное ускорение (из локальной системы в глобальную)
        Eigen::Vector3d global_acc = R * meas_acc;
//        global_acc(2) -= 9.80665; // Коррекция гравитаци

        // Сохраняем глобальное ускорение в структуру imu_output
        imu_output.accelerationGlobal = global_acc;

        // Записываем глобальное ускорение в массив состояния для дальнейшей интеграции
        X[116][0] = global_acc(0);
        X[117][0] = global_acc(1);
        X[118][0] = global_acc(2);

        // Интегрируем ускорение в скорость
        integrate(X[116][0], X[121][0] , X[121][1], real_dt); // скорость по оси X
        integrate(X[117][0], X[122][0] , X[122][1], real_dt); // скорость по оси Y
        integrate(X[118][0], X[123][0] , X[123][1], real_dt); // скорость по оси Z

        imu_output.speedGlobal = Eigen::Vector3d(X[121][0], X[122][0], X[123][0]);

        // Интегрируем скорость в координаты (позицию)
        integrate(X[121][0], X[124][0] , X[124][1], real_dt); // координата по оси X
        integrate(X[122][0], X[125][0] , X[125][1], real_dt); // координата по оси Y
        integrate(X[123][0], X[126][0] , X[126][1], real_dt); // координата по оси Z

        imu_output.coordinateGlobal = Eigen::Vector3d(X[124][0], X[125][0], X[126][0]);

        // 3) интегрируем локальную скорость в локальную координату
        double v0 = true_linear_vel(0);
        double v1 = true_linear_vel(1);
        double v2 = true_linear_vel(2);
            //    (нужно добавить два новых массива stateLocalSpeedPrev и stateLocalPosPrev)
            integrate(v0, X_loc_speed[0], X_loc_speed_prev[0], real_dt);
            integrate(v1, X_loc_speed[1], X_loc_speed_prev[1], real_dt);
            integrate(v2, X_loc_speed[2], X_loc_speed_prev[2], real_dt);
            imu_output.speedLocal = Eigen::Vector3d(X_loc_speed[0], X_loc_speed[1], X_loc_speed[2]);

            integrate(imu_output.speedLocal.x(), X_loc_pos[0], X_loc_pos_prev[0], real_dt);
            integrate(imu_output.speedLocal.y(), X_loc_pos[1], X_loc_pos_prev[1], real_dt);
            integrate(imu_output.speedLocal.z(), X_loc_pos[2], X_loc_pos_prev[2], real_dt);
            imu_output.coordinateLocal = Eigen::Vector3d(X_loc_pos[0], X_loc_pos[1], X_loc_pos[2]);

    }
    void updateCoordinate(double x, double y, double z)
    {
        X[124][1] =x;
        X[125][1] =y;
        X[126][1] =z;
    };

    void integrate(double &input, double &output, double &prevOutput, double dt)
    {
        output = prevOutput + dt*input;
        prevOutput = output;
    }

    // Возвращает структуру с текущими измерениями
    IMUOutput getOutput() const {
        return imu_output;
    }

private:
    double resolution; // квантование (0.01)

    // Генератор случайных чисел
    std::random_device rd;
    std::mt19937 gen;
    Eigen::Vector3d linear_acc;

    double X_loc_speed[3], X_loc_speed_prev[3];
    double X_loc_pos[3],   X_loc_pos_prev[3];

    // Нормальные распределения для шума измерений ориентации
    std::normal_distribution<> noise_dist_orientation_roll;
    std::normal_distribution<> noise_dist_orientation_pitch;
    std::normal_distribution<> noise_dist_orientation_yaw;
    // Для гироскопа
    std::normal_distribution<> noise_dist_gyro;
    // Для акселерометра по каждой оси
    std::normal_distribution<> noise_dist_acc_x;
    std::normal_distribution<> noise_dist_acc_y;
    std::normal_distribution<> noise_dist_acc_z;

    // Функция для получения шума акселерометра по оси
    double noiseAcc(int axis) {
        if (axis == 0)
            return noise(noise_dist_acc_x);
        else if (axis == 1)
            return noise(noise_dist_acc_y);
        else
            return noise(noise_dist_acc_z);
    }

    // Функция квантования: округление до ближайшего кратного resolution
    double quantize(double value, double resolution) {
        return std::round(value / resolution) * resolution;
    }

    // Универсальная функция для получения случайного числа из распределения
    double noise(std::normal_distribution<>& dist) {
        return dist(gen);
    }
    // Фильтры для измерений ориентации
    SecondOrderFilter roll_filter;
    SecondOrderFilter pitch_filter;
    SecondOrderFilter yaw_filter;
    // Массивы фильтров для гироскопа и акселерометра (по 3 канала)
    SecondOrderFilter* gyro_filters[3];
    SecondOrderFilter* acc_filters[3];

    // Смещения (систематическая ошибка) для ориентации и акселерометра
    Eigen::Vector3d orientation_bias; // крен, дифферент, курс
    Eigen::Vector3d gyro_bias;        // для гироскопа (принимаем 0)
    Eigen::Vector3d acc_bias;         // акселерометр по осям

    // Для вычисления ускорения по разности линейных скоростей
    Eigen::Vector3d prev_linear_vel;
    bool first_update;


    // Выходные данные измерений
    IMUOutput imu_output;
};

#endif // SENSORIMU_H
