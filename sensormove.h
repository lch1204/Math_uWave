#ifndef SENSORMOVE_H
#define SENSORMOVE_H

#include "readerjson.h"
#include "sensor_uwave.h"
#include <iostream>
#include <ostream>
#include <qt5/QtCore/qglobal.h>
#include <random>
#include <chrono>

using namespace std;

class SensorMove
{
    ReaderJson config;
    vector<Sensor_uWave> sensors;
    vector<Obstacle> obstacles;
    double simulation_time = 0.0;
    bool is_waiting = false;
    std::chrono::time_point<std::chrono::steady_clock> timeout_start;
    double loss_probability = 0.1; // Вероятность потери сигнала
    std::mt19937 gen{std::random_device{}()}; // Генератор случайных чисел
    vector<RayPath> measurement_results; // Сохраненные результаты измерений

public:
    SensorMove();

    void run_simulation();

    void addPositionAUV(double x, double y, double z);
    void addPositionModem(double x, double y, double z);
    void readConfig(const string &config_file);
    void returnXY();

    // Проверка потери сигнала
    bool check_signal_loss(double procent_loss) const {
        std::mt19937 gen{std::random_device{}()}; // Генератор случайных чисел
        // Получаем вероятность потери сигнала из конфига (по умолчанию 0.1)
        double loss_probability_config = config.get<double>("signal_loss_probability");

        if (config.get<bool>("signal_loss_probability_flag"))
        {
            // Генерация случайного события (потеря сигнала)
            std::bernoulli_distribution dist(loss_probability_config);
            return dist(gen); // Возвращает true, если сигнал потерян
        }
        else
        {
            std::bernoulli_distribution dist(procent_loss);
            return dist(gen); // Возвращает true, если сигнал потерян
        }

    }

    // Запуск таймаута при потере сигнала
    void start_timeout() {
        is_waiting = true;
        timeout_start = std::chrono::steady_clock::now();
        std::cout << "Signal lost! Starting timeout..." << endl;
    }

    // Основной метод выполнения измерений
    void perform_measurements();

private:
    void process_delays(const vector<double>& delays, size_t sensor_id);
    void run_measurement_cycle();;
};

#endif // SENSORMOVE_H
