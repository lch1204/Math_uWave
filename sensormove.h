#ifndef SENSORMOVE_H
#define SENSORMOVE_H

#include "readerjson.h"
#include "sensor_uwave.h"
#include <deque>
#include <iostream>
#include <mutex>
#include <ostream>
#include <qt5/QtCore/qglobal.h>
#include <random>
#include <chrono>

using namespace std;

extern double X[2000][2];
extern QVector<double> K;

class SensorMove
{
    ReaderJson config;

    vector<Obstacle> obstacles;
    double simulation_time = 0.0;
    bool is_waiting = false;
    std::chrono::time_point<std::chrono::steady_clock> timeout_start;
    double loss_probability = 0.1; // Вероятность потери сигнала
    std::mt19937 gen{std::random_device{}()}; // Генератор случайных чисел
    vector<RayPath> measurement_results; // Сохраненные результаты измерений

    std::atomic<bool> new_hydro_measurement{false};
    double last_distance{0.0};
    std::mutex data_mutex;

public:
    SensorMove();

    void run_simulation(); ///для вызова в основном тике без аппарата

    void run_simulation_with_MathAUV(double x, double y, double z, double dt); ///для вызова в основном тике мат моделью аппарата

    vector<Sensor_uWave> sensors;

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

    bool new_measurement_available = false;

    bool delayAfterMeasurement = false;
    double delayDt = 0;
    double lastDistance =0;

    // В классе SensorMove добавить метод:
    std::optional<double> getCurrentDistance() const {
        if(new_measurement_available) {
            return sensors[1].dist;
        }
        return std::nullopt;
    }

    // Основной метод выполнения измерений
    void perform_measurements(double dt);

    std::deque<HydroMeasurement> hydro_buffer;

    // Метод для обновления данных от гидроакустики (вызывается извне при получении)
    void updateHydroacousticData(double distance) {
        std::lock_guard<std::mutex> lock(data_mutex);
        hydro_buffer.emplace_back(HydroMeasurement{
            distance,
            get_current_time()
        });
    }

    // Метод для проверки и получения данных
    std::optional<HydroMeasurement> getNextHydroData() {
        std::lock_guard<std::mutex> lock(data_mutex);
        if(!hydro_buffer.empty()) {
            auto data = hydro_buffer.front();
            hydro_buffer.pop_front();
            return data;
        }
        return std::nullopt;
    }

    double get_current_time() {
        static auto start_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;
        return elapsed.count();
    }

    bool updateData = false;

    double getOutput()
    {
        if (updateData)
        {
            updateData = false;
            return lastDistance;
        }
        else
        {
            return -1;
        }
    }

private:
    void process_delays(const vector<double>& delays, size_t sensor_id);
    void run_measurement_cycle(double dt);
};

#endif // SENSORMOVE_H
