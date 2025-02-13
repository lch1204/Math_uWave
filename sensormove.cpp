#include "sensormove.h"
#include <algorithm>
#include <iostream>
#include <qt5/QtCore/qglobal.h>
#include <random>
#include <QDebug>
#include <thread>

using namespace std;

SensorMove::SensorMove() {

}

void SensorMove::run_simulation() {
    double time_step = config.get<double>("simulation.time_step"); //шаг времени симуляции
    double duration = config.get<double>("simulation.duration"); //время симуляции
    bool real_time = config.get<bool>("simulation.real_time_mode");


    while(simulation_time < duration) {
        auto& transmitter = sensors[0];
        transmitter.update_position(time_step);
        auto cycle_start = std::chrono::steady_clock::now();

        // Основной цикл измерений
        run_measurement_cycle();

        // Обновление времени симуляции
        simulation_time += time_step;

        if(real_time) {
            // Задержка для реального времени
            auto cycle_end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(cycle_end - cycle_start).count();
            if(elapsed < time_step) {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(time_step - elapsed)
                    );
            }
        }
    }
}

void SensorMove::addPositionAUV(double x, double y, double z)
{
    sensors.emplace_back(Point3D{x, y, z}, config);
}

void SensorMove::addPositionModem(double x, double y, double z)
{
    sensors.emplace_back(Point3D{x, y, z}, config);
}

void SensorMove::readConfig(const string &config_file)
{
    config.load(config_file);
    obstacles = config.get_obstacles();

    //инициализация начального местоположения датчиков
    try {
        auto positions = config.get<vector<vector<double>>>("sensor.positions");
        for(const auto& pos : positions) {
            sensors.emplace_back(Point3D{pos[0], pos[1], pos[2]}, config);
        }
    } catch (const std::exception& e) {
        std::cerr << "Ошибка при чтении 'sensor.positions': " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }


    // Настройка мобильности инициатора
    if(config.get<bool>("initiator.mobile")) {
        sensors[0].set_mobility(true);
        auto vel = config.get<vector<double>>("initiator.velocity");
        sensors[0].set_velocity({vel[0], vel[1], vel[2]});
    }
}

void SensorMove::perform_measurements() {
    auto& transmitter = sensors[0]; // Инициатор (первый датчик)


    for (size_t i = 1; i < sensors.size(); ++i) {
        auto& receiver = sensors[i];


        // 1. Расчет путей распространения сигнала
        SignalPropagation propagation(config);
        propagation.calculate_paths(
            transmitter.get_position(),
            receiver.get_position(),
            obstacles
            );

        // 2. Выбор пути через вероятностный алгоритм
        const RayPath* selected_path = propagation.selectPath();

        // 3. Обработка результатов
        measurement_results.push_back(*selected_path);
        auto delays = receiver.receive_signal(
            transmitter.get_position(),
            * selected_path
            );
        if (selected_path) {
            // Сохранение результатов

            double distance = delays[0] ;

            // Логирование
            qDebug() << "Measurement to sensor" << i
                     << "| Delay:" << distance
                     << "s | weight:" << selected_path->weight;
        } else {
            qDebug() << "No valid path to sensor" << i;
        }

        // 4. Учет задержки между измерениями (из конфига)
        if (config.get<bool>("measurement_delay_flag"))
        {
            double measurement_delay = config.get<double>("measurement_delay");
            std::this_thread::sleep_for(
                std::chrono::duration<double>(measurement_delay)
                );
        }
        else
        {
            qDebug() << "delays[0]" << delays[0]/config.get<double>("sound_speed");
            std::this_thread::sleep_for(
                std::chrono::duration<double>(delays[0]/config.get<double>("sound_speed")+config.get<double>("measurement_delay"))
                );
        }
    }
}

void SensorMove::process_delays(const vector<double> &delays, size_t sensor_id) {
    if(delays.empty()) {
        cout << "No signal from sensor " << sensor_id << endl;
        return;
    }

    double min_delay = *min_element(delays.begin(), delays.end());
    double distance = min_delay * config.get<double>("sound_speed");

    cout << "Sensor " << sensor_id << ": "
         << distance << " meters ("
         << delays.size() << " paths)" << endl;
}

void SensorMove::run_measurement_cycle() {
    if(is_waiting) {
        // Логика таймаута
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - timeout_start).count();
        if(elapsed >= config.get<double>("timeout")) {
            is_waiting = false;
            qDebug() << "Timeout finished. Resuming operations.";
        } else {
            qDebug() << "Waiting..." << (config.get<double>("timeout") - elapsed) << "s left";
            return;
        }
    }
    Point3D tr =sensors[0].get_position();
    Point3D rx = sensors[1].get_position();
    double res = sqrt(pow(tr.x-rx.x,2)+pow(tr.y-rx.y,2)+pow(tr.z-rx.z,2)); //расстояние между двумя маяками ответчиками

    double procent_loss =sqrt(10.01*res-10.01)/100; //параметр
    qDebug() << "расстояние между маяками" << res<< "процент потерь" << procent_loss*100;
    // Проверка потери сигнала
    if(check_signal_loss(procent_loss)) {
        start_timeout();
        return;
    }

    // Основная логика измерений
    perform_measurements();
}
