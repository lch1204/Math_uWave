#include "sensormove.h"
#include <iostream>

using namespace std;

SensorMove::SensorMove(const string &config_file) {
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

void SensorMove::run_simulation() {
    double time_step = config.get<double>("simulation.time_step");
    double duration = config.get<double>("simulation.duration");

    while(sim_time < duration) {
        auto& transmitter = sensors[0];
        transmitter.update_position(time_step);

        for(size_t i = 1; i < sensors.size(); ++i) {
            auto& receiver = sensors[i];

            SignalPropagation propagation(config);
            propagation.calculate_paths(
                transmitter.get_position(),
                receiver.get_position(),
                obstacles
                );

            auto delays = receiver.receive_signal(
                transmitter.get_position(),
                propagation
                );

            process_delays(delays, i);
        }

        sim_time += time_step;
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
