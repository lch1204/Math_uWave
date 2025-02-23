#ifndef SENSOR_UWAVE_H
#define SENSOR_UWAVE_H

#include "stucturs.h"
#include "readerjson.h"
#include "signalpropagation.h"
#include <random>

class Sensor_uWave
{
    Point3D position, velocity;
    ReaderJson& config;
    default_random_engine gen;
    normal_distribution<double> noise_dist;

    bool is_mobile = false;

public:
    Sensor_uWave(const Point3D& pos, ReaderJson& cfg)
        : position(pos), config(cfg),
        noise_dist(config.get<double>("sensor.noise_mean"),
                   config.get<double>("sensor.noise_stddev")) {}

    void update_position(double dt) {
        if(!is_mobile) return;
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;
        position.z += velocity.z * dt;
    }

    void update_position_fromAUV(double x, double y, double z) {
        position.x = x;
        position.y = y;
        position.z = z;
    }

    double getHydroacousticDistData() const {
        return dist;
    }

    vector<double> receive_signal(const RayPath& selected_path) {
        vector<double> delays;

        // Добавляем шум к задержке выбранного пути
        double distance = selected_path.delay*config.get<double>("sound_speed") + noise_dist(gen);
        delays.push_back(distance);

        return delays;
    }

    const Point3D& get_position() const { return position; }
    void set_mobility(bool mobile) { is_mobile = mobile; }
    void set_velocity(const Point3D& vel) { velocity = vel; }
    double dist = 0; //дистанция между модемами
};

#endif // SENSOR_UWAVE_H
