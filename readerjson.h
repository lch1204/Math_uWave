#ifndef READERJSON_H
#define READERJSON_H

#include "include/nlohmann/json_fwd.hpp"
#include "stucturs.h"
#include <nlohmann/json.hpp>
#include <vector>
#include <unordered_map>
#include <cmath>


using namespace std;

class ReaderJson
{

public:
    ReaderJson();

    nlohmann::json config;
    unordered_map<string, double> reflection_loss_table;

    template<typename T>
    T get(const std::string& key) const {
        const nlohmann::json* current = &config;
        std::istringstream iss(key);
        std::string token;

        while (std::getline(iss, token, '.')) {
            if (!current->contains(token)) {
                throw std::runtime_error("Key not found in config: " + key);
            }
            current = &(*current)[token];
        }

        return current->get<T>();
    }

    struct AquariumWalls {
        bool enabled = false; //стены бассейна false - отключены, иначе учитываются
        double front_z = 100.0;
        double back_z = -100.0;
        double left_x = -100.0;
        double right_x = 100.0;
        double reflection_loss = 0.7;
    } walls;

    void load(const string& filename);



    vector<Obstacle> get_obstacles() const;

    double get_reflection_loss(const string& surface_type) const;

    vector<Obstacle> get_aquarium_obstacles() const;
};

#endif // READERJSON_H
