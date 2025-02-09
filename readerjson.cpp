#include "readerjson.h"
#include <fstream>

using namespace std;

ReaderJson::ReaderJson() {}

void ReaderJson::load(const string &filename) {
    std::ifstream file(filename);
    file >> config;

    reflection_loss_table = {
        {"concrete", 0.85},
        {"silt", 0.4},
        {"sand", 0.65},
        {"water_surface", 0.15},
        {"metal", 0.95},
        {"vegetation", 0.3}
    };

    if(config.contains("aquarium_walls")) {
        auto& walls_cfg = config["aquarium_walls"];
        walls.enabled = walls_cfg["enabled"];
        walls.reflection_loss = walls_cfg.value("reflection_loss", 0.7);
        walls.front_z = walls_cfg.value("front_z", 100.0);
        walls.back_z = walls_cfg.value("back_z", -100.0);
        walls.left_x = walls_cfg.value("left_x", -100.0);
        walls.right_x = walls_cfg.value("right_x", 100.0);
    }
}

vector<Obstacle> ReaderJson::get_obstacles() const {
    vector<Obstacle> obstacles;
    const double INF = numeric_limits<double>::infinity();

    // Пользовательские препятствия
    for (const auto& obs : config["obstacles"]) {
        vector<Point3D> vertices;
        for (const auto& coord : obs["coordinates"]) {
            vertices.emplace_back(
                coord[0].get<double>(), // X
                coord[1].get<double>(), // Y
                coord[2].get<double>()  // Z
                );
        }

        obstacles.emplace_back(
            vertices,
            obs["surface_type"].get<std::string>(),
            obs.value("obstacle_type", "generic"),
            obs.value("reflection_loss", get_reflection_loss(obs["surface_type"]))
            );
    }

    // Стенки акватории
    if (walls.enabled) {
        const auto add_wall = [&](std::vector<Point3D> vertices) {
            obstacles.emplace_back(
                vertices,
                "aquarium_wall",
                "vertical",
                walls.reflection_loss
                );
        };

        // Передняя и задняя стенки (по Z)
        add_wall({
            {walls.left_x, -INF, walls.front_z},
            {walls.right_x, INF, walls.front_z}
        });
        add_wall({
            {walls.left_x, -INF, walls.back_z},
            {walls.right_x, INF, walls.back_z}
        });

        // Левая и правая стенки (по X)
        add_wall({
            {walls.left_x, -INF, walls.back_z},
            {walls.left_x, INF, walls.front_z}
        });
        add_wall({
            {walls.right_x, -INF, walls.back_z},
            {walls.right_x, INF, walls.front_z}
        });
    }

    return obstacles;
}

double ReaderJson::get_reflection_loss(const string &surface_type) const
{
    auto it = reflection_loss_table.find(surface_type);
    return it != reflection_loss_table.end() ? it->second : 0.5;
}

vector<Obstacle> ReaderJson::get_aquarium_obstacles() const {
    vector<Obstacle> obstacles;

    // Константа для бесконечности
    const double INF = std::numeric_limits<double>::infinity();

    // Дно
    obstacles.emplace_back(
        vector<Point3D>{
            { -INF, -INF, config["aquarium"]["depth"] },
            {  INF,  INF, config["aquarium"]["depth"] }
        },
        config["aquarium"]["bottom_type"].get<string>(),
        "horizontal",
        get_reflection_loss(config["aquarium"]["bottom_type"].get<string>())
        );

    // Поверхность воды
    obstacles.emplace_back(
        vector<Point3D>{
            { -INF, -INF, config["aquarium"]["surface_z"] },
            {  INF,  INF, config["aquarium"]["surface_z"] }
        },
        "water_surface",
        "horizontal",
        get_reflection_loss("water_surface")
        );

    return obstacles;
}

