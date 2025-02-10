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
        walls.front_y = walls_cfg.value("front_y", 100.0);
        walls.back_y = walls_cfg.value("back_y", -100.0);
        walls.left_x = walls_cfg.value("left_x", -100.0);
        walls.right_x = walls_cfg.value("right_x", 100.0);
        walls.bottom_z = walls_cfg.value("bottom_z", 100.0);
        walls.top_z = walls_cfg.value("top_z", 0.0);
    }
}

vector<Obstacle> ReaderJson::get_obstacles() const {
    vector<Obstacle> obstacles;
    const double INF = numeric_limits<double>::infinity();

    // 1. Добавляем дно
    if(config.contains("aquarium")) {
        obstacles.emplace_back(
            vector<Point3D>{
                {-INF, -INF, config["aquarium"]["depth"].get<double>()},
                { INF,  INF, config["aquarium"]["depth"].get<double>()}
            },
            config["aquarium"]["bottom_type"].get<string>(),
            "horizontal",  // Тип препятствия
            get_reflection_loss(config["aquarium"]["bottom_type"].get<string>())
            );
    }

    // 2. Поверхность воды
    obstacles.emplace_back(
        vector<Point3D>{
            { -INF, -INF, config["aquarium"]["surface_z"] },
            {  INF,  INF, config["aquarium"]["surface_z"] }
        },
        "water_surface",
        "horizontal",
        get_reflection_loss("water_surface")
        );

    // 3. Стенки акватории (вертикальные по осям X и Y)
    if (walls.enabled) {
        const auto add_wall = [&](vector<Point3D> vertices, string surface) {
            obstacles.emplace_back(
                vertices,
                surface,
                "vertical", // Тип препятствия
                walls.reflection_loss // Коэффициент отражения
                );
        };

        // Стенки по оси X (левая и правая)
        add_wall(
            {{walls.left_x, -INF, walls.bottom_z}, {walls.left_x, INF, walls.top_z}}, // Левая стенка
            "left_wall"
            );
        add_wall(
            {{walls.right_x, -INF, walls.bottom_z}, {walls.right_x, INF, walls.top_z}}, // Правая стенка
            "right_wall"
            );

        // Стенки по оси Y (передняя и задняя)
        add_wall(
            {{-INF, walls.front_y, walls.bottom_z}, {INF, walls.front_y, walls.top_z}}, // Передняя стенка
            "front_wall"
            );
        add_wall(
            {{-INF, walls.back_y, walls.bottom_z}, {INF, walls.back_y, walls.top_z}}, // Задняя стенка
            "back_wall"
            );
    }

    return obstacles;
}
double ReaderJson::get_reflection_loss(const string &surface_type) const
{
    auto it = reflection_loss_table.find(surface_type);
    return it != reflection_loss_table.end() ? it->second : 0.5;
}

