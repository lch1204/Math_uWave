#include "signalpropagation.h"
#include <QDebug>
#include <cmath>
#include <iostream>
#include <ostream>
#include <qt5/QtCore/qglobal.h>
#include <random>

// Расстояние между двумя точками
double SignalPropagation::distance(const Point3D& a, const Point3D& b) const {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

// Угловой фактор отражения
double SignalPropagation::angular_factor(const Point3D& incident, const Obstacle& obs) const {
    Point3D normal;

    // Определяем нормаль в зависимости от типа поверхности
    if (obs.surface_type == "water_surface") {
        normal = {0, 0, -1}; // Нормаль вниз для поверхности воды
    } else if (obs.surface_type == "silt" || obs.surface_type == "sand") {
        normal = {0, 0, 1}; // Нормаль вверх для дна
    } else if (obs.surface_type == "aquarium_wall") {
        if (obs.vertices[0].x == obs.vertices[1].x) {
            normal = {1, 0, 0}; // Нормаль для вертикальных стенок по X
        } else {
            normal = {0, 1, 0}; // Нормаль для вертикальных стенок по Y
        }
    }

    // Вычисляем косинус угла между направлением сигнала и нормалью
    double dot = incident.x * normal.x + incident.y * normal.y + incident.z * normal.z;
    double mag = sqrt(incident.x * incident.x + incident.y * incident.y + incident.z * incident.z);
    qDebug() << "incident.x " << incident.x << "normal.x " << normal.x ;
    qDebug() << "dot" << dot << "mag" << mag;
    return std::max(0.0, dot / mag); // Угловой фактор не может быть отрицательным
}

// Основной метод для расчета путей сигнала
void SignalPropagation::calculate_paths(const Point3D& tx, const Point3D& rx, const vector<Obstacle>& obstacles) {
    multipaths.clear();

    // Проверяем, есть ли прямой путь (Line of Sight, LOS)
    bool has_los = true;
    for (const auto& obs : obstacles) {
        if (line_intersects_obstacle(tx, rx, obs)) {
            has_los = false;
            break;
        }
    }

    // Если прямой путь есть, добавляем его в список
    if (has_los) {
        double distance_los = distance(tx, rx);
        multipaths.push_back({
            {tx, rx}, // Путь (точки)
            1.0,      // Затухание (прямой путь без потерь)
            distance_los / config.get<double>("sound_speed"), // Задержка
            1.0       // Вес (прямой путь имеет максимальный вес)
        });
    }
    // std::cout <<" has_los = "<< has_los << std::endl;


    // Рассчитываем отраженные пути
    for (const auto& obs : obstacles) {
        // Находим точку отражения
        Point3D reflection_point = calculate_reflection_point(tx, rx, obs);
        qDebug() <<"fff";

        // Проверяем, есть ли LOS от передатчика до точки отражения и от точки отражения до приемника
        bool has_los_to_reflection = true;
        bool has_los_from_reflection = true;

        for (const auto& other_obs : obstacles) {
            if (line_intersects_obstacle(tx, reflection_point, other_obs)) {
                has_los_to_reflection = false;
                break;
            }
            if (line_intersects_obstacle(reflection_point, rx, other_obs)) {
                has_los_from_reflection = false;
                break;
            }
        }

        // std::cout <<" has_los_to_reflection = "<< has_los_to_reflection << std::endl;
        // std::cout <<" has_los_from_reflection = "<< has_los_from_reflection << std::endl;

        // Если путь свободен, добавляем отраженный путь
        if (has_los_to_reflection && has_los_from_reflection) {
            double distance_to_reflection = distance(tx, reflection_point);
            double distance_from_reflection = distance(reflection_point, rx);
            double total_distance = distance_to_reflection + distance_from_reflection;

            // Угловой фактор и затухание
            qDebug() << "reflection_point.x" << reflection_point.x << "reflection_point.y" << reflection_point.y << "reflection_point.z " << reflection_point.z ;
            Point3D incident_dir = {reflection_point.x - tx.x, reflection_point.y - tx.y, reflection_point.z - tx.z};
            double angle_factor = angular_factor(incident_dir, obs);
            qDebug() << "angle_factor" << angle_factor;
            double attenuation = pow(obs.reflection_loss, 1) * angle_factor;
            // double attenuation = 0.5;

            // Добавляем путь в список
            multipaths.push_back({
                {tx, reflection_point, rx}, // Путь (точки)
                attenuation,                // Затухание
                total_distance / config.get<double>("sound_speed"), // Задержка
                attenuation                 // Вес
            });
            qDebug() << "multipaths.push_back attenuation" << attenuation;
        }
    }
}

// Вычисление точки отражения
Point3D SignalPropagation::calculate_reflection_point(const Point3D& tx, const Point3D& rx, const Obstacle& obs) {
    // Для горизонтальных поверхностей (дно, поверхность воды)
    if (obs.surface_type == "water_surface" || obs.surface_type == "silt" || obs.surface_type == "sand") {
        double surface_z = obs.vertices[0].z;
        double rx_dist = rx.z - surface_z;
        qDebug() << "rx.x" << rx.x << "rx.y" << rx.y << "surface_z - rx_dist"  << surface_z - rx_dist;
        return {rx.x, rx.y, surface_z - rx_dist}; // Зеркальное отражение
    }

    // Для вертикальных стенок
    Point3D normal = calculate_surface_normal(obs);
    Point3D intersection = {obs.vertices[0].x, obs.vertices[0].y, obs.vertices[0].z}; // Упрощенное пересечение
    return reflect_point(rx, intersection, normal); // Отражение точки
}

// Проверка пересечения линии с препятствием
bool SignalPropagation::line_intersects_obstacle(const Point3D& p1, const Point3D& p2, const Obstacle& obs) {
    // Упрощенная проверка для горизонтальных и вертикальных поверхностей
    if (obs.surface_type == "water_surface" || obs.surface_type == "silt" || obs.surface_type == "sand") {
        return (p1.z > obs.vertices[0].z && p2.z < obs.vertices[0].z) ||
               (p1.z < obs.vertices[0].z && p2.z > obs.vertices[0].z);
    } else if (obs.surface_type == "aquarium_wall") {
        return (p1.x > obs.vertices[0].x && p2.x < obs.vertices[0].x) ||
               (p1.x < obs.vertices[0].x && p2.x > obs.vertices[0].x);
    }
    return false;
}

// Отражение точки относительно плоскости
Point3D SignalPropagation::reflect_point(const Point3D& point, const Point3D& surface_point, const Point3D& normal) {
    Point3D v = point - surface_point;
    double d = 2 * (v.x * normal.x + v.y * normal.y + v.z * normal.z);
    return {
        point.x - d * normal.x,
        point.y - d * normal.y,
        point.z - d * normal.z
    };
}

// Вычисление нормали к поверхности
Point3D SignalPropagation::calculate_surface_normal(const Obstacle& obs) {
    if (obs.surface_type == "water_surface" || obs.surface_type == "silt" || obs.surface_type == "sand") {
        return {0, 0, 1}; // Нормаль вверх для горизонтальных поверхностей
    } else if (obs.surface_type == "aquarium_wall") {
        return {1, 0, 0}; // Нормаль для вертикальных стенок
    }
    return {0, 0, 1}; // По умолчанию
}

const RayPath *SignalPropagation::selectPath() const {
    if(multipaths.empty()) return nullptr;
    qDebug() << "multipaths.size()"<< multipaths.size();

    // Суммируем все веса
    double total_weight = 0.0;
    for(const auto& path : multipaths) {
        total_weight += path.weight;
    }

    // Генерируем случайное число
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    double r = dis(gen) * total_weight;

    // Выбираем подходящий путь
    double cumulative = 0.0;
    for(const auto& path : multipaths) {
        cumulative += path.weight;
        if(r <= cumulative) {
            return &path;
        }
    }

    return &multipaths.back(); // fallback
}
