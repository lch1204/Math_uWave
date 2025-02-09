#include "signalpropagation.h"
#include "qglobal.h"
#include <QDebug>


double SignalPropagation::distance(const Point3D &a, const Point3D &b) const {
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2) + pow(a.z-b.z, 2));
}

double SignalPropagation::angular_factor(const Point3D &incident, const Obstacle &obs) const {
    Point3D normal;
    // Определение нормали для стенок акватории
    if(obs.surface_type == "aquarium_wall") {
        if(obs.vertices[0].x == obs.vertices[1].x) { // Вертикальные стенки
            normal = {static_cast<double>(obs.vertices[0].x > 0 ? -1 : 1), 0, 0};
        } else { // Горизонтальные стенки
            normal = {0, 0, static_cast<double>(obs.vertices[0].z > 0 ? -1 : 1)};
        }
    } else {
        if (obs.surface_type == "water_surface") {
            normal = {0, 0, -1}; // Нормаль вниз (сигнал идет из воды в воздух)
        } else if (obs.surface_type == "silt" || obs.surface_type == "sand") {
            normal = {0, 0, 1}; // Нормаль вверх (сигнал отражается от дна)
        } else {
            // Для других препятствий используем вертикальную нормаль
            normal = {0, 1, 0};
        }
    }

    double dot = incident.x*normal.x + incident.y*normal.y + incident.z*normal.z;
    double mag = sqrt(incident.x*incident.x + incident.y*incident.y + incident.z*incident.z);
    return max(0.0, dot/mag);
}

void SignalPropagation::calculate_paths(const Point3D &tx, const Point3D &rx, const vector<Obstacle> &obstacles) {
    multipaths.clear();

    // Прямой путь (Line of Sight, LOS)
    bool has_los = true; //предполагаем, что есть прямой путь
    for (const auto& obs : obstacles) {
        if (line_intersects_obstacle(tx, rx, obs)) {
            has_los = false;
            break;
        }
    }

    if (has_los) {
        double distance_los = distance(tx, rx);
        multipaths.push_back({
            {tx, rx}, // Путь (точки)
            1.0,      // Затухание (прямой путь без потерь)
            distance_los / config.get<double>("sound_speed"), // Задержка
            1.0       // Вес (прямой путь имеет максимальный вес)
        });
    }

    // Отраженные пути
    for (const auto& obs : obstacles) {
        // Рассчитываем точку отражения (упрощенная модель)
        Point3D reflection_point = calculate_reflection_point(tx, rx, obs);

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

        if (has_los_to_reflection && has_los_from_reflection) {
            // Длина пути до отражения и от отражения до приемника
            double distance_to_reflection = distance(tx, reflection_point);
            double distance_from_reflection = distance(reflection_point, rx);
            double total_distance = distance_to_reflection + distance_from_reflection;

            // Угловой фактор
            Point3D incident_dir = {reflection_point.x - tx.x,
                                    reflection_point.y - tx.y,
                                    reflection_point.z - tx.z};
            double angle_factor = angular_factor(incident_dir, obs);

            // Затухание сигнала
            double reflection_loss = obs.reflection_loss;
            double attenuation = pow(reflection_loss, 1) * angle_factor;

            // Вес пути
            double weight = attenuation;
            // Добавляем путь в список
            multipaths.push_back({
                {tx, reflection_point, rx}, // Путь (точки)
                attenuation,                // Затухание
                total_distance / config.get<double>("sound_speed"), // Задержка
                weight                      // Вес
            });
            qDebug() << "weight" << weight;
        }
    }
}

Point3D SignalPropagation::calculate_reflection_point(const Point3D& tx,
                                                      const Point3D& rx,
                                                      const Obstacle& obs) {
    // Для горизонтальных поверхностей
    if (obs.obstacle_type == "horizontal") {
        double surface_z = obs.vertices[0].z;
        double tx_dist = tx.z - surface_z;
        double rx_dist = rx.z - surface_z;
        return {rx.x, rx.y, surface_z - rx_dist};
    }

    // Для вертикальных стенок
    Point3D normal = calculate_surface_normal(obs);
    Point3D intersection = find_ray_intersection(tx, rx, obs);
    return reflect_point(rx, intersection, normal);
}


bool SignalPropagation::line_intersects_obstacle(const Point3D &p1, const Point3D &p2, const Obstacle &obs) {
    // Преобразование вершин препятствия в AABB
    Point3D min_point = obs.vertices[0];
    Point3D max_point = obs.vertices[0];
    //поиск минимального и максимального значения по осям
    for (const auto& v : obs.vertices) {
        min_point.x = std::min(min_point.x, v.x);
        min_point.y = std::min(min_point.y, v.y);
        min_point.z = std::min(min_point.z, v.z);
        max_point.x = std::max(max_point.x, v.x);
        max_point.y = std::max(max_point.y, v.y);
        max_point.z = std::max(max_point.z, v.z);
    }
    return line_intersects_cube(p1, p2, min_point, max_point);
}

bool SignalPropagation::line_intersects_cube(const Point3D &p1, const Point3D &p2, const Point3D &cube_min, const Point3D &cube_max) {
    // Алгоритм пересечения луча и AABB (Slab Method)
    double tmin = 0.0;
    double tmax = 1.0;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;

    // Проверка для каждой оси
    for (int i = 0; i < 3; ++i) {
        double inv_dir;
        double t0, t1;

        if (i == 0) { // X-axis
            inv_dir = 1.0 / dx;
            t0 = (cube_min.x - p1.x) * inv_dir;
            t1 = (cube_max.x - p1.x) * inv_dir;
        } else if (i == 1) { // Y-axis
            inv_dir = 1.0 / dy;
            t0 = (cube_min.y - p1.y) * inv_dir;
            t1 = (cube_max.y - p1.y) * inv_dir;
        } else { // Z-axis
            inv_dir = 1.0 / dz;
            t0 = (cube_min.z - p1.z) * inv_dir;
            t1 = (cube_max.z - p1.z) * inv_dir;
        }

        if (inv_dir < 0.0) std::swap(t0, t1); //если движение от x2 к x1 то границы инвертируются

        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);

        if (tmin > tmax) return false;
    }

    // Проверка, что пересечение в пределах отрезка [p1, p2]
    return (tmin <= 1.0 && tmax >= 0.0);
}

Point3D SignalPropagation::reflect_point(const Point3D &point, const Point3D &surface_point, const Point3D &normal) {
    Point3D v = point - surface_point;
    double d = 2 * (v.x*normal.x + v.y*normal.y + v.z*normal.z);
    return {
        point.x - d*normal.x,
        point.y - d*normal.y,
        point.z - d*normal.z
    };
}

Point3D SignalPropagation::find_ray_intersection(const Point3D &tx,
                                                 const Point3D &rx,
                                                 const Obstacle &obs) {
    // Получаем нормаль и точку на плоскости препятствия
    Point3D normal = calculate_surface_normal(obs);
    Point3D point_on_plane = obs.vertices[0]; // Берем первую вершину как точку на плоскости

    // Уравнение плоскости: normal ⋅ (X - point_on_plane) = 0
    // Перепишем в виде: normal ⋅ X = normal ⋅ point_on_plane
    double plane_d = normal.x * point_on_plane.x
                     + normal.y * point_on_plane.y
                     + normal.z * point_on_plane.z;

    // Вычисляем параметр t для луча tx -> rx
    Point3D dir = rx - tx;
    double denominator = normal.x * dir.x + normal.y * dir.y + normal.z * dir.z;

    if (fabs(denominator) < 1e-6) {
        // Луч параллелен плоскости, пересечения нет
        return tx; // Возвращаем заглушку
    }

    double t = (plane_d - (normal.x * tx.x + normal.y * tx.y + normal.z * tx.z)) / denominator;
    return tx + dir * t;
}

Point3D SignalPropagation::calculate_surface_normal(const Obstacle &obs) {
    if (obs.obstacle_type == "horizontal") {
        return {0, 0, 1}; // Нормаль вверх для горизонтальных поверхностей
    }
    else if (obs.obstacle_type == "vertical_x") {
        return {1, 0, 0}; // Нормаль для стенок по X
    }
    else if (obs.obstacle_type == "vertical_y") {
        return {0, 1, 0}; // Нормаль для стенок по Y
    }
    else {
        // Для произвольных поверхностей вычисляем нормаль через векторное произведение
        if (obs.vertices.size() >= 3) {
            Point3D v1 = obs.vertices[1] - obs.vertices[0];
            Point3D v2 = obs.vertices[2] - obs.vertices[0];
            return {
                v1.y * v2.z - v1.z * v2.y,
                v1.z * v2.x - v1.x * v2.z,
                v1.x * v2.y - v1.y * v2.x
            };
        }
        return {0, 0, 1}; // Значение по умолчанию
    }
}
