#include "signalpropagation.h"
#include <QDebug>
#include <cmath>
#include <iostream>
#include <qt5/QtCore/qglobal.h>
#include <random>

// Расстояние между двумя точками
double SignalPropagation::distance(const Point3D& a, const Point3D& b) const {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

// Угловой фактор отражения
double SignalPropagation::angular_factor(const Point3D &incident, const Obstacle &obs) const {
    if (std::isnan(incident.x)) {
        qDebug() << "Некорректное направление падения!";
        return 0.0;
    }

    Point3D normal ;
        calculate_surface_normal(obs);
    double dot = incident.x * normal.x + incident.y * normal.y + incident.z * normal.z;
    double mag = sqrt(pow(incident.x, 2) + pow(incident.y, 2) + pow(incident.z, 2));

    return std::max(0.0, dot / mag);
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

        // Проверяем, есть ли LOS от передатчика до точки отражения и от точки отражения до приемника
        bool has_los_to_reflection = true;

        // В цикле проверки LOS:
        for (const auto& other_obs : obstacles) {
            // Пропускаем текущее препятствие (отражение от него уже учтено)
            if (&other_obs == &obs) continue;

            // Проверка валидности reflection_point
            if (std::isnan(reflection_point.x)) {
                has_los_to_reflection = false;
                break;
            }
        }

        // Если путь свободен, добавляем отраженный путь
        if (has_los_to_reflection) {
            double distance_to_reflection = distance(tx, reflection_point);
            double distance_from_reflection = distance(reflection_point, rx);
            double total_distance =distance_to_reflection+distance_from_reflection;
            Point3D norm = calculate_surface_normal(obs);
            // Угловой фактор и затухание
            double angle_factor = calculate_angle_cosine(tx, reflection_point, norm);
            // qDebug() << "angle_factor" << angle_factor;
            double attenuation = abs(pow(obs.reflection_loss, 1) * angle_factor);

            // Добавляем путь в список
            multipaths.push_back({
                {tx, reflection_point, rx}, // Путь (точки)
                attenuation,                // Затухание
                total_distance / config.get<double>("sound_speed"), // Задержка
                attenuation                 // Вес
            });
            // qDebug() << "multipaths.push_back attenuation" << attenuation;
        }
    }
}

// Вычисление точки отражения
Point3D SignalPropagation::calculate_reflection_point(const Point3D& tx, const Point3D& rx, const Obstacle& obs) {
    // Проверка на валидность вершин препятствия
    if (obs.vertices.empty()) {
        qDebug() << "Obstacle has no vertices!";
        return {NAN, NAN, NAN};
    }

    // Для горизонтальных поверхностей (дно, поверхность воды)
    if (obs.surface_type == "water_surface" || obs.surface_type == "silt" || obs.surface_type == "sand")
    {
        return calculate_reflection_point_horizontal(tx,rx,obs.vertices[0].z);
    }

    // Для вертикальных стенок акватории
    if (obs.surface_type == "aquarium_wall") {
        // // Убедимся, что препятствие имеет корректный тип
        if (obs.obstacle_type.empty()) {
            qDebug() << "Unknown obstacle type!";
            return {NAN, NAN, NAN};
        }
        return calculate_reflection_point_vertical(tx, rx, obs);
    }

    // Если тип препятствия неизвестен
    qDebug() << "Unknown surface type!";
    return {NAN, NAN, NAN};
}

// Проверка пересечения линии с препятствием
bool SignalPropagation::line_intersects_obstacle(const Point3D& p1, const Point3D& p2, const Obstacle& obs) {
    // Если точки невалидны, пересечения нет
    if (std::isnan(p1.x)) return false;

    // Для горизонтальных поверхностей
    if (obs.surface_type == "water_surface" || obs.surface_type == "silt") {
        double surface_z = obs.vertices[0].z;
        return (p1.z - surface_z) * (p2.z - surface_z) < 0;
    }

    // Для вертикальных стенок
    if (obs.surface_type == "aquarium_wall") {
        // Определяем границы стены
        double wall_min, wall_max, axis_val;
        if (obs.obstacle_type == "left_wall" || obs.obstacle_type == "right_wall") {
            wall_min = std::min(obs.vertices[0].y, obs.vertices[1].y);
            wall_max = std::max(obs.vertices[0].y, obs.vertices[1].y);
            axis_val = obs.vertices[0].x;
            // Проверка пересечения по X
            if ((p1.x - axis_val) * (p2.x - axis_val) >= 0) return false;
            // Расчет точки пересечения
            double t = (axis_val - p1.x) / (p2.x - p1.x);
            double y_intersect = p1.y + t * (p2.y - p1.y);
            return (y_intersect >= wall_min && y_intersect <= wall_max);
        } else if (obs.obstacle_type == "front_wall" || obs.obstacle_type == "back_wall") {
            wall_min = std::min(obs.vertices[0].x, obs.vertices[1].x);
            wall_max = std::max(obs.vertices[0].x, obs.vertices[1].x);
            axis_val = obs.vertices[0].y;
            // Проверка пересечения по Y
            if ((p1.y - axis_val) * (p2.y - axis_val) >= 0) return false;
            // Расчет точки пересечения
            double t = (axis_val - p1.y) / (p2.y - p1.y);
            double x_intersect = p1.x + t * (p2.x - p1.x);
            return (x_intersect >= wall_min && x_intersect <= wall_max);
        }
    }

    return false;
}

// Отражение точки относительно плоскости
Point3D SignalPropagation::reflect_point(const Point3D& point, const Point3D& surface_point, const Point3D& normal) {
    Point3D v = point - surface_point;
    double d = 2 * (v.x * normal.x + v.y * normal.y + v.z * normal.z);
    qDebug() << "reflect_point -"<< "point.x - d * normal.x" << point.x - d * normal.x;
    return {
        point.x - d * normal.x,
        point.y - d * normal.y,
        point.z - d * normal.z
    };
}

Point3D SignalPropagation::calculate_reflection_point_horizontal(const Point3D &tx, const Point3D &rx, double surface_z)
{
    // 1. Зеркально отражаем приемник относительно поверхности
    Point3D mirrored_rx = rx;
    mirrored_rx.z = 2 * surface_z - rx.z;

    // 2. Находим параметр t пересечения луча tx -> mirrored_rx с поверхностью z=surface_z
    double dz = mirrored_rx.z - tx.z;

    // Если луч параллелен поверхности (dz == 0), отражение невозможно
    if (std::abs(dz) < 1e-6) {
        return {NAN, NAN, NAN};
    }

    double t = (surface_z - tx.z) / dz;

    // 3. Вычисляем координаты точки пересечения
    double x = tx.x + t * (mirrored_rx.x - tx.x);
    double y = tx.y + t * (mirrored_rx.y - tx.y);
    double z = surface_z;
    qDebug() <<"point_horizontal x" << x <<"y" <<y <<"z"<< z;

    return {x, y, z};
}

double SignalPropagation::calculate_angle_cosine(const Point3D &tx, const Point3D &point_reflection, const Point3D &surface_normal)
{
    // 1. Вычисляем вектор направления от tx к точке отражения
    Point3D dir = {
        point_reflection.x - tx.x,
        point_reflection.y - tx.y,
        point_reflection.z - tx.z
    };

    // 2. Вычисляем длину вектора
    double length = sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    qDebug() << "length" << length;

    // Если точки совпадают или вектор нулевой, возвращаем NaN
    if (length < 1e-6) {
        return NAN;
    }

    // 3. Нормализуем вектор
    dir.x = dir.x / length;
    dir.y = dir.y / length;
    dir.z = dir.z / length;

    // 4. Нормализуем нормаль к поверхности
    double normal_length = sqrt(surface_normal.x * surface_normal.x +
                                surface_normal.y * surface_normal.y +
                                surface_normal.z * surface_normal.z);
    Point3D normal = {
        surface_normal.x / normal_length,
        surface_normal.y / normal_length,
        surface_normal.z / normal_length
    };

    // 5. Вычисляем косинус угла между направлением и нормалью
    double cosine = dir.x * normal.x + dir.y * normal.y + dir.z * normal.z;
    cosine = sqrt(1 - pow(cosine,2));
    if (dir.z<0) cosine = -cosine;

    return cosine;
}

Point3D SignalPropagation::calculate_reflection_point_vertical(const Point3D &tx, const Point3D &rx, const Obstacle &obs)
{
    // 1. Определяем параметры стены
    double wall_pos;
    char reflection_axis; // 'x' или 'y'
    bool is_positive_side;

    if (obs.obstacle_type == "left_wall" || obs.obstacle_type == "right_wall") {
        wall_pos = obs.vertices[0].x;
        reflection_axis = 'x';
        is_positive_side = (obs.obstacle_type == "right_wall");
    }
    else if (obs.obstacle_type == "front_wall" || obs.obstacle_type == "back_wall") {
        wall_pos = obs.vertices[0].y;
        reflection_axis = 'y';
        is_positive_side = (obs.obstacle_type == "front_wall");
    }
    else {
        return {NAN, NAN, NAN};
    }

    // 2. Зеркальное отражение приемника
    Point3D mirrored_rx = rx;
    if (reflection_axis == 'x') {
        mirrored_rx.x = 2 * wall_pos - rx.x;
    } else {
        mirrored_rx.y = 2 * wall_pos - rx.y;
    }

    // 3. Находим параметр t пересечения луча tx -> mirrored_rx со стеной
    double delta;
    if (reflection_axis == 'x') {
        delta = mirrored_rx.x - tx.x;
        if (fabs(delta) < 1e-6) return {NAN, NAN, NAN};
        double t = (wall_pos - tx.x) / delta;
        if (t < 0 || t > 1) return {NAN, NAN, NAN};
        qDebug() <<"XВычисляем координаты точки пересечения x"<< wall_pos <<"y" << tx.y + t * (mirrored_rx.y - tx.y) << "z" << tx.z + t * (mirrored_rx.z - tx.z);
        // 4. Вычисляем координаты точки пересечения
        return {
            wall_pos,
            tx.y + t * (mirrored_rx.y - tx.y),
            tx.z + t * (mirrored_rx.z - tx.z)
        };
    }
    else {
        delta = mirrored_rx.y - tx.y;
        if (fabs(delta) < 1e-6) return {NAN, NAN, NAN};
        double t = (wall_pos - tx.y) / delta;
        if (t < 0 || t > 1) return {NAN, NAN, NAN};
        qDebug() <<"Y Вычисляем координаты точки пересечения x"<< tx.x + t * (mirrored_rx.x - tx.x)
                 <<"y" << wall_pos << "z" << tx.z + t * (mirrored_rx.z - tx.z);
        return {
            tx.x + t * (mirrored_rx.x - tx.x),
            wall_pos,
            tx.z + t * (mirrored_rx.z - tx.z)
        };
    }
}

// Вычисление нормали к поверхности
Point3D SignalPropagation::calculate_surface_normal(const Obstacle &obs) const {
    if (obs.surface_type == "aquarium_wall") {
        if (obs.obstacle_type == "left_wall") {
            return {1, 0, 0}; // Нормаль вправо для левой стенки
        } else if (obs.obstacle_type == "right_wall") {
            return {-1, 0, 0}; // Нормаль влево для правой стенки
        } else if (obs.obstacle_type == "front_wall") {
            return {0, -1, 0}; // Нормаль назад для передней стенки
        } else if (obs.obstacle_type == "back_wall") {
            return {0, 1, 0}; // Нормаль вперед для задней стенки
        }
    }
    // Для горизонтальных поверхностей
    return {0, 0, 1};
}

const RayPath *SignalPropagation::selectPath() const {
    if(multipaths.empty()) return nullptr;
    qDebug() << "multipaths.size()"<< multipaths.size();

    // Суммируем все веса
    double total_weight = 0.0;
    for(const auto& path : multipaths) {
        qDebug() << "weight" << path.weight;
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
