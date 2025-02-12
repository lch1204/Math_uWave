#ifndef SIGNALPROPAGATION_H
#define SIGNALPROPAGATION_H

#include "stucturs.h"
#include "readerjson.h"

// Модель распространения сигнала
class SignalPropagation {
    vector<RayPath> multipaths; // Список путей сигнала
    ReaderJson& config;         // Конфигурация из JSON

public:
    SignalPropagation(ReaderJson& cfg) : config(cfg) {}

    // Основной метод для расчета путей сигнала
    void calculate_paths(const Point3D& tx, const Point3D& rx, const vector<Obstacle>& obstacles);

    // Возвращает список путей
    const vector<RayPath>& get_paths() const { return multipaths; }
    const RayPath* selectPath() const; // Выбор пути на основе весов
    Point3D calculate_surface_normal(const Obstacle& obs) const; // Вычисление нормали к поверхности
private:
    // Вспомогательные методы
    double distance(const Point3D& a, const Point3D& b) const; // Расстояние между двумя точками
    double angular_factor(const Point3D& incident, const Obstacle& obs) const; // Угловой фактор отражения
    Point3D calculate_reflection_point(const Point3D& tx, const Point3D& rx, const Obstacle& obs); // Точка отражения
    bool line_intersects_obstacle(const Point3D& p1, const Point3D& p2, const Obstacle& obs); // Проверка пересечения с препятствием
    Point3D reflect_point(const Point3D& point, const Point3D& surface_point, const Point3D& normal); // Отражение точки
    Point3D calculate_reflection_point_horizontal(const Point3D& tx, const Point3D& rx, double surface_z);
    double calculate_angle_cosine(const Point3D& tx, const Point3D& point_reflection, const Point3D &surface_normal); //Вычисляет косинус угла между направлением от tx к точке отражения и вертикальной осью (ось Z).
    Point3D calculate_reflection_point_vertical(const Point3D& tx, const Point3D& rx, const Obstacle& obs);

};

#endif // SIGNALPROPAGATION_H
