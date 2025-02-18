#ifndef STUCTURS_H
#define STUCTURS_H

#include <string>
#include <vector>

using namespace std;

//Содержит все структуры данных

struct HydroMeasurement {
    double distance;
    double timestamp;
};

struct Point3D {
    double x, y, z;
    bool operator!=(const Point3D& other) const
    {
        return x != other.x || y != other.y || z != other.z;
    } //возвращает сравнение двух точек, равные ли у них координаты

    Point3D() : x(0), y(0), z(0) {}

    Point3D(double x_, double y_, double z_)
        : x(x_), y(y_), z(z_) {}
};

// Арифметические операторы для Point3D

inline Point3D operator+(const Point3D &a, const Point3D &b) {
    return Point3D(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Point3D operator-(const Point3D &a, const Point3D &b) {
    return Point3D(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Point3D operator*(const Point3D &p, double scalar) {
    return Point3D(p.x * scalar, p.y * scalar, p.z * scalar);
}

inline Point3D operator*(double scalar, const Point3D &p) {
    return p * scalar;
}

//препятствие
struct Obstacle {
    std::vector<Point3D> vertices;
    std::string surface_type;
    std::string obstacle_type; // "horizontal", "vertical_x" и т.д.
    double reflection_loss = 0.5;

    // Конструктор по умолчанию
    Obstacle() = default;

    // Пользовательский конструктор
    Obstacle(
        std::vector<Point3D> v,
        std::string st,
        std::string ot,
        double rl
        ) : vertices(std::move(v)),
        surface_type(std::move(st)),
        obstacle_type(std::move(ot)),
        reflection_loss(rl) {}
};

//путь луча
struct RayPath {
    vector<Point3D> path;
    double attenuation; //затухание
    double delay;
    double weight;
};


#endif // STUCTURS_H
