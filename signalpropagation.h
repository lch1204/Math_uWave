#ifndef SIGNALPROPAGATION_H
#define SIGNALPROPAGATION_H
#include "stucturs.h"
#include "readerjson.h"

// Модель распространения сигнала
class SignalPropagation
{
    vector<RayPath> multipaths;
    Point3D last_tx, last_rx;
    ReaderJson& config;

    double distance(const Point3D& a, const Point3D& b) const;

    double angular_factor(const Point3D& incident, const Obstacle& obs) const;

public:
    SignalPropagation(ReaderJson &cfg)
        : last_tx({0, 0, 0}),   // Инициализация last_tx
        last_rx({0, 0, 0}),   // Инициализация last_rx
        config(cfg)
    {}

    void calculate_paths(const Point3D& tx, const Point3D& rx,
                         const vector<Obstacle>& obstacles);

    const vector<RayPath>& get_paths() const { return multipaths; }

    Point3D calculate_reflection_point(const Point3D& tx,
                                       const Point3D& rx,
                                       const Obstacle& obs);

    bool line_intersects_obstacle(const Point3D& p1, const Point3D& p2,
                                  const Obstacle& obs);

    bool line_intersects_cube(const Point3D& p1, const Point3D& p2,
                              const Point3D& cube_min, const Point3D& cube_max);

    Point3D reflect_point(const Point3D& point,
                          const Point3D& surface_point,
                          const Point3D& normal);

    Point3D find_ray_intersection(const Point3D& tx,
                                  const Point3D& rx,
                                  const Obstacle& obs);

    Point3D calculate_surface_normal(const Obstacle& obs);
};

#endif // SIGNALPROPAGATION_H
