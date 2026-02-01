#ifndef BALLISTIC_SOLVE_PROJECTILE_HPP
#define BALLISTIC_SOLVE_PROJECTILE_HPP

#include <Eigen/Dense>
#include <functional>

namespace ballistic_solve
{
    class Projectile
    {
    public:
        using DragCoefficient = std::function<double(double)>;

    public:
        const double mass;

        const double area;

        const DragCoefficient drag_coefficient;

    public:
        Projectile();

        Projectile(double mass, double area, DragCoefficient drag_coefficient);

    public:
        [[nodiscard]] Projectile with_mass(double value) const;

        [[nodiscard]] Projectile with_area(double value) const;

        [[nodiscard]] Projectile with_drag_coefficient(double value) const;

        [[nodiscard]] Projectile with_drag_coefficient(DragCoefficient function) const;
    };
}

#endif // BALLISTIC_SOLVE_PROJECTILE_HPP