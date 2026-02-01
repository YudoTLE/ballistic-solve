#ifndef BALLISTIC_SOLVE_PROJECTILE_INL
#define BALLISTIC_SOLVE_PROJECTILE_INL

#include "./projectile.hpp"

namespace ballistic_solve
{
    template <concepts::MachBasedDragCoefficient DragCoefficient>
    Projectile<DragCoefficient>::Projectile()
        : mass(1.0), area(1.0), drag_coefficient([](double)
                                                 { return 0.47; })
    {
    }

    template <concepts::MachBasedDragCoefficient DragCoefficient>
    Projectile<DragCoefficient>::Projectile(double mass, double area, DragCoefficient drag_coefficient)
        : mass(mass), area(area), drag_coefficient(drag_coefficient)
    {
    }

    template <concepts::MachBasedDragCoefficient DragCoefficient>
    auto Projectile<DragCoefficient>::with_mass(double value) const
    {
        return Projectile(value, this->area, this->drag_coefficient);
    }

    template <concepts::MachBasedDragCoefficient DragCoefficient>
    auto Projectile<DragCoefficient>::with_area(double value) const
    {
        return Projectile(this->mass, value, this->drag_coefficient);
    }

    template <concepts::MachBasedDragCoefficient DragCoefficient>
    auto Projectile<DragCoefficient>::with_drag_coefficient(double value) const
    {
        return Projectile(this->mass, this->area, [value](double)
                          { return value; });
    }

    template <concepts::MachBasedDragCoefficient DragCoefficient>
    template <concepts::MachBasedDragCoefficient F>
    auto Projectile<DragCoefficient>::with_drag_coefficient(F &&function) const
    {
        return Projectile(this->mass, this->area, std::forward<F>(function));
    }
}

#endif // BALLISTIC_SOLVE_PROJECTILE_INL