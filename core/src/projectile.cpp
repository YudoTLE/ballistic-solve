#include "ballistic_solve/projectile.hpp"

namespace ballistic_solve
{
    Projectile::Projectile()
        : mass(1.0),
          area(1.0),
          drag_coefficient([](double)
                          { return 0.47; })
    {
    }

    Projectile::Projectile(
        double mass,
        double area,
        Projectile::DragCoefficient drag_coefficient)
        : mass(std::move(mass)),
          area(std::move(area)),
          drag_coefficient(std::move(drag_coefficient))
    {
    }

    Projectile Projectile::with_mass(double value) const
    {
        return Projectile(
            value,
            this->area,
            this->drag_coefficient);
    }

    Projectile Projectile::with_area(double value) const
    {
        return Projectile(
            this->mass,
            value,
            this->drag_coefficient);
    }

    Projectile Projectile::with_drag_coefficient(double value) const
    {
        return Projectile(
            this->mass,
            this->area,
            [value](double)
            { return value; });
    }

    Projectile Projectile::with_drag_coefficient(Projectile::DragCoefficient function) const
    {
        return Projectile(
            this->mass,
            this->area,
            function);
    }
}