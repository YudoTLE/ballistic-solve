#include "ballistic_solve/projectile.hpp"

namespace ballistic_solve
{
    Projectile::Projectile()
        : mass(0.0),
          area(0.0),
          drag_coefficient([](double)
                           { return 0.0; })
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

    Projectile Projectile::baseball_standard()
    {
        return Projectile(
            0.145,
            0.00426,
            [](double)
            { return 0.30; });
    }

    Projectile Projectile::soccer_ball_standard()
    {
        return Projectile(
            0.430,
            0.0388,
            [](double)
            { return 0.25; });
    }

    Projectile Projectile::basketball_standard()
    {
        return Projectile(
            0.624,
            0.0457,
            [](double)
            { return 0.30; });
    }

    Projectile Projectile::golf_ball_standard()
    {
        return Projectile(
            0.0459,
            0.00143,
            [](double)
            { return 0.25; });
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