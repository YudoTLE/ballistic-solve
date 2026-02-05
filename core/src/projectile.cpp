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

    Projectile Projectile::m61_vulcan_round()
    {
        return Projectile(
            0.100,
            3.14159e-4,
            [](double mach)
            {
                if (mach < 0.8)
                {
                    return 0.295;
                }
                if (mach < 1.0)
                {
                    double t = (mach - 0.8) / 0.2;
                    return 0.295 + t * (0.60 - 0.295);
                }
                if (mach < 1.2)
                {
                    double t = (mach - 1.0) / 0.2;
                    return 0.60 - t * (0.60 - 0.50);
                }
                if (mach < 2.0)
                {
                    double t = (mach - 1.2) / 0.8;
                    return 0.50 - t * (0.50 - 0.45);
                }
                if (mach < 3.5)
                {
                    double t = (mach - 2.0) / 1.5;
                    return 0.45 - t * (0.45 - 0.40);
                }
                return 0.40;
            });
    }

    Projectile Projectile::gsh30_round()
    {
        return Projectile(
            0.390,
            7.069e-4,
            [](double mach)
            {
                if (mach < 0.8)
                {
                    return 0.280;
                }
                if (mach < 1.0)
                {
                    double t = (mach - 0.8) / 0.2;
                    return 0.280 + t * (0.60 - 0.280);
                }
                if (mach < 1.2)
                {
                    double t = (mach - 1.0) / 0.2;
                    return 0.60 - t * (0.60 - 0.50);
                }
                if (mach < 2.5)
                {
                    double t = (mach - 1.2) / 1.3;
                    return 0.50 - t * (0.50 - 0.44);
                }
                if (mach < 3.0)
                {
                    double t = (mach - 2.5) / 0.5;
                    return 0.44 - t * (0.44 - 0.42);
                }
                return 0.42;
            });
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