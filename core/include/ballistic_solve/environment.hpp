#ifndef BALLISTIC_SOLVE_ENVIRONMENT_HPP
#define BALLISTIC_SOLVE_ENVIRONMENT_HPP

#include <Eigen/Dense>
#include <functional>

namespace ballistic_solve
{

    class Environment
    {
    public:
        using AirDensity = std::function<double(Eigen::Vector3d)>;
        using WindVelocity = std::function<Eigen::Vector3d(Eigen::Vector3d)>;
        using Temperature = std::function<double(Eigen::Vector3d)>;

    public:
        const Eigen::Vector3d gravity;

        const AirDensity air_density;

        const WindVelocity wind_velocity;

        const Temperature temperature;

    public:
        Environment();

        Environment(Eigen::Vector3d gravity,
                    AirDensity air_density,
                    WindVelocity wind_velocity,
                    Temperature temperature);

    public:
        [[nodiscard]] Environment with_gravity(double value) const;

        [[nodiscard]] Environment with_gravity(Eigen::Vector3d function) const;

        [[nodiscard]] Environment with_air_density(double value) const;

        [[nodiscard]] Environment with_air_density(AirDensity function) const;

        [[nodiscard]] Environment with_wind_velocity(Eigen::Vector3d value) const;

        [[nodiscard]] Environment with_wind_velocity(WindVelocity function) const;

        [[nodiscard]] Environment with_temperature(double value) const;

        [[nodiscard]] Environment with_temperature(Temperature function) const;
    };
}

#endif // BALLISTIC_SOLVE_ENVIRONMENT_HPP