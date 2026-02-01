#include "ballistic_solve/environment.hpp"

namespace ballistic_solve
{
    Environment::Environment()
        : gravity(Eigen::Vector3d(0, 0, -9.81)),
          air_density(
              [](Eigen::Vector3d pos)
              {
                double h = std::max(0.0, pos.z()); 
                double term = 1.0 - (0.0065 * h) / 288.15;
                return 1.225 * std::pow(term, 4.256); }),
          wind_velocity(
              [](Eigen::Vector3d)
              { return Eigen::Vector3d::Zero(); }),
          temperature(
              [](Eigen::Vector3d pos)
              {
                double h = std::max(0.0, pos.z());
                return 288.15 - 0.0065 * h; })
    {
    }

    Environment::Environment(
        Eigen::Vector3d gravity,
        Environment::AirDensity air_density,
        Environment::WindVelocity wind_velocity,
        Environment::Temperature temperature)
        : gravity(std::move(gravity)),
          air_density(std::move(air_density)),
          wind_velocity(std::move(wind_velocity)),
          temperature(std::move(temperature))
    {
    }

    Environment Environment::with_gravity(double value) const
    {
        return Environment(
            Eigen::Vector3d(0, 0, -value),
            this->air_density,
            this->wind_velocity,
            this->temperature);
    }

    Environment Environment::with_gravity(Eigen::Vector3d value) const
    {
        return Environment(
            value,
            this->air_density,
            this->wind_velocity,
            this->temperature);
    }

    Environment Environment::with_air_density(double value) const
    {
        return Environment(
            this->gravity,
            [value](Eigen::Vector3d)
            { return value; },
            this->wind_velocity,
            this->temperature);
    }

    Environment Environment::with_air_density(Environment::AirDensity function) const
    {
        return Environment(
            this->gravity,
            function,
            this->wind_velocity,
            this->temperature);
    }

    Environment Environment::with_wind_velocity(Eigen::Vector3d value) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            [value](Eigen::Vector3d)
            { return value; },
            this->temperature);
    }

    Environment Environment::with_wind_velocity(Environment::WindVelocity function) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            function,
            this->temperature);
    }

    Environment Environment::with_temperature(double value) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            this->wind_velocity,
            [value](Eigen::Vector3d)
            { return value; });
    }

    Environment Environment::with_temperature(Environment::Temperature function) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            this->wind_velocity,
            function);
    }
}