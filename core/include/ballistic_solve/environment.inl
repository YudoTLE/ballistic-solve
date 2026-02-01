#ifndef BALLISTIC_SOLVE_ENVIRONMENT_INL
#define BALLISTIC_SOLVE_ENVIRONMENT_INL

#include "./environment.hpp"

namespace ballistic_solve
{
    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    Environment<AirDensity, WindVelocity, Temperature>::Environment()
        : gravity(Eigen::Vector3d(0, 0, -9.81)),
          air_density([](Eigen::Vector3d)
                      { return 1.225; }),
          wind_velocity([](Eigen::Vector3d)
                        { return Eigen::Vector3d(0, 0, 0); }),
          temperature([](Eigen::Vector3d)
                      { return 288.15; })
    {
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    Environment<AirDensity, WindVelocity, Temperature>::Environment(
        Eigen::Vector3d gravity,
        AirDensity air_density,
        WindVelocity wind_velocity,
        Temperature temperature)
        : gravity(gravity),
          air_density(air_density),
          wind_velocity(wind_velocity),
          temperature(temperature)
    {
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_gravity(double value) const
    {
        return Environment(
            Eigen::Vector3d(0, 0, -value),
            this->air_density,
            this->wind_velocity,
            this->temperature);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_gravity(Eigen::Vector3d value) const
    {
        return Environment(
            value,
            this->air_density,
            this->wind_velocity,
            this->temperature);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_air_density(double value) const
    {
        return Environment(
            this->gravity,
            [value](Eigen::Vector3d)
            { return value; },
            this->wind_velocity,
            this->temperature);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    template <concepts::AltitudeBasedAirDensity F>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_air_density(F &&function) const
    {
        return Environment(
            this->gravity,
            [f = std::forward<F>(function)](Eigen::Vector3d pos)
            { return f(pos.z()); },
            this->wind_velocity,
            this->temperature);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    template <concepts::SpatialBasedAirDensity F>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_air_density(F &&function) const
    {
        return Environment(
            this->gravity,
            std::forward<F>(function),
            this->wind_velocity,
            this->temperature);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_wind_velocity(Eigen::Vector3d value) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            [value](Eigen::Vector3d)
            { return value; },
            this->temperature);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    template <concepts::AltitudeBasedWindVelocity F>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_wind_velocity(F &&function) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            [f = std::forward<F>(function)](Eigen::Vector3d pos)
            { return f(pos.z()); },
            this->temperature);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    template <concepts::SpatialBasedWindVelocity F>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_wind_velocity(F &&function) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            std::forward<F>(function),
            this->temperature);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_temperature(double value) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            this->wind_velocity,
            [value](Eigen::Vector3d)
            { return value; });
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    template <concepts::AltitudeBasedTemperature F>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_temperature(F &&function) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            this->wind_velocity,
            [f = std::forward<F>(function)](Eigen::Vector3d pos)
            { return f(pos.z()); });
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    template <concepts::SpatialBasedTemperature F>
    auto Environment<AirDensity, WindVelocity, Temperature>::with_temperature(F &&function) const
    {
        return Environment(
            this->gravity,
            this->air_density,
            this->wind_velocity,
            std::forward<F>(function));
    }
}

#endif // BALLISTIC_SOLVE_ENVIRONMENT_INL