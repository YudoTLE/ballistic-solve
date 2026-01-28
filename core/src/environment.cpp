#include "ballistic-solve/environment.hpp"

#include <cmath>
#include <random>

namespace ballistic_solve
{
    Environment::Environment(
        const Eigen::Vector3d &gravity,
        const Environment::AirDensity &air_density,
        const Eigen::Vector3d &wind_velocity)
        : gravity(gravity),
          air_density(air_density),
          wind_velocity(wind_velocity)
    {
    }

    Environment Environment::vacuum()
    {
        return Environment(
            {0, 0, -9.81},
            [](double alt)
            { return 0.0; },
            {0, 0, 0});
    }

    Environment Environment::standard()
    {
        return Environment(
            {0, 0, -9.81},
            [](double alt)
            { return 1.225 * std::exp(-alt / 8500.0); },
            {0, 0, 0});
    }

    Environment Environment::standard_with_random_wind(std::optional<unsigned int> seed)
    {
        static std::random_device rd;
        static std::mt19937 gen(seed.value_or(rd()));
        std::normal_distribution<> horizontal(0.0, 10.0);
        std::normal_distribution<> vertical(0.0, 1.0);

        return Environment(
            {0, 0, -9.81},
            [](double alt)
            { return 1.225 * std::exp(-alt / 8500.0); },
            {horizontal(gen), horizontal(gen), vertical(gen)});
    }

    Environment Environment::standard_with_random_strong_wind(std::optional<unsigned int> seed)
    {
        static std::random_device rd;
        static std::mt19937 gen(seed.value_or(rd()));
        std::normal_distribution<> horizontal(0.0, 25.0);
        std::normal_distribution<> vertical(0.0, 5.0);

        return Environment(
            {0, 0, -9.81},
            [](double alt)
            { return 1.225 * std::exp(-alt / 8500.0); },
            {horizontal(gen), horizontal(gen), vertical(gen)});
    }

    Environment Environment::standard_with_random_extreme_wind(std::optional<unsigned int> seed)
    {
        static std::random_device rd;
        static std::mt19937 gen(seed.value_or(rd()));
        std::normal_distribution<> horizontal(0.0, 50.0);
        std::normal_distribution<> vertical(0.0, 10.0);

        return Environment(
            {0, 0, -9.81},
            [](double alt)
            { return 1.225 * std::exp(-alt / 8500.0); },
            {horizontal(gen), horizontal(gen), vertical(gen)});
    }

    Environment Environment::standard_with_random_catastrophic_wind(std::optional<unsigned int> seed)
    {
        static std::random_device rd;
        static std::mt19937 gen(seed.value_or(rd()));
        std::normal_distribution<> horizontal(0.0, 100.0);
        std::normal_distribution<> vertical(0.0, 20.0);

        return Environment(
            {0, 0, -9.81},
            [](double alt)
            { return 1.225 * std::exp(-alt / 8500.0); },
            {horizontal(gen), horizontal(gen), vertical(gen)});
    }
}