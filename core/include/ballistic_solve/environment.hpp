#ifndef BALLISTIC_SOLVE_ENVIRONMENT_HPP
#define BALLISTIC_SOLVE_ENVIRONMENT_HPP

#include <Eigen/Dense>
#include <concepts>
#include <type_traits>

namespace ballistic_solve
{
    namespace concepts
    {
        template <typename F>
        concept AltitudeBasedAirDensity =
            std::regular_invocable<F, double> &&
            std::copy_constructible<F> &&
            std::convertible_to<std::invoke_result_t<F, double>, double>;

        template <typename F>
        concept SpatialBasedAirDensity =
            std::regular_invocable<F, Eigen::Vector3d> &&
            std::copy_constructible<F> &&
            std::convertible_to<std::invoke_result_t<F, Eigen::Vector3d>, double>;

        template <typename F>
        concept AltitudeBasedWindVelocity =
            std::regular_invocable<F, double> &&
            std::copy_constructible<F> &&
            std::convertible_to<std::invoke_result_t<F, double>, Eigen::Vector3d>;

        template <typename F>
        concept SpatialBasedWindVelocity =
            std::regular_invocable<F, Eigen::Vector3d> &&
            std::copy_constructible<F> &&
            std::convertible_to<std::invoke_result_t<F, Eigen::Vector3d>, Eigen::Vector3d>;

        template <typename F>
        concept AltitudeBasedTemperature =
            std::regular_invocable<F, double> &&
            std::copy_constructible<F> &&
            std::convertible_to<std::invoke_result_t<F, double>, double>;

        template <typename F>
        concept SpatialBasedTemperature =
            std::regular_invocable<F, Eigen::Vector3d> &&
            std::copy_constructible<F> &&
            std::convertible_to<std::invoke_result_t<F, Eigen::Vector3d>, double>;
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature>
    class Environment
    {
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

        [[nodiscard]] auto with_gravity(double value) const;

        [[nodiscard]] auto with_gravity(Eigen::Vector3d function) const;

        [[nodiscard]] auto with_air_density(double value) const;

        template <concepts::AltitudeBasedAirDensity F>
        [[nodiscard]] auto with_air_density(F &&function) const;

        template <concepts::SpatialBasedAirDensity F>
        [[nodiscard]] auto with_air_density(F &&function) const;

        [[nodiscard]] auto with_wind_velocity(Eigen::Vector3d value) const;

        template <concepts::AltitudeBasedWindVelocity F>
        [[nodiscard]] auto with_wind_velocity(F &&function) const;

        template <concepts::SpatialBasedWindVelocity F>
        [[nodiscard]] auto with_wind_velocity(F &&function) const;

        [[nodiscard]] auto with_temperature(double value) const;

        template <concepts::AltitudeBasedTemperature F>
        [[nodiscard]] auto with_temperature(F &&function) const;

        template <concepts::SpatialBasedTemperature F>
        [[nodiscard]] auto with_temperature(F &&function) const;
    };
}

#include "./environment.inl"

#endif // BALLISTIC_SOLVE_ENVIRONMENT_HPP