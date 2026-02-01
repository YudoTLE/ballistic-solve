#ifndef BALLISTIC_SOLVE_BALLISTIC_HPP
#define BALLISTIC_SOLVE_BALLISTIC_HPP

#include "./environment.hpp"
#include "./projectile.hpp"
#include <Eigen/dense>
#include <optional>
#include <vector>
#include <numeric>

namespace ballistic_solve
{
    namespace concepts
    {
        template <typename F>
        concept TimeBasedTargetPosition =
            std::regular_invocable<F, double> &&
            std::copy_constructible<F> &&
            std::convertible_to<std::invoke_result_t<F, double>, Eigen::Vector3d>;
    }
    
    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    class Ballistic
    {
    private:
        using State = Eigen::Matrix<double, 6, 1>;

    public:
        struct Trajectory
        {
            const std::vector<Eigen::Vector3d> positions;
            const std::vector<double> times;
        };

        struct Solution
        {
            const Eigen::Vector3d direction;
            const double time;
        };

        struct Integration
        {
            const double max_step_size = std::numeric_limits<double>::infinity();
            const double initial_step_size = 1e-3;
            const double absolute_tolerance = 1e-6;
            const double relative_tolerance = 1e-3;
        };

        struct Targeting
        {
            const double h = 1e-3;
            const double time_scan_interval = 0.5;
            const std::uintmax_t angle_max_iteration = 16;
            const std::uintmax_t time_max_iteration = 16;
        };

    public:
        const Environment<AirDensity, WindVelocity, Temperature> environment;
        const Projectile<DragCoefficient> projectile;
        const Integration integration;
        const Targeting targeting;

    public:
        Ballistic(
            Environment<AirDensity, WindVelocity, Temperature> environment,
            Projectile<DragCoefficient> projectile,
            Integration integration = Integration{},
            Targeting targeting = Targeting{});

        [[nodiscard]] Trajectory simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector3d &direction,
            std::pair<double, double> time_range) const;

        [[nodiscard]] Trajectory simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector2d &angles,
            std::pair<double, double> time_range) const;

        [[nodiscard]] Eigen::Vector3d simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector3d &direction,
            double time) const;

        [[nodiscard]] Eigen::Vector3d simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector2d &angles,
            double time) const;

        template <concepts::TimeBasedTargetPosition F>
        [[nodiscard]] std::optional<Solution> solve_earliest(
            F target_position,
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            std::pair<double, double> time_range) const;

        template <concepts::TimeBasedTargetPosition F>
        [[nodiscard]] std::optional<Solution> solve_latest(
            F target_position,
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            std::pair<double, double> time_range) const;

    private:
        void system(const State &x, State &dxdt, double time) const;

        [[nodiscard]] Eigen::Vector2d find_best_angles(
            const Eigen::Vector3d &target_position,
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            double time) const;
    };

}

#include "./ballistic.inl"

#endif // BALLISTIC_SOLVE_BALLISTIC_HPP