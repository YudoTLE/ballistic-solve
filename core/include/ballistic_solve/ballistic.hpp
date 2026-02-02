#ifndef BALLISTIC_SOLVE_BALLISTIC_HPP
#define BALLISTIC_SOLVE_BALLISTIC_HPP

#include "./environment.hpp"
#include "./projectile.hpp"
#include <Eigen/dense>
#include <optional>
#include <functional>
#include <vector>
#include <numeric>

namespace ballistic_solve
{
    class Ballistic
    {
    private:
        using State = Eigen::Matrix<double, 6, 1>;

        using TargetPosition = std::function<Eigen::Vector3d(double)>;

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

        struct Targeting
        {
            const double time_scan_interval = 0.5;
        };

    public:
        const Environment environment;
        const Projectile projectile;
        const Targeting targeting;

    public:
        Ballistic(
            Environment environment,
            Projectile projectile,
            Targeting targeting = Targeting{});

    public:
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

        [[nodiscard]] std::optional<Solution> solve_earliest(
            TargetPosition target_position,
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            std::pair<double, double> time_range) const;

        [[nodiscard]] std::optional<Solution> solve_latest(
            TargetPosition target_position,
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

#endif // BALLISTIC_SOLVE_BALLISTIC_HPP