#ifndef BALLISTIC_SOLVE_BALLISTIC_HPP
#define BALLISTIC_SOLVE_BALLISTIC_HPP

#include "./environment.hpp"
#include "./projectile.hpp"
#include <Eigen/dense>
#include <optional>
#include <functional>
#include <vector>

namespace ballistic_solve
{
    /**
     * @brief Ballistic trajectory solver for projectile motion with environmental effects.
     *
     * This class provides methods to simulate projectile trajectories and solve firing
     * solutions for moving or stationary targets. It accounts for gravity, air drag,
     * wind, and other environmental factors.
     */
    class Ballistic
    {
    private:
        /**
         * @brief State vector representation: [position_x, position_y, position_z, velocity_x, velocity_y, velocity_z]
         */
        using State = Eigen::Matrix<double, 6, 1>;

        /**
         * @brief Function type for time-varying target position.
         *
         * @param time Time in seconds
         * @return Target position vector at the given time
         */
        using TargetPosition = std::function<Eigen::Vector3d(double)>;

    public:
        /**
         * @brief Contains the computed trajectory of a projectile.
         */
        struct Trajectory
        {
            /** @brief Sequence of 3D positions along the trajectory in meters */
            const std::vector<Eigen::Vector3d> positions;

            /** @brief Corresponding time values for each position in seconds */
            const std::vector<double> times;
        };

        /**
         * @brief Contains a firing solution for intercepting a target.
         */
        struct Solution
        {
            /** @brief Unit direction vector to aim the projectile */
            const Eigen::Vector3d direction;

            /** @brief Time of intercept in seconds */
            const double time;

            /** @brief Intercept error in meters */
            const double error;

            /** @brief Computation time in seconds */
            const double computation_time;
        };

    public:
        /** @brief Environmental conditions affecting the projectile */
        const Environment environment;

        /** @brief Physical properties of the projectile */
        const Projectile projectile;

    public:
        /**
         * @brief Constructs a ballistic solver with given environment and projectile.
         *
         * @param environment Environmental conditions (gravity, air density, wind, temperature)
         * @param projectile Projectile properties (mass, area, drag coefficient)
         */
        Ballistic(
            Environment environment,
            Projectile projectile);

    public:
        /**
         * @brief Simulates projectile trajectory over a time range with specified direction.
         *
         * @param platform_position Initial position of the launching platform in meters
         * @param platform_velocity Velocity of the launching platform in m/s
         * @param projectile_speed Muzzle speed of the projectile relative to platform in m/s
         * @param direction Unit direction vector for the projectile launch
         * @param time_range Time interval [start, end] for simulation in seconds
         * @return Trajectory containing positions and times along the flight path
         */
        [[nodiscard]] Trajectory simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector3d &direction,
            std::pair<double, double> time_range) const;

        /**
         * @brief Simulates projectile trajectory over a time range with specified angles.
         *
         * @param platform_position Initial position of the launching platform in meters
         * @param platform_velocity Velocity of the launching platform in m/s
         * @param projectile_speed Muzzle speed of the projectile relative to platform in m/s
         * @param angles Launch angles [azimuth, elevation] in radians
         * @param time_range Time interval [start, end] for simulation in seconds
         * @return Trajectory containing positions and times along the flight path
         */
        [[nodiscard]] Trajectory simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector2d &angles,
            std::pair<double, double> time_range) const;

        /**
         * @brief Simulates projectile position at a specific time with specified direction.
         *
         * @param platform_position Initial position of the launching platform in meters
         * @param platform_velocity Velocity of the launching platform in m/s
         * @param projectile_speed Muzzle speed of the projectile relative to platform in m/s
         * @param direction Unit direction vector for the projectile launch
         * @param time Time at which to compute position in seconds
         * @return Projectile position at the specified time in meters
         */
        [[nodiscard]] Eigen::Vector3d simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector3d &direction,
            double time) const;

        /**
         * @brief Simulates projectile position at a specific time with specified angles.
         *
         * @param platform_position Initial position of the launching platform in meters
         * @param platform_velocity Velocity of the launching platform in m/s
         * @param projectile_speed Muzzle speed of the projectile relative to platform in m/s
         * @param angles Launch angles [azimuth, elevation] in radians
         * @param time Time at which to compute position in seconds
         * @return Projectile position at the specified time in meters
         */
        [[nodiscard]] Eigen::Vector3d simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector2d &angles,
            double time) const;

        /**
         * @brief Simulates projectile position at a specific time with the best possible angles.
         *
         * @param platform_position Initial position of the launching platform in meters
         * @param platform_velocity Velocity of the launching platform in m/s
         * @param projectile_speed Muzzle speed of the projectile relative to platform in m/s
         * @param angles Launch angles [azimuth, elevation] in radians
         * @param time Time at which to compute position in seconds
         * @return Pair of projectile position at the specified time in meters and optimal direction
         */
        [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> simulate_best(
            const Eigen::Vector3d &target_position,
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            double time) const;

        /**
         * @brief Finds the earliest firing solution to intercept a moving target.
         *
         * Searches for the earliest time within the specified range where the projectile
         * can intercept the target. Uses a time-scanning approach with the given interval.
         *
         * @param target_position Function providing target position as a function of time
         * @param platform_position Initial position of the launching platform in meters
         * @param platform_velocity Velocity of the launching platform in m/s
         * @param projectile_speed Muzzle speed of the projectile relative to platform in m/s
         * @param time_range Time interval [start, end] to search for solutions in seconds
         * @param time_scan_interval Time step for scanning potential intercept times (default: 0.5 s)
         * @return Optional Solution containing firing direction and intercept time, or nullopt if no solution exists
         */
        [[nodiscard]] std::optional<Solution> solve_earliest(
            TargetPosition target_position,
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            std::pair<double, double> time_range,
            double time_scan_interval = 0.5) const;

        /**
         * @brief Finds the latest firing solution to intercept a moving target.
         *
         * Searches for the latest time within the specified range where the projectile
         * can intercept the target. Uses a time-scanning approach with the given interval.
         *
         * @param target_position Function providing target position as a function of time
         * @param platform_position Initial position of the launching platform in meters
         * @param platform_velocity Velocity of the launching platform in m/s
         * @param projectile_speed Muzzle speed of the projectile relative to platform in m/s
         * @param time_range Time interval [start, end] to search for solutions in seconds
         * @param time_scan_interval Time step for scanning potential intercept times (default: 0.5 s)
         * @return Optional Solution containing firing direction and intercept time, or nullopt if no solution exists
         */
        [[nodiscard]] std::optional<Solution> solve_latest(
            TargetPosition target_position,
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            std::pair<double, double> time_range,
            double time_scan_interval = 0.5) const;

        /**
         * @brief Finds optimal launch angles to hit a stationary target at a specific time.
         *
         * Computes the azimuth and elevation angles that minimize the intercept error
         * for reaching the target position at the specified time.
         *
         * @param target_position Position of the target in meters
         * @param platform_position Initial position of the launching platform in meters
         * @param platform_velocity Velocity of the launching platform in m/s
         * @param projectile_speed Muzzle speed of the projectile relative to platform in m/s
         * @param time Time at which intercept should occur in seconds
         * @return Optimal launch angles [azimuth, elevation] in radians
         */
        [[nodiscard]] Eigen::Vector2d find_best_angles(
            const Eigen::Vector3d &target_position,
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            double time) const;
        
        /**
         * @brief Finds optimal launch direction to hit a stationary target at a specific time.
         *
         * Computes the direction that minimize the intercept error
         * for reaching the target position at the specified time.
         *
         * @param target_position Position of the target in meters
         * @param platform_position Initial position of the launching platform in meters
         * @param platform_velocity Velocity of the launching platform in m/s
         * @param projectile_speed Muzzle speed of the projectile relative to platform in m/s
         * @param time Time at which intercept should occur in seconds
         * @return Optimal launch direction
         */
        [[nodiscard]] Eigen::Vector3d find_best_direction(
            const Eigen::Vector3d &target_position,
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            double time) const;

    private:
        void system(const State &x, State &dxdt, double time) const;

        [[nodiscard]] Trajectory simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector3d &direction,
            bool is_normalized_direction,
            std::pair<double, double> time_range) const;

        [[nodiscard]] Eigen::Vector3d simulate(
            const Eigen::Vector3d &platform_position,
            const Eigen::Vector3d &platform_velocity,
            double projectile_speed,
            const Eigen::Vector3d &direction,
            bool is_normalized_direction,
            double time) const;
    };
}

#endif // BALLISTIC_SOLVE_BALLISTIC_HPP