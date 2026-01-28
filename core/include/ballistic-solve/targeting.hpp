#ifndef BALLISTIC_SOLVE_TARGETING_HPP
#define BALLISTIC_SOLVE_TARGETING_HPP

#include "ballistic-solve/environment.hpp"
#include "ballistic-solve/projectile.hpp"
#include "ballistic-solve/simulation.hpp"

#include <Eigen/Dense>
#include <functional>
#include <optional>
#include <utility>
#include <cstdint>

namespace ballistic_solve
{
    /**
     * @brief Complete targeting solution for engaging a target.
     *
     * Contains launch angles, direction vector, and time of flight.
     */
    struct TargetingSolution
    {
        const std::pair<double, double> angles; ///< (azimuth, elevation) in radians
        const Eigen::Vector3d direction;        ///< Launch direction (unit vector)
        const double time;                      ///< Time of flight (seconds)
    };

    /**
     * @brief Configuration for the ballistic solver.
     *
     * Controls search strategy and iteration limits for finding firing solutions.
     */
    struct TargetingOptions
    {
        /**
         * @brief Strategy for nested angle search.
         */
        enum class AngleSearchStrategy
        {
            AzimuthThenElevation, ///< Search azimuth in inner loop, elevation in outer loop
            ElevationThenAzimuth  ///< Search elevation in inner loop, azimuth in outer loop
        };

        /// Strategy for nested angle search
        const AngleSearchStrategy angle_search_strategy = AngleSearchStrategy::AzimuthThenElevation;

        /// Maximum iterations for azimuth angle search
        const std::uintmax_t azimuth_max_iter = 16;

        /// Maximum iterations for elevation angle search
        const std::uintmax_t elevation_max_iter = 16;

        /// Maximum iterations for time-of-flight search
        const std::uintmax_t time_max_iter = 16;

        /// Step size for numerical differentiation
        const double h = 1e-3;

        /// Time step for coarse scanning in time search (seconds)
        const double time_scan_step = 0.5;
    };

    /**
     * @brief Find best azimuth angle for a given elevation and time.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param target_position Target position
     * @param elevation_angle Fixed elevation angle (radians)
     * @param time Flight time (seconds)
     * @param integration_options Integration parameters
     * @param targeting_options Targeting configuration
     * @return double Best azimuth angle (radians)
     */
    double find_best_azimuth_angle(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &target_position,
        const double elevation_angle,
        const double time,
        const IntegrationOptions &integration_options = IntegrationOptions{},
        const TargetingOptions &targeting_options = TargetingOptions{});

    /**
     * @brief Find best elevation angle for a given azimuth and time.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param target_position Target position
     * @param azimuth_angle Fixed azimuth angle (radians)
     * @param time Flight time (seconds)
     * @param integration_options Integration parameters
     * @param targeting_options Targeting configuration
     * @return double Best elevation angle (radians)
     */
    double find_best_elevation_angle(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &target_position,
        const double azimuth_angle,
        const double time,
        const IntegrationOptions &integration_options = IntegrationOptions{},
        const TargetingOptions &targeting_options = TargetingOptions{});

    /**
     * @brief Find best launch angles (azimuth, elevation) for a given time.
     *
     * Uses nested search based on the configured angle search strategy.
     * Minimizes squared distance to target.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param target_position Target position
     * @param time Flight time (seconds)
     * @param integration_options Integration parameters
     * @param targeting_options Targeting configuration
     * @return std::pair<double, double> (azimuth, elevation) in radians
     */
    std::pair<double, double> find_best_angles(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &target_position,
        const double time,
        const IntegrationOptions &integration_options = IntegrationOptions{},
        const TargetingOptions &targeting_options = TargetingOptions{});

    /**
     * @brief Find best launch direction for a given time.
     *
     * Convenience function that returns a direction vector instead of angles.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param target_position Target position
     * @param time Flight time (seconds)
     * @param integration_options Integration parameters
     * @param targeting_options Targeting configuration
     * @return Eigen::Vector3d Best launch direction (unit vector)
     */
    Eigen::Vector3d find_best_direction(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &target_position,
        const double time,
        const IntegrationOptions &integration_options = IntegrationOptions{},
        const TargetingOptions &targeting_options = TargetingOptions{});

    /**
     * @brief Find earliest time when target can be intercepted.
     *
     * Searches forward through the time range to find the first moment when
     * the projectile can reach the (possibly moving) target. Uses coarse
     * scanning followed by root refinement.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param target_position Function returning target position at given time
     * @param time_range (min_time, max_time) search range in seconds
     * @param integration_options Integration parameters
     * @param targeting_options Targeting configuration
     * @return std::optional<double> Earliest intercept time in seconds, or nullopt if no solution exists
     */
    std::optional<double> find_earliest_time(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::function<Eigen::Vector3d(double)> &target_position,
        const std::pair<double, double> &time_range,
        const IntegrationOptions &integration_options = IntegrationOptions{},
        const TargetingOptions &targeting_options = TargetingOptions{});

    /**
     * @brief Find latest time when target can be intercepted.
     *
     * Searches backward through the time range to find the last moment when
     * the projectile can reach the (possibly moving) target. Uses coarse
     * scanning followed by root refinement.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param target_position Function returning target position at given time
     * @param time_range (min_time, max_time) search range in seconds
     * @param integration_options Integration parameters
     * @param targeting_options Targeting configuration
     * @return std::optional<double> Latest intercept time in seconds, or nullopt if no solution exists
     */
    std::optional<double> find_latest_time(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::function<Eigen::Vector3d(double)> &target_position,
        const std::pair<double, double> &time_range,
        const IntegrationOptions &integration_options = IntegrationOptions{},
        const TargetingOptions &targeting_options = TargetingOptions{});

    /**
     * @brief Find earliest firing solution for intercepting a target.
     *
     * Combines find_earliest_time with find_best_angles to produce a complete
     * targeting solution at the earliest possible intercept time.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param target_position Function returning target position at given time
     * @param time_range (min_time, max_time) search range in seconds
     * @param integration_options Integration parameters
     * @param targeting_options Targeting configuration
     * @return std::optional<TargetingSolution> Complete firing solution, or nullopt if no solution exists
     */
    std::optional<TargetingSolution> find_earliest_firing_solution(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::function<Eigen::Vector3d(double)> &target_position,
        const std::pair<double, double> &time_range,
        const IntegrationOptions &integration_options = IntegrationOptions{},
        const TargetingOptions &targeting_options = TargetingOptions{});

    /**
     * @brief Find latest firing solution for intercepting a target.
     *
     * Combines find_latest_time with find_best_angles to produce a complete
     * targeting solution at the latest possible intercept time.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param target_position Function returning target position at given time
     * @param time_range (min_time, max_time) search range in seconds
     * @param integration_options Integration parameters
     * @param targeting_options Targeting configuration
     * @return std::optional<TargetingSolution> Complete firing solution, or nullopt if no solution exists
     */
    std::optional<TargetingSolution> find_latest_firing_solution(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::function<Eigen::Vector3d(double)> &target_position,
        const std::pair<double, double> &time_range,
        const IntegrationOptions &integration_options = IntegrationOptions{},
        const TargetingOptions &targeting_options = TargetingOptions{});
}

#endif // BALLISTIC_SOLVE_TARGETING_HPP