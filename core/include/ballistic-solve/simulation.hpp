#ifndef BALLISTIC_SOLVE_SIMULATION_HPP
#define BALLISTIC_SOLVE_SIMULATION_HPP

#include "ballistic-solve/environment.hpp"
#include "ballistic-solve/projectile.hpp"
#include "ballistic-solve/trajectory.hpp"
#include "ballistic-solve/platform.hpp"

#include <Eigen/Dense>
#include <utility>

namespace ballistic_solve
{
    /**
     * @brief Configuration for the adaptive ODE integrator.
     *
     * Controls the Dormand-Prince (DOPRI5) Runge-Kutta adaptive integration_options
     * used for trajectory integration.
     */
    struct IntegrationOptions
    {
        const double max_step = 1e3;    ///< Maximum step size (seconds)
        const double first_step = 1e-3; ///< Initial step size (seconds)
        const double atol = 1e-6;       ///< Absolute error tolerance
        const double rtol = 1e-3;       ///< Relative error tolerance
    };

    /**
     * @brief Convert spherical coordinates to 3D direction vector.
     *
     * @param angles Pair of (azimuth, elevation) angles in radians
     * @return Eigen::Vector3d Unit direction vector
     */
    Eigen::Vector3d to_direction(const std::pair<double, double> &angles);

    /**
     * @brief Convert 3D direction vector to spherical coordinates.
     *
     * @param direction Direction vector (need not be normalized)
     * @return std::pair<double, double> (azimuth, elevation) in radians
     */
    std::pair<double, double> to_angles(const Eigen::Vector3d &direction);

    /**
     * @brief Compute complete ballistic trajectory with recorded time steps.
     *
     * Simulates projectile motion under gravity, drag, and wind effects using
     * adaptive Runge-Kutta integration. Records position at each integration step.
     *
     * @param environment Environmental conditions (gravity, air density, wind)
     * @param projectile Projectile properties (mass, drag coefficient, area)
     * @param platform Launch platform (position, velocity, muzzle velocity)
     * @param direction Unit vector indicating launch direction
     * @param time Simulation time (seconds)
     * @param integration_options Integration parameters (step sizes, tolerances)
     * @return Trajectory Complete trajectory with positions and times
     */
    Trajectory compute_trajectory(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &direction,
        const double time,
        const IntegrationOptions &integration_options = IntegrationOptions{});

    /**
     * @brief Compute complete ballistic trajectory from launch angles.
     *
     * Convenience overload that accepts spherical coordinates instead of
     * direction vector.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param angles Pair of (azimuth, elevation) angles in radians
     * @param time Simulation time (seconds)
     * @param integration_options Integration parameters
     * @return Trajectory Complete trajectory with positions and times
     */
    Trajectory compute_trajectory(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::pair<double, double> &angles,
        const double time,
        const IntegrationOptions &integration_options = IntegrationOptions{});

    /**
     * @brief Compute final position only (no trajectory recording).
     *
     * More efficient than compute_trajectory when only the endpoint is needed.
     * Uses the same physics simulation but doesn't record intermediate steps.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param direction Unit vector indicating launch direction
     * @param time Simulation time (seconds)
     * @param integration_options Integration parameters
     * @return Eigen::Vector3d Final position after given simulation time
     */
    Eigen::Vector3d compute_point(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &direction,
        const double time,
        const IntegrationOptions &integration_options = IntegrationOptions{});

    /**
     * @brief Compute final position only from launch angles.
     *
     * Convenience overload that accepts spherical coordinates.
     *
     * @param environment Environmental conditions
     * @param projectile Projectile properties
     * @param platform Launch platform
     * @param angles Pair of (azimuth, elevation) angles in radians
     * @param time Simulation time (seconds)
     * @param integration_options Integration parameters
     * @return Eigen::Vector3d Final position after given simulation time
     */
    Eigen::Vector3d compute_point(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::pair<double, double> &angles,
        const double time,
        const IntegrationOptions &integration_options = IntegrationOptions{});
}

#endif // BALLISTIC_SOLVE_SIMULATION_HPP