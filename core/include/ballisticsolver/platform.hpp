#ifndef BALLISTICSOLVER_PLATFORM_HPP
#define BALLISTICSOLVER_PLATFORM_HPP

#include <Eigen/Dense>

namespace ballisticsolver
{
    /**
     * @brief Launch platform properties for ballistic simulation.
     * 
     * Represents the firing platform (e.g., ground position, moving vehicle, aircraft)
     * including its position, velocity, and the muzzle velocity of the weapon system.
     * The projectile's initial velocity is the vector sum of platform velocity and
     * muzzle velocity in the launch direction.
     */
    class Platform
    {
    public:
        /// Position of the launch platform in 3D space (meters)
        const Eigen::Vector3d position;
        
        /// Velocity of the platform itself (m/s). For stationary platforms, this is zero.
        const Eigen::Vector3d velocity;

        /// Muzzle velocity magnitude (m/s). Speed of projectile relative to platform.
        const double muzzle_velocity;

    public:
        /**
         * @brief Construct a launch platform.
         * 
         * @param position 3D position vector in meters
         * @param velocity 3D velocity vector in m/s (platform motion)
         * @param muzzle_velocity Scalar muzzle velocity in m/s (projectile speed relative to platform)
         */
        Platform(
            const Eigen::Vector3d &position,
            const Eigen::Vector3d &velocity,
            const double muzzle_velocity);
    };
}

#endif // BALLISTICSOLVER_PLATFORM_HPP