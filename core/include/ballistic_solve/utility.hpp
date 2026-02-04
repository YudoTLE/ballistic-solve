#ifndef BALLISTIC_SOLVE_UTILITY_HPP
#define BALLISTIC_SOLVE_UTILITY_HPP

#include <Eigen/Dense>

namespace ballistic_solve
{
    /**
     * @brief Converts angular representation to a unit direction vector.
     * 
     * Converts azimuth and elevation angles to a 3D unit direction vector
     * using spherical coordinate conversion.
     * 
     * @param angles Vector containing [azimuth, elevation] in radians
     *               - azimuth: horizontal angle measured from the positive x-axis
     *               - elevation: vertical angle measured from the xy-plane
     * @return Unit direction vector in 3D space
     */
    [[nodiscard]] Eigen::Vector3d to_direction(const Eigen::Vector2d &angles);

    /**
     * @brief Converts a direction vector to angular representation.
     * 
     * Converts a 3D direction vector to azimuth and elevation angles
     * using inverse spherical coordinate conversion.
     * 
     * @param direction Direction vector in 3D space (need not be normalized)
     * @return Vector containing [azimuth, elevation] in radians
     *         - azimuth: horizontal angle measured from the positive x-axis, range [-π, π]
     *         - elevation: vertical angle measured from the xy-plane, range [-π/2, π/2]
     */
    [[nodiscard]] Eigen::Vector2d to_angles(const Eigen::Vector3d &direction);
}

#endif // BALLISTIC_SOLVE_UTILITY_HPP