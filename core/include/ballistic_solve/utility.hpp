

#ifndef BALLISTIC_SOLVE_UTILITY_HPP
#define BALLISTIC_SOLVE_UTILITY_HPP

#include "Eigen/Dense"

namespace ballistic_solve
{
    [[nodiscard]] Eigen::Vector3d to_direction(const Eigen::Vector2d &angles);

    [[nodiscard]] Eigen::Vector2d to_angles(const Eigen::Vector3d &direction);
}

#endif // BALLISTIC_SOLVE_UTILITY_HPP