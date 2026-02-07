#include "ballistic_solve/utility.hpp"

namespace ballistic_solve
{
    Eigen::Vector3d to_direction(const Eigen::Vector2d &angles)
    {
        return Eigen::Vector3d(
            std::cos(angles.y()) * std::cos(angles.x()),
            std::cos(angles.y()) * std::sin(angles.x()),
            std::sin(angles.y()));
    }

    Eigen::Vector2d to_angles(const Eigen::Vector3d &direction)
    {
        Eigen::Vector3d normalized_direction = direction.normalized();
        return Eigen::Vector2d(
            std::atan2(normalized_direction.y(), normalized_direction.x()),
            std::asin(normalized_direction.z()));
    }
}
