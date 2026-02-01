#include "ballistic_solve/utility.hpp"

namespace ballistic_solve
{
    Eigen::Vector3d to_direction(const Eigen::Vector2d &angles)
    {
        return Eigen::Vector3d(
            std::cos(angles.x()) * std::cos(angles.y()),
            std::cos(angles.x()) * std::sin(angles.y()),
            std::sin(angles.x()));
    }

    Eigen::Vector2d to_angles(const Eigen::Vector3d &direction)
    {
        return Eigen::Vector2d(
            std::atan2(direction.y(), direction.x()),
            std::asin(direction.z()));
    }
}
