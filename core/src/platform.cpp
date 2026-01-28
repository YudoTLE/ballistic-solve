#include "ballisticsolver/platform.hpp"

namespace ballisticsolver
{
    Platform::Platform(
        const Eigen::Vector3d &position,
        const Eigen::Vector3d &velocity,
        const double muzzle_velocity)
        : position(position),
          velocity(velocity),
          muzzle_velocity(muzzle_velocity)
    {
    }
}