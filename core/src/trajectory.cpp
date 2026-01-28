#include "ballistic-solve/trajectory.hpp"

namespace ballistic_solve
{
    Trajectory::Trajectory(
        const std::vector<Eigen::Vector3d> &positions,
        const std::vector<double> &times)
        : positions(positions), times(times)
    {
    }

    size_t Trajectory::size() const
    {
        return positions.size();
    }

    bool Trajectory::empty() const
    {
        return positions.empty();
    }
}