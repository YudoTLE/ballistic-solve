#ifndef BALLISTIC_SOLVE_TRAJECTORY_HPP
#define BALLISTIC_SOLVE_TRAJECTORY_HPP

#include <Eigen/Dense>
#include <vector>

namespace ballistic_solve
{
    /**
     * @brief Container for a computed ballistic trajectory.
     *
     * Stores the position history and corresponding time stamps of a projectile
     * as computed by the trajectory integration. Positions and times are recorded
     * at each adaptive integration step.
     */
    class Trajectory
    {
    public:
        /// Sequence of 3D positions along the trajectory
        const std::vector<Eigen::Vector3d> positions;

        /// Corresponding time stamps for each position (seconds)
        const std::vector<double> times;

    public:
        /**
         * @brief Construct a trajectory from position and time data.
         *
         * @param positions Vector of 3D positions
         * @param times Vector of time stamps (must have same length as positions)
         */
        Trajectory(const std::vector<Eigen::Vector3d> &positions,
                   const std::vector<double> &times);

        /**
         * @brief Get the number of recorded points in the trajectory.
         *
         * @return size_t Number of trajectory points
         */
        size_t size() const;

        /**
         * @brief Check if the trajectory is empty.
         *
         * @return true if no points are recorded, false otherwise
         */
        bool empty() const;
    };
}

#endif // BALLISTIC_SOLVE_TRAJECTORY_HPP