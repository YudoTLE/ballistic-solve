#ifndef BALLISTIC_SOLVE_TOOLS_H
#define BALLISTIC_SOLVE_TOOLS_H

#include <Eigen/Dense>
#include <numbers>
#include <functional>

namespace ballistic_solve
{
    /**
     * @brief Find a bracketing interval containing a root using TOMS748 algorithm.
     *
     * Returns an interval [a, b] where f(a) and f(b) have opposite signs,
     * guaranteeing a root exists within the interval by the intermediate value theorem.
     *
     * @param f Objective function to find root of
     * @param a_x Lower bound of search interval
     * @param b_x Upper bound of search interval
     * @param max_iter Maximum number of iterations (default: 32)
     * @return std::pair<double, double> Bracket [a, b] containing the root
     *
     * @note f(a_x) and f(b_x) must have opposite signs
     */
    std::pair<double, double> bracket_find_root(
        const std::function<double(double)> &f,
        const double a_x, const double b_x,
        std::uintmax_t max_iter = 16);

    /**
     * @brief Find minimum of a unimodal (single basin) function using Brent's method.
     *
     * Efficiently finds the global minimum for convex or unimodal functions.
     * Uses parabolic interpolation with golden section search as fallback.
     *
     * @param f Objective function to minimize
     * @param lo_x Lower bound of search interval
     * @param hi_x Upper bound of search interval
     * @param max_iter Maximum number of iterations (default: 32)
     * @return std::pair<double, double> (x_min, f(x_min)) - location and value of minimum
     */
    std::pair<double, double> basin_find_minima(
        const std::function<double(double)> &f,
        const double lo_x, const double hi_x,
        std::uintmax_t max_iter = 16);

    /**
     * @brief Find global minimum of a sine-like periodic function with one complete wave.
     *
     * Specialized algorithm for functions with approximately sinusoidal behavior
     * containing a single global minimum per period. Uses boundary sampling to
     * detect topology, then partitions the domain via root-finding and applies
     * Brent's method to each subregion.
     *
     * Algorithm strategy:
     * 1. Sample near boundaries to detect if minimum is trivial (at boundary)
     * 2. If function is monotonic, search entire interval
     * 3. If multimodal, find level curve crossing and partition domain
     * 4. Apply basin optimization to each partition independently
     *
     * @param f Objective function to minimize (should be sine-like)
     * @param lo_x Lower bound of search interval
     * @param hi_x Upper bound of search interval
     * @param h Step size for boundary probing (default: 1e-3)
     * @param max_iter Maximum iterations per optimization call (default: 32)
     * @return std::pair<double, double> (x_min, f(x_min)) - global minimum
     *
     * @note Assumes function has at most one complete oscillation in [lo_x, hi_x]
     * @note Requires hi_x - lo_x > 2*h for proper boundary sampling
     */
    std::pair<double, double> sinlike_find_minima(
        const std::function<double(double)> &f,
        const double lo_x, const double hi_x,
        const double h = 1e-3,
        std::uintmax_t max_iter = 16);
}

#endif // BALLISTIC_SOLVE_TOOLS_H