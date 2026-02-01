#ifndef BALLISTIC_SOLVE_TOOLS_INL
#define BALLISTIC_SOLVE_TOOLS_INL

#include "./tools.hpp"

#include <boost/math/tools/minima.hpp>
#include <boost/math/tools/roots.hpp>
#include <limits>

static int digits = std::numeric_limits<double>::digits;
static boost::math::tools::eps_tolerance<double> tol(digits);

namespace ballistic_solve
{
    template <concepts::ScalarFunction F>
    std::pair<double, double> bracket_find_root(
        F &&f,
        const double a_x, const double b_x,
        std::uintmax_t max_iter)
    {
        return boost::math::tools::toms748_solve(std::forward<F>(f), a_x, b_x, tol, max_iter);
    }

    template <concepts::ScalarFunction F>
    std::pair<double, double> basin_find_minima(
        F &&f,
        const double lo_x, const double hi_x,
        std::uintmax_t max_iter)
    {
        return boost::math::tools::brent_find_minima(std::forward<F>(f), lo_x, hi_x, digits, max_iter);
    }

    template <concepts::ScalarFunction F>
    std::pair<double, double> sinlike_find_minima(
        F &&f,
        const double lo_x, const double hi_x,
        const double h,
        std::uintmax_t max_iter)
    {
        double near_lo_x = lo_x + h;
        double near_hi_x = hi_x - h;

        std::pair<double, double> anchor{lo_x, f(lo_x)};
        std::pair<double, double> near_lo{near_lo_x, f(near_lo_x)};
        std::pair<double, double> near_hi{near_hi_x, f(near_hi_x)};

        if (std::min(near_lo.second, near_hi.second) > anchor.second)
        {
            return anchor;
        }

        std::pair<double, double> global_minima = anchor;
        if (std::max(near_lo.second, near_hi.second) < anchor.second)
        {
            std::pair<double, double> local_minima = basin_find_minima(
                f,
                lo_x,
                hi_x,
                max_iter);

            if (local_minima.second < global_minima.second)
            {
                global_minima = local_minima;
            }
        }
        else
        {
            auto [near_root_lo_x, near_root_hi_x] = bracket_find_root(
                [&](const double x) -> double
                { return f(x) - anchor.second; },
                near_lo_x,
                near_hi_x,
                max_iter);

            if (near_lo.second < anchor.second)
            {
                std::pair<double, double> local_minima = basin_find_minima(
                    f,
                    lo_x,
                    near_root_lo_x,
                    max_iter);

                if (local_minima.second < global_minima.second)
                {
                    global_minima = local_minima;
                }
            }
            if (near_hi.second < anchor.second)
            {
                std::pair<double, double> local_minima = basin_find_minima(
                    f,
                    near_root_hi_x,
                    hi_x,
                    max_iter);

                if (local_minima.second < global_minima.second)
                {
                    global_minima = local_minima;
                }
            }
        }

        return global_minima;
    }
}

#endif // BALLISTIC_SOLVE_TOOLS_INL