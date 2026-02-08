#ifndef BALLISTIC_SOLVE_CONSTANTS_HPP
#define BALLISTIC_SOLVE_CONSTANTS_HPP

#include <boost/math/tools/roots.hpp>
#include <limits>

namespace ballistic_solve
{
    namespace constants
    {
        inline constexpr int digits = std::numeric_limits<double>::digits;
        inline constexpr double h = 1e-6;

        namespace angle_finding {
            inline constexpr std::uintmax_t max_iterations = 25;
        }

        namespace root_finding {
            inline const boost::math::tools::eps_tolerance<double> tolerance(digits);
            inline constexpr std::uintmax_t max_iterations = 40;
        }

        namespace integration
        {
            inline constexpr double max_step_size = std::numeric_limits<double>::infinity();
            inline constexpr double initial_step_size = 1e-3;
            inline constexpr double abs_tolerance = 1e-6;
            inline constexpr double rel_tolerance = 1e-3;
        }
    }
}

#endif // BALLISTIC_SOLVE_CONSTANTS_HPP