#ifndef BALLISTIC_SOLVE_TOOLS_HPP
#define BALLISTIC_SOLVE_TOOLS_HPP

#include <concepts>

namespace ballistic_solve
{
    namespace concepts
    {
        template <typename F>
        concept ScalarFunction =
            std::regular_invocable<F, double> &&
            std::copy_constructible<F> &&
            std::convertible_to<std::invoke_result_t<F, double>, double>;
    }

    template <concepts::ScalarFunction F>
    std::pair<double, double> bracket_find_root(
        F &&f,
        const double a_x, const double b_x,
        std::uintmax_t max_iter);

    template <concepts::ScalarFunction F>
    std::pair<double, double> basin_find_minima(
        F &&f,
        const double lo_x, const double hi_x,
        std::uintmax_t max_iter);

    template <concepts::ScalarFunction F>
    std::pair<double, double> sinlike_find_minima(
        F &&f,
        const double lo_x, const double hi_x,
        const double h,
        std::uintmax_t max_iter);
}

#include "./tools.inl"

#endif // BALLISTIC_SOLVE_TOOLS_HPP