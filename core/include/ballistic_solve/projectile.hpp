#ifndef BALLISTIC_SOLVE_PROJECTILE_HPP
#define BALLISTIC_SOLVE_PROJECTILE_HPP

#include <concepts>
#include <type_traits>

namespace ballistic_solve
{
    namespace concepts
    {
        template <typename F>
        concept MachBasedDragCoefficient =
            std::regular_invocable<F, double> &&
            std::copy_constructible<F> &&
            std::convertible_to<std::invoke_result_t<F, double>, double>;
    }

    template <concepts::MachBasedDragCoefficient DragCoefficient>
    class Projectile
    {
    public:
        const double mass;

        const double area;

        const DragCoefficient drag_coefficient;

    public:
        Projectile();

        Projectile(double mass, double area, DragCoefficient drag_coefficient);

    public:
        [[nodiscard]] auto with_mass(double value) const;

        [[nodiscard]] auto with_area(double val) const;

        [[nodiscard]] auto with_drag_coefficient(double value) const;

        template <concepts::MachBasedDragCoefficient F>
        [[nodiscard]] auto with_drag_coefficient(F &&function) const;
    };
}

#include "./projectile.inl"

#endif // BALLISTIC_SOLVE_PROJECTILE_HPP