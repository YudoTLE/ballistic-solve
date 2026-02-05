#ifndef BALLISTIC_SOLVE_PROJECTILE_HPP
#define BALLISTIC_SOLVE_PROJECTILE_HPP

#include <Eigen/Dense>
#include <functional>

namespace ballistic_solve
{
    /**
     * @brief Represents a projectile with physical properties for ballistic trajectory calculations.
     *
     * This class encapsulates the physical characteristics of a projectile that affect its
     * flight dynamics, including mass, cross-sectional area, and drag coefficient.
     * All properties are immutable; modification methods return new instances.
     */
    class Projectile
    {
    public:
        /**
         * @brief Function type for drag coefficient calculation.
         *
         * The drag coefficient may vary with Mach number. This function
         * takes Mach number as input and returns the corresponding drag coefficient.
         *
         * @param mach The Mach number of the projectile (dimensionless)
         * @return The drag coefficient (dimensionless)
         */
        using DragCoefficient = std::function<double(double)>;

    public:
        /** @brief Mass of the projectile in kilograms */
        const double mass;

        /** @brief Cross-sectional area of the projectile in square meters */
        const double area;

        /** @brief Function returning drag coefficient based on velocity */
        const DragCoefficient drag_coefficient;

    public:
        /**
         * @brief Default constructor.
         *
         * Creates a projectile with all parameters set to zero:
         * - mass: 0.0 kg
         * - area: 0.0 m²
         * - drag_coefficient: 0.0 (constant)
         *
         * Use builder methods (with_*) to configure desired parameters, or use
         * static factory methods for common projectile types.
         */
        Projectile();

        /**
         * @brief Constructs a projectile with specified properties.
         *
         * @param mass Mass of the projectile in kg (must be positive)
         * @param area Cross-sectional area in m² (must be positive)
         * @param drag_coefficient Function that returns drag coefficient for a given velocity
         */
        Projectile(double mass, double area, DragCoefficient drag_coefficient);

    public:
        /**
         * @brief Creates a standard baseball projectile.
         *
         * Official baseball properties:
         * - mass: 0.145 kg (145 grams, MLB regulation)
         * - area: 0.00426 m² (diameter 73mm, radius 36.5mm)
         * - drag_coefficient: 0.30 (typical for baseball with seams)
         *
         * @return Projectile with standard baseball properties
         */
        [[nodiscard]] static Projectile baseball_standard();

        /**
         * @brief Creates a standard soccer ball projectile.
         *
         * FIFA regulation soccer ball:
         * - mass: 0.430 kg (430 grams, FIFA standard)
         * - area: 0.0388 m² (diameter 22cm, radius 11cm)
         * - drag_coefficient: 0.25 (smooth modern ball)
         *
         * @return Projectile with standard soccer ball properties
         */
        [[nodiscard]] static Projectile soccer_ball_standard();

        /**
         * @brief Creates a standard basketball projectile.
         *
         * NBA regulation basketball:
         * - mass: 0.624 kg (624 grams)
         * - area: 0.0457 m² (diameter 24.1cm, radius 12.05cm)
         * - drag_coefficient: 0.30 (textured surface)
         *
         * @return Projectile with standard basketball properties
         */
        [[nodiscard]] static Projectile basketball_standard();

        /**
         * @brief Creates a standard golf ball projectile.
         *
         * USGA regulation golf ball:
         * - mass: 0.0459 kg (45.9 grams)
         * - area: 0.00143 m² (diameter 42.7mm, radius 21.35mm)
         * - drag_coefficient: 0.25 (dimpled surface reduces drag)
         *
         * @return Projectile with standard golf ball properties
         */
        [[nodiscard]] static Projectile golf_ball_standard();

    public:
        /**
         * @brief Creates a new projectile with a different mass.
         *
         * @param value New mass in kg (must be positive)
         * @return A new Projectile instance with the updated mass
         */
        [[nodiscard]] Projectile with_mass(double value) const;

        /**
         * @brief Creates a new projectile with a different cross-sectional area.
         *
         * @param value New cross-sectional area in m² (must be positive)
         * @return A new Projectile instance with the updated area
         */
        [[nodiscard]] Projectile with_area(double value) const;

        /**
         * @brief Creates a new projectile with a constant drag coefficient.
         *
         * @param value Constant drag coefficient (dimensionless, typically 0.1 to 1.0)
         * @return A new Projectile instance with a constant drag coefficient function
         */
        [[nodiscard]] Projectile with_drag_coefficient(double value) const;

        /**
         * @brief Creates a new projectile with a custom drag coefficient function.
         *
         * This allows for Mach-dependent drag coefficients, which is important
         * for accurate modeling at transonic and supersonic velocities.
         *
         * @param function Function that computes drag coefficient from Mach number
         * @return A new Projectile instance with the updated drag coefficient function
         */
        [[nodiscard]] Projectile with_drag_coefficient(DragCoefficient function) const;
    };
}

#endif // BALLISTIC_SOLVE_PROJECTILE_HPP