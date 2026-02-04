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
         * The drag coefficient may vary with velocity (Mach number). This function
         * takes velocity as input and returns the corresponding drag coefficient.
         *
         * @param velocity The velocity of the projectile in m/s
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
         * Creates a projectile with default values:
         * - mass: 1.0 kg
         * - area: 1.0 m²
         * - drag_coefficient: constant 0.47 (approximate value for a sphere)
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
         * This allows for velocity-dependent drag coefficients, which is important
         * for accurate modeling at transonic and supersonic velocities.
         *
         * @param function Function that computes drag coefficient from velocity
         * @return A new Projectile instance with the updated drag coefficient function
         */
        [[nodiscard]] Projectile with_drag_coefficient(DragCoefficient function) const;
    };
}

#endif // BALLISTIC_SOLVE_PROJECTILE_HPP