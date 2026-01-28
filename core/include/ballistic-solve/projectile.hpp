#ifndef BALLISTIC_SOLVE_PROJECTILE_HPP
#define BALLISTIC_SOLVE_PROJECTILE_HPP

namespace ballistic_solve
{
    /**
     * @brief Physical properties of a projectile for ballistic simulation.
     * 
     * Encapsulates the mass, cross-sectional area, and drag coefficient
     * needed to compute aerodynamic forces during flight.
     */
    class Projectile
    {
    public:
        /// Mass of the projectile in kilograms
        const double mass;
        
        /// Cross-sectional area in square meters (perpendicular to velocity)
        const double area;
        
        /// Dimensionless drag coefficient (Cd). Typical values:
        /// - Sphere: ~0.47
        /// - Streamlined body: ~0.04
        /// - Flat plate: ~1.28
        const double drag_coefficient;

        /**
         * @brief Construct a projectile with specified properties.
         * 
         * @param mass Mass in kilograms (default: 1 kg)
         * @param area Cross-sectional area in square meters (default: 1 m²)
         * @param drag_coefficient Dimensionless drag coefficient (default: 0.47, sphere)
         */
        Projectile(
            const double mass = 1.0,
            const double area = 1.0,
            const double drag_coefficient = 0.47);
    };
}

#endif // BALLISTIC_SOLVE_PROJECTILE_HPP