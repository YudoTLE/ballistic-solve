#ifndef BALLISTIC_SOLVE_ENVIRONMENT_HPP
#define BALLISTIC_SOLVE_ENVIRONMENT_HPP

#include <functional>
#include <optional>
#include <Eigen/Dense>

namespace ballistic_solve
{
    /**
     * @brief Environmental conditions for ballistic simulation.
     *
     * Encapsulates atmospheric and gravitational conditions affecting projectile motion,
     * including gravity, altitude-dependent air density, and wind velocity.
     */
    class Environment
    {
    public:
        /// Function type for air density as a function of altitude: ρ(h) in kg/m³
        using AirDensity = std::function<double(double)>;

        /// Gravitational acceleration vector in m/s² (typically [0, 0, -9.81])
        const Eigen::Vector3d gravity;

        /// Air density function ρ(altitude) in kg/m³, where altitude is in meters
        const AirDensity air_density;

        /// Wind velocity vector in m/s
        const Eigen::Vector3d wind_velocity;

        /**
         * @brief Construct a custom environment.
         *
         * @param gravity Gravitational acceleration vector (m/s²)
         * @param air_density Function mapping altitude (m) to air density (kg/m³)
         * @param wind_velocity Wind velocity vector (m/s)
         */
        Environment(
            const Eigen::Vector3d &gravity,
            const AirDensity &air_density,
            const Eigen::Vector3d &wind_velocity);

        /**
         * @brief Create a vacuum environment (no air, no wind).
         *
         * Uses standard gravity but zero air density everywhere.
         * Useful for testing or space-like conditions.
         *
         * @return Environment Vacuum environment with g = -9.81 m/s²
         */
        static Environment vacuum();

        /**
         * @brief Create standard Earth atmosphere at sea level.
         *
         * Uses:
         * - Standard gravity: 9.81 m/s² (downward)
         * - Exponential atmosphere: ρ(h) = 1.225 * exp(-h/8500) kg/m³
         * - No wind
         *
         * @return Environment Standard Earth conditions
         */
        static Environment standard();

        /**
         * @brief Create standard atmosphere with moderate random wind.
         *
         * Wind is sampled from normal distributions:
         * - Horizontal components (x, y): σ = 10 m/s (~22 mph)
         * - Vertical component (z): σ = 1 m/s
         *
         * Represents typical windy day conditions.
         *
         * @param seed Optional random seed for reproducibility
         * @return Environment Standard conditions with moderate random wind
         */
        static Environment standard_with_random_wind(std::optional<unsigned int> seed = std::nullopt);

        /**
         * @brief Create standard atmosphere with strong random wind.
         *
         * Wind is sampled from normal distributions:
         * - Horizontal components (x, y): σ = 25 m/s (~56 mph)
         * - Vertical component (z): σ = 5 m/s
         *
         * Represents tropical storm or gale force wind conditions.
         *
         * @param seed Optional random seed for reproducibility
         * @return Environment Standard conditions with strong random wind
         */
        static Environment standard_with_random_strong_wind(std::optional<unsigned int> seed = std::nullopt);

        /**
         * @brief Create standard atmosphere with extreme random wind.
         *
         * Wind is sampled from normal distributions:
         * - Horizontal components (x, y): σ = 50 m/s (~112 mph)
         * - Vertical component (z): σ = 10 m/s
         *
         * Represents Category 3 hurricane conditions.
         *
         * @param seed Optional random seed for reproducibility
         * @return Environment Standard conditions with extreme random wind
         */
        static Environment standard_with_random_extreme_wind(std::optional<unsigned int> seed = std::nullopt);

        /**
         * @brief Create standard atmosphere with catastrophic random wind.
         *
         * Wind is sampled from normal distributions:
         * - Horizontal components (x, y): σ = 100 m/s (~224 mph)
         * - Vertical component (z): σ = 20 m/s
         *
         * Represents Category 5+ hurricane or strongest tornado conditions.
         * Useful for stress testing numerical stability.
         *
         * @param seed Optional random seed for reproducibility
         * @return Environment Standard conditions with catastrophic random wind
         */
        static Environment standard_with_random_catastrophic_wind(std::optional<unsigned int> seed = std::nullopt);
    };
}

#endif // BALLISTIC_SOLVE_ENVIRONMENT_HPP