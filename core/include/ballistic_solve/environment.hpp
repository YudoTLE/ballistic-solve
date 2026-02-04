#ifndef BALLISTIC_SOLVE_ENVIRONMENT_HPP
#define BALLISTIC_SOLVE_ENVIRONMENT_HPP

#include <Eigen/Dense>
#include <functional>

namespace ballistic_solve
{
    /**
     * @brief Represents environmental conditions affecting projectile flight.
     * 
     * This class encapsulates spatially-varying environmental parameters including
     * gravity, air density, wind velocity, and temperature. These parameters can be
     * constant or vary with position in 3D space.
     * All properties are immutable; modification methods return new instances.
     */
    class Environment
    {
    public:
        /**
         * @brief Function type for air density calculation.
         * 
         * @param position Position vector (x, y, z) in meters
         * @return Air density in kg/m³
         */
        using AirDensity = std::function<double(Eigen::Vector3d)>;

        /**
         * @brief Function type for wind velocity calculation.
         * 
         * @param position Position vector (x, y, z) in meters
         * @return Wind velocity vector in m/s
         */
        using WindVelocity = std::function<Eigen::Vector3d(Eigen::Vector3d)>;

        /**
         * @brief Function type for temperature calculation.
         * 
         * @param position Position vector (x, y, z) in meters
         * @return Temperature in Kelvin
         */
        using Temperature = std::function<double(Eigen::Vector3d)>;

    public:
        /** @brief Gravity vector in m/s² (typically pointing downward in z-direction) */
        const Eigen::Vector3d gravity;

        /** @brief Function returning air density for a given position */
        const AirDensity air_density;

        /** @brief Function returning wind velocity vector for a given position */
        const WindVelocity wind_velocity;

        /** @brief Function returning temperature for a given position */
        const Temperature temperature;

    public:
        /**
         * @brief Default constructor.
         * 
         * Creates an environment with Earth-like atmospheric conditions:
         * - gravity: (0, 0, -9.81) m/s² (standard Earth gravity pointing downward)
         * - air_density: ISA (International Standard Atmosphere) model
         *   ρ(h) = 1.225 * (1 - 0.0065h/288.15)^4.256 kg/m³
         *   where h is altitude (z-coordinate) clamped to non-negative values
         * - wind_velocity: zero wind (0, 0, 0) m/s at all positions
         * - temperature: ISA temperature model
         *   T(h) = 288.15 - 0.0065h K (15°C at sea level, -6.5°C/km lapse rate)
         *   where h is altitude (z-coordinate) clamped to non-negative values
         */
        Environment();

        /**
         * @brief Constructs an environment with specified conditions.
         * 
         * @param gravity Gravity vector in m/s²
         * @param air_density Function that returns air density for a given position
         * @param wind_velocity Function that returns wind velocity for a given position
         * @param temperature Function that returns temperature for a given position
         */
        Environment(Eigen::Vector3d gravity,
                    AirDensity air_density,
                    WindVelocity wind_velocity,
                    Temperature temperature);

    public:
        /**
         * @brief Creates a new environment with constant gravity magnitude.
         * 
         * Sets gravity to (0, 0, -value), pointing downward along the z-axis.
         * 
         * @param value Gravity magnitude in m/s² (must be positive)
         * @return A new Environment instance with the updated gravity
         */
        [[nodiscard]] Environment with_gravity(double value) const;

        /**
         * @brief Creates a new environment with a custom gravity vector.
         * 
         * @param vector Gravity vector in m/s² (can point in any direction)
         * @return A new Environment instance with the updated gravity
         */
        [[nodiscard]] Environment with_gravity(Eigen::Vector3d vector) const;

        /**
         * @brief Creates a new environment with constant air density.
         * 
         * @param value Constant air density in kg/m³ (must be positive, ~1.225 at sea level)
         * @return A new Environment instance with constant air density at all positions
         */
        [[nodiscard]] Environment with_air_density(double value) const;

        /**
         * @brief Creates a new environment with a custom air density function.
         * 
         * Useful for modeling altitude-dependent density or other spatial variations.
         * 
         * @param function Function that computes air density from position
         * @return A new Environment instance with the updated air density function
         */
        [[nodiscard]] Environment with_air_density(AirDensity function) const;

        /**
         * @brief Creates a new environment with constant wind velocity.
         * 
         * @param value Constant wind velocity vector in m/s
         * @return A new Environment instance with constant wind at all positions
         */
        [[nodiscard]] Environment with_wind_velocity(Eigen::Vector3d value) const;

        /**
         * @brief Creates a new environment with a custom wind velocity function.
         * 
         * Useful for modeling altitude-dependent winds or spatially-varying wind fields.
         * 
         * @param function Function that computes wind velocity from position
         * @return A new Environment instance with the updated wind velocity function
         */
        [[nodiscard]] Environment with_wind_velocity(WindVelocity function) const;

        /**
         * @brief Creates a new environment with constant temperature.
         * 
         * @param value Constant temperature in Kelvin (must be positive, ~288.15 K at sea level)
         * @return A new Environment instance with constant temperature at all positions
         */
        [[nodiscard]] Environment with_temperature(double value) const;

        /**
         * @brief Creates a new environment with a custom temperature function.
         * 
         * Useful for modeling altitude-dependent temperature or other spatial variations.
         * 
         * @param function Function that computes temperature from position
         * @return A new Environment instance with the updated temperature function
         */
        [[nodiscard]] Environment with_temperature(Temperature function) const;
    };
}

#endif // BALLISTIC_SOLVE_ENVIRONMENT_HPP