#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>

#include "ballistic_solve/environment.hpp"
#include "ballistic_solve/projectile.hpp"
#include "ballistic_solve/ballistic.hpp"
#include "ballistic_solve/utility.hpp"

namespace nb = nanobind;
namespace bs = ballistic_solve;

NB_MODULE(_core, m)
{
    nb::class_<bs::Environment>(m, "Environment",
                                "Represents environmental conditions affecting projectile flight.\n\n"
                                "This class encapsulates spatially-varying environmental parameters including\n"
                                "gravity, air density, wind velocity, and temperature. These parameters can be\n"
                                "constant or vary with position in 3D space.\n"
                                "All properties are immutable; modification methods return new instances.")
        .def(nb::init<>(),
             "Default constructor.\n\n"
             "Creates an environment with all parameters set to zero:\n"
             "- gravity: (0, 0, 0) m/s²\n"
             "- air_density: 0 kg/m³ at all positions\n"
             "- wind_velocity: (0, 0, 0) m/s at all positions\n"
             "- temperature: 0 K at all positions")
        .def(
            nb::init<
                Eigen::Vector3d,
                bs::Environment::AirDensity,
                bs::Environment::WindVelocity,
                bs::Environment::Temperature>(),
            nb::arg("gravity"),
            nb::arg("air_density"),
            nb::arg("wind_velocity"),
            nb::arg("temperature"),
            "Constructs an environment with specified conditions.\n\n"
            "Parameters:\n"
            "    gravity: Gravity vector in m/s²\n"
            "    air_density: Function that returns air density for a given position\n"
            "    wind_velocity: Function that returns wind velocity for a given position\n"
            "    temperature: Function that returns temperature for a given position")
        .def_static("earth_standard", &bs::Environment::earth_standard,
                    "Creates an Earth standard atmosphere environment.\n\n"
                    "Uses International Standard Atmosphere (ISA) model:\n"
                    "- gravity: (0, 0, -9.81) m/s²\n"
                    "- air_density: ISA model ρ(h) = 1.225 * (1 - 0.0065h/288.15)^4.256 kg/m³\n"
                    "- wind_velocity: zero wind\n"
                    "- temperature: ISA model T(h) = 288.15 - 0.0065h K\n\n"
                    "Returns:\n"
                    "    Environment with Earth-like conditions")
        .def_static("moon_standard", &bs::Environment::moon_standard,
                    "Creates a lunar environment.\n\n"
                    "Moon conditions:\n"
                    "- gravity: (0, 0, -1.62) m/s² (approximately 1/6 of Earth's gravity)\n"
                    "- air_density: 0 kg/m³ (vacuum - no atmosphere)\n"
                    "- wind_velocity: (0, 0, 0) m/s (no atmosphere, no wind)\n"
                    "- temperature: 0 K (set to zero; actual lunar temperature varies dramatically)\n\n"
                    "Returns:\n"
                    "    Environment with Moon-like conditions")
        .def_static("mars_standard", &bs::Environment::mars_standard,
                    "Creates a Martian environment.\n\n"
                    "Mars conditions:\n"
                    "- gravity: (0, 0, -3.71) m/s² (approximately 38% of Earth's gravity)\n"
                    "- air_density: 0.020 kg/m³ (thin CO₂ atmosphere, about 1% of Earth's)\n"
                    "- wind_velocity: (0, 0, 0) m/s (no wind by default, though Mars has dust storms)\n"
                    "- temperature: 210 K (approximately -63°C, average surface temperature)\n\n"
                    "Returns:\n"
                    "    Environment with Mars-like conditions")
        .def_ro("gravity", &bs::Environment::gravity,
                "Gravity vector in m/s² (typically pointing downward in z-direction)")
        .def_ro("air_density", &bs::Environment::air_density,
                "Function returning air density (kg/m³) for a given position")
        .def_ro("wind_velocity", &bs::Environment::wind_velocity,
                "Function returning wind velocity vector (m/s) for a given position")
        .def_ro("temperature", &bs::Environment::temperature,
                "Function returning temperature (K) for a given position")
        .def(
            "with_gravity",
            nb::overload_cast<double>(&bs::Environment::with_gravity, nb::const_),
            nb::arg("value"),
            "Creates a new environment with constant gravity magnitude.\n\n"
            "Sets gravity to (0, 0, -value), pointing downward along the z-axis.\n\n"
            "Parameters:\n"
            "    value: Gravity magnitude in m/s² (must be positive)\n\n"
            "Returns:\n"
            "    A new Environment instance with the updated gravity")
        .def(
            "with_gravity",
            nb::overload_cast<Eigen::Vector3d>(&bs::Environment::with_gravity, nb::const_),
            nb::arg("vector"),
            "Creates a new environment with a custom gravity vector.\n\n"
            "Parameters:\n"
            "    vector: Gravity vector in m/s² (can point in any direction)\n\n"
            "Returns:\n"
            "    A new Environment instance with the updated gravity")
        .def(
            "with_air_density",
            nb::overload_cast<double>(&bs::Environment::with_air_density, nb::const_),
            nb::arg("value"),
            "Creates a new environment with constant air density.\n\n"
            "Parameters:\n"
            "    value: Constant air density in kg/m³ (must be positive, ~1.225 at sea level)\n\n"
            "Returns:\n"
            "    A new Environment instance with constant air density at all positions")
        .def(
            "with_air_density",
            nb::overload_cast<bs::Environment::AirDensity>(&bs::Environment::with_air_density, nb::const_),
            nb::arg("function"),
            "Creates a new environment with a custom air density function.\n\n"
            "Useful for modeling altitude-dependent density or other spatial variations.\n\n"
            "Parameters:\n"
            "    function: Function that computes air density from position\n\n"
            "Returns:\n"
            "    A new Environment instance with the updated air density function")
        .def(
            "with_wind_velocity",
            nb::overload_cast<Eigen::Vector3d>(&bs::Environment::with_wind_velocity, nb::const_),
            nb::arg("value"),
            "Creates a new environment with constant wind velocity.\n\n"
            "Parameters:\n"
            "    value: Constant wind velocity vector in m/s\n\n"
            "Returns:\n"
            "    A new Environment instance with constant wind at all positions")
        .def(
            "with_wind_velocity",
            nb::overload_cast<bs::Environment::WindVelocity>(&bs::Environment::with_wind_velocity, nb::const_),
            nb::arg("function"),
            "Creates a new environment with a custom wind velocity function.\n\n"
            "Useful for modeling altitude-dependent winds or spatially-varying wind fields.\n\n"
            "Parameters:\n"
            "    function: Function that computes wind velocity from position\n\n"
            "Returns:\n"
            "    A new Environment instance with the updated wind velocity function")
        .def(
            "with_temperature",
            nb::overload_cast<double>(&bs::Environment::with_temperature, nb::const_),
            nb::arg("value"),
            "Creates a new environment with constant temperature.\n\n"
            "Parameters:\n"
            "    value: Constant temperature in Kelvin (must be positive, ~288.15 K at sea level)\n\n"
            "Returns:\n"
            "    A new Environment instance with constant temperature at all positions")
        .def(
            "with_temperature",
            nb::overload_cast<bs::Environment::Temperature>(&bs::Environment::with_temperature, nb::const_),
            nb::arg("function"),
            "Creates a new environment with a custom temperature function.\n\n"
            "Useful for modeling altitude-dependent temperature or other spatial variations.\n\n"
            "Parameters:\n"
            "    function: Function that computes temperature from position\n\n"
            "Returns:\n"
            "    A new Environment instance with the updated temperature function");

    nb::class_<bs::Projectile>(m, "Projectile",
                               "Represents a projectile with physical properties for ballistic trajectory calculations.\n\n"
                               "This class encapsulates the physical characteristics of a projectile that affect its\n"
                               "flight dynamics, including mass, cross-sectional area, and drag coefficient.\n"
                               "All properties are immutable; modification methods return new instances.")
        .def(nb::init<>(),
             "Default constructor.\n\n"
             "Creates a projectile with all parameters set to zero:\n"
             "- mass: 0.0 kg\n"
             "- area: 0.0 m²\n"
             "- drag_coefficient: 0.0 (constant)\n\n"
             "Use builder methods (with_*) to configure desired parameters, or use\n"
             "static factory methods for common projectile types.")
        .def(
            nb::init<double, double, bs::Projectile::DragCoefficient>(),
            nb::arg("mass"),
            nb::arg("area"),
            nb::arg("drag_coefficient"),
            "Constructs a projectile with specified properties.\n\n"
            "Parameters:\n"
            "    mass: Mass of the projectile in kg (must be positive)\n"
            "    area: Cross-sectional area in m² (must be positive)\n"
            "    drag_coefficient: Function that returns drag coefficient for a given velocity")
        .def_static("baseball_standard", &bs::Projectile::baseball_standard,
                    "Creates a standard baseball projectile.\n\n"
                    "Official baseball properties:\n"
                    "- mass: 0.145 kg (145 grams, MLB regulation)\n"
                    "- area: 0.00426 m² (diameter 73mm, radius 36.5mm)\n"
                    "- drag_coefficient: 0.30 (typical for baseball with seams)\n\n"
                    "Returns:\n"
                    "    Projectile with standard baseball properties")
        .def_static("soccer_ball_standard", &bs::Projectile::soccer_ball_standard,
                    "Creates a standard soccer ball projectile.\n\n"
                    "FIFA regulation soccer ball:\n"
                    "- mass: 0.430 kg (430 grams, FIFA standard)\n"
                    "- area: 0.0388 m² (diameter 22cm, radius 11cm)\n"
                    "- drag_coefficient: 0.25 (smooth modern ball)\n\n"
                    "Returns:\n"
                    "    Projectile with standard soccer ball properties")
        .def_static("basketball_standard", &bs::Projectile::basketball_standard,
                    "Creates a standard basketball projectile.\n\n"
                    "NBA regulation basketball:\n"
                    "- mass: 0.624 kg (624 grams)\n"
                    "- area: 0.0457 m² (diameter 24.1cm, radius 12.05cm)\n"
                    "- drag_coefficient: 0.30 (textured surface)\n\n"
                    "Returns:\n"
                    "    Projectile with standard basketball properties")
        .def_static("golf_ball_standard", &bs::Projectile::golf_ball_standard,
                    "Creates a standard golf ball projectile.\n\n"
                    "USGA regulation golf ball:\n"
                    "- mass: 0.0459 kg (45.9 grams)\n"
                    "- area: 0.00143 m² (diameter 42.7mm, radius 21.35mm)\n"
                    "- drag_coefficient: 0.25 (dimpled surface reduces drag)\n\n"
                    "Returns:\n"
                    "    Projectile with standard golf ball properties")
        .def_ro("mass", &bs::Projectile::mass,
                "Mass of the projectile in kilograms")
        .def_ro("area", &bs::Projectile::area,
                "Cross-sectional area of the projectile in square meters")
        .def_ro("drag_coefficient", &bs::Projectile::drag_coefficient,
                "Function returning drag coefficient (dimensionless) based on velocity")
        .def(
            "with_mass",
            &bs::Projectile::with_mass,
            nb::arg("value"),
            "Creates a new projectile with a different mass.\n\n"
            "Parameters:\n"
            "    value: New mass in kg (must be positive)\n\n"
            "Returns:\n"
            "    A new Projectile instance with the updated mass")
        .def(
            "with_area",
            &bs::Projectile::with_area,
            nb::arg("value"),
            "Creates a new projectile with a different cross-sectional area.\n\n"
            "Parameters:\n"
            "    value: New cross-sectional area in m² (must be positive)\n\n"
            "Returns:\n"
            "    A new Projectile instance with the updated area")
        .def(
            "with_drag_coefficient",
            nb::overload_cast<double>(&bs::Projectile::with_drag_coefficient, nb::const_),
            nb::arg("value"),
            "Creates a new projectile with a constant drag coefficient.\n\n"
            "Parameters:\n"
            "    value: Constant drag coefficient (dimensionless, typically 0.1 to 1.0)\n\n"
            "Returns:\n"
            "    A new Projectile instance with a constant drag coefficient function")
        .def(
            "with_drag_coefficient",
            nb::overload_cast<bs::Projectile::DragCoefficient>(&bs::Projectile::with_drag_coefficient, nb::const_),
            nb::arg("function"),
            "Creates a new projectile with a custom drag coefficient function.\n\n"
            "This allows for Mach-dependent drag coefficients, which is important\n"
            "for accurate modeling at transonic and supersonic velocities.\n\n"
            "Parameters:\n"
            "    function: Function that computes drag coefficient from Mach number\n\n"
            "Returns:\n"
            "    A new Projectile instance with the updated drag coefficient function");

    nb::class_<bs::Ballistic::Trajectory>(m, "Trajectory",
                                          "Contains the computed trajectory of a projectile.")
        .def_ro("positions", &bs::Ballistic::Trajectory::positions,
                "Sequence of 3D positions along the trajectory in meters")
        .def_ro("times", &bs::Ballistic::Trajectory::times,
                "Corresponding time values for each position in seconds");

    nb::class_<bs::Ballistic::Solution>(m, "Solution",
                                        "Contains a firing solution for intercepting a target.")
        .def_ro("direction", &bs::Ballistic::Solution::direction,
                "Unit direction vector to aim the projectile")
        .def_ro("time", &bs::Ballistic::Solution::time,
                "Time of intercept in seconds");

    nb::class_<bs::Ballistic>(m, "Ballistic",
                              "Ballistic trajectory solver for projectile motion with environmental effects.\n\n"
                              "This class provides methods to simulate projectile trajectories and solve firing\n"
                              "solutions for moving or stationary targets. It accounts for gravity, air drag,\n"
                              "wind, and other environmental factors.")
        .def(
            nb::init<bs::Environment, bs::Projectile>(),
            nb::arg("environment"),
            nb::arg("projectile"),
            "Constructs a ballistic solver with given environment and projectile.\n\n"
            "Parameters:\n"
            "    environment: Environmental conditions (gravity, air density, wind, temperature)\n"
            "    projectile: Projectile properties (mass, area, drag coefficient)")
        .def(
            nb::init<bs::Environment, bs::Projectile>(),
            nb::arg("environment"),
            nb::arg("projectile"))
        .def_ro("environment", &bs::Ballistic::environment,
                "Environmental conditions affecting the projectile")
        .def_ro("projectile", &bs::Ballistic::projectile,
                "Physical properties of the projectile")
        .def(
            "simulate",
            nb::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, double, const Eigen::Vector3d &, std::pair<double, double>>(&bs::Ballistic::simulate, nb::const_),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("direction"),
            nb::arg("time_range"),
            "Simulates projectile trajectory over a time range with specified direction.\n\n"
            "Parameters:\n"
            "    platform_position: Initial position of the launching platform in meters\n"
            "    platform_velocity: Velocity of the launching platform in m/s\n"
            "    projectile_speed: Muzzle speed of the projectile relative to platform in m/s\n"
            "    direction: Unit direction vector for the projectile launch\n"
            "    time_range: Time interval (start, end) for simulation in seconds\n\n"
            "Returns:\n"
            "    Trajectory containing positions and times along the flight path")
        .def(
            "simulate",
            nb::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, double, const Eigen::Vector2d &, std::pair<double, double>>(&bs::Ballistic::simulate, nb::const_),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("angles"),
            nb::arg("time_range"),
            "Simulates projectile trajectory over a time range with specified angles.\n\n"
            "Parameters:\n"
            "    platform_position: Initial position of the launching platform in meters\n"
            "    platform_velocity: Velocity of the launching platform in m/s\n"
            "    projectile_speed: Muzzle speed of the projectile relative to platform in m/s\n"
            "    angles: Launch angles [azimuth, elevation] in radians\n"
            "    time_range: Time interval (start, end) for simulation in seconds\n\n"
            "Returns:\n"
            "    Trajectory containing positions and times along the flight path")
        .def(
            "simulate",
            nb::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, double, const Eigen::Vector3d &, double>(&bs::Ballistic::simulate, nb::const_),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("direction"),
            nb::arg("time"),
            "Simulates projectile position at a specific time with specified direction.\n\n"
            "Parameters:\n"
            "    platform_position: Initial position of the launching platform in meters\n"
            "    platform_velocity: Velocity of the launching platform in m/s\n"
            "    projectile_speed: Muzzle speed of the projectile relative to platform in m/s\n"
            "    direction: Unit direction vector for the projectile launch\n"
            "    time: Time at which to compute position in seconds\n\n"
            "Returns:\n"
            "    Projectile position at the specified time in meters")
        .def(
            "simulate",
            nb::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, double, const Eigen::Vector2d &, double>(&bs::Ballistic::simulate, nb::const_),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("angles"),
            nb::arg("time"),
            "Simulates projectile position at a specific time with specified angles.\n\n"
            "Parameters:\n"
            "    platform_position: Initial position of the launching platform in meters\n"
            "    platform_velocity: Velocity of the launching platform in m/s\n"
            "    projectile_speed: Muzzle speed of the projectile relative to platform in m/s\n"
            "    angles: Launch angles [azimuth, elevation] in radians\n"
            "    time: Time at which to compute position in seconds\n\n"
            "Returns:\n"
            "    Projectile position at the specified time in meters")
        .def(
            "solve_earliest",
            &bs::Ballistic::solve_earliest,
            nb::arg("target_position"),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("time_range"),
            nb::arg("time_scan_interval") = 0.5,
            "Finds the earliest firing solution to intercept a moving target.\n\n"
            "Searches for the earliest time within the specified range where the projectile\n"
            "can intercept the target. Uses a time-scanning approach with the given interval.\n\n"
            "Parameters:\n"
            "    target_position: Function providing target position as a function of time\n"
            "    platform_position: Initial position of the launching platform in meters\n"
            "    platform_velocity: Velocity of the launching platform in m/s\n"
            "    projectile_speed: Muzzle speed of the projectile relative to platform in m/s\n"
            "    time_range: Time interval (start, end) to search for solutions in seconds\n"
            "    time_scan_interval: Time step for scanning potential intercept times (default: 0.5 s)\n\n"
            "Returns:\n"
            "    Optional Solution containing firing direction and intercept time, or None if no solution exists")
        .def(
            "solve_latest",
            &bs::Ballistic::solve_latest,
            nb::arg("target_position"),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("time_range"),
            nb::arg("time_scan_interval") = 0.5,
            "Finds the latest firing solution to intercept a moving target.\n\n"
            "Searches for the latest time within the specified range where the projectile\n"
            "can intercept the target. Uses a time-scanning approach with the given interval.\n\n"
            "Parameters:\n"
            "    target_position: Function providing target position as a function of time\n"
            "    platform_position: Initial position of the launching platform in meters\n"
            "    platform_velocity: Velocity of the launching platform in m/s\n"
            "    projectile_speed: Muzzle speed of the projectile relative to platform in m/s\n"
            "    time_range: Time interval (start, end) to search for solutions in seconds\n"
            "    time_scan_interval: Time step for scanning potential intercept times (default: 0.5 s)\n\n"
            "Returns:\n"
            "    Optional Solution containing firing direction and intercept time, or None if no solution exists")
        .def(
            "find_best_angles",
            &bs::Ballistic::find_best_angles,
            nb::arg("target_position"),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("time"),
            "Finds optimal launch angles to hit a stationary target at a specific time.\n\n"
            "Computes the azimuth and elevation angles that minimize the intercept error\n"
            "for reaching the target position at the specified time.\n\n"
            "Parameters:\n"
            "    target_position: Position of the target in meters\n"
            "    platform_position: Initial position of the launching platform in meters\n"
            "    platform_velocity: Velocity of the launching platform in m/s\n"
            "    projectile_speed: Muzzle speed of the projectile relative to platform in m/s\n"
            "    time: Time at which intercept should occur in seconds\n\n"
            "Returns:\n"
            "    Optimal launch angles [azimuth, elevation] in radians")
        .def(
            "find_best_direction",
            &bs::Ballistic::find_best_direction,
            nb::arg("target_position"),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("time"),
            "Finds optimal launch direction to hit a stationary target at a specific time.\n\n"
            "Computes the direction that minimizes the intercept error\n"
            "for reaching the target position at the specified time.\n\n"
            "Parameters:\n"
            "    target_position: Position of the target in meters\n"
            "    platform_position: Initial position of the launching platform in meters\n"
            "    platform_velocity: Velocity of the launching platform in m/s\n"
            "    projectile_speed: Muzzle speed of the projectile relative to platform in m/s\n"
            "    time: Time at which intercept should occur in seconds\n\n"
            "Returns:\n"
            "    Optimal launch direction as a unit vector");

    m.def(
        "to_direction",
        &bs::to_direction,
        nb::arg("angles"),
        "Converts angular representation to a unit direction vector.\n\n"
        "Converts azimuth and elevation angles to a 3D unit direction vector\n"
        "using spherical coordinate conversion.\n\n"
        "Parameters:\n"
        "    angles: Vector containing [azimuth, elevation] in radians\n"
        "            - azimuth: horizontal angle measured from the positive x-axis\n"
        "            - elevation: vertical angle measured from the xy-plane\n\n"
        "Returns:\n"
        "    Unit direction vector in 3D space");
    m.def(
        "to_angles",
        &bs::to_angles,
        nb::arg("direction"),
        "Converts a direction vector to angular representation.\n\n"
        "Converts a 3D direction vector to azimuth and elevation angles\n"
        "using inverse spherical coordinate conversion.\n\n"
        "Parameters:\n"
        "    direction: Direction vector in 3D space (need not be normalized)\n\n"
        "Returns:\n"
        "    Vector containing [azimuth, elevation] in radians\n"
        "    - azimuth: horizontal angle measured from the positive x-axis, range [-π, π]\n"
        "    - elevation: vertical angle measured from the xy-plane, range [-π/2, π/2]");
}