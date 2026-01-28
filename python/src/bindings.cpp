#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>

#include "ballistic-solve/environment.hpp"
#include "ballistic-solve/platform.hpp"
#include "ballistic-solve/projectile.hpp"
#include "ballistic-solve/simulation.hpp"
#include "ballistic-solve/trajectory.hpp"
#include "ballistic-solve/targeting.hpp"
#include "ballistic-solve/tools.hpp"

namespace nb = nanobind;
namespace bs = ballistic_solve;

NB_MODULE(_core, m)
{
    m.doc() = "Ballistic trajectory simulation and optimization library";

    nb::class_<bs::Environment>(m, "Environment",
                                "Environmental conditions for ballistic simulation.\n\n"
                                "Encapsulates atmospheric and gravitational conditions affecting projectile motion.")
        .def(
            nb::init<
                const Eigen::Vector3d &,
                const bs::Environment::AirDensity &,
                const Eigen::Vector3d &>(),
            "Construct a custom environment.",
            nb::arg("gravity"), "Gravitational acceleration vector (m/s²)",
            nb::arg("air_density"), "Function mapping altitude (m) to air density (kg/m³)",
            nb::arg("wind_velocity"), "Wind velocity vector (m/s)")
        .def_ro("gravity", &bs::Environment::gravity,
                "Gravitational acceleration vector in m/s²")
        .def_ro("air_density", &bs::Environment::air_density,
                "Air density function ρ(altitude) in kg/m³")
        .def_ro("wind_velocity", &bs::Environment::wind_velocity,
                "Wind velocity vector in m/s")
        .def_static("vacuum", &bs::Environment::vacuum,
                    "Create a vacuum environment (no air, no wind).\n\n"
                    "Uses standard gravity but zero air density. Useful for testing.")
        .def_static("standard", &bs::Environment::standard,
                    "Create standard Earth atmosphere at sea level.\n\n"
                    "Uses standard gravity (9.81 m/s²) and exponential atmosphere model.")
        .def_static("standard_with_random_wind", &bs::Environment::standard_with_random_wind,
                    "Create standard atmosphere with moderate random wind.\n\n"
                    "Wind: horizontal σ=10 m/s (~22 mph), vertical σ=1 m/s",
                    nb::arg("seed") = std::nullopt, "Optional random seed for reproducibility")
        .def_static("standard_with_random_strong_wind", &bs::Environment::standard_with_random_strong_wind,
                    "Create standard atmosphere with strong random wind.\n\n"
                    "Wind: horizontal σ=25 m/s (~56 mph), vertical σ=5 m/s. Tropical storm conditions.",
                    nb::arg("seed") = std::nullopt, "Optional random seed for reproducibility")
        .def_static("standard_with_random_extreme_wind", &bs::Environment::standard_with_random_extreme_wind,
                    "Create standard atmosphere with extreme random wind.\n\n"
                    "Wind: horizontal σ=50 m/s (~112 mph), vertical σ=10 m/s. Category 3 hurricane.",
                    nb::arg("seed") = std::nullopt, "Optional random seed for reproducibility")
        .def_static("standard_with_random_catastrophic_wind", &bs::Environment::standard_with_random_catastrophic_wind,
                    "Create standard atmosphere with catastrophic random wind.\n\n"
                    "Wind: horizontal σ=100 m/s (~224 mph), vertical σ=20 m/s. Category 5+ hurricane.\n"
                    "Useful for stress testing numerical stability.",
                    nb::arg("seed") = std::nullopt, "Optional random seed for reproducibility");

    nb::class_<bs::Projectile>(m, "Projectile",
                               "Physical properties of a projectile for ballistic simulation.\n\n"
                               "Encapsulates the mass, cross-sectional area, and drag coefficient\n"
                               "needed to compute aerodynamic forces during flight.")
        .def(
            nb::init<double, double, double>(),
            "Construct a projectile with specified properties.",
            nb::arg("mass") = 1.0, "Mass in kilograms (default: 1 kg)",
            nb::arg("area") = 1.0, "Cross-sectional area in square meters (default: 1 m²)",
            nb::arg("drag_coefficient") = 0.47, "Dimensionless drag coefficient (default: 0.47, sphere)")
        .def_ro("mass", &bs::Projectile::mass,
                "Mass of the projectile in kilograms")
        .def_ro("area", &bs::Projectile::area,
                "Cross-sectional area in square meters (perpendicular to velocity)")
        .def_ro("drag_coefficient", &bs::Projectile::drag_coefficient,
                "Dimensionless drag coefficient. Typical values:\n"
                "- Sphere: ~0.47\n"
                "- Streamlined body: ~0.04\n"
                "- Flat plate: ~1.28");

    nb::class_<bs::Platform>(m, "Platform",
                             "Launch platform properties for ballistic simulation.\n\n"
                             "Represents the firing platform (e.g., ground position, moving vehicle, aircraft)\n"
                             "including its position, velocity, and the muzzle velocity of the weapon system.\n"
                             "The projectile's initial velocity is the vector sum of platform velocity and\n"
                             "muzzle velocity in the launch direction.")
        .def(
            nb::init<
                const Eigen::Vector3d &,
                const Eigen::Vector3d &,
                const double>(),
            "Construct a launch platform.",
            nb::arg("position"), "3D position vector in meters",
            nb::arg("velocity"), "3D velocity vector in m/s (platform motion)",
            nb::arg("muzzle_velocity"), "Scalar muzzle velocity in m/s (projectile speed relative to platform)")
        .def_ro("position", &bs::Platform::position,
                "Position of the launch platform in 3D space (meters)")
        .def_ro("velocity", &bs::Platform::velocity,
                "Velocity of the platform itself (m/s). For stationary platforms, this is zero.")
        .def_ro("muzzle_velocity", &bs::Platform::muzzle_velocity,
                "Muzzle velocity magnitude (m/s). Speed of projectile relative to platform.");

    nb::class_<bs::Trajectory>(m, "Trajectory",
                               "Container for a computed ballistic trajectory.\n\n"
                               "Stores the position history and corresponding time stamps of a projectile\n"
                               "as computed by the trajectory integration. Positions and times are recorded\n"
                               "at each adaptive integration step.")
        .def(nb::init<const std::vector<Eigen::Vector3d> &, const std::vector<double> &>(),
             "Construct a trajectory from position and time data.",
             nb::arg("positions"), "Vector of 3D positions",
             nb::arg("times"), "Vector of time stamps (must have same length as positions)")
        .def("size", &bs::Trajectory::size,
             "Get the number of recorded points in the trajectory.")
        .def("empty", &bs::Trajectory::empty,
             "Check if the trajectory is empty.")
        .def_ro("positions", &bs::Trajectory::positions,
                "Sequence of 3D positions along the trajectory")
        .def_ro("times", &bs::Trajectory::times,
                "Corresponding time stamps for each position (seconds)");

    nb::class_<bs::IntegrationOptions>(m, "IntegrationOptions",
                                       "Configuration for the adaptive ODE integrator.\n\n"
                                       "Controls the Dormand-Prince (DOPRI5) Runge-Kutta adaptive stepper.")
        .def(nb::init<const double, const double, const double, const double>(),
             nb::arg("max_step") = 1e3, "Maximum step size (seconds)",
             nb::arg("first_step") = 1e-3, "Initial step size (seconds)",
             nb::arg("atol") = 1e-6, "Absolute error tolerance",
             nb::arg("rtol") = 1e-3, "Relative error tolerance")
        .def_ro("max_step", &bs::IntegrationOptions::max_step,
                "Maximum step size (seconds)")
        .def_ro("first_step", &bs::IntegrationOptions::first_step,
                "Initial step size (seconds)")
        .def_ro("atol", &bs::IntegrationOptions::atol,
                "Absolute error tolerance")
        .def_ro("rtol", &bs::IntegrationOptions::rtol,
                "Relative error tolerance");

    nb::class_<bs::TargetingSolution>(m, "TargetingSolution",
                                      "Complete targeting solution for engaging a target.\n\n"
                                      "Contains launch angles, direction vector, and time of flight.")
        .def_ro("angles", &bs::TargetingSolution::angles,
                "Launch angles as (azimuth, elevation) in radians")
        .def_ro("direction", &bs::TargetingSolution::direction,
                "Launch direction (unit vector)")
        .def_ro("time", &bs::TargetingSolution::time,
                "Time of flight (seconds)");

    nb::enum_<bs::TargetingOptions::AngleSearchStrategy>(m, "AngleSearchStrategy",
                                                         "Strategy for nested angle optimization.")
        .value("AzimuthThenElevation", bs::TargetingOptions::AngleSearchStrategy::AzimuthThenElevation,
               "Optimize azimuth in inner loop, elevation in outer loop")
        .value("ElevationThenAzimuth", bs::TargetingOptions::AngleSearchStrategy::ElevationThenAzimuth,
               "Optimize elevation in inner loop, azimuth in outer loop");

    nb::class_<bs::TargetingOptions>(m, "TargetingOptions",
                                     "Configuration for the ballistic solver.\n\n"
                                     "Controls search strategy and iteration limits for finding firing solutions.")
        .def(nb::init<
                 const bs::TargetingOptions::AngleSearchStrategy,
                 const std::uintmax_t,
                 const std::uintmax_t,
                 const std::uintmax_t,
                 const double,
                 const double>(),
             nb::arg("angle_search_strategy") = bs::TargetingOptions::AngleSearchStrategy::AzimuthThenElevation, "Strategy for nested angle search",
             nb::arg("azimuth_max_iter") = 16, "Maximum iterations for azimuth angle search",
             nb::arg("elevation_max_iter") = 16, "Maximum iterations for elevation angle search",
             nb::arg("time_max_iter") = 16, "Maximum iterations for time-of-flight search",
             nb::arg("h") = 1e-3, "Step size for numerical differentiation",
             nb::arg("time_scan_step") = 0.5, "Time step for coarse scanning in time search (seconds)")
        .def_ro("angle_search_strategy", &bs::TargetingOptions::angle_search_strategy,
                "Strategy for nested angle search")
        .def_ro("azimuth_max_iter", &bs::TargetingOptions::azimuth_max_iter,
                "Maximum iterations for azimuth angle search")
        .def_ro("elevation_max_iter", &bs::TargetingOptions::elevation_max_iter,
                "Maximum iterations for elevation angle search")
        .def_ro("time_max_iter", &bs::TargetingOptions::time_max_iter,
                "Maximum iterations for time-of-flight search")
        .def_ro("h", &bs::TargetingOptions::h,
                "Step size for numerical differentiation")
        .def_ro("time_scan_step", &bs::TargetingOptions::time_scan_step,
                "Time step for coarse scanning in time search (seconds)");

    m.def("to_direction", &bs::to_direction,
          "Convert spherical coordinates to 3D direction vector.\n\n"
          "Args:\n"
          "    angles: Pair of (azimuth, elevation) angles in radians\n\n"
          "Returns:\n"
          "    Unit direction vector",
          nb::arg("angles"));

    m.def("to_angles", &bs::to_angles,
          "Convert 3D direction vector to spherical coordinates.\n\n"
          "Args:\n"
          "    direction: Direction vector (need not be normalized)\n\n"
          "Returns:\n"
          "    Tuple of (azimuth, elevation) in radians",
          nb::arg("direction"));

    m.def("compute_trajectory",
          nb::overload_cast<
              const bs::Environment &,
              const bs::Projectile &,
              const bs::Platform &,
              const Eigen::Vector3d &,
              const double,
              const bs::IntegrationOptions &>(&bs::compute_trajectory),
          "Compute complete ballistic trajectory with recorded time steps.\n\n"
          "Simulates projectile motion under gravity, drag, and wind using adaptive\n"
          "Runge-Kutta integration. Records position at each integration step.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    direction: Unit vector indicating launch direction\n"
          "    time: Simulation time (seconds)\n"
          "    integration_options: Integration parameters\n\n"
          "Returns:\n"
          "    Complete trajectory with positions and times",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("direction"),
          nb::arg("time"),
          nb::arg("integration_options") = bs::IntegrationOptions{});

    m.def("compute_trajectory",
          nb::overload_cast<
              const bs::Environment &,
              const bs::Projectile &,
              const bs::Platform &,
              const std::pair<double, double> &,
              const double,
              const bs::IntegrationOptions &>(&bs::compute_trajectory),
          "Compute complete ballistic trajectory from launch angles.\n\n"
          "Convenience overload that accepts (azimuth, elevation) angles.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    angles: Pair of (azimuth, elevation) angles in radians\n"
          "    time: Simulation time (seconds)\n"
          "    integration_options: Integration parameters\n\n"
          "Returns:\n"
          "    Complete trajectory with positions and times",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("angles"),
          nb::arg("time"),
          nb::arg("integration_options") = bs::IntegrationOptions{});

    m.def("compute_point",
          nb::overload_cast<
              const bs::Environment &,
              const bs::Projectile &,
              const bs::Platform &,
              const Eigen::Vector3d &,
              const double,
              const bs::IntegrationOptions &>(&bs::compute_point),
          "Compute final position only (no trajectory recording).\n\n"
          "More efficient than compute_trajectory when only the endpoint is needed.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    direction: Unit vector indicating launch direction\n"
          "    time: Simulation time (seconds)\n"
          "    integration_options: Integration parameters\n\n"
          "Returns:\n"
          "    Final position after given simulation time",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("direction"),
          nb::arg("time"),
          nb::arg("integration_options") = bs::IntegrationOptions{});

    m.def("compute_point",
          nb::overload_cast<
              const bs::Environment &,
              const bs::Projectile &,
              const bs::Platform &,
              const std::pair<double, double> &,
              const double,
              const bs::IntegrationOptions &>(&bs::compute_point),
          "Compute final position only from launch angles.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    angles: Pair of (azimuth, elevation) angles in radians\n"
          "    time: Simulation time (seconds)\n"
          "    integration_options: Integration parameters\n\n"
          "Returns:\n"
          "    Final position after given simulation time",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("angles"),
          nb::arg("time"),
          nb::arg("integration_options") = bs::IntegrationOptions{});

    m.def("find_best_azimuth_angle", &bs::find_best_azimuth_angle,
          "Find best azimuth angle for a given elevation and time.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    target_position: Target position\n"
          "    elevation_angle: Fixed elevation angle (radians)\n"
          "    time: Flight time (seconds)\n"
          "    integration_options: Integration parameters\n"
          "    targeting_options: Targeting configuration\n\n"
          "Returns:\n"
          "    Best azimuth angle (radians)",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("target_position"),
          nb::arg("elevation_angle"),
          nb::arg("time"),
          nb::arg("integration_options") = bs::IntegrationOptions{},
          nb::arg("targeting_options") = bs::TargetingOptions{});

    m.def("find_best_elevation_angle", &bs::find_best_elevation_angle,
          "Find best elevation angle for a given azimuth and time.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    target_position: Target position\n"
          "    azimuth_angle: Fixed azimuth angle (radians)\n"
          "    time: Flight time (seconds)\n"
          "    integration_options: Integration parameters\n"
          "    targeting_options: Targeting configuration\n\n"
          "Returns:\n"
          "    Best elevation angle (radians)",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("target_position"),
          nb::arg("azimuth_angle"),
          nb::arg("time"),
          nb::arg("integration_options") = bs::IntegrationOptions{},
          nb::arg("targeting_options") = bs::TargetingOptions{});

    m.def("find_best_angles", &bs::find_best_angles,
          "Find best launch angles (azimuth, elevation) for a given time.\n\n"
          "Uses nested search based on the configured angle search strategy.\n"
          "Minimizes squared distance to target.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    target_position: Target position\n"
          "    time: Flight time (seconds)\n"
          "    integration_options: Integration parameters\n"
          "    targeting_options: Targeting configuration\n\n"
          "Returns:\n"
          "    Tuple of (azimuth, elevation) in radians",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("target_position"),
          nb::arg("time"),
          nb::arg("integration_options") = bs::IntegrationOptions{},
          nb::arg("targeting_options") = bs::TargetingOptions{});

    m.def("find_best_direction", &bs::find_best_direction,
          "Find best launch direction for a given time.\n\n"
          "Convenience function that returns a direction vector instead of angles.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    target_position: Target position\n"
          "    time: Flight time (seconds)\n"
          "    integration_options: Integration parameters\n"
          "    targeting_options: Targeting configuration\n\n"
          "Returns:\n"
          "    Best launch direction (unit vector)",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("target_position"),
          nb::arg("time"),
          nb::arg("integration_options") = bs::IntegrationOptions{},
          nb::arg("targeting_options") = bs::TargetingOptions{});

    m.def("find_earliest_time", &bs::find_earliest_time,
          "Find earliest time when target can be intercepted.\n\n"
          "Searches forward through the time range to find the first moment when\n"
          "the projectile can reach the (possibly moving) target. Uses coarse\n"
          "scanning followed by root refinement.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    target_position: Function returning target position at given time\n"
          "    time_range: Tuple of (min_time, max_time) in seconds\n"
          "    integration_options: Integration parameters\n"
          "    targeting_options: Targeting configuration\n\n"
          "Returns:\n"
          "    Earliest intercept time in seconds, or None if no solution exists",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("target_position"),
          nb::arg("time_range"),
          nb::arg("integration_options") = bs::IntegrationOptions{},
          nb::arg("targeting_options") = bs::TargetingOptions{});

    m.def("find_latest_time", &bs::find_latest_time,
          "Find latest time when target can be intercepted.\n\n"
          "Searches backward through the time range to find the last moment when\n"
          "the projectile can reach the (possibly moving) target. Uses coarse\n"
          "scanning followed by root refinement.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    target_position: Function returning target position at given time\n"
          "    time_range: Tuple of (min_time, max_time) in seconds\n"
          "    integration_options: Integration parameters\n"
          "    targeting_options: Targeting configuration\n\n"
          "Returns:\n"
          "    Latest intercept time in seconds, or None if no solution exists",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("target_position"),
          nb::arg("time_range"),
          nb::arg("integration_options") = bs::IntegrationOptions{},
          nb::arg("targeting_options") = bs::TargetingOptions{});

    m.def("find_earliest_firing_solution", &bs::find_earliest_firing_solution,
          "Find earliest firing solution for intercepting a target.\n\n"
          "Combines find_earliest_time with find_best_angles to produce a complete\n"
          "targeting solution at the earliest possible intercept time.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    target_position: Function returning target position at given time\n"
          "    time_range: Tuple of (min_time, max_time) in seconds\n"
          "    integration_options: Integration parameters\n"
          "    targeting_options: Targeting configuration\n\n"
          "Returns:\n"
          "    Complete firing solution, or None if no solution exists",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("target_position"),
          nb::arg("time_range"),
          nb::arg("integration_options") = bs::IntegrationOptions{},
          nb::arg("targeting_options") = bs::TargetingOptions{});

    m.def("find_latest_firing_solution", &bs::find_latest_firing_solution,
          "Find latest firing solution for intercepting a target.\n\n"
          "Combines find_latest_time with find_best_angles to produce a complete\n"
          "targeting solution at the latest possible intercept time.\n\n"
          "Args:\n"
          "    environment: Environmental conditions\n"
          "    projectile: Projectile properties\n"
          "    platform: Launch platform\n"
          "    target_position: Function returning target position at given time\n"
          "    time_range: Tuple of (min_time, max_time) in seconds\n"
          "    integration_options: Integration parameters\n"
          "    targeting_options: Targeting configuration\n\n"
          "Returns:\n"
          "    Complete firing solution, or None if no solution exists",
          nb::arg("environment"),
          nb::arg("projectile"),
          nb::arg("platform"),
          nb::arg("target_position"),
          nb::arg("time_range"),
          nb::arg("integration_options") = bs::IntegrationOptions{},
          nb::arg("targeting_options") = bs::TargetingOptions{});

    m.def("bracket_find_root", &bs::bracket_find_root,
          "Find a bracketing interval containing a root using TOMS748 algorithm.\n\n"
          "Returns an interval [a, b] where f(a) and f(b) have opposite signs,\n"
          "guaranteeing a root exists within the interval by the intermediate value theorem.\n\n"
          "Args:\n"
          "    f: Objective function to find root of\n"
          "    a_x: Lower bound of search interval\n"
          "    b_x: Upper bound of search interval\n"
          "    max_iter: Maximum number of iterations (default: 32)\n\n"
          "Returns:\n"
          "    Bracket (a, b) containing the root\n\n"
          "Note:\n"
          "    f(a_x) and f(b_x) must have opposite signs",
          nb::arg("f"),
          nb::arg("a_x"),
          nb::arg("b_x"),
          nb::arg("max_iter") = 16);

    m.def("basin_find_minima", &bs::basin_find_minima,
          "Find minimum of a unimodal (single basin) function using Brent's method.\n\n"
          "Efficiently finds the global minimum for convex or unimodal functions.\n"
          "Uses parabolic interpolation with golden section search as fallback.\n\n"
          "Args:\n"
          "    f: Objective function to minimize\n"
          "    lo_x: Lower bound of search interval\n"
          "    hi_x: Upper bound of search interval\n"
          "    max_iter: Maximum number of iterations (default: 32)\n\n"
          "Returns:\n"
          "    Tuple of (x_min, f(x_min)) - location and value of minimum",
          nb::arg("f"),
          nb::arg("lo_x"),
          nb::arg("hi_x"),
          nb::arg("max_iter") = 16);

    m.def("sinlike_find_minima", &bs::sinlike_find_minima,
          "Find global minimum of a sine-like periodic function with one complete wave.\n\n"
          "Specialized algorithm for functions with approximately sinusoidal behavior\n"
          "containing a single global minimum per period. Uses boundary sampling to\n"
          "detect topology, then partitions the domain via root-finding and applies\n"
          "Brent's method to each subregion.\n\n"
          "Algorithm strategy:\n"
          "1. Sample near boundaries to detect if minimum is trivial (at boundary)\n"
          "2. If function is monotonic, search entire interval\n"
          "3. If multimodal, find level curve crossing and partition domain\n"
          "4. Apply basin optimization to each partition independently\n\n"
          "Args:\n"
          "    f: Objective function to minimize (should be sine-like)\n"
          "    lo_x: Lower bound of search interval\n"
          "    hi_x: Upper bound of search interval\n"
          "    h: Step size for boundary probing (default: 1e-3)\n"
          "    max_iter: Maximum iterations per optimization call (default: 32)\n\n"
          "Returns:\n"
          "    Tuple of (x_min, f(x_min)) - global minimum\n\n"
          "Note:\n"
          "    - Assumes function has at most one complete oscillation in [lo_x, hi_x]\n"
          "    - Requires hi_x - lo_x > 2*h for proper boundary sampling",
          nb::arg("f"),
          nb::arg("lo_x"),
          nb::arg("hi_x"),
          nb::arg("h") = 1e-3,
          nb::arg("max_iter") = 16);
}