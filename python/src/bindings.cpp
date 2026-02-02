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
    nb::class_<bs::Environment>(m, "Environment")
        .def(nb::init<>())
        .def(
            nb::init<
                Eigen::Vector3d,
                bs::Environment::AirDensity,
                bs::Environment::WindVelocity,
                bs::Environment::Temperature>(),
            nb::arg("gravity"),
            nb::arg("air_density"),
            nb::arg("wind_velocity"),
            nb::arg("temperature"))
        .def_ro("gravity", &bs::Environment::gravity)
        .def_ro("air_density", &bs::Environment::air_density)
        .def_ro("wind_velocity", &bs::Environment::wind_velocity)
        .def_ro("temperature", &bs::Environment::temperature)
        .def(
            "with_gravity",
            nb::overload_cast<double>(&bs::Environment::with_gravity, nb::const_))
        .def(
            "with_gravity",
            nb::overload_cast<Eigen::Vector3d>(&bs::Environment::with_gravity, nb::const_))
        .def(
            "with_air_density",
            nb::overload_cast<double>(&bs::Environment::with_air_density, nb::const_))
        .def(
            "with_air_density",
            nb::overload_cast<bs::Environment::AirDensity>(&bs::Environment::with_air_density, nb::const_))
        .def(
            "with_wind_velocity",
            nb::overload_cast<Eigen::Vector3d>(&bs::Environment::with_wind_velocity, nb::const_))
        .def(
            "with_wind_velocity",
            nb::overload_cast<bs::Environment::WindVelocity>(&bs::Environment::with_wind_velocity, nb::const_))
        .def(
            "with_temperature",
            nb::overload_cast<double>(&bs::Environment::with_temperature, nb::const_))
        .def(
            "with_temperature",
            nb::overload_cast<bs::Environment::Temperature>(&bs::Environment::with_temperature, nb::const_));

    nb::class_<bs::Projectile>(m, "Projectile")
        .def(nb::init<>())
        .def(
            nb::init<double, double, bs::Projectile::DragCoefficient>(),
            nb::arg("mass"),
            nb::arg("area"),
            nb::arg("drag_coefficient"))
        .def_ro("mass", &bs::Projectile::mass)
        .def_ro("area", &bs::Projectile::area)
        .def_ro("drag_coefficient", &bs::Projectile::drag_coefficient)
        .def(
            "with_mass",
            &bs::Projectile::with_mass,
            nb::arg("value"))
        .def(
            "with_area",
            &bs::Projectile::with_area,
            nb::arg("value"))
        .def(
            "with_drag_coefficient",
            nb::overload_cast<double>(&bs::Projectile::with_drag_coefficient, nb::const_),
            nb::arg("value"))
        .def(
            "with_drag_coefficient",
            nb::overload_cast<bs::Projectile::DragCoefficient>(&bs::Projectile::with_drag_coefficient, nb::const_),
            nb::arg("function"));

    nb::class_<bs::Ballistic::Trajectory>(m, "Trajectory")
        .def_ro("positions", &bs::Ballistic::Trajectory::positions)
        .def_ro("times", &bs::Ballistic::Trajectory::times);

    nb::class_<bs::Ballistic::Solution>(m, "Solution")
        .def_ro("direction", &bs::Ballistic::Solution::direction)
        .def_ro("time", &bs::Ballistic::Solution::time);

    nb::class_<bs::Ballistic>(m, "Ballistic")
        .def(
            nb::init<bs::Environment, bs::Projectile>(),
            nb::arg("environment"),
            nb::arg("projectile"))
        .def(
            nb::init<bs::Environment, bs::Projectile>(),
            nb::arg("environment"),
            nb::arg("projectile"))
        .def_ro("environment", &bs::Ballistic::environment)
        .def_ro("projectile", &bs::Ballistic::projectile)
        .def(
            "simulate",
            nb::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, double, const Eigen::Vector3d &, std::pair<double, double>>(&bs::Ballistic::simulate, nb::const_),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("direction"),
            nb::arg("time_range"))
        .def(
            "simulate",
            nb::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, double, const Eigen::Vector2d &, std::pair<double, double>>(&bs::Ballistic::simulate, nb::const_),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("angles"),
            nb::arg("time_range"))
        .def(
            "simulate",
            nb::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, double, const Eigen::Vector3d &, double>(&bs::Ballistic::simulate, nb::const_),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("direction"),
            nb::arg("time"))
        .def(
            "simulate",
            nb::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, double, const Eigen::Vector2d &, double>(&bs::Ballistic::simulate, nb::const_),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("angles"),
            nb::arg("time"))
        .def(
            "solve_earliest",
            &bs::Ballistic::solve_earliest,
            nb::arg("target_position"),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("time_range"),
            nb::arg("time_scan_interval") = 0.5)
        .def(
            "solve_latest",
            &bs::Ballistic::solve_latest,
            nb::arg("target_position"),
            nb::arg("platform_position"),
            nb::arg("platform_velocity"),
            nb::arg("projectile_speed"),
            nb::arg("time_range"),
            nb::arg("time_scan_interval") = 0.5);

    m.def(
        "to_direction",
        &bs::to_direction,
        nb::arg("angles"));
    m.def(
        "to_angles",
        &bs::to_angles,
        nb::arg("direction"));
}