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

using AirDensity = std::function<double(Eigen::Vector3d)>;
using WindVelocity = std::function<Eigen::Vector3d(Eigen::Vector3d)>;
using Temperature = std::function<double(Eigen::Vector3d)>;
using DragCoefficient = std::function<double(double)>;
using TargetPosition = std::function<Eigen::Vector3d(double)>;

using ConcreteEnvironment = bs::Environment<AirDensity, WindVelocity, Temperature>;
using ConcreteProjectile = bs::Projectile<DragCoefficient>;
using ConcreteBallistic = bs::Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>;

NB_MODULE(_core, m)
{
    nb::class_<ConcreteEnvironment>(m, "Environment")
        .def(
            nb::init<
                Eigen::Vector3d,
                AirDensity,
                WindVelocity,
                Temperature>(),
            nb::arg("gravity"),
            nb::arg("air_density"),
            nb::arg("wind_velocity"),
            nb::arg("temperature"))
        .def(nb::init<>())
        .def_ro("gravity", &ConcreteEnvironment::gravity)
        .def_ro("air_density", &ConcreteEnvironment::air_density)
        .def_ro("wind_velocity", &ConcreteEnvironment::wind_velocity)
        .def_ro("temperature", &ConcreteEnvironment::temperature)
        // with_gravity
        .def("with_gravity", [](const ConcreteEnvironment &self, double value)
             { return self.with_gravity(value); }, nb::arg("value"))
        .def("with_gravity", [](const ConcreteEnvironment &self, Eigen::Vector3d value)
             { return self.with_gravity(value); }, nb::arg("value"))
        // with_air_density
        .def("with_air_density", [](const ConcreteEnvironment &self, double value)
             { return self.with_air_density(value); }, nb::arg("value"))
        .def("with_air_density", [](const ConcreteEnvironment &self, std::function<double(double)> f)
             { return self.with_air_density(std::move(f)); }, nb::arg("altitude_function"))
        .def("with_air_density", [](const ConcreteEnvironment &self, AirDensity f)
             { return self.with_air_density(std::move(f)); }, nb::arg("spatial_function"))
        // with_wind_velocity
        .def("with_wind_velocity", [](const ConcreteEnvironment &self, Eigen::Vector3d value)
             { return self.with_wind_velocity(value); }, nb::arg("value"))
        .def("with_wind_velocity", [](const ConcreteEnvironment &self, std::function<Eigen::Vector3d(double)> f)
             { return self.with_wind_velocity(std::move(f)); }, nb::arg("altitude_function"))
        .def("with_wind_velocity", [](const ConcreteEnvironment &self, WindVelocity f)
             { return self.with_wind_velocity(std::move(f)); }, nb::arg("spatial_function"))
        // with_temperature
        .def("with_temperature", [](const ConcreteEnvironment &self, double value)
             { return self.with_temperature(value); }, nb::arg("value"))
        .def("with_temperature", [](const ConcreteEnvironment &self, std::function<double(double)> f)
             { return self.with_temperature(std::move(f)); }, nb::arg("altitude_function"))
        .def("with_temperature", [](const ConcreteEnvironment &self, Temperature f)
             { return self.with_temperature(std::move(f)); }, nb::arg("spatial_function"));

    nb::class_<ConcreteProjectile>(m, "Projectile")
        .def(
            nb::init<double, double, DragCoefficient>(),
            nb::arg("mass"),
            nb::arg("area"),
            nb::arg("drag_coefficient"))
        .def(nb::init<>())
        .def_ro("mass", &ConcreteProjectile::mass)
        .def_ro("area", &ConcreteProjectile::area)
        .def_ro("drag_coefficient", &ConcreteProjectile::drag_coefficient)
        // with_mass
        .def("with_mass", [](const ConcreteProjectile &self, double value)
             { return self.with_mass(value); }, nb::arg("value"))
        // with_area
        .def("with_area", [](const ConcreteProjectile &self, double value)
             { return self.with_area(value); }, nb::arg("value"))
        // with_drag_coefficient
        .def("with_drag_coefficient", [](const ConcreteProjectile &self, double value)
             { return self.with_drag_coefficient(value); }, nb::arg("value"))
        .def("with_drag_coefficient", [](const ConcreteProjectile &self, DragCoefficient f)
             { return self.with_drag_coefficient(std::move(f)); }, nb::arg("function"));

    nb::class_<ConcreteBallistic::Trajectory>(m, "Trajectory")
        .def_ro("positions", &ConcreteBallistic::Trajectory::positions)
        .def_ro("times", &ConcreteBallistic::Trajectory::times);

    nb::class_<ConcreteBallistic::Solution>(m, "Solution")
        .def_ro("direction", &ConcreteBallistic::Solution::direction)
        .def_ro("time", &ConcreteBallistic::Solution::time);

    nb::class_<ConcreteBallistic::Integration>(m, "Integration")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double>(),
             nb::arg("max_step_size"),
             nb::arg("initial_step_size"),
             nb::arg("absolute_tolerance"),
             nb::arg("relative_tolerance"))
        .def_ro("max_step_size", &ConcreteBallistic::Integration::max_step_size)
        .def_ro("initial_step_size", &ConcreteBallistic::Integration::initial_step_size)
        .def_ro("absolute_tolerance", &ConcreteBallistic::Integration::absolute_tolerance)
        .def_ro("relative_tolerance", &ConcreteBallistic::Integration::relative_tolerance);

    nb::class_<ConcreteBallistic::Targeting>(m, "Targeting")
        .def(nb::init<>())
        .def(nb::init<double, double, std::uintmax_t, std::uintmax_t>(),
             nb::arg("h"),
             nb::arg("time_scan_interval"),
             nb::arg("angle_max_iteration"),
             nb::arg("time_max_iteration"))
        .def_ro("h", &ConcreteBallistic::Targeting::h)
        .def_ro("time_scan_interval", &ConcreteBallistic::Targeting::time_scan_interval)
        .def_ro("angle_max_iteration", &ConcreteBallistic::Targeting::angle_max_iteration)
        .def_ro("time_max_iteration", &ConcreteBallistic::Targeting::time_max_iteration);

    // Ballistic main class
    nb::class_<ConcreteBallistic>(m, "Ballistic")
        .def(nb::init<ConcreteEnvironment, ConcreteProjectile>(),
             nb::arg("environment"),
             nb::arg("projectile"))
        .def(nb::init<ConcreteEnvironment, ConcreteProjectile, ConcreteBallistic::Integration>(),
             nb::arg("environment"),
             nb::arg("projectile"),
             nb::arg("integration"))
        .def(nb::init<ConcreteEnvironment, ConcreteProjectile, ConcreteBallistic::Integration, ConcreteBallistic::Targeting>(),
             nb::arg("environment"),
             nb::arg("projectile"),
             nb::arg("integration"),
             nb::arg("targeting"))
        .def_ro("environment", &ConcreteBallistic::environment)
        .def_ro("projectile", &ConcreteBallistic::projectile)
        .def_ro("integration", &ConcreteBallistic::integration)
        .def_ro("targeting", &ConcreteBallistic::targeting)
        // simulates
        .def("simulate", [](const ConcreteBallistic &self, const Eigen::Vector3d &platform_position, const Eigen::Vector3d &platform_velocity, double projectile_speed, const Eigen::Vector3d &direction, std::pair<double, double> time_range)
             { return self.simulate(platform_position, platform_velocity, projectile_speed, direction, time_range); }, nb::arg("platform_position"), nb::arg("platform_velocity"), nb::arg("projectile_speed"), nb::arg("direction"), nb::arg("time_range"))
        .def("simulate", [](const ConcreteBallistic &self, const Eigen::Vector3d &platform_position, const Eigen::Vector3d &platform_velocity, double projectile_speed, const Eigen::Vector2d &angles, std::pair<double, double> time_range)
             { return self.simulate(platform_position, platform_velocity, projectile_speed, angles, time_range); }, nb::arg("platform_position"), nb::arg("platform_velocity"), nb::arg("projectile_speed"), nb::arg("angles"), nb::arg("time_range"))
        .def("simulate", [](const ConcreteBallistic &self, const Eigen::Vector3d &platform_position, const Eigen::Vector3d &platform_velocity, double projectile_speed, const Eigen::Vector3d &direction, double time)
             { return self.simulate(platform_position, platform_velocity, projectile_speed, direction, time); }, nb::arg("platform_position"), nb::arg("platform_velocity"), nb::arg("projectile_speed"), nb::arg("direction"), nb::arg("time"))
        .def("simulate", [](const ConcreteBallistic &self, const Eigen::Vector3d &platform_position, const Eigen::Vector3d &platform_velocity, double projectile_speed, const Eigen::Vector2d &angles, double time)
             { return self.simulate(platform_position, platform_velocity, projectile_speed, angles, time); }, nb::arg("platform_position"), nb::arg("platform_velocity"), nb::arg("projectile_speed"), nb::arg("angles"), nb::arg("time"))
        // solves
        .def("solve_earliest", [](const ConcreteBallistic &self, TargetPosition target_position, const Eigen::Vector3d &platform_position, const Eigen::Vector3d &platform_velocity, double projectile_speed, std::pair<double, double> time_range)
             { return self.solve_earliest(std::move(target_position), platform_position, platform_velocity, projectile_speed, time_range); }, nb::arg("target_position"), nb::arg("platform_position"), nb::arg("platform_velocity"), nb::arg("projectile_speed"), nb::arg("time_range"))
        .def("solve_latest", [](const ConcreteBallistic &self, TargetPosition target_position, const Eigen::Vector3d &platform_position, const Eigen::Vector3d &platform_velocity, double projectile_speed, std::pair<double, double> time_range)
             { return self.solve_latest(std::move(target_position), platform_position, platform_velocity, projectile_speed, time_range); }, nb::arg("target_position"), nb::arg("platform_position"), nb::arg("platform_velocity"), nb::arg("projectile_speed"), nb::arg("time_range"));

    m.def("to_direction", &bs::to_direction, nb::arg("angles"));
    m.def("to_angles", &bs::to_angles, nb::arg("direction"));
}