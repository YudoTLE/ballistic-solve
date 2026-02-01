#ifndef BALLISTIC_SOLVE_BALLISTIC_INL
#define BALLISTIC_SOLVE_BALLISTIC_INL

#include "./ballistic.hpp"

#include "./tools.hpp"
#include "./utility.hpp"
#include <boost/numeric/odeint.hpp>
#include <numbers>

namespace ballistic_solve
{
    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::Ballistic(
        Environment<AirDensity, WindVelocity, Temperature> environment,
        Projectile<DragCoefficient> projectile,
        Ballistic::Integration integration,
        Ballistic::Targeting targeting)
        : environment(std::move(environment)),
          projectile(std::move(projectile)),
          integration(std::move(integration)),
          targeting(std::move(targeting))
    {
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    typename Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::Trajectory
    Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::simulate(
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const Eigen::Vector3d &direction,
        const std::pair<double, double> time_range) const
    {
        auto stepper = boost::numeric::odeint::make_controlled(
            this->integration.absolute_tolerance,
            this->integration.relative_tolerance,
            this->integration.max_step_size,
            boost::numeric::odeint::runge_kutta_dopri5<State>());

        std::vector<Eigen::Vector3d> trajectory_positions;
        std::vector<double> trajectory_times;

        auto observer = [&](const State &x, double t)
        {
            trajectory_positions.push_back(x.head<3>());
            trajectory_times.push_back(t);
        };

        State x;
        x << platform_position, platform_velocity + direction * projectile_speed;

        boost::numeric::odeint::integrate_adaptive(
            stepper,
            [this](const State &x, State &dxdt, const double t)
            { this->system(x, dxdt, t); },
            x,
            time_range.first,
            time_range.second,
            this->integration.initial_step_size,
            observer);

        return Trajectory{
            .positions = trajectory_positions,
            .times = trajectory_times};
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    typename Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::Trajectory
    Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::simulate(
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const Eigen::Vector2d &angles,
        const std::pair<double, double> time_range) const
    {
        return this->simulate(
            platform_position,
            platform_velocity,
            projectile_speed,
            to_direction(angles),
            time_range);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    Eigen::Vector3d Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::simulate(
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const Eigen::Vector3d &direction,
        const double time) const
    {
        auto stepper = boost::numeric::odeint::make_controlled(
            this->integration.absolute_tolerance,
            this->integration.relative_tolerance,
            this->integration.max_step_size,
            boost::numeric::odeint::runge_kutta_dopri5<State>());

        State x;
        x << platform_position, platform_velocity + direction * projectile_speed;

        boost::numeric::odeint::integrate_adaptive(
            stepper,
            [this](const State &x, State &dxdt, const double t)
            { this->system(x, dxdt, t); },
            x,
            0.0,
            time,
            this->integration.initial_step_size);

        return x.head<3>();
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    Eigen::Vector3d Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::simulate(
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const Eigen::Vector2d &angles,
        const double time) const
    {
        return this->simulate(
            platform_position,
            platform_velocity,
            projectile_speed,
            to_direction(angles),
            time);
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    template <concepts::TimeBasedTargetPosition F>
    std::optional<typename Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::Solution>
    Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::solve_earliest(
        F target_position,
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const std::pair<double, double> time_range) const
    {
        Eigen::Vector2d best_angles;
        double best_angles_penalty;

        auto objective = [&](const double time) -> double
        {
            Eigen::Vector3d current_target_position = target_position(time);

            Eigen::Vector2d angles = this->find_best_angles(
                current_target_position,
                platform_position,
                platform_velocity,
                projectile_speed,
                time);

            Eigen::Vector3d computed_point = this->simulate(
                platform_position,
                platform_velocity,
                projectile_speed,
                angles,
                time);

            // positive = overshoot, negative = undershoot
            double angles_penalty = (current_target_position - platform_position).dot(computed_point - current_target_position);

            if (angles_penalty < best_angles_penalty)
            {
                best_angles = angles;
                best_angles_penalty = angles_penalty;
            }

            return angles_penalty;
        };

        std::optional<double> overshoot_time, undershoot_time;

        for (double time = time_range.first;
             time <= time_range.second;
             time = std::min(time + this->targeting.time_scan_interval, time_range.second))
        {
            best_angles = Eigen::Vector2d::Zero();
            best_angles_penalty = std::numeric_limits<double>::infinity();

            double value = objective(time);
            if (value == 0)
            {
                return Solution{
                    .direction = to_direction(best_angles),
                    .time = time};
            }
            if (value > 0)
            {
                if (undershoot_time.has_value())
                {
                    auto [near_root_lo, near_root_hi] = bracket_find_root(
                        objective,
                        undershoot_time.value(),
                        time,
                        this->targeting.time_max_iteration);

                    return Solution{
                        .direction = to_direction(best_angles),
                        .time = (near_root_lo + near_root_hi) * 0.5};
                }

                overshoot_time = time;
            }
            if (value < 0)
            {
                if (overshoot_time.has_value())
                {
                    auto [near_root_lo, near_root_hi] = bracket_find_root(
                        objective,
                        overshoot_time.value(),
                        time,
                        this->targeting.time_max_iteration);

                    return Solution{
                        .direction = to_direction(best_angles),
                        .time = (near_root_lo + near_root_hi) * 0.5};
                }

                undershoot_time = time;
            }

            if (time == time_range.second)
            {
                break;
            }
        }

        return std::nullopt;
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    template <concepts::TimeBasedTargetPosition F>
    std::optional<typename Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::Solution>
    Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::solve_latest(
        F target_position,
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const std::pair<double, double> time_range) const
    {
        auto objective = [&](const double time) -> double
        {
            Eigen::Vector3d current_target_position = target_position(time);

            Eigen::Vector2d angles = this->find_best_angles(
                current_target_position,
                platform_position,
                platform_velocity,
                projectile_speed,
                time);

            Eigen::Vector3d computed_point = this->simulate(
                platform_position,
                platform_velocity,
                projectile_speed,
                angles,
                time);

            // positive = overshoot, negative = undershoot
            return (current_target_position - platform_position).dot(computed_point - current_target_position);
        };

        std::optional<double> overshoot_time, undershoot_time;

        for (double current_time = time_range.second;
             current_time >= time_range.first;
             current_time = std::max(current_time - this->targeting.time_scan_interval, time_range.first))
        {
            std::optional<double> time;

            double value = objective(current_time);
            if (value == 0)
            {
                time = current_time;
            }
            if (value > 0)
            {
                if (undershoot_time.has_value())
                {
                    auto [near_root_lo, near_root_hi] = bracket_find_root(
                        objective,
                        current_time,
                        undershoot_time.value(),
                        this->targeting.time_max_iteration);

                    time = (near_root_lo + near_root_hi) * 0.5;
                }

                overshoot_time = current_time;
            }
            if (value < 0)
            {
                if (overshoot_time.has_value())
                {
                    auto [near_root_lo, near_root_hi] = bracket_find_root(
                        objective,
                        current_time,
                        overshoot_time.value(),
                        this->targeting.time_max_iteration);

                    time = (near_root_lo + near_root_hi) * 0.5;
                }

                undershoot_time = current_time;
            }

            if (time.has_value())
            {
                Eigen::Vector2d best_angles = this->find_best_angles(
                    target_position(time.value()),
                    platform_position,
                    platform_velocity,
                    projectile_speed,
                    time.value()
                );

                return Solution{
                    .direction = to_direction(best_angles),
                    .time = time.value()
                };
            }

            if (current_time == time_range.first)
            {
                break;
            }
        }

        return std::nullopt;
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    void Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::system(const State &x, State &dxdt, const double t) const
    {
        Eigen::Vector3d position = x.head<3>();
        Eigen::Vector3d velocity = x.tail<3>();

        Eigen::Vector3d relative_velocity = velocity - this->environment.wind_velocity(position);
        double relative_velocity_magnitude = relative_velocity.norm();

        Eigen::Vector3d drag_force = Eigen::Vector3d::Zero();
        if (relative_velocity_magnitude > 0)
        {
            drag_force = -0.5 *
                         this->environment.air_density(position) *
                         this->projectile.drag_coefficient(relative_velocity_magnitude * this->environment.temperature(position)) *
                         this->projectile.area *
                         relative_velocity_magnitude *
                         relative_velocity;
        }

        Eigen::Vector3d acceleration = this->environment.gravity + drag_force / this->projectile.mass;

        dxdt.head<3>() = velocity;
        dxdt.tail<3>() = acceleration;
    }

    template <
        concepts::SpatialBasedAirDensity AirDensity,
        concepts::SpatialBasedWindVelocity WindVelocity,
        concepts::SpatialBasedTemperature Temperature,
        concepts::MachBasedDragCoefficient DragCoefficient>
    Eigen::Vector2d Ballistic<AirDensity, WindVelocity, Temperature, DragCoefficient>::find_best_angles(
        const Eigen::Vector3d &target_position,
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const double time) const
    {
        auto find_best_azimuth = [&](const double elevation) -> std::pair<double, double>
        {
            auto objective = [&](const double azimuth) -> double
            {
                Eigen::Vector3d computed_point = this->simulate(
                    platform_position,
                    platform_velocity,
                    projectile_speed,
                    Eigen::Vector2d(azimuth, elevation),
                    time);
                return (target_position - computed_point).squaredNorm();
            };

            return sinlike_find_minima(
                objective, 0.0, std::numbers::pi * 2,
                this->targeting.h, this->targeting.angle_max_iteration);
        };

        auto objective = [&](const double elevation)
        {
            return find_best_azimuth(elevation).second;
        };

        double best_elevation = basin_find_minima(
                                    objective,
                                    std::numbers::pi * -0.5,
                                    std::numbers::pi * 0.5,
                                    this->targeting.angle_max_iteration)
                                    .first;
        double best_azimuth = find_best_azimuth(best_elevation)
                                  .first;

        return Eigen::Vector2d(best_azimuth, best_elevation);
    }
}

#endif // BALLISTIC_SOLVE_BALLISTIC_INL