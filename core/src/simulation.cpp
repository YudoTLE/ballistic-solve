#include "ballistic-solve/simulation.hpp"

#include <boost/numeric/odeint.hpp>

using StateType = Eigen::Matrix<double, 6, 1>;

namespace ballistic_solve
{
    Eigen::Vector3d to_direction(const std::pair<double, double> &angles)
    {
        const auto &[azimuth_angle, elevation_angle] = angles;

        return Eigen::Vector3d(
            std::cos(elevation_angle) * std::cos(azimuth_angle),
            std::cos(elevation_angle) * std::sin(azimuth_angle),
            std::sin(elevation_angle));
    }

    std::pair<double, double> to_angles(const Eigen::Vector3d &direction)
    {
        return {
            std::atan2(direction.y(), direction.x()),
            std::asin(direction.z())};
    }

    Trajectory compute_trajectory(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &direction,
        const double time,
        const IntegrationOptions &integration_options)
    {
        auto dopri5_stepper = boost::numeric::odeint::make_controlled(
            integration_options.atol,
            integration_options.rtol,
            integration_options.max_step,
            boost::numeric::odeint::runge_kutta_dopri5<StateType>());

        auto ballistic_system = [&](const StateType &x, StateType &dxdt, const double t)
        {
            Eigen::Vector3d position = x.head<3>();
            Eigen::Vector3d velocity = x.tail<3>();

            double rho = environment.air_density(position.z());

            Eigen::Vector3d relative_velocity = velocity - environment.wind_velocity;
            double relative_velocity_magnitude = relative_velocity.norm();

            Eigen::Vector3d drag_force = Eigen::Vector3d::Zero();
            if (relative_velocity_magnitude > 0)
            {
                drag_force = -0.5 *
                             rho *
                             projectile.drag_coefficient *
                             projectile.area *
                             relative_velocity_magnitude *
                             relative_velocity;
            }

            Eigen::Vector3d acceleration = environment.gravity + drag_force / projectile.mass;

            dxdt.head<3>() = velocity;
            dxdt.tail<3>() = acceleration;
        };

        std::vector<Eigen::Vector3d> trajectory_positions;
        std::vector<double> trajectory_times;

        auto observer = [&](const StateType &x, double t)
        {
            trajectory_positions.push_back(x.head<3>());
            trajectory_times.push_back(t);
        };

        StateType x;
        x << platform.position, direction * platform.muzzle_velocity + platform.velocity;

        boost::numeric::odeint::integrate_adaptive(
            dopri5_stepper,
            ballistic_system,
            x,
            0.0,
            time,
            integration_options.first_step,
            observer);

        return Trajectory(std::move(trajectory_positions), std::move(trajectory_times));
    }

    Trajectory compute_trajectory(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::pair<double, double> &angles,
        const double time,
        const IntegrationOptions &integration_options)
    {
        return compute_trajectory(
            environment,
            projectile,
            platform,
            to_direction(angles),
            time,
            integration_options);
    }

    Eigen::Vector3d compute_point(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &direction,
        const double time,
        const IntegrationOptions &integration_options)
    {
        auto dopri5_stepper = boost::numeric::odeint::make_controlled(
            integration_options.atol,
            integration_options.rtol,
            integration_options.max_step,
            boost::numeric::odeint::runge_kutta_dopri5<StateType>());

        auto ballistic_system = [&](const StateType &x, StateType &dxdt, const double t)
        {
            Eigen::Vector3d position = x.head<3>();
            Eigen::Vector3d velocity = x.tail<3>();

            double rho = environment.air_density(position.z());

            Eigen::Vector3d relative_velocity = velocity - environment.wind_velocity;
            double relative_velocity_magnitude = relative_velocity.norm();

            Eigen::Vector3d drag_force = Eigen::Vector3d::Zero();
            if (relative_velocity_magnitude > 0)
            {
                drag_force = -0.5 *
                             rho *
                             projectile.drag_coefficient *
                             projectile.area *
                             relative_velocity_magnitude *
                             relative_velocity;
            }

            Eigen::Vector3d acceleration = environment.gravity + drag_force / projectile.mass;

            dxdt.head<3>() = velocity;
            dxdt.tail<3>() = acceleration;
        };

        StateType x;
        x << platform.position, direction * platform.muzzle_velocity + platform.velocity;

        boost::numeric::odeint::integrate_adaptive(
            dopri5_stepper,
            ballistic_system,
            x,
            0.0,
            time,
            integration_options.first_step);

        return x.head<3>();
    }

    Eigen::Vector3d compute_point(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::pair<double, double> &angles,
        const double time,
        const IntegrationOptions &integration_options)
    {
        return compute_point(
            environment,
            projectile,
            platform,
            to_direction(angles),
            time,
            integration_options);
    }
}