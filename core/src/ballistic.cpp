#include "ballistic_solve/ballistic.hpp"

#include "ballistic_solve/constants.hpp"
#include "ballistic_solve/utility.hpp"
#include <unsupported/Eigen/LevenbergMarquardt>
#include <boost/numeric/odeint.hpp>
#include <boost/math/tools/minima.hpp>
#include <boost/math/tools/roots.hpp>
#include <limits>
#include <numbers>

namespace ballistic_solve
{
    Ballistic::Ballistic(
        Environment environment,
        Projectile projectile,
        Ballistic::Targeting targeting)
        : environment(std::move(environment)),
          projectile(std::move(projectile)),
          targeting(std::move(targeting))
    {
    }

    Ballistic::Trajectory Ballistic::simulate(
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const Eigen::Vector3d &direction,
        const std::pair<double, double> time_range) const
    {
        auto stepper = boost::numeric::odeint::make_controlled(
            constants::integration::abs_tolerance,
            constants::integration::rel_tolerance,
            constants::integration::max_step_size,
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
            constants::integration::initial_step_size,
            observer);

        return Trajectory{
            .positions = trajectory_positions,
            .times = trajectory_times};
    }

    Ballistic::Trajectory Ballistic::simulate(
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

    Eigen::Vector3d Ballistic::simulate(
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const Eigen::Vector3d &direction,
        const double time) const
    {
        auto stepper = boost::numeric::odeint::make_controlled(
            constants::integration::abs_tolerance,
            constants::integration::rel_tolerance,
            constants::integration::max_step_size,
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
            constants::integration::initial_step_size);

        return x.head<3>();
    }

    Eigen::Vector3d Ballistic::simulate(
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

    std::optional<Ballistic::Solution> Ballistic::solve_earliest(
        Ballistic::TargetPosition target_position,
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

        for (double current_time = time_range.first;
             current_time <= time_range.second;
             current_time = std::min(current_time + this->targeting.time_scan_interval, time_range.second))
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
                    std::uintmax_t max_iterations = constants::root_finding::max_iterations;
                    auto [near_root_lo, near_root_hi] = boost::math::tools::toms748_solve(
                        objective,
                        undershoot_time.value(),
                        current_time,
                        constants::root_finding::tolerance,
                        max_iterations);

                    time = (near_root_lo + near_root_hi) * 0.5;
                }

                overshoot_time = current_time;
            }
            if (value < 0)
            {
                if (overshoot_time.has_value())
                {
                    std::uintmax_t max_iterations = constants::root_finding::max_iterations;
                    auto [near_root_lo, near_root_hi] = boost::math::tools::toms748_solve(
                        objective,
                        overshoot_time.value(),
                        current_time,
                        constants::root_finding::tolerance,
                        max_iterations);

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
                    time.value());

                return Solution{
                    .direction = to_direction(best_angles),
                    .time = time.value()};
            }

            if (current_time == time_range.second)
            {
                break;
            }
        }

        return std::nullopt;
    }

    std::optional<Ballistic::Solution> Ballistic::solve_latest(
        Ballistic::TargetPosition target_position,
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
                    std::uintmax_t max_iterations = constants::root_finding::max_iterations;
                    auto [near_root_lo, near_root_hi] = boost::math::tools::toms748_solve(
                        objective,
                        current_time,
                        undershoot_time.value(),
                        constants::root_finding::tolerance,
                        max_iterations);

                    time = (near_root_lo + near_root_hi) * 0.5;
                }

                overshoot_time = current_time;
            }
            if (value < 0)
            {
                if (overshoot_time.has_value())
                {
                    std::uintmax_t max_iterations = constants::root_finding::max_iterations;
                    auto [near_root_lo, near_root_hi] = boost::math::tools::toms748_solve(
                        objective,
                        current_time,
                        overshoot_time.value(),
                        constants::root_finding::tolerance,
                        max_iterations);

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
                    time.value());

                return Solution{
                    .direction = to_direction(best_angles),
                    .time = time.value()};
            }

            if (current_time == time_range.first)
            {
                break;
            }
        }

        return std::nullopt;
    }

    void Ballistic::system(const State &x, State &dxdt, const double t) const
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

    struct AngleFunctor : Eigen::DenseFunctor<double>
    {
        const Ballistic *ballistic;
        const Eigen::Vector3d target_position;
        const Eigen::Vector3d platform_position;
        const Eigen::Vector3d platform_velocity;
        const double projectile_speed;
        const double time;

        AngleFunctor(const Ballistic *ballistic,
                     const Eigen::Vector3d &target_position,
                     const Eigen::Vector3d &platform_position,
                     const Eigen::Vector3d &platform_velocity,
                     double projectile_speed,
                     double time)
            : DenseFunctor<double>(2, 3),
              ballistic(ballistic),
              target_position(target_position),
              platform_position(platform_position),
              platform_velocity(platform_velocity),
              projectile_speed(projectile_speed),
              time(time)
        {
        }

        int operator()(const InputType &angles, ValueType &fvec) const
        {
            Eigen::Vector3d computed_point = ballistic->simulate(
                platform_position,
                platform_velocity,
                projectile_speed,
                Eigen::Vector2d(angles[0], angles[1]),
                time);

            Eigen::Vector3d diff = target_position - computed_point;
            fvec[0] = diff.x();
            fvec[1] = diff.y();
            fvec[2] = diff.z();

            return 0;
        }

        int df(const InputType &angles, JacobianType &fjac) const
        {
            Eigen::VectorXd fvec_plus(3), fvec_minus(3);
            Eigen::VectorXd angles_h = angles;

            for (int i = 0; i < 2; i++)
            {
                angles_h(i) += constants::h;
                (*this)(angles_h, fvec_plus);

                angles_h(i) -= 2 * constants::h;
                (*this)(angles_h, fvec_minus);

                angles_h(i) = angles(i);

                fjac.col(i) = (fvec_plus - fvec_minus) / (2 * constants::h);
            }

            return 0;
        }
    };

    Eigen::Vector2d Ballistic::find_best_angles(
        const Eigen::Vector3d &target_position,
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const double time) const
    {
        Eigen::VectorXd angles(2);
        angles << 0.0, 0.0;

        AngleFunctor functor(this, target_position, platform_position,
                             platform_velocity, projectile_speed, time);
        Eigen::LevenbergMarquardt<AngleFunctor> lm(functor);

        lm.minimize(angles);

        return Eigen::Vector2d(angles[0], angles[1]);
    }
}