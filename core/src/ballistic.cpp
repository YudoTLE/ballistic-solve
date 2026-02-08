#include "ballistic_solve/ballistic.hpp"

#include "ballistic_solve/constants.hpp"
#include "ballistic_solve/utility.hpp"
#include <unsupported/Eigen/LevenbergMarquardt>
#include <boost/numeric/odeint.hpp>
#include <boost/math/tools/minima.hpp>
#include <boost/math/tools/roots.hpp>
#include <chrono>

namespace ballistic_solve
{
    Ballistic::Ballistic(
        Environment environment,
        Projectile projectile)
        : environment(std::move(environment)),
          projectile(std::move(projectile))
    {
    }

    Ballistic::Trajectory Ballistic::simulate(
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const Eigen::Vector3d &direction,
        const bool is_normalized_direction,
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
        x << platform_position,
            platform_velocity + (is_normalized_direction ? direction : direction.normalized()) * projectile_speed;

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
        const Eigen::Vector3d &direction,
        const std::pair<double, double> time_range) const
    {
        return this->simulate(
            platform_position,
            platform_velocity,
            projectile_speed,
            direction,
            false,
            time_range);
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
            true,
            time_range);
    }

    Eigen::Vector3d Ballistic::simulate(
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const Eigen::Vector3d &direction,
        const bool is_normalized_direction,
        const double time) const
    {
        auto stepper = boost::numeric::odeint::make_controlled(
            constants::integration::abs_tolerance,
            constants::integration::rel_tolerance,
            constants::integration::max_step_size,
            boost::numeric::odeint::runge_kutta_dopri5<State>());

        State x;
        x << platform_position,
            platform_velocity + (is_normalized_direction ? direction : direction.normalized()) * projectile_speed;

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
        const Eigen::Vector3d &direction,
        const double time) const
    {
        return this->simulate(
            platform_position,
            platform_velocity,
            projectile_speed,
            direction,
            false,
            time);
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
            true,
            time);
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> Ballistic::simulate_best(
        const Eigen::Vector3d &target_position,
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const double time) const
    {
        Eigen::Vector3d direction = this->find_best_direction(
            target_position,
            platform_position,
            platform_velocity,
            projectile_speed,
            time);

        return {this->simulate(
                    platform_position,
                    platform_velocity,
                    projectile_speed,
                    direction,
                    true,
                    time),
                direction};
    }

    std::optional<Ballistic::Solution> Ballistic::solve_earliest(
        Ballistic::TargetPosition target_position,
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const std::pair<double, double> time_range,
        const double time_scan_interval) const
    {
        auto start = std::chrono::high_resolution_clock::now();

        auto objective = [&](double time)
        {
            Eigen::Vector3d txy = target_position(time);

            auto [computed_point, _] = this->simulate_best(
                txy, platform_position, platform_velocity, projectile_speed, time);

            return (txy - platform_position).dot(computed_point - txy);
        };
        auto solution = [&](double a, double b, double fa, double fb) -> std::optional<Solution>
        {
            std::uintmax_t max_iter = constants::root_finding::max_iterations;

            auto [p, q] = boost::math::tools::toms748_solve(
                objective,
                a,
                b,
                fa,
                fb,
                constants::root_finding::tolerance,
                max_iter);

            double time = p;
            if (p != q)
            {
                Eigen::Vector3d tp = target_position(p);
                Eigen::Vector3d tq = target_position(q);

                Eigen::Vector3d pp = this->simulate_best(
                                             tp,
                                             platform_position,
                                             platform_velocity,
                                             projectile_speed,
                                             p)
                                         .first;
                Eigen::Vector3d pq = this->simulate_best(
                                             tq,
                                             platform_position,
                                             platform_velocity,
                                             projectile_speed,
                                             q)
                                         .first;

                if (std::min((tp - pp).norm(), (tq - pq).norm()) > 1e-6) // TODO: USE CONSTANT
                {
                    return std::nullopt;
                }

                max_iter = constants::root_finding::max_iterations - max_iter;

                auto [x, y] = boost::math::tools::toms748_solve(
                    objective,
                    p,
                    q,
                    (tp - platform_position).dot(pp - tp),
                    (tq - platform_position).dot(pq - tq),
                    constants::root_finding::tolerance,
                    max_iter);

                time = (x + y) * 0.5;
            }

            Eigen::Vector3d txy = target_position(time);

            auto [computed_point, direction] = this->simulate_best(
                txy,
                platform_position,
                platform_velocity,
                projectile_speed,
                time);
            double error = (txy - computed_point).norm();
            if (error > 1e-9) // TODO: USE CONSTANT
            {
                return std::nullopt;
            }
            double computation_time = std::chrono::duration<double>(
                                          std::chrono::high_resolution_clock::now() - start)
                                          .count();

            return Solution{
                .direction = direction,
                .time = time,
                .error = error,
                .computation_time = computation_time};
        };

        std::optional<double> overshoot_time, undershoot_time;
        double overshoot_error, undershoot_error;

        for (double current_time = time_range.first;
             current_time <= time_range.second;
             current_time = std::min(current_time + time_scan_interval, time_range.second))
        {
            double current_error = objective(current_time);

            if (current_error >= 0)
            {
                if (undershoot_time.has_value())
                {
                    std::optional<Solution> s = solution(
                        undershoot_time.value(),
                        current_time,
                        undershoot_error,
                        current_error);
                    if (s.has_value())
                    {
                        return s;
                    }

                    undershoot_time = std::nullopt;
                }

                overshoot_time = current_time;
                overshoot_error = current_error;
            }
            else
            {
                if (overshoot_time.has_value())
                {
                    std::optional<Solution> s = solution(
                        overshoot_time.value(),
                        current_time,
                        overshoot_error,
                        current_error);
                    if (s.has_value())
                    {
                        return s;
                    }

                    overshoot_time = std::nullopt;
                }

                undershoot_time = current_time;
                undershoot_error = current_error;
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
        const std::pair<double, double> time_range,
        const double time_scan_interval) const
    {
        auto start = std::chrono::high_resolution_clock::now();

        auto objective = [&](double time)
        {
            Eigen::Vector3d txy = target_position(time);

            auto [computed_point, _] = this->simulate_best(
                txy, platform_position, platform_velocity, projectile_speed, time);

            return (txy - platform_position).dot(computed_point - txy);
        };
        auto solution = [&](double a, double b, double fa, double fb) -> std::optional<Solution>
        {
            std::uintmax_t max_iter = constants::root_finding::max_iterations;

            auto [p, q] = boost::math::tools::toms748_solve(
                objective,
                a,
                b,
                fa,
                fb,
                constants::root_finding::tolerance,
                max_iter);

            double time = p;
            if (p != q)
            {
                Eigen::Vector3d tp = target_position(p);
                Eigen::Vector3d tq = target_position(q);

                Eigen::Vector3d pp = this->simulate_best(
                                             tp,
                                             platform_position,
                                             platform_velocity,
                                             projectile_speed,
                                             p)
                                         .first;
                Eigen::Vector3d pq = this->simulate_best(
                                             tq,
                                             platform_position,
                                             platform_velocity,
                                             projectile_speed,
                                             q)
                                         .first;

                if (std::min((tp - pp).norm(), (tq - pq).norm()) > 1e-6) // TODO: USE CONSTANT
                {
                    return std::nullopt;
                }

                max_iter = constants::root_finding::max_iterations - max_iter;

                auto [x, y] = boost::math::tools::toms748_solve(
                    objective,
                    p,
                    q,
                    (tp - platform_position).dot(pp - tp),
                    (tq - platform_position).dot(pq - tq),
                    constants::root_finding::tolerance,
                    max_iter);

                time = (x + y) * 0.5;
            }

            Eigen::Vector3d txy = target_position(time);

            auto [computed_point, direction] = this->simulate_best(
                txy,
                platform_position,
                platform_velocity,
                projectile_speed,
                time);
            double error = (txy - computed_point).norm();
            if (error > 1e-9) // TODO: USE CONSTANT
            {
                return std::nullopt;
            }
            double computation_time = std::chrono::duration<double>(
                                          std::chrono::high_resolution_clock::now() - start)
                                          .count();

            return Solution{
                .direction = direction,
                .time = time,
                .error = error,
                .computation_time = computation_time};
        };

        std::optional<double> overshoot_time, undershoot_time;
        double overshoot_error, undershoot_error;

        for (double current_time = time_range.second;
             current_time >= time_range.first;
             current_time = std::max(current_time - time_scan_interval, time_range.first))
        {
            double current_error = objective(current_time);

            if (current_error >= 0)
            {
                if (undershoot_time.has_value())
                {
                    std::optional<Solution> s = solution(
                        current_time,
                        undershoot_time.value(),
                        current_error,
                        undershoot_error);
                    if (s.has_value())
                    {
                        return s;
                    }

                    undershoot_time = std::nullopt;
                }

                overshoot_time = current_time;
                overshoot_error = current_error;
            }
            else
            {
                if (overshoot_time.has_value())
                {
                    std::optional<Solution> s = solution(
                        current_time,
                        overshoot_time.value(),
                        current_error,
                        overshoot_error);
                    if (s.has_value())
                    {
                        return s;
                    }

                    overshoot_time = std::nullopt;
                }

                undershoot_time = current_time;
                undershoot_error = current_error;
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
        Eigen::VectorXd angles = to_angles(target_position - platform_position);

        AngleFunctor functor(this, target_position, platform_position,
                             platform_velocity, projectile_speed, time);
        Eigen::LevenbergMarquardt<AngleFunctor> lm(functor);

        lm.setMaxfev(constants::angle_finding::max_iterations);
        lm.minimize(angles);

        return Eigen::Vector2d(angles[0], angles[1]);
    }

    Eigen::Vector3d Ballistic::find_best_direction(
        const Eigen::Vector3d &target_position,
        const Eigen::Vector3d &platform_position,
        const Eigen::Vector3d &platform_velocity,
        const double projectile_speed,
        const double time) const
    {
        return to_direction(this->find_best_angles(
            target_position,
            platform_position,
            platform_velocity,
            projectile_speed,
            time));
    }
}