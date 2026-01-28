#include "ballistic-solve/targeting.hpp"

#include "ballistic-solve/simulation.hpp"
#include "ballistic-solve/tools.hpp"
#include <numbers>
#include <optional>

namespace ballistic_solve
{
    double find_best_azimuth_angle(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &target_position,
        const double elevation_angle,
        const double time,
        const IntegrationOptions &integration_options,
        const TargetingOptions &targeting_options)
    {
        auto objective = [&](const double azimuth_angle) -> double
        {
            Eigen::Vector3d computed_point = compute_point(
                environment,
                projectile,
                platform,
                std::make_pair(azimuth_angle, elevation_angle),
                time,
                integration_options);

            return (target_position - computed_point).squaredNorm();
        };

        return sinlike_find_minima(
                   objective, 0.0, std::numbers::pi * 2,
                   targeting_options.h, targeting_options.azimuth_max_iter)
            .first;
    }

    double find_best_elevation_angle(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &target_position,
        const double azimuth_angle,
        const double time,
        const IntegrationOptions &integration_options,
        const TargetingOptions &targeting_options)
    {
        auto objective = [&](const double elevation_angle) -> double
        {
            Eigen::Vector3d computed_point = compute_point(
                environment,
                projectile,
                platform,
                std::make_pair(azimuth_angle, elevation_angle),
                time,
                integration_options);

            return (target_position - computed_point).squaredNorm();
        };

        return sinlike_find_minima(
                   objective, 0.0, std::numbers::pi * 2,
                   targeting_options.h, targeting_options.elevation_max_iter)
            .first;
    }

    std::pair<double, double> find_best_angles(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &target_position,
        const double time,
        const IntegrationOptions &integration_options,
        const TargetingOptions &targeting_options)
    {
        switch (targeting_options.angle_search_strategy)
        {
        case TargetingOptions::AngleSearchStrategy::AzimuthThenElevation:
        {
            auto objective = [&](const double elevation_angle) -> double
            {
                double azimuth_angle = find_best_azimuth_angle(
                    environment,
                    projectile,
                    platform,
                    target_position,
                    elevation_angle,
                    time,
                    integration_options,
                    targeting_options);

                Eigen::Vector3d computed_point = compute_point(
                    environment,
                    projectile,
                    platform,
                    std::make_pair(azimuth_angle, elevation_angle),
                    time,
                    integration_options);

                return (target_position - computed_point).squaredNorm();
            };

            double elevation_angle = basin_find_minima(
                                         objective,
                                         std::numbers::pi * -0.5,
                                         std::numbers::pi * 0.5,
                                         targeting_options.elevation_max_iter)
                                         .first;
            double azimuth_angle = find_best_azimuth_angle(
                environment,
                projectile,
                platform,
                target_position,
                elevation_angle,
                time,
                integration_options,
                targeting_options);

            return {azimuth_angle, elevation_angle};
        }

        case TargetingOptions::AngleSearchStrategy::ElevationThenAzimuth:
        {
            auto objective = [&](const double azimuth_angle) -> double
            {
                double elevation_angle = find_best_elevation_angle(
                    environment,
                    projectile,
                    platform,
                    target_position,
                    azimuth_angle,
                    time,
                    integration_options,
                    targeting_options);

                Eigen::Vector3d computed_point = compute_point(
                    environment,
                    projectile,
                    platform,
                    std::make_pair(azimuth_angle, elevation_angle),
                    time,
                    integration_options);

                return (target_position - computed_point).squaredNorm();
            };

            double azimuth_angle = basin_find_minima(
                                       objective,
                                       std::numbers::pi * -0.5,
                                       std::numbers::pi * 0.5,
                                       targeting_options.azimuth_max_iter)
                                       .first;
            double elevation_angle = find_best_elevation_angle(
                environment,
                projectile,
                platform,
                target_position,
                azimuth_angle,
                time,
                integration_options,
                targeting_options);

            return {azimuth_angle, elevation_angle};
        }
        }
    }

    Eigen::Vector3d find_best_direction(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const Eigen::Vector3d &target_position,
        const double time,
        const IntegrationOptions &integration_options,
        const TargetingOptions &targeting_options)
    {
        return to_direction(find_best_angles(
            environment,
            projectile,
            platform,
            target_position,
            time,
            integration_options,
            targeting_options));
    }

    std::optional<double> find_earliest_time(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::function<Eigen::Vector3d(double)> &target_position,
        const std::pair<double, double> &time_range,
        const IntegrationOptions &integration_options,
        const TargetingOptions &targeting_options)
    {
        auto &[min_time, max_time] = time_range;

        auto objective = [&](const double time) -> double
        {
            Eigen::Vector3d current_target_position = target_position(time);

            std::pair<double, double> angles = find_best_angles(
                environment,
                projectile,
                platform,
                current_target_position,
                time,
                integration_options,
                targeting_options);

            Eigen::Vector3d computed_point = compute_point(
                environment,
                projectile,
                platform,
                angles,
                time,
                integration_options);

            // positive = overshoot, negative = undershoot
            return (current_target_position - platform.position).dot(computed_point - current_target_position);
        };

        std::optional<double> overshoot_time, undershoot_time;

        for (double time = min_time; time <= max_time; time = std::min(time + targeting_options.time_scan_step, max_time))
        {
            double value = objective(time);
            if (value == 0)
            {
                return time;
            }
            if (value > 0)
            {
                if (undershoot_time.has_value())
                {
                    auto [near_root_lo, near_root_hi] = bracket_find_root(
                        objective,
                        undershoot_time.value(),
                        time,
                        targeting_options.time_max_iter);

                    return (near_root_lo + near_root_hi) * 0.5;
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
                        targeting_options.time_max_iter);

                    return (near_root_lo + near_root_hi) * 0.5;
                }

                undershoot_time = time;
            }

            if (time == max_time)
            {
                break;
            }
        }

        return std::nullopt;
    }

    std::optional<double> find_latest_time(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::function<Eigen::Vector3d(double)> &target_position,
        const std::pair<double, double> &time_range,
        const IntegrationOptions &integration_options,
        const TargetingOptions &targeting_options)
    {
        auto &[min_time, max_time] = time_range;

        auto objective = [&](const double time) -> double
        {
            Eigen::Vector3d current_target_position = target_position(time);

            std::pair<double, double> angles = find_best_angles(
                environment,
                projectile,
                platform,
                current_target_position,
                time,
                integration_options,
                targeting_options);

            Eigen::Vector3d computed_point = compute_point(
                environment,
                projectile,
                platform,
                angles,
                time,
                integration_options);

            // positive = overshoot, negative = undershoot
            return (current_target_position - platform.position).dot(computed_point - current_target_position);
        };

        std::optional<double> overshoot_time, undershoot_time;

        for (double time = max_time; time >= min_time; time = std::max(time - targeting_options.time_scan_step, min_time))
        {
            double value = objective(time);
            if (value == 0)
            {
                return time;
            }
            if (value > 0)
            {
                if (undershoot_time.has_value())
                {
                    auto [near_root_lo, near_root_hi] = bracket_find_root(
                        objective,
                        time,
                        undershoot_time.value(),
                        targeting_options.time_max_iter);

                    return (near_root_lo + near_root_hi) * 0.5;
                }

                overshoot_time = time;
            }
            if (value < 0)
            {
                if (overshoot_time.has_value())
                {
                    auto [near_root_lo, near_root_hi] = bracket_find_root(
                        objective,
                        time,
                        overshoot_time.value(),
                        targeting_options.time_max_iter);

                    return (near_root_lo + near_root_hi) * 0.5;
                }

                undershoot_time = time;
            }

            if (time == min_time)
            {
                break;
            }
        }

        return std::nullopt;
    }

    std::optional<TargetingSolution> find_earliest_firing_solution(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::function<Eigen::Vector3d(double)> &target_position,
        const std::pair<double, double> &time_range,
        const IntegrationOptions &integration_options,
        const TargetingOptions &targeting_options)
    {
        std::optional<double> time = find_earliest_time(
            environment,
            projectile,
            platform,
            target_position,
            time_range,
            integration_options,
            targeting_options);
        if (!time.has_value())
        {
            return std::nullopt;
        }

        std::pair<double, double> angles = find_best_angles(
            environment,
            projectile,
            platform,
            target_position(time.value()),
            time.value(),
            integration_options,
            targeting_options);
        Eigen::Vector3d direction = to_direction(angles);

        return TargetingSolution{
            .angles = angles,
            .direction = direction,
            .time = time.value()};
    }

    std::optional<TargetingSolution> find_latest_firing_solution(
        const Environment &environment,
        const Projectile &projectile,
        const Platform &platform,
        const std::function<Eigen::Vector3d(double)> &target_position,
        const std::pair<double, double> &time_range,
        const IntegrationOptions &integration_options,
        const TargetingOptions &targeting_options)
    {
        std::optional<double> time = find_latest_time(
            environment,
            projectile,
            platform,
            target_position,
            time_range,
            integration_options,
            targeting_options);
        if (!time.has_value())
        {
            return std::nullopt;
        }

        std::pair<double, double> angles = find_best_angles(
            environment,
            projectile,
            platform,
            target_position(time.value()),
            time.value(),
            integration_options,
            targeting_options);
        Eigen::Vector3d direction = to_direction(angles);

        return TargetingSolution{
            .angles = angles,
            .direction = direction,
            .time = time.value()};
    }
}