#include "turtlebot3_wall_following_node/target_point_finder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace turtlebot3
{

TargetPointFinder::TargetPointFinder(const Params& params, rclcpp::Logger logger) :
    params_{params}, logger_{logger}
{
}

std::optional<Eigen::Isometry3d> TargetPointFinder::find_target_point(
    const std::vector<LaserDetection>& detections,
    const Eigen::Isometry3d& pose) const
{
    const rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    std::vector<const LaserDetection*> in_range_detections;

    for (const auto& detection : detections)
    {
        const bool in_range{(detection.angle >= params_.min_sweep_angle) &&
                            (detection.angle <= params_.max_sweep_angle)};

        if (in_range)
        {
            in_range_detections.push_back(&detection);
        }
    }

    std::sort(in_range_detections.begin(),
        in_range_detections.end(),
        [](const LaserDetection* a, const LaserDetection* b)
        {
            auto angle_from_forward = [](float angle)
            {
                const float dist_from_zero{std::abs(angle)};
                const float dist_from_2pi{std::abs(angle - static_cast<float>(2.0 * M_PI))};
                return std::min(dist_from_zero, dist_from_2pi);
            };
            return angle_from_forward(a->angle) < angle_from_forward(b->angle);
        });

    const LaserDetection* nearest{nullptr};
    float min_distance{std::numeric_limits<float>::max()};

    for (const auto* detection : in_range_detections)
    {
        // Track minimum distance
        if (detection->distance < min_distance)
        {
            min_distance = detection->distance;
            nearest = detection;
        }

        // Prioritize: if this forward-facing detection is below threshold, use it immediately
        if (detection->distance < params_.min_wall_distance)
        {
            nearest = detection;
            RCLCPP_DEBUG_THROTTLE(logger_,
                steady_clock,
                1000,
                "Selected forward detection below threshold: angle=%.3f rad (%.1f°), dist=%.3f m",
                detection->angle,
                detection->angle * 180.0 / M_PI,
                detection->distance);
            break;
        }
    }

    RCLCPP_INFO_THROTTLE(logger_,
        steady_clock,
        1000,
        "=== SWEEP SEARCH === total_detections=%zu, in_range=%zu",
        detections.size(),
        in_range_detections.size());

    RCLCPP_DEBUG_THROTTLE(logger_,
        steady_clock,
        1000,
        "Sweep angles: min=%.2f rad (%.1f°), max=%.2f rad (%.1f°)",
        params_.min_sweep_angle,
        params_.min_sweep_angle * 180.0 / M_PI,
        params_.max_sweep_angle,
        params_.max_sweep_angle * 180.0 / M_PI);

    // If we found a point and it's within max detection range, use it as target
    if (nearest && nearest->distance <= params_.max_detection_range)
    {
        RCLCPP_INFO_THROTTLE(logger_,
            steady_clock,
            1000,
            "Target detection found: angle=%.3f rad (%.1f°), distance=%.3f m, base_link_pos=[%.3f, "
            "%.3f]",
            nearest->angle,
            nearest->angle * 180.0 / M_PI,
            nearest->distance,
            nearest->x(),
            nearest->y());

        const Eigen::Isometry3d base_link_T_detection{std::invoke(
            [&nearest]
            {
                Eigen::Isometry3d t{Eigen::Isometry3d::Identity()};
                t.translation() = Eigen::Vector3d{nearest->x(), nearest->y(), 0.0};
                return t;
            })};

        const Eigen::Isometry3d target_point_odom{pose * base_link_T_detection};

        RCLCPP_DEBUG_STREAM_THROTTLE(logger_,
            steady_clock,
            1000,
            "Transform base_link_T_detection:\n"
                << base_link_T_detection.matrix() << "\nTransform odom_T_target:\n"
                << target_point_odom.matrix());

        RCLCPP_INFO_THROTTLE(logger_,
            steady_clock,
            1000,
            "Target set in odom frame: pos=[%.3f, %.3f, %.3f]",
            target_point_odom.translation().x(),
            target_point_odom.translation().y(),
            target_point_odom.translation().z());

        return target_point_odom;
    }
    else
    {
        // Clear target if no valid point found
        RCLCPP_DEBUG_THROTTLE(logger_,
            steady_clock,
            1000,
            "No valid target found (nearest=%s, max_range=%.2f)",
            nearest ? "exists but out of range" : "not found",
            params_.max_detection_range);
        return std::nullopt;
    }
}

} // namespace turtlebot3
