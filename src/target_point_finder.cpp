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
    const Eigen::Isometry3d& pose)
{
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    // Collect all in-range detections using simple min/max angle check
    std::vector<const LaserDetection*> in_range_detections;
    int out_of_range_count = 0;

    for (const auto& detection : detections)
    {
        // Simple range check: detection.angle must be between min and max
        bool in_range = (detection.angle >= params_.min_sweep_angle) &&
                        (detection.angle <= params_.max_sweep_angle);

        if (in_range)
        {
            in_range_detections.push_back(&detection);
        }
        else
        {
            out_of_range_count++;
        }
    }

    // Sort detections by angle distance from forward (0°)
    // This prioritizes forward-facing detections
    std::sort(in_range_detections.begin(),
        in_range_detections.end(),
        [](const LaserDetection* a, const LaserDetection* b)
        {
            // Calculate angular distance from forward (0° or 2π)
            auto angle_from_forward = [](float angle)
            {
                float dist_from_zero = std::abs(angle);
                float dist_from_2pi = std::abs(angle - 2.0f * M_PI);
                return std::min(dist_from_zero, dist_from_2pi);
            };
            return angle_from_forward(a->angle) < angle_from_forward(b->angle);
        });

    // Selection strategy:
    // 1. Scan from front to back: if any detection is below min_wall_distance, choose it
    // 2. Otherwise, choose the detection with minimum distance
    const LaserDetection* nearest = nullptr;
    float min_distance = std::numeric_limits<float>::max();

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

    int in_range_count = in_range_detections.size();

    RCLCPP_INFO_THROTTLE(logger_,
        steady_clock,
        1000,
        "=== SWEEP SEARCH === total_detections=%zu, in_range=%d, out_of_range=%d",
        detections.size(),
        in_range_count,
        out_of_range_count);

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

        // 1. Create transform for detection in base_link frame
        const Eigen::Isometry3d base_link_T_detection = std::invoke(
            [&nearest]
            {
                Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
                t.translation() = Eigen::Vector3d{nearest->x(), nearest->y(), 0.0};
                return t;
            });

        // 2. Transform to odom frame: odom_T_target = odom_T_base_link * base_link_T_detection
        Eigen::Isometry3d target_point_odom = pose * base_link_T_detection;

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
