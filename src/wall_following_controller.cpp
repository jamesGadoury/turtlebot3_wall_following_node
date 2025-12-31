#include "turtlebot3_wall_following_node/wall_following_controller.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

namespace turtlebot3
{

WallFollowingController::WallFollowingController(rclcpp::Logger logger,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster) :
    WallFollowingController(Config{}, logger, tf_broadcaster)
{
}

WallFollowingController::WallFollowingController(const Config& config,
    rclcpp::Logger logger,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster) :
    config_{config},
    robot_params_{get_turtlebot3_params()},
    logger_{logger},
    tf_broadcaster_{tf_broadcaster}
{
}

void WallFollowingController::reset()
{
    target_point_odom_.reset();
    reached_min_distance_ = false;
}

void WallFollowingController::find_and_update_target_point(
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
        bool in_range = (detection.angle >= config_.min_sweep_angle) &&
                        (detection.angle <= config_.max_sweep_angle);

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
        if (detection->distance < config_.min_wall_distance)
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
        config_.min_sweep_angle,
        config_.min_sweep_angle * 180.0 / M_PI,
        config_.max_sweep_angle,
        config_.max_sweep_angle * 180.0 / M_PI);

    // If we found a point and it's within max detection range, use it as target
    if (nearest && nearest->distance <= config_.max_detection_range)
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
        target_point_odom_ = pose * base_link_T_detection;

        RCLCPP_DEBUG_STREAM_THROTTLE(logger_,
            steady_clock,
            1000,
            "Transform base_link_T_detection:\n"
                << base_link_T_detection.matrix() << "\nTransform odom_T_target:\n"
                << target_point_odom_->matrix());

        RCLCPP_INFO_THROTTLE(logger_,
            steady_clock,
            1000,
            "Target set in odom frame: pos=[%.3f, %.3f, %.3f]",
            target_point_odom_->translation().x(),
            target_point_odom_->translation().y(),
            target_point_odom_->translation().z());
    }
    else
    {
        // Clear target if no valid point found
        RCLCPP_DEBUG_THROTTLE(logger_,
            steady_clock,
            1000,
            "No valid target found (nearest=%s, max_range=%.2f)",
            nearest ? "exists but out of range" : "not found",
            config_.max_detection_range);
        target_point_odom_.reset();
    }
}

ControlInput WallFollowingController::update(const SystemResponse& input)
{
    ControlInput output;
    output.cmd_vel.linear.x = 0.0;
    output.cmd_vel.angular.z = 0.0;
    output.is_complete = false;

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    // Log controller inputs
    Eigen::Vector3d robot_position = input.pose.translation();
    double robot_yaw = std::atan2(input.pose.rotation()(1, 0), input.pose.rotation()(0, 0));
    RCLCPP_INFO_THROTTLE(logger_,
        steady_clock,
        1000,
        "=== CONTROLLER INPUT === detections=%zu, mode=%s, has_target=%d, robot_pos=[%.3f, %.3f], "
        "robot_yaw=%.3f rad (%.1f°)",
        input.detections.size(),
        config_.continuous ? "CONTINUOUS" : "ONE-SHOT",
        target_point_odom_.has_value(),
        robot_position.x(),
        robot_position.y(),
        robot_yaw,
        robot_yaw * 180.0 / M_PI);

    // Step 1: Find and update target point
    // In continuous mode: always update
    // In one-shot mode: update only if not already set
    if (config_.continuous || !target_point_odom_.has_value())
    {
        find_and_update_target_point(input.detections, input.pose);
    }

    // Step 2: If target_point_odom_ is set, move forward and optionally align
    if (target_point_odom_.has_value())
    {

        // Broadcast TF for target point in odom frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = input.timestamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "wall_follow_target_point";

        // Convert Eigen::Isometry3d to Transform using tf2_eigen
        // Note: toMsg returns Pose, so we extract components and build Transform
        auto pose_msg = tf2::toMsg(*target_point_odom_);
        transform_stamped.transform.translation.x = pose_msg.position.x;
        transform_stamped.transform.translation.y = pose_msg.position.y;
        transform_stamped.transform.translation.z = pose_msg.position.z;
        transform_stamped.transform.rotation = pose_msg.orientation;

        tf_broadcaster_->sendTransform(transform_stamped);

        // Step 3: Compute errors - distance and angle from robot to target in odom frame
        Eigen::Vector3d direction_to_target = target_point_odom_->translation() - robot_position;
        double distance_to_target = direction_to_target.norm();
        double angle_to_target_odom = std::atan2(direction_to_target.y(), direction_to_target.x());

        // Compute angle error in robot frame
        double angle_error = angle_to_target_odom - robot_yaw - config_.angle_setpoint;

        // Normalize angle_error to [-π, π]
        while (angle_error > M_PI)
            angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI)
            angle_error += 2.0 * M_PI;

        // Compute distance error (positive when too far, negative when too close)
        double distance_error = distance_to_target - config_.min_wall_distance;

        // Check if we're close enough to wall
        bool within_min_distance = distance_to_target <= config_.min_wall_distance;

        // Update state for one-shot mode
        if (!config_.continuous && within_min_distance)
        {
            reached_min_distance_ = true;
        }

        // Log errors and control state
        RCLCPP_INFO_THROTTLE(logger_,
            steady_clock,
            500,
            "=== ERRORS === distance_error=%.3f m (target=%.3f, actual=%.3f), "
            "angle_error=%.3f rad (%.1f°), within_min_dist=%d, reached_min=%d, phase=%s",
            distance_error,
            config_.min_wall_distance,
            distance_to_target,
            angle_error,
            angle_error * 180.0 / M_PI,
            within_min_distance,
            reached_min_distance_,
            (!config_.continuous && !reached_min_distance_)  ? "APPROACH"
            : (!config_.continuous && reached_min_distance_) ? "ALIGN"
                                                             : "FOLLOW");

        RCLCPP_DEBUG_STREAM_THROTTLE(logger_,
            steady_clock,
            1000,
            "Robot pose (odom):\n"
                << input.pose.matrix() << "\nTarget pose (odom):\n"
                << target_point_odom_->matrix());

        // Two-phase behavior for one-shot mode
        if (!config_.continuous)
        {
            if (!reached_min_distance_)
            {
                // Phase 1: Approach - only move forward, no rotation
                output.cmd_vel.linear.x =
                    std::clamp(config_.forward_speed, 0.0, robot_params_.max_linear_velocity);
                output.cmd_vel.angular.z = 0.0;
            }
            else
            {
                // Phase 2: Rotate - stop forward motion, only rotate to align
                output.cmd_vel.linear.x = 0.0;

                if (std::abs(angle_error) < config_.wall_alignment_tolerance)
                {
                    // Alignment complete
                    output.is_complete = true;
                    output.cmd_vel.angular.z = 0.0;
                    return output;
                }
                else
                {
                    // Apply rotation
                    double angular_velocity =
                        (angle_error > 0) ? config_.angular_speed : -config_.angular_speed;
                    output.cmd_vel.angular.z = std::clamp(angular_velocity,
                        -robot_params_.max_angular_velocity,
                        robot_params_.max_angular_velocity);
                }
            }
        }
        else
        {
            // Continuous mode: always move forward and correct angle error
            output.cmd_vel.linear.x =
                std::clamp(config_.forward_speed, 0.0, robot_params_.max_linear_velocity);

            double angular_velocity = 0.0;
            if (std::abs(angle_error) > config_.wall_alignment_tolerance)
            {
                angular_velocity =
                    (angle_error > 0) ? config_.angular_speed : -config_.angular_speed;
                RCLCPP_DEBUG_THROTTLE(logger_,
                    steady_clock,
                    500,
                    "Continuous mode: Applying rotation (angle_error=%.3f rad > tolerance=%.3f "
                    "rad)",
                    std::abs(angle_error),
                    config_.wall_alignment_tolerance);
            }
            else
            {
                RCLCPP_DEBUG_THROTTLE(logger_,
                    steady_clock,
                    500,
                    "Continuous mode: No rotation needed (angle_error=%.3f rad <= tolerance=%.3f "
                    "rad)",
                    std::abs(angle_error),
                    config_.wall_alignment_tolerance);
            }

            output.cmd_vel.angular.z = std::clamp(angular_velocity,
                -robot_params_.max_angular_velocity,
                robot_params_.max_angular_velocity);
        }
    }
    else
    {
        // No target found, stop
        RCLCPP_WARN_THROTTLE(logger_,
            steady_clock,
            1000,
            "No target point found! detections=%zu, max_range=%.2f - stopping",
            input.detections.size(),
            config_.max_detection_range);
        output.cmd_vel.linear.x = 0.0;
        output.cmd_vel.angular.z = 0.0;
    }

    // Log controller outputs
    RCLCPP_INFO_THROTTLE(logger_,
        steady_clock,
        500,
        "=== CONTROLLER OUTPUT === cmd_vel: linear.x=%.3f m/s, angular.z=%.3f rad/s (%.1f°/s), "
        "is_complete=%d",
        output.cmd_vel.linear.x,
        output.cmd_vel.angular.z,
        output.cmd_vel.angular.z * 180.0 / M_PI,
        output.is_complete);

    return output;
}

} // namespace turtlebot3
