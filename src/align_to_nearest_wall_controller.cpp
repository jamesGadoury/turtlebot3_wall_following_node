#include "turtlebot3_wall_following_node/align_to_nearest_wall_controller.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>

namespace turtlebot3
{

AlignToNearestWallController::AlignToNearestWallController(
    rclcpp::Logger logger,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster) :
    AlignToNearestWallController(Config{}, logger, tf_broadcaster)
{
}

AlignToNearestWallController::AlignToNearestWallController(
    const Config& config,
    rclcpp::Logger logger,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster) :
    config_{config},
    robot_params_{get_turtlebot3_params()},
    logger_{logger},
    tf_broadcaster_{tf_broadcaster}
{
}

void AlignToNearestWallController::reset()
{
    target_point_odom_.reset();
}

void AlignToNearestWallController::find_and_set_target_point(
    const std::vector<LaserDetection>& detections,
    const Eigen::Isometry3d& pose)
{
    // Find nearest point in sweep angle range
    // The sweep range is centered around 0 (forward x axis of robot), wrapping around 2π
    const double half_sweep = config_.sweep_angle_range / 2.0;
    const double upper_bound = half_sweep;  // [0, half_sweep]
    const double lower_bound = 2.0 * M_PI - half_sweep;  // [2π - half_sweep, 2π]

    const LaserDetection* nearest = nullptr;
    float min_distance = std::numeric_limits<float>::max();

    for (const auto& detection : detections)
    {
        // Check if detection is within sweep angle range
        // Either in [0, upper_bound] or [lower_bound, 2π]
        const bool in_range = (detection.angle <= upper_bound) ||
                             (detection.angle >= lower_bound);

        if (in_range && detection.distance < min_distance)
        {
            min_distance = detection.distance;
            nearest = &detection;
        }
    }

    // If nearest point is within min_wall_distance, transform to odom and set target_point_odom_
    if (nearest && nearest->distance <= config_.min_wall_distance)
    {
        // Transform from base_footprint to odom
        Eigen::Vector3d target_in_base(nearest->x(), nearest->y(), 0.0);
        target_point_odom_ = pose * target_in_base;
    }
}

ControlInput AlignToNearestWallController::update(const SystemResponse& input)
{
    ControlInput output;
    output.cmd_vel.linear.x = 0.0;
    output.cmd_vel.angular.z = 0.0;
    output.is_complete = false;

    // Step 1: Find and set target point only if not already set
    if (!target_point_odom_.has_value())
    {
        find_and_set_target_point(input.detections, input.pose);
    }

    // Step 2: If target_point_odom_ is set, rotate; otherwise move forward
    if (target_point_odom_.has_value())
    {
        // Broadcast TF for target point in odom frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = input.timestamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "align_target_point";
        transform_stamped.transform.translation.x = target_point_odom_->x();
        transform_stamped.transform.translation.y = target_point_odom_->y();
        transform_stamped.transform.translation.z = target_point_odom_->z();

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);

        // Step 3: Compute angle from robot to target in odom frame
        Eigen::Vector3d robot_position = input.pose.translation();
        Eigen::Vector3d direction_to_target = *target_point_odom_ - robot_position;
        double angle_to_target_odom = std::atan2(direction_to_target.y(), direction_to_target.x());

        // Get robot's current yaw in odom frame
        Eigen::Matrix3d rotation = input.pose.rotation();
        double robot_yaw = std::atan2(rotation(1, 0), rotation(0, 0));

        // Compute angle error in robot frame
        double angle_error = angle_to_target_odom - robot_yaw - config_.angle_setpoint;

        // Normalize angle_error to [-π, π]
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

        // Log current angle and error
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_INFO_THROTTLE(logger_, steady_clock, 500,
            "AlignToNearestWall: angle_to_target=%.3f rad (%.1f°), error=%.3f rad (%.1f°)",
            angle_to_target_odom - robot_yaw, (angle_to_target_odom - robot_yaw) * 180.0 / M_PI,
            angle_error, angle_error * 180.0 / M_PI);

        // Check if aligned within tolerance
        if (std::abs(angle_error) < config_.wall_alignment_tolerance)
        {
            output.is_complete = true;
            return output;
        }

        // Rotate towards target
        // If angle_error > 0, target is to the left, rotate left (positive angular velocity)
        // If angle_error < 0, target is to the right, rotate right (negative angular velocity)
        const double angular_velocity = (angle_error > 0) ?
            config_.angular_speed : -config_.angular_speed;

        output.cmd_vel.angular.z = std::clamp(
            angular_velocity,
            -robot_params_.max_angular_velocity,
            robot_params_.max_angular_velocity
        );
    }
    else
    {
        // No target locked, move forward
        output.cmd_vel.linear.x = std::clamp(
            config_.forward_speed,
            0.0,
            robot_params_.max_linear_velocity
        );
    }

    return output;
}

} // namespace turtlebot3
