#include "turtlebot3_wall_following_node/wall_following_controller.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace turtlebot3
{

WallFollowingController::WallFollowingController(rclcpp::Logger logger,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster) :
    WallFollowingController(Params{}, logger, tf_broadcaster)
{
}

WallFollowingController::WallFollowingController(const Params& params,
    rclcpp::Logger logger,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster) :
    params_{params},
    robot_params_{get_turtlebot3_params()},
    target_finder_{params.finder_params, logger},
    logger_{logger},
    tf_broadcaster_{tf_broadcaster}
{
}

void WallFollowingController::reset()
{
    // No state to reset in continuous mode
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
        "=== WALL FOLLOWING CONTROLLER INPUT === detections=%zu, robot_pos=[%.3f, %.3f], "
        "robot_yaw=%.3f rad (%.1f°)",
        input.detections.size(),
        robot_position.x(),
        robot_position.y(),
        robot_yaw,
        robot_yaw * 180.0 / M_PI);

    // Step 1: Find target point (always update in continuous mode)
    std::optional<Eigen::Isometry3d> target_point_odom =
        target_finder_.find_target_point(input.detections, input.pose);

    // Step 2: If target_point is found, move forward and align
    if (target_point_odom.has_value())
    {
        // Broadcast TF for target point in odom frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = input.timestamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "wall_follow_target_point";

        // Convert Eigen::Isometry3d to Transform using tf2_eigen
        auto pose_msg = tf2::toMsg(*target_point_odom);
        transform_stamped.transform.translation.x = pose_msg.position.x;
        transform_stamped.transform.translation.y = pose_msg.position.y;
        transform_stamped.transform.translation.z = pose_msg.position.z;
        transform_stamped.transform.rotation = pose_msg.orientation;

        tf_broadcaster_->sendTransform(transform_stamped);

        // Step 3: Compute errors - distance and angle from robot to target in odom frame
        Eigen::Vector3d direction_to_target = target_point_odom->translation() - robot_position;
        double distance_to_target = direction_to_target.norm();
        double angle_to_target_odom = std::atan2(direction_to_target.y(), direction_to_target.x());

        // Compute angle error in robot frame
        double angle_error = angle_to_target_odom - robot_yaw - params_.angle_setpoint;

        // Normalize angle_error to [-π, π]
        while (angle_error > M_PI)
            angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI)
            angle_error += 2.0 * M_PI;

        // Compute distance error (positive when too far, negative when too close)
        double distance_error = distance_to_target - params_.finder_params.min_wall_distance;

        // Log errors and control state
        RCLCPP_INFO_THROTTLE(logger_,
            steady_clock,
            500,
            "=== ERRORS === distance_error=%.3f m (target=%.3f, actual=%.3f), "
            "angle_error=%.3f rad (%.1f°)",
            distance_error,
            params_.finder_params.min_wall_distance,
            distance_to_target,
            angle_error,
            angle_error * 180.0 / M_PI);

        RCLCPP_DEBUG_STREAM_THROTTLE(logger_,
            steady_clock,
            1000,
            "Robot pose (odom):\n"
                << input.pose.matrix() << "\nTarget pose (odom):\n"
                << target_point_odom->matrix());

        // Continuous mode: always move forward and correct angle error
        output.cmd_vel.linear.x =
            std::clamp(params_.forward_speed, 0.0, robot_params_.max_linear_velocity);

        double angular_velocity = 0.0;
        if (std::abs(angle_error) > params_.wall_alignment_tolerance)
        {
            angular_velocity = (angle_error > 0) ? params_.angular_speed : -params_.angular_speed;
            RCLCPP_DEBUG_THROTTLE(logger_,
                steady_clock,
                500,
                "Continuous mode: Applying rotation (angle_error=%.3f rad > tolerance=%.3f rad)",
                std::abs(angle_error),
                params_.wall_alignment_tolerance);
        }
        else
        {
            RCLCPP_DEBUG_THROTTLE(logger_,
                steady_clock,
                500,
                "Continuous mode: No rotation needed (angle_error=%.3f rad <= tolerance=%.3f rad)",
                std::abs(angle_error),
                params_.wall_alignment_tolerance);
        }

        output.cmd_vel.angular.z = std::clamp(angular_velocity,
            -robot_params_.max_angular_velocity,
            robot_params_.max_angular_velocity);
    }
    else
    {
        // No target found, stop
        RCLCPP_WARN_THROTTLE(logger_,
            steady_clock,
            1000,
            "No target point found! detections=%zu, max_range=%.2f - stopping",
            input.detections.size(),
            params_.finder_params.max_detection_range);
        output.cmd_vel.linear.x = 0.0;
        output.cmd_vel.angular.z = 0.0;
    }

    // Log controller outputs
    RCLCPP_INFO_THROTTLE(logger_,
        steady_clock,
        500,
        "=== WALL FOLLOWING CONTROLLER OUTPUT === cmd_vel: linear.x=%.3f m/s, angular.z=%.3f "
        "rad/s (%.1f°/s), is_complete=%d",
        output.cmd_vel.linear.x,
        output.cmd_vel.angular.z,
        output.cmd_vel.angular.z * 180.0 / M_PI,
        output.is_complete);

    return output;
}

} // namespace turtlebot3
