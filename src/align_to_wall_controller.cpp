#include "turtlebot3_wall_following_node/align_to_wall_controller.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace turtlebot3
{

AlignToWallController::AlignToWallController(rclcpp::Logger logger,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster) :
    AlignToWallController(Params{}, logger, tf_broadcaster)
{
}

AlignToWallController::AlignToWallController(const Params& params,
    rclcpp::Logger logger,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster) :
    params_{params},
    robot_params_{get_turtlebot3_params()},
    target_finder_{params.finder_params, logger},
    heading_controller_{params.heading_params},
    logger_{logger},
    tf_broadcaster_{tf_broadcaster}
{
}

void AlignToWallController::reset()
{
    target_point_odom_.reset();
    reached_min_distance_ = false;
}

ControlInput AlignToWallController::update(const SystemResponse& input)
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
        "=== ALIGN TO WALL CONTROLLER INPUT === detections=%zu, has_target=%d, robot_pos=[%.3f, "
        "%.3f], robot_yaw=%.3f rad (%.1f°)",
        input.detections.size(),
        target_point_odom_.has_value(),
        robot_position.x(),
        robot_position.y(),
        robot_yaw,
        robot_yaw * 180.0 / M_PI);

    // Step 1: Find target point once (only if not already set)
    if (!target_point_odom_.has_value())
    {
        target_point_odom_ = target_finder_.find_target_point(input.detections, input.pose);
    }

    // Step 2: If target_point_odom_ is set, execute two-phase alignment
    if (target_point_odom_.has_value())
    {
        // Broadcast TF for target point in odom frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = input.timestamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "align_target_point";

        // Convert Eigen::Isometry3d to Transform using tf2_eigen
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

        // Compute target heading: angle to target plus desired offset
        double target_heading = angle_to_target_odom - params_.angle_setpoint;

        // Compute heading error using heading controller
        double heading_error = HeadingController::normalize_angle(target_heading - robot_yaw);

        // Compute distance error (positive when too far, negative when too close)
        double distance_error = distance_to_target - params_.finder_params.min_wall_distance;

        // Check if we're close enough to wall
        bool within_min_distance = distance_to_target <= params_.finder_params.min_wall_distance;

        // Update state
        if (within_min_distance)
        {
            reached_min_distance_ = true;
        }

        // Log errors and control state
        RCLCPP_INFO_THROTTLE(logger_,
            steady_clock,
            500,
            "=== ERRORS === distance_error=%.3f m (target=%.3f, actual=%.3f), "
            "heading_error=%.3f rad (%.1f°), within_min_dist=%d, reached_min=%d, phase=%s",
            distance_error,
            params_.finder_params.min_wall_distance,
            distance_to_target,
            heading_error,
            heading_error * 180.0 / M_PI,
            within_min_distance,
            reached_min_distance_,
            !reached_min_distance_ ? "APPROACH" : "ALIGN");

        RCLCPP_DEBUG_STREAM_THROTTLE(logger_,
            steady_clock,
            1000,
            "Robot pose (odom):\n"
                << input.pose.matrix() << "\nTarget pose (odom):\n"
                << target_point_odom_->matrix());

        // Two-phase behavior
        if (!reached_min_distance_)
        {
            // Phase 1: Approach - only move forward, no rotation
            output.cmd_vel.linear.x =
                std::clamp(params_.forward_speed, 0.0, robot_params_.max_linear_velocity);
            output.cmd_vel.angular.z = 0.0;
        }
        else
        {
            // Phase 2: Rotate - stop forward motion, only rotate to align
            output.cmd_vel.linear.x = 0.0;

            if (heading_controller_.is_aligned(robot_yaw, target_heading))
            {
                // Alignment complete
                output.is_complete = true;
                output.cmd_vel.angular.z = 0.0;
                return output;
            }
            else
            {
                // Apply rotation using heading controller
                output.cmd_vel.angular.z = heading_controller_.compute_angular_velocity(robot_yaw,
                    target_heading,
                    robot_params_.max_angular_velocity);
            }
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
            params_.finder_params.max_detection_range);
        output.cmd_vel.linear.x = 0.0;
        output.cmd_vel.angular.z = 0.0;
    }

    // Log controller outputs
    RCLCPP_INFO_THROTTLE(logger_,
        steady_clock,
        500,
        "=== ALIGN TO WALL CONTROLLER OUTPUT === cmd_vel: linear.x=%.3f m/s, angular.z=%.3f rad/s "
        "(%.1f°/s), is_complete=%d",
        output.cmd_vel.linear.x,
        output.cmd_vel.angular.z,
        output.cmd_vel.angular.z * 180.0 / M_PI,
        output.is_complete);

    return output;
}

} // namespace turtlebot3
