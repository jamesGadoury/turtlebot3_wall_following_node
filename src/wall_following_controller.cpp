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
    heading_controller_{params.heading_params},
    logger_{logger},
    tf_broadcaster_{tf_broadcaster}
{
}

void WallFollowingController::reset()
{
}

ControlInput WallFollowingController::update(const SystemResponse& input)
{
    ControlInput output;
    output.cmd_vel.linear.x = 0.0;
    output.cmd_vel.angular.z = 0.0;
    output.is_complete = false;

    const rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    const Eigen::Vector3d robot_position{input.pose.translation()};
    const double robot_yaw{std::atan2(input.pose.rotation()(1, 0), input.pose.rotation()(0, 0))};
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

    const std::optional<Eigen::Isometry3d> target_point_odom{
        target_finder_.find_target_point(input.detections, input.pose)};

    // Step 2: If target_point is found, move forward and align
    if (target_point_odom.has_value())
    {
        geometry_msgs::msg::TransformStamped transform_stamped{
            tf2::eigenToTransform(*target_point_odom)};
        transform_stamped.header.stamp = input.timestamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "wall_follow_target_point";

        tf_broadcaster_->sendTransform(transform_stamped);

        const Eigen::Vector3d direction_to_target{
            target_point_odom->translation() - robot_position};
        const double distance_to_target{direction_to_target.norm()};
        const double angle_to_target_odom{
            std::atan2(direction_to_target.y(), direction_to_target.x())};

        const double target_heading{angle_to_target_odom - params_.angle_setpoint};
        const double heading_error{HeadingController::normalize_angle(target_heading - robot_yaw)};
        const double distance_error{distance_to_target - params_.finder_params.min_wall_distance};

        RCLCPP_INFO_THROTTLE(logger_,
            steady_clock,
            500,
            "=== ERRORS === distance_error=%.3f m (target=%.3f, actual=%.3f), "
            "heading_error=%.3f rad (%.1f°)",
            distance_error,
            params_.finder_params.min_wall_distance,
            distance_to_target,
            heading_error,
            heading_error * 180.0 / M_PI);

        RCLCPP_DEBUG_STREAM_THROTTLE(logger_,
            steady_clock,
            1000,
            "Robot pose (odom):\n"
                << input.pose.matrix() << "\nTarget pose (odom):\n"
                << target_point_odom->matrix());

        output.cmd_vel.linear.x =
            std::clamp(params_.forward_speed, 0.0, robot_params_.max_linear_velocity);

        output.cmd_vel.angular.z = heading_controller_.compute_angular_velocity(robot_yaw,
            target_heading,
            robot_params_.max_angular_velocity);

        if (heading_controller_.is_aligned(robot_yaw, target_heading))
        {
            RCLCPP_DEBUG_THROTTLE(logger_,
                steady_clock,
                500,
                "Continuous mode: No rotation needed (heading_error=%.3f rad <= tolerance=%.3f "
                "rad)",
                std::abs(heading_error),
                params_.heading_params.alignment_tolerance);
        }
        else
        {
            RCLCPP_DEBUG_THROTTLE(logger_,
                steady_clock,
                500,
                "Continuous mode: Applying rotation (heading_error=%.3f rad > tolerance=%.3f rad)",
                std::abs(heading_error),
                params_.heading_params.alignment_tolerance);
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
