#pragma once

#include "turtlebot3_wall_following_node/controller_interface.hpp"
#include "turtlebot3_wall_following_node/target_point_finder.hpp"
#include "turtlebot3_wall_following_node/turtlebot3_params.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <cmath>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace turtlebot3
{

/**
 * @brief Controller that aligns to the nearest wall once and completes
 *
 * This controller operates in two phases:
 * 1. Approach: Move forward until within min_wall_distance
 * 2. Align: Stop and rotate until aligned with the wall
 *
 * The target point is found only once at the start.
 */
class AlignToWallController : public ControllerInterface
{
public:
    struct Config
    {
        // Target point finder parameters
        TargetPointFinder::Params finder_params;

        // Angular speed for rotation (rad/s)
        double angular_speed{0.1};

        // Forward speed (m/s)
        double forward_speed{0.1};

        // Target angle to maintain to wall (rad, -90 degrees = along -y axis)
        double angle_setpoint{-M_PI / 2.0};

        // Tolerance for angle alignment (rad)
        double wall_alignment_tolerance{0.1};
    };

    AlignToWallController(rclcpp::Logger logger,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

    AlignToWallController(const Config& config,
        rclcpp::Logger logger,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

    void reset();
    ControlInput update(const SystemResponse& input) override;

private:
    Config config_;
    Turtlebot3Params robot_params_;
    TargetPointFinder target_finder_;
    bool reached_min_distance_{false};
    std::optional<Eigen::Isometry3d> target_point_odom_;
    rclcpp::Logger logger_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace turtlebot3
