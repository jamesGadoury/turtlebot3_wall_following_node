#pragma once

#include "turtlebot3_wall_following_node/controller_interface.hpp"
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
 * @brief Controller that follows a wall
 *
 * This controller tracks the nearest wall point and maintains alignment.
 * Can operate in two modes:
 * - Continuous mode: Continuously updates target and follows wall indefinitely
 * - One-shot mode: Aligns to initial target once and completes
 */
class WallFollowingController : public ControllerInterface
{
public:
    struct Config
    {
        // Continuous mode: true = continuous wall following, false = align once and complete
        bool continuous{true};

        // Minimum sweep angle for finding nearest point (rad, 0 = forward, π/2 = left, -π/2 =
        // right)
        double min_sweep_angle{-M_PI};

        // Maximum sweep angle for finding nearest point (rad, 0 = forward, π/2 = left, -π/2 =
        // right)
        double max_sweep_angle{0.0};

        // Target distance to wall - start aligning when closer than this (m)
        double min_wall_distance{0.5};

        // Maximum detection range to consider (m) - ignore points beyond this distance
        double max_detection_range{3.5};

        // Angular speed for rotation (rad/s)
        double angular_speed{0.1};

        // Forward speed (m/s)
        double forward_speed{0.1};

        // Target angle to maintain to wall (rad, -90 degrees = along -y axis)
        double angle_setpoint{-M_PI / 2.0};

        // Tolerance for angle alignment (rad)
        double wall_alignment_tolerance{0.1};
    };

    WallFollowingController(rclcpp::Logger logger,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

    WallFollowingController(const Config& config,
        rclcpp::Logger logger,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

    void reset();
    ControlInput update(const SystemResponse& input) override;

private:
    Config config_;
    Turtlebot3Params robot_params_;
    bool reached_min_distance_{false}; // For one-shot mode: true when within min_wall_distance
    std::optional<Eigen::Isometry3d> target_point_odom_;
    rclcpp::Logger logger_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void find_and_update_target_point(const std::vector<LaserDetection>& detections,
        const Eigen::Isometry3d& pose);
};

} // namespace turtlebot3
