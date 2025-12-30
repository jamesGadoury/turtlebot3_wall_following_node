#pragma once

#include "turtlebot3_wall_following_node/controller_interface.hpp"
#include "turtlebot3_wall_following_node/turtlebot3_params.hpp"

#include <cmath>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace turtlebot3
{

/**
 * @brief Controller that aligns the robot to face the nearest wall
 *
 * This controller rotates the robot until the nearest detected point
 * is directly in front of the robot (within tolerance), then signals completion.
 */
class AlignToNearestWallController : public ControllerInterface
{
public:
    struct Config
    {
        // Total sweep angle range for finding nearest point (rad)
        // This range is centered around 0 (forward), wrapping around 2π
        // E.g., π means look from [0, π/2] and [2π - π/2, 2π]
        double sweep_angle_range{M_PI / 2.0};  // +/- 45 degrees from forward x axis

        // Minimum distance to consider a wall point (m)
        double min_wall_distance{0.5};

        // Angular speed for rotation (rad/s)
        double angular_speed{0.1};

        // Forward speed when no target locked (m/s)
        double forward_speed{0.1};

        // Target angle to align to (rad, 0 = straight ahead, -90 degrees = along -y axis)
        double angle_setpoint{-M_PI};

        // Tolerance for alignment completion (rad)
        double wall_alignment_tolerance{0.1};
    };

    AlignToNearestWallController(
        rclcpp::Logger logger,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

    AlignToNearestWallController(
        const Config& config,
        rclcpp::Logger logger,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

    void reset();
    ControlInput update(const SystemResponse& input) override;

private:
    Config config_;
    Turtlebot3Params robot_params_;
    std::optional<Eigen::Vector3d> target_point_odom_;
    rclcpp::Logger logger_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void find_and_set_target_point(const std::vector<LaserDetection>& detections,
                                    const Eigen::Isometry3d& pose);
};

} // namespace turtlebot3
