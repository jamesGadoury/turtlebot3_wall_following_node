#pragma once

#include "turtlebot3_wall_following_node/controller_interface.hpp"
#include "turtlebot3_wall_following_node/heading_controller.hpp"
#include "turtlebot3_wall_following_node/target_point_finder.hpp"
#include "turtlebot3_wall_following_node/turtlebot3_params.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace turtlebot3
{

/**
 * @brief Controller that continuously follows a wall
 *
 * This controller continuously updates the target point and follows the wall indefinitely.
 * The target is recalculated on every update to track the nearest wall point.
 */
class WallFollowingController : public ControllerInterface
{
public:
    struct Params
    {
        // Target point finder parameters
        TargetPointFinder::Params finder_params{
            .min_sweep_angle = 3 * M_PI / 2.0,  // 270° (right side)
            .max_sweep_angle = 2 * M_PI - 0.01, // ~360° (forward)
            .min_wall_distance = 0.25,
            .max_detection_range = 1.5,
        };

        // Heading controller parameters
        HeadingController::Params heading_params{
            .angular_speed = 0.8,
            .alignment_tolerance = 0.2,
        };

        // Forward speed (m/s)
        double forward_speed{0.05};

        // Target angle to maintain to wall (rad, -90 degrees = along -y axis)
        double angle_setpoint{-M_PI / 2.0};
    };

    WallFollowingController(rclcpp::Logger logger,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

    WallFollowingController(const Params& params,
        rclcpp::Logger logger,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

    void reset();
    ControlInput update(const SystemResponse& input) override;

private:
    Params params_;
    Turtlebot3Params robot_params_;
    TargetPointFinder target_finder_;
    HeadingController heading_controller_;
    rclcpp::Logger logger_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace turtlebot3
