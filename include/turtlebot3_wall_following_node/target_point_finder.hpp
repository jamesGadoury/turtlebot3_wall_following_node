#pragma once

#include "turtlebot3_wall_following_node/controller_interface.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace turtlebot3
{

/**
 * @brief Finds target points for wall following based on laser detections
 *
 * This class encapsulates the logic for finding the nearest wall point
 * within a specified angular sweep range.
 */
class TargetPointFinder
{
public:
    struct Params
    {
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
    };

    TargetPointFinder(const Params& params, rclcpp::Logger logger);

    /**
     * @brief Find target point from laser detections
     *
     * @param detections Laser scan detections in base_link frame
     * @param pose Current robot pose in odom frame (odom_T_base_link)
     * @return Target point in odom frame, or std::nullopt if no valid target found
     */
    std::optional<Eigen::Isometry3d>
    find_target_point(const std::vector<LaserDetection>& detections, const Eigen::Isometry3d& pose);

private:
    Params params_;
    rclcpp::Logger logger_;
};

} // namespace turtlebot3
