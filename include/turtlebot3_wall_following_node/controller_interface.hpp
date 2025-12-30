#pragma once

#include "turtlebot3_wall_following_node/laser_detection.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>

namespace turtlebot3
{

/**
 * @brief Output command from a controller
 */
struct ControlInput
{
    geometry_msgs::msg::Twist cmd_vel;
    bool is_complete{false};
};

/**
 * @brief Input state for controllers
 */
struct SystemResponse
{
    Eigen::Isometry3d pose;
    std::vector<LaserDetection> detections;
};

/**
 * @brief Abstract interface for wall following controllers
 */
class ControllerInterface
{
public:
    virtual ~ControllerInterface() = default;

    /**
     * @brief Compute control output given current state
     * @param input Current sensor state
     * @return Control output with velocity command and completion status
     */
    virtual ControlInput update(const SystemResponse& input) = 0;
};

} // namespace turtlebot3
