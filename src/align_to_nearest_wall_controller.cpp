#include "turtlebot3_wall_following_node/align_to_nearest_wall_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace turtlebot3
{

AlignToNearestWallController::AlignToNearestWallController() :
    AlignToNearestWallController(Config{})
{
}

AlignToNearestWallController::AlignToNearestWallController(const Config& config) :
    config_{config},
    robot_params_{get_turtlebot3_params()}
{
}

void AlignToNearestWallController::reset()
{
    target_point_.reset();
}

ControlInput AlignToNearestWallController::update(const SystemResponse& input)
{
    ControlInput output;
    output.cmd_vel.linear.x = 0.0;
    output.cmd_vel.angular.z = 0.0;
    output.is_complete = false;

    // Step 1: Find nearest point in sweep angle range
    const LaserDetection* nearest = nullptr;
    float min_distance = std::numeric_limits<float>::max();

    for (const auto& detection : input.detections)
    {
        // Check if detection is within sweep angle range
        if (detection.angle >= config_.sweep_angle_min &&
            detection.angle <= config_.sweep_angle_max)
        {
            if (detection.distance < min_distance)
            {
                min_distance = detection.distance;
                nearest = &detection;
            }
        }
    }

    // Step 2: If nearest point is within min_wall_distance, update target_point_
    if (nearest && nearest->distance <= config_.min_wall_distance)
    {
        target_point_ = *nearest;
    }

    // Step 3: If target_point_ is set, rotate; otherwise move forward
    if (target_point_.has_value())
    {
        // Step 4: Compute angle error and check for completion
        const double angle_to_target = target_point_->angle;
        const double angle_error = angle_to_target - config_.angle_setpoint;

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
