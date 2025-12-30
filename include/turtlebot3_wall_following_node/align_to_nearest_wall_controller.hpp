#pragma once

#include "turtlebot3_wall_following_node/controller_interface.hpp"
#include "turtlebot3_wall_following_node/turtlebot3_params.hpp"

#include <cmath>
#include <optional>

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
        // Sweep angle range for finding nearest point (rad)
        double sweep_angle_min = -M_PI / 2;  // -90 degrees
        double sweep_angle_max = M_PI / 2;   // +90 degrees

        // Minimum distance to consider a wall point (m)
        double min_wall_distance = 0.5;

        // Angular speed for rotation (rad/s)
        double angular_speed = 0.5;

        // Forward speed when no target locked (m/s)
        double forward_speed = 0.1;

        // Target angle to align to (rad, 0 = straight ahead)
        double angle_setpoint = 0.0;

        // Tolerance for alignment completion (rad)
        double wall_alignment_tolerance = 0.05;
    };

    AlignToNearestWallController();
    explicit AlignToNearestWallController(const Config& config);

    void reset();
    ControlInput update(const SystemResponse& input) override;

private:
    Config config_;
    Turtlebot3Params robot_params_;
    std::optional<LaserDetection> target_point_;
};

} // namespace turtlebot3
