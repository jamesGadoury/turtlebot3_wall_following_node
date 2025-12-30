#pragma once

#include "turtlebot3_wall_following_node/controller_interface.hpp"
#include "turtlebot3_wall_following_node/turtlebot3_params.hpp"

namespace turtlebot3
{

/**
 * @brief Controller that follows a wall on the right side
 *
 * This controller maintains a target distance from the wall on the right
 * while moving forward. It adjusts angular velocity to correct for
 * deviations from the target distance.
 */
class RightWallFollowingController : public ControllerInterface
{
public:
    struct Config
    {
    };

    RightWallFollowingController();
    explicit RightWallFollowingController(const Config& config);

    ControlInput update(const SystemResponse& input) override;

private:
    Config config_;
    Turtlebot3Params robot_params_;

    double compute_right_wall_distance(const std::vector<LaserDetection>& detections) const;
    double compute_front_clearance(const std::vector<LaserDetection>& detections) const;
};

} // namespace turtlebot3
