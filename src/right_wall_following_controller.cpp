#include "turtlebot3_wall_following_node/right_wall_following_controller.hpp"

namespace turtlebot3
{

RightWallFollowingController::RightWallFollowingController() :
    RightWallFollowingController(Config{})
{
}

RightWallFollowingController::RightWallFollowingController(const Config& config) :
    config_{config},
    robot_params_{get_turtlebot3_params()}
{
}

ControlInput RightWallFollowingController::compute(const SystemResponse& /*input*/)
{
    ControlInput output;
    return output;
}

double RightWallFollowingController::compute_right_wall_distance(
    const std::vector<LaserDetection>& /*detections*/) const
{
    return 0.0;
}

double RightWallFollowingController::compute_front_clearance(
    const std::vector<LaserDetection>& /*detections*/) const
{
    return 0.0;
}

} // namespace turtlebot3
