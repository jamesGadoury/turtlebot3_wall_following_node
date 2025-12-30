#include "turtlebot3_wall_following_node/align_to_nearest_wall_controller.hpp"

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
}

ControlInput AlignToNearestWallController::compute(const SystemResponse& /*input*/)
{
    ControlInput output;
    return output;
}

} // namespace turtlebot3
