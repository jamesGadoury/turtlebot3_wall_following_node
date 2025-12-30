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

ControllerOutput AlignToNearestWallController::compute(const ControllerInput& /*input*/)
{
    ControllerOutput output;
    return output;
}

} // namespace turtlebot3
