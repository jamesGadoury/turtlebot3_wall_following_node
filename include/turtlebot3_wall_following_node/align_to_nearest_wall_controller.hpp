#pragma once

#include "turtlebot3_wall_following_node/controller_interface.hpp"
#include "turtlebot3_wall_following_node/turtlebot3_params.hpp"

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
    };

    AlignToNearestWallController();
    explicit AlignToNearestWallController(const Config& config);

    void reset() override;
    ControllerOutput compute(const ControllerInput& input) override;

private:
    Config config_;
    Turtlebot3Params robot_params_;
};

} // namespace turtlebot3
