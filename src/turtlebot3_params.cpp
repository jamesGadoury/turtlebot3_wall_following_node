#include "turtlebot3_wall_following_node/turtlebot3_params.hpp"

#include <cmath>

namespace turtlebot3
{

void throw_bad_turtlebot3_model_setting()
{
    std::stringstream ss;
    ss << "TURTLEBOT3_MODEL env variable must be set to one of the following: [ ";
    for (const auto& option : TURTLEBOT3_OPTIONS)
    {
        ss << option;
        ss << " ";
    }
    ss << "]";
    throw std::runtime_error(ss.str());
}

void validate_turtlebot3_model()
{
    if (!TURTLEBOT3_MODEL)
    {
        throw_bad_turtlebot3_model_setting();
    }

    const std::string model{TURTLEBOT3_MODEL};

    if (std::find(TURTLEBOT3_OPTIONS.cbegin(), TURTLEBOT3_OPTIONS.cend(), model) ==
        TURTLEBOT3_OPTIONS.end())
    {
        throw_bad_turtlebot3_model_setting();
    }
}

std::string get_turtlebot3_model()
{
    validate_turtlebot3_model();
    return std::string{TURTLEBOT3_MODEL};
}

Turtlebot3Params get_turtlebot3_params()
{
    const std::string model{get_turtlebot3_model()};

    if (model == BURGER_MODEL)
    {
        return Turtlebot3Params{
            model,                  // model
            BURGER_MAX_LIN_VEL,     // 0.22 m/s
            BURGER_MAX_ANG_VEL      // 2.84 rad/s
        };
    }

    if (model == WAFFLE_MODEL)
    {
        return Turtlebot3Params{
            model,                  // model
            WAFFLE_MAX_LIN_VEL,     // 0.26 m/s
            WAFFLE_MAX_ANG_VEL      // 1.82 rad/s
        };
    }

    throw std::runtime_error("shouldn't happen");
}

} // namespace turtlebot3