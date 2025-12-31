#pragma once

#include <cstdlib>
#include <string>
#include <vector>

namespace turtlebot3
{

const static double BURGER_MAX_LIN_VEL{0.22};
const static double BURGER_MAX_ANG_VEL{2.84};

const static double WAFFLE_MAX_LIN_VEL{0.26};
const static double WAFFLE_MAX_ANG_VEL{1.82};

const static auto TURTLEBOT3_MODEL{std::getenv("TURTLEBOT3_MODEL")};

const static std::string BURGER_MODEL{"burger"};
const static std::string WAFFLE_MODEL{"waffle"};

const static std::vector TURTLEBOT3_OPTIONS{BURGER_MODEL, WAFFLE_MODEL};

void throw_bad_turtlebot3_model_setting();
void validate_turtlebot3_model();
std::string get_turtlebot3_model();

/**
 * @brief TurtleBot3 platform-specific parameters
 */
struct Turtlebot3Params
{
    std::string model;

    // Velocity limits (platform-specific)
    double max_linear_velocity;
    double max_angular_velocity;
};

Turtlebot3Params get_turtlebot3_params();

} // namespace turtlebot3