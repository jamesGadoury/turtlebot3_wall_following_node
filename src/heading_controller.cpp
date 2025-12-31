#include "turtlebot3_wall_following_node/heading_controller.hpp"

#include <algorithm>
#include <cmath>

namespace turtlebot3
{

HeadingController::HeadingController(const Params& params) : params_{params}
{
}

double HeadingController::compute_angular_velocity(double current_heading,
    double target_heading,
    double max_angular_velocity) const
{
    const double heading_error = normalize_angle(target_heading - current_heading);

    if (std::abs(heading_error) <= params_.alignment_tolerance)
    {
        return 0.0;
    }

    const double angular_velocity =
        (heading_error > 0) ? params_.angular_speed : -params_.angular_speed;

    return std::clamp(angular_velocity, -max_angular_velocity, max_angular_velocity);
}

bool HeadingController::is_aligned(double current_heading, double target_heading) const
{
    const double heading_error = normalize_angle(target_heading - current_heading);
    return std::abs(heading_error) <= params_.alignment_tolerance;
}

double HeadingController::normalize_angle(double angle)
{
    double normalized = angle;
    while (normalized > M_PI)
        normalized -= 2.0 * M_PI;
    while (normalized < -M_PI)
        normalized += 2.0 * M_PI;
    return normalized;
}

} // namespace turtlebot3
