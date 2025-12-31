#pragma once

#include <cmath>

namespace turtlebot3
{

/**
 * @brief Controller for computing angular velocity to reach a target heading
 *
 * This class encapsulates the logic for computing angular velocity
 * based on the current heading error and alignment tolerance.
 */
class HeadingController
{
public:
    struct Params
    {
        // Angular speed for rotation (rad/s)
        double angular_speed{0.2};

        // Tolerance for angle alignment (rad)
        double alignment_tolerance{0.2};
    };

    HeadingController() = default;
    explicit HeadingController(const Params& params);

    /**
     * @brief Compute angular velocity to reach target heading
     *
     * @param current_heading Current heading angle (rad)
     * @param target_heading Target heading angle (rad)
     * @param max_angular_velocity Maximum allowed angular velocity (rad/s)
     * @return Angular velocity command (rad/s), 0 if within tolerance
     */
    double compute_angular_velocity(double current_heading,
        double target_heading,
        double max_angular_velocity) const;

    /**
     * @brief Check if current heading is aligned with target
     *
     * @param current_heading Current heading angle (rad)
     * @param target_heading Target heading angle (rad)
     * @return true if within alignment tolerance
     */
    bool is_aligned(double current_heading, double target_heading) const;

    /**
     * @brief Normalize angle to [-π, π]
     *
     * @param angle Input angle (rad)
     * @return Normalized angle in range [-π, π]
     */
    static double normalize_angle(double angle);

private:
    Params params_;
};

} // namespace turtlebot3
