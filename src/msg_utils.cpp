#include "turtlebot3_wall_following_node/msg_utils.hpp"

#include <rclcpp/rclcpp.hpp>

using std::string;
using std::vector;

namespace turtlebot3
{

vector<LaserDetection> to_laser_detections(const sensor_msgs::msg::LaserScan& scan)
{
    vector<LaserDetection> detections;
    float angle{scan.angle_min};

    for (const float range : scan.ranges)
    {
        if (!std::isfinite(range))
        {
            continue;
        }
        detections.emplace_back(LaserDetection{range, angle});

        angle += scan.angle_increment;
    }

    return detections;
}
} // namespace turtlebot3
