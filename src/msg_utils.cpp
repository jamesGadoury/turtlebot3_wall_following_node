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

    for (float range : scan.ranges)
    {
        if (!std::isfinite(range))
        {
            continue;
        }
        LaserDetection detection;
        detection.distance = range;
        detection.angle = angle;
        detections.emplace_back(std::move(detection));

        angle += scan.angle_increment;
    }

    return detections;
}
} // namespace turtlebot3
