#include "turtlebot3_wall_following_node/msg_utils.hpp"

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

string to_string(const geometry_msgs::msg::Pose& pose)
{
    std::stringstream ss;
    ss << "{";
    ss << "position: {";
    ss << "x: " << pose.position.x << ", ";
    ss << "y: " << pose.position.y << ", ";
    ss << "z: " << pose.position.z;
    ss << "},";
    ss << "orientation: {";
    ss << "x: " << pose.orientation.x << ", ";
    ss << "y: " << pose.orientation.y << ", ";
    ss << "z: " << pose.orientation.z << ", ";
    ss << "w: " << pose.orientation.w;
    ss << "}";
    return ss.str();
}

string to_string(const nav_msgs::msg::Odometry& odom)
{
    std::stringstream ss;
    ss << "{";
    ss << "pose: {";
    ss << "pose: {";
    ss << to_string(odom.pose.pose);
    ss << "}";
    ss << "}";
    ss << "}";
    return ss.str();
}

} // namespace turtlebot3
