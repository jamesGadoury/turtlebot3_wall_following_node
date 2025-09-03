#include "turtlebot3_wall_following_node/msg_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

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

// TODO: should make more specific (to_sphere_marker) and parameterize more
visualization_msgs::msg::Marker to_marker(const Eigen::Isometry3d& transform)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "odom"; // must exist in TF or match RViz Fixed Frame
    // m.header.stamp = node->now();
    // m.ns = "demo";
    m.id = 0;
    m.pose.position.x = transform.translation().x();
    m.pose.position.y = transform.translation().y();
    m.pose.position.z = transform.translation().z();

    Eigen::Quaterniond q(transform.rotation());
    q.normalize();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.3; // meters
    m.scale.y = 0.3;
    m.scale.z = 0.3;

    // Color (RGBA in [0,1])
    m.color.r = 0.2f;
    m.color.g = 0.7f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;

    // Keep on screen for 0 = forever (until overridden/DELETED)
    m.lifetime = rclcpp::Duration::from_seconds(0.0);

    return m;
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
