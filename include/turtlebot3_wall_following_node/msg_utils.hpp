#pragma once

#include "turtlebot3_wall_following_node/laser_detection.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace turtlebot3
{

std::vector<LaserDetection> to_laser_detections(const sensor_msgs::msg::LaserScan& scan);

visualization_msgs::msg::Marker to_marker(const LaserDetection& detection);

std::string to_string(const geometry_msgs::msg::Pose& pose);

std::string to_string(const nav_msgs::msg::Odometry& odom);

} // namespace turtlebot3
