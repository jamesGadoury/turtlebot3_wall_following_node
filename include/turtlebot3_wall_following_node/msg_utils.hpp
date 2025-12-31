#pragma once

#include "turtlebot3_wall_following_node/laser_detection.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace turtlebot3
{

std::vector<LaserDetection> to_laser_detections(const sensor_msgs::msg::LaserScan& scan);

} // namespace turtlebot3
