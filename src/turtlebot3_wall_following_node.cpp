#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlebot3_wall_following_node/laser_detection.hpp"
#include "turtlebot3_wall_following_node/point.hpp"

#include <chrono>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>

using std::vector;

namespace turtlebot3
{

Point find_nearest_point(const vector<Point>& points)
{
    // TODO: clean up above types and this func
    std::optional<double> min_distance;
    std::optional<Point> nearest_point;
    for (auto p : map.cartesian)
    {
        if (std::isnan(p.x) || std::isnan(p.y))
        {
            continue;
        }
        const auto d = distance(p);
        if (!min_distance.has_value() || d < min_distance.value())
        {
            min_distance = d;
            nearest_point = p;
        }
    }
    if (!nearest_point.has_value())
    {
        throw std::runtime_error("wtf");
    }
    return nearest_point.value();
}

class WallFollower : public rclcpp::Node
{
public:
    struct Config
    {
        // QoS profile with queue size of 10
        // TODO: is qos of 10 desirable?
        rclcpp::QoS default_qos{10};
        std::string odom_topic;
        std::string scan_topic;
    };

    static Config default_config()
    {
        Config c;
        c.odom_topic = "/odom";
        c.scan_topic = "/scan";
        return c;
    }

    WallFollower(const Config& config = default_config()) :
        Node("wall_follower"),
        odom_subscription_{create_subscription<nav_msgs::msg::Odometry>(config.odom_topic,
            config.default_qos,
            [this](nav_msgs::msg::Odometry::UniquePtr msg)
            { last_pose_update_ = msg->pose.pose; })},
        scan_subscription_{create_subscription<sensor_msgs::msg::LaserScan>(config.scan_topic,
            config.default_qos,
            [this](sensor_msgs::msg::LaserScan::UniquePtr msg) { last_scan_update_ = *msg; })},
        update_timer_{create_wall_timer(std::chrono::milliseconds(5), [this] { update(); })}
    {
    }

    bool waiting_for_updates()
    {
        return !last_pose_update_.has_value() || !last_scan_update_.has_value();
    }

    void update()
    {
        if (waiting_for_updates())
        {
            return;
        }

        const auto& pose{last_pose_update_.value()};

        RCLCPP_INFO(get_logger(), "[update] pose='%s'", to_string(pose).c_str());
        const DistanceMap distance_map{to_distance_map(last_scan_update_.value())};

        const Point nearest_point{find_nearest_point(distance_map)};
        RCLCPP_INFO(get_logger(),
            "[update] nearest_point={x: '%f', y: '%f'}",
            nearest_point.x,
            nearest_point.y);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    std::optional<geometry_msgs::msg::Pose> last_pose_update_;
    std::optional<sensor_msgs::msg::LaserScan> last_scan_update_;
};
} // namespace turtlebot3

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtlebot3::WallFollower>());
    rclcpp::shutdown();
}
