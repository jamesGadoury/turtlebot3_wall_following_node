#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlebot3_wall_following_node/laser_detection.hpp"
#include "turtlebot3_wall_following_node/msg_utils.hpp"

#include <chrono>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

using std::optional;
using std::vector;

namespace turtlebot3
{

LaserDetection find_nearest_detection(const vector<LaserDetection>& detections)
{
    optional<LaserDetection> nearest_detection;
    for (auto detection : detections)
    {
        if (!nearest_detection.has_value() ||
            detection.distance < nearest_detection.value().distance)
        {
            nearest_detection = detection;
        }
    }
    if (!nearest_detection.has_value())
    {
        throw std::runtime_error("wtf");
    }
    return nearest_detection.value();
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
        std::string marker_topic;
    };

    static Config default_config()
    {
        Config c;
        c.odom_topic = "/odom";
        c.scan_topic = "/scan";
        c.marker_topic = "/marker";
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
        marker_publisher_{create_publisher<visualization_msgs::msg::Marker>(config.marker_topic,
            config.default_qos)},
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

        RCLCPP_DEBUG(get_logger(), "[update] pose='%s'", to_string(pose).c_str());
        const vector<LaserDetection> detections{to_laser_detections(last_scan_update_.value())};

        const LaserDetection nearest{find_nearest_detection(detections)};
        RCLCPP_DEBUG(get_logger(),
            "[update] nearest_point={x: '%f', y: '%f'}",
            nearest.x(),
            nearest.y());

        marker_publisher_->publish(to_marker(nearest));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
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
