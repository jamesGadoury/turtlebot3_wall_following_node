#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>

namespace turtlebot3
{

std::string to_string(const geometry_msgs::msg::Pose& pose)
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

std::string to_string(const nav_msgs::msg::Odometry& odom)
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

struct Point
{
    float x;
    float y;
};

struct DistanceMap
{
    // Metadata from LaserScan
    double angle_min;       // start angle of the scan [rad]
    double angle_max;       // end angle of the scan [rad]
    double angle_increment; // angular distance between measurements [rad]
    double range_min;       // minimum valid range [m]
    double range_max;       // maximum valid range [m]

    // Measured ranges (distance values, clipped by min/max)
    std::vector<float> ranges;

    // Precomputed (x,y) points in the robot's local frame
    std::vector<Point> cartesian;

    // Utility: number of beams
    size_t size() const
    {
        return ranges.size();
    }

    // Utility: get angle for beam i
    double angle(size_t i) const
    {
        return angle_min + i * angle_increment;
    }
};

DistanceMap to_distance_map(const sensor_msgs::msg::LaserScan& msg)
{
    DistanceMap map;
    map.angle_min = msg.angle_min;
    map.angle_max = msg.angle_max;
    map.angle_increment = msg.angle_increment;
    map.range_min = msg.range_min;
    map.range_max = msg.range_max;
    map.ranges = msg.ranges;

    // Precompute Cartesian coordinates (x, y) for each valid range
    map.cartesian.reserve(msg.ranges.size());
    for (size_t i = 0; i < msg.ranges.size(); ++i)
    {
        const float r = msg.ranges[i];
        if (std::isfinite(r) && r >= msg.range_min && r <= msg.range_max)
        {
            const double angle = msg.angle_min + i * msg.angle_increment;
            const float x = r * std::cos(angle);
            const float y = r * std::sin(angle);
            Point p;
            p.x = x;
            p.y = y;
            map.cartesian.emplace_back(p);
        }
        else
        {
            // Use NaN or skip invalid values
            Point p;
            p.x = std::numeric_limits<float>::quiet_NaN();
            p.y = std::numeric_limits<float>::quiet_NaN();
            map.cartesian.emplace_back(p);
        }
    }

    return map;
}

// TODO: Should we just populate this in above DistanceMap instead?
//       Also, better name?
struct DistanceRay
{
    double distance;
    double angle;
};

double distance(const Point& point)
{
    return std::sqrt(point.x * point.x + point.y * point.y);
}

Point find_nearest_point(const DistanceMap& map)
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
