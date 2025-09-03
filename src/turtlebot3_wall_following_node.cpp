#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlebot3_wall_following_node/laser_detection.hpp"
#include "turtlebot3_wall_following_node/msg_utils.hpp"

#include <Eigen/Geometry>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace turtlebot3
{

LaserDetection find_nearest_detection(const std::vector<LaserDetection>& detections)
{
    std::optional<LaserDetection> nearest_detection;
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
        tf_buffer_{std::make_unique<tf2_ros::Buffer>(get_clock())},
        tf_listener_{std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true)},
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
        return !last_scan_update_.has_value();
    }

    void update()
    {
        // TODO: should separate out the scan processing into a callback for the scan, rather than
        //       always computing here, we still want this to update on its own, as we will want
        //       to make the update rate higher for current node responsibilities
        if (waiting_for_updates())
        {
            return;
        }

        // TODO: parameterize
        const std::string target_frame{"odom"};
        const std::string source_frame{"base_footprint"};
        auto ts = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        auto transform = tf2::transformToEigen(ts.transform); // returns Isometry3d

        const std::vector<LaserDetection> detections{
            to_laser_detections(last_scan_update_.value())};

        const LaserDetection nearest_detection{find_nearest_detection(detections)};
        const Eigen::Vector3d nearest_point(
            transform * Eigen::Vector3d(static_cast<double>(nearest_detection.x()),
                            static_cast<double>(nearest_detection.y()),
                            0));
        RCLCPP_DEBUG(get_logger(),
            "[update] nearest_point={x: '%f', y: '%f'}",
            nearest_point.x(),
            nearest_point.y());

        // TODO: should I just add a Vector3d overload?
        marker_publisher_->publish(
            to_marker(Eigen::Translation3d(nearest_point) * Eigen::Isometry3d::Identity()));
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    std::optional<sensor_msgs::msg::LaserScan> last_scan_update_;
};
} // namespace turtlebot3

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtlebot3::WallFollower>());
    rclcpp::shutdown();
}
