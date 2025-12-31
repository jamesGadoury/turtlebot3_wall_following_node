#include "turtlebot3_wall_following_node/align_to_wall_controller.hpp"
#include "turtlebot3_wall_following_node/controller_interface.hpp"
#include "turtlebot3_wall_following_node/laser_detection.hpp"
#include "turtlebot3_wall_following_node/msg_utils.hpp"
#include "turtlebot3_wall_following_node/wall_following_controller.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace turtlebot3
{

enum class WallFollowerState
{
    ALIGNING_TO_WALL,
    FOLLOWING_WALL
};

class WallFollower : public rclcpp::Node
{
public:
    struct Config
    {
        rclcpp::QoS default_qos{10};
        std::string odom_topic;
        std::string scan_topic;
        std::string cmd_vel_topic;
    };

    static Config default_config()
    {
        Config c;
        c.odom_topic = "/odom";
        c.scan_topic = "/scan";
        c.cmd_vel_topic = "/cmd_vel";
        return c;
    }

    WallFollower(const Config& config = default_config()) :
        Node("wall_follower"),
        tf_buffer_{std::make_unique<tf2_ros::Buffer>(get_clock())},
        tf_listener_{std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true)},
        tf_broadcaster_{std::make_shared<tf2_ros::TransformBroadcaster>(this)},
        scan_subscription_{create_subscription<sensor_msgs::msg::LaserScan>(config.scan_topic,
            config.default_qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { handle_laser_scan(msg); })},
        cmd_vel_publisher_{create_publisher<geometry_msgs::msg::TwistStamped>(config.cmd_vel_topic,
            config.default_qos)},
        time_since_startup_{std::chrono::steady_clock::now()},
        current_state_{WallFollowerState::ALIGNING_TO_WALL}
    {
        // Create alignment controller (one-shot mode)
        {
            const AlignToWallController::Config config{
                .finder_params =
                    {
                        .min_sweep_angle = -0.262, // -15° from forward
                        .max_sweep_angle = 0.262,  // +15° from forward
                        .min_wall_distance = 0.3,
                        .max_detection_range = 3.5,
                    },
                .angular_speed = 0.2,
                .forward_speed = 0.05,
                .angle_setpoint = -M_PI / 2.0,
                .wall_alignment_tolerance = 0.2};
            align_controller_ =
                std::make_unique<AlignToWallController>(config, get_logger(), tf_broadcaster_);
        }

        // Create wall following controller (continuous mode)
        {
            const WallFollowingController::Config config{
                .finder_params =
                    {
                        .min_sweep_angle = 3 * M_PI / 2.0, // 270° (right side)
                        .max_sweep_angle = 2 * M_PI,       // 360° (forward)
                        .min_wall_distance = 0.3,
                        .max_detection_range = 1.5,
                    },
                .angular_speed = 0.2,
                .forward_speed = 0.05,
                .angle_setpoint = -M_PI / 2.0,
                .wall_alignment_tolerance = 0.2};
            follow_controller_ =
                std::make_unique<WallFollowingController>(config, get_logger(), tf_broadcaster_);
        }

        RCLCPP_INFO(get_logger(), "WallFollower initialized in ALIGNING_TO_WALL state");
    }

    void handle_laser_scan(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const std::string target_frame{"odom"};
        const std::string source_frame{"base_footprint"};

        try
        {
            auto ts = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            pose_ = tf2::transformToEigen(ts.transform);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(),
                *get_clock(),
                1000,
                "Could not get transform at scan time: %s",
                ex.what());
            return;
        }

        detections_ = to_laser_detections(*msg);

        RCLCPP_INFO_THROTTLE(get_logger(),
            *get_clock(),
            2000,
            "Update: scan_ranges=%zu, detections=%zu, state=%d",
            msg->ranges.size(),
            detections_.size(),
            static_cast<int>(current_state_));

        handle_motion();
    }

    void handle_motion()
    {
        SystemResponse input;
        input.timestamp = get_clock()->now();
        input.pose = pose_;
        input.detections = detections_;

        ControlInput output;

        switch (current_state_)
        {
        case WallFollowerState::ALIGNING_TO_WALL:
            output = align_controller_->update(input);
            break;
        case WallFollowerState::FOLLOWING_WALL:
            output = follow_controller_->update(input);
            break;
        }

        // Publish velocity command
        geometry_msgs::msg::TwistStamped cmd_msg;
        cmd_msg.header.frame_id = "";
        cmd_msg.header.stamp = get_clock()->now();
        cmd_msg.twist = output.cmd_vel;
        cmd_vel_publisher_->publish(cmd_msg);

        // Handle state transition
        if (output.is_complete)
        {
            handle_transition();
        }
    }

    void handle_transition()
    {
        switch (current_state_)
        {
        case WallFollowerState::ALIGNING_TO_WALL:
            RCLCPP_INFO(get_logger(), "Alignment complete, transitioning to FOLLOWING_WALL");
            current_state_ = WallFollowerState::FOLLOWING_WALL;
            break;
        case WallFollowerState::FOLLOWING_WALL:
            // Right wall following runs indefinitely
            break;
        }
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
    std::chrono::steady_clock::time_point time_since_startup_;
    Eigen::Isometry3d pose_;
    std::vector<LaserDetection> detections_;

    // State machine
    WallFollowerState current_state_;
    std::unique_ptr<AlignToWallController> align_controller_;
    std::unique_ptr<WallFollowingController> follow_controller_;
};

} // namespace turtlebot3

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtlebot3::WallFollower>());
    rclcpp::shutdown();
}
