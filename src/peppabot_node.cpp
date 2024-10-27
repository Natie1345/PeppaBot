#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>

class PeppaBotNode : public rclcpp::Node {
public:
    PeppaBotNode() : Node("peppabot_node"), goal_index_(0), object_detected_(false) {
        // Define the cylinder coordinates as goal positions
        goals_ = {
            createGoalPose(0.668685, 2.58418, 0.0, 1.0),
            createGoalPose(5.16823, 1.32238, 0.0, 1.0),
            createGoalPose(5.20808, 0.112293, 0.0, 1.0),
            createGoalPose(5.09059, -1.67338, 0.0, 1.0),
            createGoalPose(5.09703, -2.50786, 0.0, 1.0),
            createGoalPose(0.888802, -3.96757, 0.0, 1.0),
            createGoalPose(1.66056, 0.741782, 0.0, 1.0),
            createGoalPose(0.69425, -1.69643, 0.0, 1.0),
            createGoalPose(2.50555, -1.69839, 0.0, 1.0)
        };

        // Publishers and subscribers for TurtleBot3-related topics
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/navigate_to_pose/goal", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);        // Velocity commands

        // Subscribe to laser scan data for detecting obstacles
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PeppaBotNode::laserScanCallback, this, std::placeholders::_1));

        // Start navigating to the first goal
        sendGoal(goals_[goal_index_]);
    }

private:
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    size_t goal_index_;
    bool object_detected_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;

    geometry_msgs::msg::PoseStamped createGoalPose(double x, double y, double orientation_z, double orientation_w) {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";  // Ensuring it's in the "map" frame
        goal_pose.header.stamp = this->now();
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.orientation.z = orientation_z;
        goal_pose.pose.orientation.w = orientation_w;
        return goal_pose;
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Detect objects using laser scan data
        bool object_detected_now = false;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            if (range < 1.0) {  // Object detected within 1 meter
                object_detected_now = true;
                break;
            }
        }

        if (object_detected_now && !object_detected_) {
            RCLCPP_INFO(this->get_logger(), "Object detected! Stopping to navigate around it.");
            object_detected_ = true;

            // Take over and navigate around the object
            avoidObstacle();
        } else if (!object_detected_now && object_detected_) {
            RCLCPP_INFO(this->get_logger(), "Object no longer detected, resuming navigation.");
            object_detected_ = false;
            // Resume navigation towards the current goal
            sendGoal(goals_[goal_index_]);
        }
    }

    void avoidObstacle() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;  // Stop forward movement
        cmd_vel_msg.angular.z = 0.5;  // Rotate to navigate around the obstacle
        cmd_vel_publisher_->publish(cmd_vel_msg);
        RCLCPP_INFO(this->get_logger(), "Navigating around the obstacle.");
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped &goal_pose) {
        goal_publisher_->publish(goal_pose);
        RCLCPP_INFO(this->get_logger(), "Published goal: [%.2f, %.2f]", goal_pose.pose.position.x, goal_pose.pose.position.y);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PeppaBotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
