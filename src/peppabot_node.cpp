#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

class PeppaBotNode : public rclcpp::Node {
public:
    PeppaBotNode() : Node("peppabot_node"), goal_index_(0), object_detected_(false) {
        goals_ = {
            createGoalPose(2.0, 3.0, 0.0, 1.0),  // Example coordinates for book 1
            createGoalPose(3.5, -2.0, 0.0, 1.0), // Example coordinates for book 2
            createGoalPose(5.0, 2.0, 0.0, 1.0),  // Example coordinates for book 3
            createGoalPose(1.0, 4.0, 0.0, 1.0)   // Example coordinates for book 4
        };

        starting_point_ = createGoalPose(0.0, 0.0, 0.0, 1.0);  // Starting point coordinates

        // Publisher for navigation goals
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("navigate_to_pose/goal", 10);

        book_selection_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/selected_book", 10, std::bind(&PeppaBotNode::bookSelectionCallback, this, std::placeholders::_1));

        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&PeppaBotNode::laserScanCallback, this, std::placeholders::_1));
    }

private:
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    geometry_msgs::msg::PoseStamped starting_point_;
    size_t goal_index_;
    bool object_detected_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr book_selection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;

    geometry_msgs::msg::PoseStamped createGoalPose(double x, double y, double orientation_z, double orientation_w) {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->now();
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.orientation.z = orientation_z;
        goal_pose.pose.orientation.w = orientation_w;
        return goal_pose;
    }

    void bookSelectionCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string selected_book = msg->data;

        if (selected_book == "book1") {
            goal_index_ = 0;
        } else if (selected_book == "book2") {
            goal_index_ = 1;
        } else if (selected_book == "book3") {
            goal_index_ = 2;
        } else if (selected_book == "book4") {
            goal_index_ = 3;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid book selection: %s", selected_book.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Navigating to %s", selected_book.c_str());
        sendGoal(goals_[goal_index_]);
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        const float min_diameter = 0.3; // Minimum diameter of 30 cm
        const float max_diameter = 1.0; // Maximum diameter limit
        const float detection_range = 0.15; // Range tolerance for detection
        bool object_detected_now = false;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            if (range < max_diameter + detection_range && range > min_diameter - detection_range) {
                object_detected_now = true;
                float angle = scan->angle_min + i * scan->angle_increment;

                // Create a new goal position to navigate around the detected object
                adjustGoalForObstacle(angle);
                break;
            }
        }

        if (object_detected_now && !object_detected_) {
            RCLCPP_INFO(this->get_logger(), "Object detected, adjusting navigation.");
            object_detected_ = true;
        } else if (!object_detected_now && object_detected_) {
            RCLCPP_INFO(this->get_logger(), "Object no longer detected, resuming navigation.");
            object_detected_ = false;
            sendGoal(goals_[goal_index_]);
        }
    }

    void adjustGoalForObstacle(float angle) {
        // Calculate new goal position to navigate around the obstacle
        float obstacle_distance = 0.5; // Distance to maintain from the obstacle
        float new_x = goals_[goal_index_].pose.position.x + obstacle_distance * cos(angle + M_PI_2); // Shift right
        float new_y = goals_[goal_index_].pose.position.y + obstacle_distance * sin(angle + M_PI_2); // Shift up

        RCLCPP_INFO(this->get_logger(), "Adjusting goal to (%.2f, %.2f)", new_x, new_y);
        sendGoal(createGoalPose(new_x, new_y, 0.0, 1.0)); // Send new goal
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped &goal_pose) {
        goal_publisher_->publish(goal_pose);
        RCLCPP_INFO(this->get_logger(), "Published goal: [%f, %f]", goal_pose.pose.position.x, goal_pose.pose.position.y);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PeppaBotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
