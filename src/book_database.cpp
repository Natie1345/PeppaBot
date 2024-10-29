#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"  // For publishing point coordinates
#include <string>
#include <map>
#include <iostream>
#include <chrono>

struct Book {
    std::string title;
    bool available;
    float x;  // X-coordinate in the map
    float y;  // Y-coordinate in the map
};

class BookDatabaseNode : public rclcpp::Node {
public:
    BookDatabaseNode() : Node("book_database_node") {
        initialize_books();
        print_available_books();

        // Subscriber for borrowing books
        borrow_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "borrow_book", 10,
            std::bind(&BookDatabaseNode::borrow_book_callback, this, std::placeholders::_1)
        );

        // Publisher for custom point topic in Rviz
        point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("book_goal_point", 10);

        // Initialize the last borrow time
        last_borrow_time_ = this->now();
    }

private:
    std::map<std::string, Book> books_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr borrow_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_;
    rclcpp::Time last_borrow_time_; // To track the last time a borrow attempt was processed

    void initialize_books() {
         books_["1984"] = Book{"1984", true, 0.668685, 2.58418};   
        books_["Pride and Prejudice"] = Book{"Pride and Prejudice", true, 5.16823, 1.32238}; 
        books_["To Kill a Mockingbird"] = Book{"To Kill a Mockingbird", true, 5.20808, 0.112293}; 
        books_["The Great Gatsby"] = Book{"The Great Gatsby", true, 5.09059, -1.67338}; 

        // Additional books with their own coordinates
        books_["The Hunger Games"] = Book{"The Hunger Games", true, 5.09703, -2.50786};   
        books_["Divergent"] = Book{"Divergent", true, 0.888802, -3.96757}; 
        books_["IT Chapter 2"] = Book{"IT Chapter 2", true, 1.66056, 0.741782}; 
        books_["IT"] = Book{"IT", true, 0.69425, -1.69643}; 
        books_["The Hobbit"] = Book{"The Hobbit", true, 2.50555, -1.69839};  
        }

    void print_available_books() {
        std::cout << "\nAvailable Books:\n";
        for (const auto& [title, book] : books_) {
            if (book.available) {
                std::cout << "- " << title << std::endl;
            }
        }
        std::cout << std::endl;
    }

    void borrow_book_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto current_time = this->now();
        auto time_diff = current_time - last_borrow_time_;

        // Only proceed if the last message was processed more than 1 second ago
        if (time_diff.seconds() >= 1.0) {
            std::string book_title = msg->data;
            auto it = books_.find(book_title);

            // Check if the book is available and update availability
            if (it != books_.end() && it->second.available) {
                it->second.available = false;
                std::cout << "\nSuccessfully borrowed: " << book_title << std::endl;

                // Publish the book's location as a point
                publish_point(it->second.x, it->second.y, book_title);
            } else {
                std::cout << "\nBook unavailable or not found: " << book_title << std::endl;
            }

            // Print the updated list of available books after each borrow request
            print_available_books();

            // Update the last borrow time
            last_borrow_time_ = current_time;
        }
    }

    void publish_point(float x, float y, const std::string& book_title) {
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.stamp = this->now();
        point_msg.header.frame_id = "map";  // Change to "odom" if necessary

        // Set point coordinates
        point_msg.point.x = x;
        point_msg.point.y = y;
        point_msg.point.z = 0.0;

        // Debug message to confirm point is being published
        std::cout << "Publishing point for: " << book_title 
                  << " at coordinates (" << x << ", " << y << ")" << std::endl;

        // Publish the point
        point_publisher_->publish(point_msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BookDatabaseNode>());
    rclcpp::shutdown();
    return 0;
}
