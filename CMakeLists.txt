cmake_minimum_required(VERSION 3.8)
project(peppabot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # Added sensor_msgs dependency
find_package(nav2_common REQUIRED)
find_package(OpenCV REQUIRED)  # Added OpenCV dependency

# Add executable
add_executable(peppabot_node src/peppabot_node.cpp)

# Link libraries
ament_target_dependencies(peppabot_node rclcpp geometry_msgs sensor_msgs)
target_link_libraries(peppabot_node ${OpenCV_LIBS})  # Link OpenCV

# Install executable
install(TARGETS
  peppabot_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
