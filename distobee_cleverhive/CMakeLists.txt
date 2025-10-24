cmake_minimum_required(VERSION 3.8)
project(distobee_cleverhive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# feed_publisher node
add_library(feed_publisher_node SHARED src/feed_publisher_node.cpp)
target_include_directories(feed_publisher_node PUBLIC ${GST_INCLUDE_DIRS})
target_link_libraries(feed_publisher_node ${GST_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(feed_publisher_node rclcpp rclcpp_components sensor_msgs image_transport cv_bridge)
rclcpp_components_register_node(
  feed_publisher_node
  PLUGIN "distobee_cleverhive::FeedPublisher"
  EXECUTABLE feed_publisher
)

# cmd_vel_odometry node
add_library(cmd_vel_odometry_node SHARED src/cmd_vel_odometry_node.cpp)
ament_target_dependencies(cmd_vel_odometry_node rclcpp rclcpp_components geometry_msgs nav_msgs tf2)
rclcpp_components_register_node(
  cmd_vel_odometry_node
  PLUGIN "distobee_cleverhive::CmdVelOdometry"
  EXECUTABLE cmd_vel_odometry
)

ament_export_targets(distobee_cleverhive-targets)
install(TARGETS feed_publisher_node cmd_vel_odometry_node
  EXPORT distobee_cleverhive-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
