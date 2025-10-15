cmake_minimum_required(VERSION 3.8)
project(distobee_gs_feeds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# C++ node
add_library(feed_publisher_node SHARED src/feed_publisher_node.cpp)
target_include_directories(feed_publisher_node PUBLIC ${GST_INCLUDE_DIRS})
target_link_libraries(feed_publisher_node ${GST_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(feed_publisher_node rclcpp rclcpp_components sensor_msgs image_transport cv_bridge)
rclcpp_components_register_node(
  feed_publisher_node
  PLUGIN "distobee_gs_feeds::FeedPublisher"
  EXECUTABLE feed_publisher
)

ament_export_targets(distobee_gs_feeds-targets)
install(TARGETS feed_publisher_node
  EXPORT distobee_gs_feeds-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/feed_preview
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
