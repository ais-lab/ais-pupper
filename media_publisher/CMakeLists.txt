cmake_minimum_required(VERSION 3.8)
project(media_publisher)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# Library
add_library(media_publisher_node
    src/media_publisher_node.cpp
)
target_include_directories(media_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(media_publisher_node
  rclcpp sensor_msgs cv_bridge OpenCV image_transport
)

# Executable
add_executable(media_sender_node 
    src/main.cpp
)
target_link_libraries(media_sender_node media_publisher_node)
ament_target_dependencies(media_sender_node
  rclcpp sensor_msgs cv_bridge OpenCV image_transport
)

# Install (Optional but good practice)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS media_publisher_node media_sender_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
