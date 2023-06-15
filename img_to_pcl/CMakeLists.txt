cmake_minimum_required(VERSION 3.5)
project(img_to_pcl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# add executables
add_executable(img_to_pcl src/img_to_pcl.cpp)
ament_target_dependencies(img_to_pcl rclcpp sensor_msgs cv_bridge pcl_conversions pcl_ros)

# install targets so ros2 run can find executable
install(TARGETS
  img_to_pcl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
