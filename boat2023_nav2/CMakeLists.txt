cmake_minimum_required(VERSION 3.5)
project(boat2023_nav2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_bev_costmap_plugin REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY src launch params
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  src/carla_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
