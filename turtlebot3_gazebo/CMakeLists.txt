################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(turtlebot3_msgs REQUIRED)

################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${turtlebot3_msgs_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "turtlebot3_msgs" 
)

# Define the libraries
set(LIB_LASER_SCAN_PROCESSOR "laser_scan_processor")
add_library(${LIB_LASER_SCAN_PROCESSOR} src/laser_scan_processor.cpp)
ament_target_dependencies(${LIB_LASER_SCAN_PROCESSOR} ${dependencies})

set(LIB_ODOM_PROCESSOR "odom_processor")
add_library(${LIB_ODOM_PROCESSOR} src/odom_processor.cpp)
ament_target_dependencies(${LIB_ODOM_PROCESSOR} ${dependencies})

set(LIB_OBSTACLE_AVOIDANCE "obstacle_avoidance")
add_library(${LIB_OBSTACLE_AVOIDANCE} src/obstacle_avoidance.cpp)
ament_target_dependencies(${LIB_OBSTACLE_AVOIDANCE} ${dependencies})

# Define the main executable
set(EXEC_MAIN "main")
add_executable(${EXEC_MAIN} src/main.cpp)
ament_target_dependencies(${EXEC_MAIN} ${dependencies})

# Link the libraries to the main executable
target_link_libraries(${EXEC_MAIN} ${LIB_LASER_SCAN_PROCESSOR} ${LIB_ODOM_PROCESSOR} ${LIB_OBSTACLE_AVOIDANCE})

# Install targets
install(TARGETS ${EXEC_MAIN} ${LIB_LASER_SCAN_PROCESSOR} ${LIB_ODOM_PROCESSOR} ${LIB_OBSTACLE_AVOIDANCE}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(turtlebot3_msgs)
ament_package()