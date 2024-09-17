################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_maze)

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
find_package(visualization_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


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
  ${GAZEBO_MSGS_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS} 
  ${visualization_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "turtlebot3_msgs" 
  "visualization_msgs"
  "gazebo_msgs"
  "rcl_interfaces"
  "cv_bridge"
  "OpenCV"
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

set(LIB_SENSOR_DATA_PROCESSOR "sensor_data_processor")
add_library(${LIB_SENSOR_DATA_PROCESSOR} src/sensor_data_processor.cpp)
ament_target_dependencies(${LIB_SENSOR_DATA_PROCESSOR} ${dependencies})

set(LIB_ROBOT_POSE_PROCESSOR "robot_pose_processor")
add_library(${LIB_ROBOT_POSE_PROCESSOR} src/robot_pose_processor.cpp)
ament_target_dependencies(${LIB_ROBOT_POSE_PROCESSOR} ${dependencies})

set(LIB_VELOCITY_COMMANDER "velocity_commander")
add_library(${LIB_VELOCITY_COMMANDER} src/velocity_commander.cpp)
ament_target_dependencies(${LIB_VELOCITY_COMMANDER} ${dependencies})

set(LIB_COLOR_DETECTOR "color_detector")
add_library(${LIB_COLOR_DETECTOR} src/color_detector.cpp)
ament_target_dependencies(${LIB_COLOR_DETECTOR} ${dependencies})

set(LIB_IMAGE_PROCESSOR "image_processor")
add_library(${LIB_IMAGE_PROCESSOR} src/image_processor.cpp)
ament_target_dependencies(${LIB_IMAGE_PROCESSOR} ${dependencies})

# Define the main executable
set(EXEC_MAIN "main")
add_executable(${EXEC_MAIN} src/main.cpp)
ament_target_dependencies(${EXEC_MAIN} ${dependencies})



# Link the libraries to the main executable
target_link_libraries(${EXEC_MAIN} 
  ${LIB_LASER_SCAN_PROCESSOR} 
  ${LIB_ODOM_PROCESSOR} 
  ${LIB_OBSTACLE_AVOIDANCE} 
  ${LIB_SENSOR_DATA_PROCESSOR}
  ${LIB_ROBOT_POSE_PROCESSOR}
  ${LIB_VELOCITY_COMMANDER}
  ${LIB_COLOR_DETECTOR}
  ${LIB_IMAGE_PROCESSOR}
)

# Install targets
install(TARGETS ${EXEC_MAIN} ${LIB_LASER_SCAN_PROCESSOR} ${LIB_ODOM_PROCESSOR} ${LIB_OBSTACLE_AVOIDANCE}
  ${LIB_SENSOR_DATA_PROCESSOR} ${LIB_ROBOT_POSE_PROCESSOR} ${LIB_VELOCITY_COMMANDER} ${LIB_COLOR_DETECTOR} ${LIB_IMAGE_PROCESSOR}
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
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(rcl_interfaces)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(OpenCV)
ament_package()