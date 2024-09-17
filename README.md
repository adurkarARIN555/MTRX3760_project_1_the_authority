# MTRX3760 Mechartonic Systems Design
# Project 1 - The Authority - Turtlebot3 Burger Maze Solving Simulation
This is the repository for the Project 1 of Mechatronics Systems Design at the University of Sydney.
## Author Information:
|Name|Student ID (SID)
|------------|------------------------
|Eashan Garg|520439298
|Savith Vinuja Karunanayaka Karunanayakage|510674179
|Arin Adurkar|520587980
|Joel Pivetta|520447488

## Introduction:
The repository contains code to execute a maze solving simulation of Turtlebot3 Burger using ROS2 Humble. The turtlebot navigates freely avoiding obstacles in simulation environment Gazebo Classic and RViz (ROS Visualisation) by using the LiDAR sensor and camera. Using the data received from the LiDAR sensor and camera, the robot takes the next step whether to move forward or rotate to avoid the obstacle infront on the basis of the algorithm. The turtlebot strictly follows the left wall to solve the maze and stops when the camera on it detects the colour green, indicating that it has reached its final destination.

![image](https://github.com/user-attachments/assets/eb2a39e0-d595-4d0d-8b1a-1fa2b161540b) Screenshot 2024-09-17 at 00 13 39

## Workspace Directory Structure:
```
~/turtlebot3_ws/                         # Workspace
  └── build/
  └── install/
  └── log/
  └── src/                               # Source directory
      └── turtlebot3_maze/                  # Package for maze solving simulations
              ├── CMakeLists.txt                  # CMake build configuration
              ├── package.xml                     # Package metadata
              ├── include/                        # Include directory for headers 
              │       ├── color_detector.hpp            # Header for color detector
              │       ├── image_processor.hpp           # Header for image processor
              │       ├── laser_scan_processor.hpp      # Header for laser scan processor
              │       ├── obstacle_avoidance.hpp        # Header for obstacle avoidance
              │       ├── odom_processor.hpp            # Header for odometry processor
              │       ├── robot_pose_processor.hpp      # Header for robot pose processor
              │       ├── sensor_data_processor.hpp     # Header for sensor data processor
              │       └── velocity_commander.hpp        # Header for velocity commander
              └── src/                            # Source files
                      ├── main.cpp                      # Main source file
                      ├── color_detector.cpp            # Source for color detection
                      ├── image_processor.cpp           # Source for image processor
                      ├── laser_scan_processor.cpp      # Source for laser scan processor
                      ├── obstacle_avoidance.cpp        # Source for obstacle avoidance
                      ├── odom_processor.cpp            # Source for odometry processor
                      ├── robot_pose_processor.cpp      # Source for robot pose processor
                      ├── sensor_data_processor.cpp     # Source for sensor data processor
                      └── velocity_commander.cpp        # Source for velocity commander
              └── worlds/
                        ├── turtlebot3_enclosedmaze.world      # Enclosed maze world structure file
                        ├── turtlebot3_floatingmaze.world      # Floating maze world structure file
                        ├── turtlebot3_opentrack.world         # Open track world structure file
              └── launch/
                        ├── turtlebot3_enclosedmaze.launch.py    # Enclosed maze world structure file
                        ├── turtlebot3_floatingmaze.launch.py    # Floating maze world structure file
                        ├── turtlebot3_opentrack.launch.py       # Open-track maze world structure file
                        ├── robot_state_publisher.launch.py      # Enclosed maze world structure 
                        ├── spawn_turtlebot3.launch.py           # Enclosed maze world structure file
                        ├── turtlebot3_launch.py                 # Floating maze world structure file
              
                        
```
Coding Standard
- Hungarian Notation
- Comments: // This is a comment
- for ()
  {
    This is a for loop;
  }

