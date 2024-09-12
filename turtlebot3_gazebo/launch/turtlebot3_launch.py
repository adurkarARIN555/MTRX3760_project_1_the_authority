import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the LaserScanProcessor node
        Node(
            package='turtlebot3_gazebo',
            executable='laser_scan_processor',
            name='laser_scan_processor',
            output='screen',
        ),
        # Launch the OdometryProcessor node
        Node(
            package='turtlebot3_gazebo',
            executable='odom_processor',
            name='odom_processor',
            output='screen',
        ),
        # Launch the ObstacleAvoidance node
        Node(
            package='turtlebot3_gazebo',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen',
        ),
        # Launch the main node
        Node(
            package='turtlebot3_gazebo',
            executable='main',
            name='main',
            output='screen',
        ),
    ])
