import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_maze',
            executable='laser_scan_processor_node',
            name='laser_scan_processor',
            output='screen',
        ),
        Node(
            package='turtlebot3_maze',
            executable='odom_processor_node',
            name='odom_processor',
            output='screen',
        ),
        Node(
            package='turtlebot3_maze',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance',
            output='screen',
        ),
        Node(
            package='turtlebot3_maze',
            executable='color_detector_node',
            name='color_detector',
            output='screen',
        ),
    ])

