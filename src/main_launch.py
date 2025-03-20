# launch/main_launch.py
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_project',
            executable='behavior_planner_node',
            name='behavior_planner',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        ),
        Node(
            package='ros2_project',
            executable='aeb_node',
            name='aeb_system',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='astar_node',
            name='astar_pathfinding',
            output='screen'
        ),
        # Add other nodes here
    ])
