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
        Node(
            package='ros2_project',
            executable='ethics_module_node',
            name='ethics_module',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='follow_gap_node',
            name='follow_gap',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='lane_detection_node',
            name='lane_detection',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='lidar_processing_node',
            name='lidar_processing',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='object_detection_node',
            name='object_detection',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='pure_pursuit_node',
            name='pure_pursuit',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='rrt_node',
            name='rrt_pathfinding',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='sensor_fusion_node',
            name='sensor_fusion',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='slam_node',
            name='slam_system',
            output='screen'
        ),
        Node(
            package='ros2_project',
            executable='wall_following_node',
            name='wall_following',
            output='screen'
        ),
    ])
