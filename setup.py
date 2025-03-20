from setuptools import setup

package_name = 'ros2_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'rclpy',  # ROS 2 Python client library
        'numpy',  # Example dependency, adjust as needed
        'tensorflow',  # For machine learning models, if you're using them
        'opencv-python',  # If you're doing image processing (like lane detection)
    ],
    data_files=[
        ('share/ament_index/resource_index/packaged_share', ['package.xml']),
    ],
    zip_safe=True,
    author='Your Name',
    author_email='your_email@example.com',
    description='ROS 2 project for autonomous system algorithms.',
    long_description='A more detailed description of the project.',
    long_description_content_type='text/markdown',
    keywords='ROS 2, autonomous vehicles, machine learning',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    entry_points={
        'console_scripts': [
            'behavior_planner_node = ros2_project.behavior_planner:main',
            'aeb_node = ros2_project.aeb:main',
            'astar_node = ros2_project.astar:main',
            'ethics_module_node = ros2_project.ethics_module:main',
            'follow_gap_node = ros2_project.follow_gap:main',
            'lane_detection_node = ros2_project.lane_detection:main',
            'lidar_processing_node = ros2_project.lidar_processing:main',
            'object_detection_node = ros2_project.object_detection:main',
            'pure_pursuit_node = ros2_project.pure_pursuit:main',
            'rrt_node = ros2_project.rrt:main',
            'sensor_fusion_node = ros2_project.sensor_fusion:main',
            'slam_node = ros2_project.slam:main',
            'wall_following_node = ros2_project.wall_following:main',
        ],
    },
)
