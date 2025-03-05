# F1TENTH Autonomous Racing 🏁

This project includes multiple modules for autonomous racing. It integrates everything from sensor data processing to motion planning, allowing you to control a self-driving car in real-world and simulated environments.

## 🚀 Key Components:
- **Automatic Emergency Braking (AEB)**:
  - **aeb.py**: Detects obstacles and triggers emergency braking.
  - **lidar_processing.py**: Processes Lidar data to detect obstacles.
  
- **Wall Following**:
  - **wall_follow.py**: Follows walls with PID control.
  - **sensor_fusion.py**: Combines data from various sensors for better accuracy.
  
- **Follow the Gap**:
  - **follow_gap.py**: Avoids obstacles by calculating the best path through open spaces.

- **SLAM (Simultaneous Localization and Mapping)**:
  - **slam.py**: Helps the car understand its position in the environment.
  - **pure_pursuit.py**: Tracks paths to follow a specific route.

- **Motion Planning**:
  - **rrt.py**: Generates random paths for the car to explore.
  - **astar.py**: Implements the A* algorithm for pathfinding.

- **Perception & Vision**:
  - **object_detection.py**: Detects objects on the track.
  - **lane_detection.py**: Identifies the lanes on the track.

- **Perception & Planning**:
  - **behavior_planner.py**: Decides the best actions based on perceived environment.

- **Robot Ethics**:
  - **ethics_module.py**: Implements decision-making constraints in autonomous driving scenarios.

## 🛠️ How to Use:
1. Clone the repository.
2. Set up your sensors and the robot.
3. Run the specific scripts to test each module.

## 📦 Requirements:
- Python 3.x
- ROS (Robot Operating System)
