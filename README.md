# F1TENTH Autonomous Racing 🏁

This project includes multiple modules for an autonomous racecar.

##  Components:
- **Automatic Emergency Braking (AEB)**:
- **Wall Following**:
- **Follow the Gap**:
- **SLAM (Simultaneous Localization and Mapping)**:
- **Motion Planning**:
- **Perception & Vision / Planning**:


## How to Use:
1. Clone the repository
2. Install ROS2 (Foxy or Galactus)
4. Create the ROS2 Workspace
5. Build the packages with colcon build, 
6. Run the specific scripts within ROS2 nodes to test each module.
7. Build your car, setting up the  Set up your sensors and a Jetson TX or another ai edge computing device. Deploy onto the hardware, run the ros2 nodes, and utilizie your control panel, interface, or controller to test the ROS2 node on the car.
8. Or you can use the RViz Simulation Tool, configure digital sensors and define mappings for a digital car.

## Requirements:

# Hardware / Physical Car Components
(you can buy a kit online from Penn University at the University of Pennsylvania's website for around $4000 USD or build all of the following yourself for around $2500)

- Traxxas RC Rally Car with a Laser-Cut chassis
- LIDAR Mounts
- LiDAR Sensors (Hokuyo is preferred)
- Brushless Motor
- Any Electronic Speed Controllers (VESC is preferred)
-  AI GPU Computing Platform / Device (prefferably an NVIDIA Jetson TX1/TX2) but you can use a simple computing device with less effiency and potential risks like a Raspberry PI if needed.
- 1-2  Battery Packs (5000 mAh) with a XT90 connector


# Software
- Python 3.x
- ROS2 Foxy, Galactus (Robot Operating System)
