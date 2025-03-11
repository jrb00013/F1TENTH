import cv2
import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


def behavior_planner(state):
    if state == "ObstacleDetected":
        return "AvoidObstacle"
    elif state == "ClearPath":
        return "Proceed"
    return "Idle"
