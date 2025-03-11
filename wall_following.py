#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
import time
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.subscription_scan = self.create_subscription(LaserScan,'scan', self.scan_callback,10)
        self.subscription_odom = self.create_subscription(Odometry,'odom', self.odom_callback,10)

        # TODO: set PID gains
        self.kp = 0.8
        self.kd = 0.06
        self.ki = 0.00005

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.L = 0.45

        self.speed = 0.0
        self.wall = 0.0
        self.TTC_start = 1.0

        self.ack_msg = AckermannDriveStamped()

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        distance_index = int(math.radians(angle)/range_data.angle_increment)
        distance = range_data.ranges[distance_index]
        return distance

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        a = self.get_range(range_data,180)
        b = self.get_range(range_data,225)

        a_r = self.get_range(range_data,90)
        b_r = self.get_range(range_data, 45)
        
        alpha = math.atan((a*math.cos(45) - b)/(a*math.sin(45)))
        D_t = b*math.cos(alpha)
        D_tplus1 = D_t + math.sin(alpha) *2*self.L

        alpha_r = math.atan((a_r*math.cos(45) - b_r)/(a_r*math.sin(45)))
        D_t = b_r*math.cos(alpha_r)
        D_tplus1_r = D_t + math.sin(alpha_r) *2*self.L

        dist = (D_tplus1 + D_tplus1_r)/2.0
        return (dist - D_tplus1)

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        self.integral += error
        self.derivative = error-self.prev_error
        angle = self.kp*error + self.ki*self.integral + self.kd*self.derivative
        self.prev_error = error
        # TODO: Use kp, ki & kd to implement a PID controller

        if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 2.0
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 2.0
        else:
            velocity = 2.0
        ts = self.get_clock().now().to_msg()
        self.ack_msg.header.frame_id = "laser"
        # drive_msg.header.stamp = ts
        self.ack_msg.drive.steering_angle = -angle
        self.ack_msg.drive.speed = velocity
        self.publisher.publish(self.ack_msg)
        # TODO: fill in drive message and publish

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = (-1)*odom_msg.twist.twist.linear.x
    
    def ebs_condition(self, scan_msg):
        # 95->175
        TTC = np.array([], dtype=np.float32)
        angle = -math.radians(95)
        # self.get_logger().info('angle_min :%f %f' % (scan_msg.angle_min, scan_msg.angle_max))
        # self.get_logger().info('speed :%f' %self.speed)
        for j in range(380,700):
            i = scan_msg.ranges[int(j)]
            if i != math.inf and i != math.nan:
                if self.wall == 1:
                    self.speed = 0.5
                # else:
                #     speed = 1.0
                
                
                v_i = self.speed * math.cos(angle)
                
                if v_i > 0:
                    TTC = np.append(TTC, [i/max([v_i,0])])
                else:
                    TTC = np.append(TTC, [100])

            angle += scan_msg.angle_increment
        
            # self.get_logger().info('j :%s' %str(scan_msg.ranges))
        # self.get_logger().info('walle :%s' %str(TTC))
        if np.min(TTC) < self.TTC_start:
            self.wall = 1.0
            self.ack_msg.header.frame_id = "laser"
            self.ack_msg.drive.speed = 0.0
            self.publisher.publish(self.ack_msg)
        else:
            self.wall = 0


    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        desired_distance = 0.5
        self.ebs_condition(msg)
        # self.get_logger().info('walle :%f' %self.wall)
        if self.wall == 1:
            self.get_logger().info('wall :%f' %self.wall)
            self.ack_msg.header.frame_id = "laser"
            self.ack_msg.drive.speed = 0.0
            self.publisher.publish(self.ack_msg)
        else:
            error = self.get_error(msg, desired_distance) # TODO: replace with error calculated by get_error()
            velocity = 1.3 # TODO: calculate desired car velocity based on error
            # self.get_logger().info('error:%f' %error)
            # self.get_logger().info('walle :%f' %self.wall)
            
            self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized in")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
