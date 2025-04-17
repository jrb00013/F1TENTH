#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        #  create subscribers and publishers
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.subscription_scan = self.create_subscription(LaserScan,'scan', self.scan_callback,10)

        #  PID gains set
        self.kp = 0.8
        self.kd = 0.06
        self.ki = 0.00005

        #  store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.L = 0.5


        # initialize EBS variables
        self.wall = 0.0
        self.TTC_start = 1.5  # threshold for triggering EBS
        self.speed = 1.0  # default forward speed
        self.ack_msg = AckermannDriveStamped()



    # get range
    def get_range(self, range_data, angle):
        index = int((math.radians(angle) - range_data.angle_min) / range_data.angle_increment)
        index = max(0, min(index, len(range_data.ranges) - 1))
        distance = range_data.ranges[index]
        return distance if not math.isinf(distance) and not math.isnan(distance) else 0.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall follow the wall to the left going counter clockwise in 
        """

        a = self.get_range(range_data,185)
        b = self.get_range(range_data,225)
        alpha = math.atan((a*math.cos(45) - b)/(a*math.sin(45)))
        D_t = b*math.cos(alpha)
        D_tplus1 = D_t + math.sin(alpha) *2*self.L
        return dist - D_tplus1

    def pid_control(self, error, velocity):
        """
        based on e  error, publish vehicle control
        """
        self.integral += error
        self.derivative = error-self.prev_error
        angle = self.kp*error + self.ki*self.integral + self.kd*self.derivative
        self.prev_error = error
        # Use kp, ki & kd to implement PID controller

        if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 1.6
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 1.6
        else:
            velocity = 0.95
        drive_msg = AckermannDriveStamped()
        ts = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.header.stamp = ts
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.speed = velocity
        self.publisher.publish(drive_msg)

    def odom_callback(self, odom_msg):
         # TODO: update current speed
            self.speed = (-1)*odom_msg.twist.twist.linear.x


    def ebs_condition(self, scan_msg):
        TTC = np.array([], dtype=np.float32)
        angle = -math.radians(175)
        self.get_logger().info('angle_min :%f %f' % (scan_msg.angle_increment, scan_msg.angle_max))
        self.get_logger().info('speed :%f' %self.speed)
        self.get_logger().info('i :%f' %i)


        for j in range(380,700):
            i = scan_msg.ranges[j]
            if i != math.inf and i != math.nan:
                if self.wall == 1:
                    self.speed = 0.3
                # else:
                #     speed = 1.0
                
                
                v_i = self.speed * math.cos(angle)
                
                if v_i > 0:
                    TTC = np.append(TTC, [i/max([v_i,0])])
                else:
                    TTC = np.append(TTC, [100])

            angle += scan_msg.angle_increment
            self.get_logger().info('walle :%s' %str(TTC))
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
        """
        desired_distance = 0.5
        self.ebs_condition(msg)
        self.get_logger().info('walle :%f' %self.wall)
        if self.wall == 1:
            self.get_logger().info('wall :%f' %self.wall)
            self.ack_msg.header.frame_id = "laser"
            self.ack_msg.drive.speed = 0.0
            self.publisher.publish(self.ack_msg)
        else:
            error = self.get_error(msg, desired_distance) #  replace with error calculated by get_error()
            velocity = 1.8 
            self.get_logger().info('error:%f' %error)
            self.get_logger().info('walle :%f' %self.wall)
            
            self.pid_control(error, velocity) # actuate the car with pid




def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
