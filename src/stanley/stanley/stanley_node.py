#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
# TODO CHECK: include needed ROS msg type headers and libraries
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

from sklearn.neighbors import KNeighborsRegressor
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R

class PidController:
    def __init__(self, kp, ki, kd, integral_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class Stanley(Node):
    """ 
    Implement Stanley controller on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('stanley_node')
        # create ROS subscribers and publishers
        self.ego_odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_sub_callback, 10)
        self.opp_odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odm_sub_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # constants
        self.steering_gain = 1.0
        self.speed_pid = PidController(1.0, 2.0, 0.1, 1.0)
        self.k = 1.0
        self.epsilon = 0.1
        self.L = 0.33
        self.offset = 0.4
        self.max_steering_angle = 0.4
        self.max_speed = 3.0

        # variables
        self.ego_odom = None
        

    def ego_odom_sub_callback(self, odom_msg):
        # extract the yaw of the ego car
        ego_ori = odom_msg.pose.pose.orientation
        ego_ori = R.from_quat([ego_ori.x, ego_ori.y, ego_ori.z, ego_ori.w])
        ego_yaw = ego_ori.as_euler('zyx')[0]
        
        # offset the position of the ego car to the center of the front axle
        ego_pos = odom_msg.pose.pose.position
        ego_pos = np.array([ego_pos.x, ego_pos.y]) + self.L * np.array([np.cos(ego_yaw), np.sin(ego_yaw)])

        # update ego odometry
        self.ego_odom = np.array([ego_pos[0], ego_pos[1], ego_yaw])


    def opp_odm_sub_callback(self, odom_msg):
        # extract yaw of the opponent car
        opp_ori = odom_msg.pose.pose.orientation
        opp_ori = R.from_quat([opp_ori.x, opp_ori.y, opp_ori.z, opp_ori.w])
        opp_yaw = opp_ori.as_euler('zyx')[0]

        # offset the position of the opponent car in negative direction of the y axis
        opp_pos = odom_msg.pose.pose.position
        opp_pos = np.array([opp_pos.x, opp_pos.y]) + self.offset * np.array([np.sin(opp_yaw), -np.cos(opp_yaw)])

        # update opponent odometry
        opp_odom = np.array([opp_pos[0], opp_pos[1], opp_yaw])

        # call the stanley controller
        self.stanley_control(self.ego_odom, opp_odom)


    def stanley_control(self, ego_odom, opp_odom):
        # extract the odometry of the ego and opponent cars
        ego_yaw = ego_odom[2]
        opp_yaw = opp_odom[2]
        ego_pos = ego_odom[:2]
        opp_pos = opp_odom[:2]

        # compute the heading error
        heading_error = opp_yaw - ego_yaw
        if heading_error > np.pi:
            heading_error -= 2 * np.pi
        if heading_error < -np.pi:
            heading_error += 2 * np.pi

        # compute the cross track error
        cross_track_error = np.cross(np.array([np.cos(opp_yaw), np.sin(opp_yaw)]), opp_pos - ego_pos)

        # compute the speed
        speed_error = np.dot(np.array([np.cos(opp_yaw), np.sin(opp_yaw)]), opp_pos - ego_pos)
        speed = self.speed_pid.compute(speed_error, 0.1)
        speed = np.clip(speed, 0.0, self.max_speed)

        # compute the steering angle
        steering_angle = heading_error + np.arctan((self.k * cross_track_error) / (self.epsilon + speed))
        steering_angle = self.steering_gain * steering_angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # publish the drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)
        self.get_logger().info('Publishing: steering_angle: %f, speed: %f' % (steering_angle, speed))

def main(args=None):
    rclpy.init(args=args)
    print("Stanley Initialized")
    stanley_node = Stanley()
    rclpy.spin(stanley_node)

    stanley_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
