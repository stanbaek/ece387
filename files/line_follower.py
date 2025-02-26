#!/usr/bin/env python3

# BSD 3-Clause License
# 
# Copyright (c) 2025, Stan Baek
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# Author: Stan Baek
# Date Created: Feb 23, 2025

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import numpy as np


class LineFollower(Node):
    """
    A ROS2 node that follows a center line using odometry and a detected line.
    It calculates the robot's position relative to the line and adjusts its movement to stay aligned.
    """
    def __init__(self):
        super().__init__('line_follower')

        # Subscribe to IMU data to get the robot's current orientation
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
                
        # Subscribe to the center line marker to get the detected line to follow
        self.center_line_sub = self.create_subscription(Marker, 'center_line', self.follow_line, 10)

        # Publish velocity commands to control the robot's movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables to store the robot's current orientation
        self.current_orientation = None

    def imu_callback(self, msg):
        """
        Callback function for IMU data.
        Updates the robot's current position and orientation.
        """
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def follow_line(self, center_line):
        """
        Callback function for the center line marker.
        Computes the robot's alignment with the line and adjusts its movement to follow it.
        """
        if self.current_orientation is None:
            return  # Wait until we have IMU data

        # Extract the first and last points of the center line
        if len(center_line.points) < 2:
            return  # Not enough points to follow a line

        start_point = center_line.points[0]
        end_point = center_line.points[-1]

        # Compute the line direction and angle
        line_dx = end_point.x - start_point.x
        line_dy = end_point.y - start_point.y
        line_angle = np.arctan2(line_dy, line_dx)  # Angle of the line relative to the x-axis

        # Find the slope (m) and intercept (k) for the line equation y = mx + k
        m = line_dy / line_dx
        k = start_point.y - m * start_point.x

        # Convert the line equation y = mx + k into the standard form ax + by + c = 0
        # Here, a = -m, b = 1, c = -k
        b = 1
        a = -m
        c = -k

        # Compute the perpendicular distance (d) from the robot to the line
        # The formula for the distance from a point (x0, y0) to the line ax + by + c = 0 is:
        # d = (a*x0 + b*y0 + c) / sqrt(a^2 + b^2)
        # Since the robot is at the origin (0, 0), d simplifies to c / sqrt(a^2 + b^2)
        distance_error = c / np.sqrt(a * a + b * b)

        # Convert quaternion to yaw angle (robot's current orientation)
        q = self.current_orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        robot_yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Compute angular error to align with the center line
        angle_error = line_angle - robot_yaw

        # Normalize angle error to range [-pi, pi] to avoid large jumps
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        # Set movement commands
        cmd = Twist()
        cmd.linear.x = 0.1  # Move forward with constant speed
        
        kh = 1.0    # heading controller gain
        kd = 5.0    # distance controller gain
        # Adjust rotation speed based on angle error and distance error
        gamma = kh * angle_error - kd * distance_error
                
        # To see this message in real-time, run the node with the `--log-level DEBUG` argument:
        # ros2 run lab8_lidar wall_detector --log-level DEBUG
        self.logger().debug(f'distance error: {d}, angle error: {angle_error}, gamma={gamma}')

        cmd.angular.z = gamma
        # Publish the velocity command
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)  # Keep the node running
    line_follower.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown ROS2


if __name__ == '__main__':
    main()