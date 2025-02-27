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
from sensor_msgs.msg import Imu
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

        # TODO: Subscribe to the center line marker to get the detected line to follow
        # Callback function: 'self.follow_line' to execute when the topic arrives
        self.center_line_sub = 0

        # Publish velocity commands to control the robot's movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables to store the robot's current orientation
        self.current_orientation = None

    def imu_callback(self, msg: Imu) -> None:
        """
        Callback function for IMU data.
        Updates the robot's current position and orientation.
        """
        self.current_orientation = msg.orientation

    def follow_line(self, center_line: Marker) -> None:
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

        # TODO: Compute the slope of the line and its angle relative to the x-axis.
        # Then, find the slope (m) and intercept (k) for the line equation y = mx + k
        # Hint: Find out the data type of start_point and end_point.
        line_dx = 0 
        line_dy = 0 
        line_angle = 0  # Angle of the line relative to the x-axis       
        m = 0   # slope
        k = 0 # y-intercept

        # TODO: Convert the line equation y = mx + k into the standard form ax + by + c = 0
        # Here, a = -m, b = 1, c = -k
        b = 0
        a = 0
        c = 0

        # TODO: Compute the perpendicular distance (d) from the robot to the line
        # Note that the robot is at the origin (0, 0)
        distance_error = 0

        # Convert quaternion to yaw angle (robot's current orientation)
        q = self.current_orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        robot_yaw = np.arctan2(siny_cosp, cosy_cosp)

        # TODO: Compute angular error to align with the center line
        angle_error = 0

        # Normalize angle error to range [-pi, pi] to avoid large jumps
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        kh = 0.0    # TODO: Pick your heading controller gain
        kd = 0.0    # TODO: Pick your distance controller gain
        # Adjust rotation speed based on angle error and distance error
        gamma = 0
        
        # To see this message in real-time, run the node with the `--log-level DEBUG` argument:
        # ros2 run lab8_lidar wall_detector --log-level DEBUG
        self.get_logger().debug(f'distance error: {distance_error}, angle error: {angle_error}, gamma={gamma}')

        # TODO: Publish the velocity command
        # Note that the forward speed is constant
        

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
