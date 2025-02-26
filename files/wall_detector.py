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
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sklearn.linear_model import RANSACRegressor
from sklearn.cluster import DBSCAN

# Define a list of colors for visualization
Colors = [
    [1.0, 0.0, 0.0],  # Red
    [0.0, 1.0, 0.0],  # Green
    [0.0, 0.0, 1.0],  # Blue
    [1.0, 1.0, 0.0],  # Yellow
    [0.0, 1.0, 1.0],  # Cyan
    [1.0, 0.0, 1.0],  # Magenta
    [0.5, 0.5, 0.5],  # Gray
]


class WallDetector(Node):
    """
    A ROS2 node that detects walls using LiDAR data.
    It clusters LiDAR points, applies RANSAC to fit lines, and publishes detected walls and center lines.
    """
    def __init__(self):
        super().__init__('wall_detector')

        # Subscribe to LiDAR to detect walls
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Subscribe to IMU data to get the robot's current orientation
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.yaw = 0  # Robot's current orientation (yaw angle)

        # Publishers for RViz visualization
        self.wall_marker_pub = self.create_publisher(Marker, 'wall_markers', 10)
        self.center_marker_pub = self.create_publisher(Marker, 'center_line', 10)
        self.get_logger().info("Wall Detector Node Started")

    def imu_callback(self, msg):
        """
        Callback function for IMU data to extract yaw (orientation of the robot).
        Converts the quaternion orientation to a yaw angle.
        """
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """
        Callback function for LiDAR scan data.
        It processes the LiDAR points, applies clustering and RANSAC, and detects wall lines.
        """
        # Extract LiDAR scan angles and ranges
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Create a boolean mask to filter points:
        # 1. Points in front of the robot (angles < pi/2 or angles > 3*pi/2)
        # 2. Points within the specified range (min_range <= r <= max_range)
        mask = ((angles < np.pi / 2) | (angles > 3 * np.pi / 2)) & \
            (np.array(msg.ranges) >= msg.range_min) & \
            (np.array(msg.ranges) <= msg.range_max)
        angles = angles[mask]
        r = np.array(msg.ranges)[mask]

        # Convert polar coordinates (angle, range) to Cartesian coordinates (x, y)
        x_vals = r * np.cos(angles)
        y_vals = r * np.sin(angles)

        # Cluster LiDAR points using DBSCAN (Density-Based Spatial Clustering of Applications with Noise)
        points = np.column_stack((x_vals, y_vals))
        clustering = DBSCAN(eps=0.25, min_samples=15).fit(points)
        unique_labels = set(clustering.labels_)

        # To see this message in real-time, run the node with the `--log-level DEBUG` argument:
        # ros2 run lab8_lidar wall_detector --log-level DEBUG
        self.get_logger().debug(f'unique labels: {unique_labels}')
                
        wall_lines = []  # Stores detected wall lines (slope, intercept)
        color_index = 0  # Index to cycle through colors for visualization
        wall_clusters = []  # Stores points belonging to detected walls

        for label in unique_labels:
            if label == -1:
                continue  # Ignore noise points (DBSCAN labels noise as -1)

            # Extract points belonging to the current cluster
            cluster_points = points[clustering.labels_ == label]
            
            # To see this message in real-time, run the node with the `--log-level DEBUG` argument:
            # ros2 run lab8_lidar wall_detector --log-level DEBUG
            self.get_logger().debug(f'label={label}, cluster size={len(cluster_points)}')
 
            if len(cluster_points) < 15:
                continue  # Skip clusters with too few points for reliable line fitting

            # Visualize the cluster in RViz (blue color)
            self.publish_marker(cluster_points[:, 0], cluster_points[:, 1], self.wall_marker_pub, [0.0, 0.0, 1.0])

            # Use RANSAC to detect lines within the cluster
            remaining_points = cluster_points.copy()

            for _ in range(2):  # Try to find up to 2 walls in the cluster
                if len(remaining_points) < 12:
                    break  # Not enough points left for line fitting

                # Initialize RANSAC for robust line fitting
                # - min_samples: min number of points are required to fit a line.
                # - residual_threshold: The maximum distance (in meters) a point
                ransac = RANSACRegressor(min_samples=10, residual_threshold=0.05)
                ransac.fit(remaining_points[:, 0].reshape(-1, 1), remaining_points[:, 1])

                inliers = ransac.inlier_mask_
                if inliers is None or not any(inliers):
                    break  # No valid inliers found

                # At this point, RANSAC has successfully fitted a line, and we have valid inliers.

                # Extract points that belong to the fitted line
                fitted_points = remaining_points[inliers]

                # Assign a color to the detected wall
                color = Colors[color_index]
                color_index = (color_index + 1) % len(Colors)

                # Visualize the detected wall in RViz
                self.publish_marker(fitted_points[:, 0], fitted_points[:, 1], self.wall_marker_pub, color)

                # Extract the slope and intercept of the fitted line
                slope = ransac.estimator_.coef_[0]
                intercept = ransac.estimator_.intercept_

                # To see this message in real-time, run the node with the `--log-level DEBUG` argument:
                # ros2 run lab8_lidar wall_detector --log-level DEBUG
                self.get_logger().debug(f'slope={slope}, intercept={intercept}')
 
                # Store the line parameters and corresponding points
                wall_lines.append((slope, intercept))
                wall_clusters.append(fitted_points)

                # Remove inliers for the next iteration to find additional lines
                remaining_points = remaining_points[~inliers]

        # To see this message in real-time, run the node with the `--log-level DEBUG` argument:
        # ros2 run lab8_lidar wall_detector --log-level DEBUG
        self.get_logger().debug(f'Detected walls: [{", ".join(f"({m:.3f}, {c:.3f})" for m, c in wall_lines)}]')

        # If at least 2 walls are detected, compute the center line
        if len(wall_lines) >= 2:
            # Sort walls by their alignment with the robot's orientation
            wall_lines.sort(key=lambda line: abs(np.arctan2(line[0], 1) - self.yaw))
            parallel_walls = wall_lines[:2]  # Select the two most parallel walls

            # Compute the midpoint between the two walls
            m1, c1 = parallel_walls[0]
            m2, c2 = parallel_walls[1]
            mid_c = (c1 + c2) / 2
            mid_m = (m1 + m2) / 2

            # Generate points for the center line
            mid_x = np.linspace(min(x_vals), max(x_vals), num=10)
            mid_y = mid_m * mid_x + mid_c

            # Visualize the center line in RViz (green color)
            self.publish_marker(mid_x, mid_y, self.center_marker_pub, [0.0, 1.0, 0.0])

    def publish_marker(self, x_vals, y_vals, publisher, color):
        """
        Helper function to publish a line marker in RViz.
        """
        marker = Marker()
        marker.header.frame_id = 'base_link'  # Reference frame for the marker
        marker.type = Marker.LINE_STRIP  # Type of marker (a line strip)
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # Line width
        marker.color.r, marker.color.g, marker.color.b = color  # Line color
        marker.color.a = 1.0  # Line opacity
        marker.points = []

        # Add points to the line strip
        for x, y in zip(x_vals, y_vals):
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            marker.points.append(p)

        # Publish the marker
        publisher.publish(marker)


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    detector = WallDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()