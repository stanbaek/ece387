#!/usr/bin/env python3

import yaml  # For reading YAML metadata
import numpy as np  # For numerical operations
import matplotlib.pyplot as plt  # For plotting the map
from PIL import Image  # For loading the PGM image file

# Step 1: Load the map metadata from the YAML file
yaml_file = "map.yaml"
with open(yaml_file, 'r') as file:
    map_metadata = yaml.safe_load(file)  # Parse YAML content into a dictionary

# Extract key parameters from the YAML file
pgm_file = map_metadata["image"]  # The filename of the PGM image (map)
resolution = map_metadata["resolution"]  # Map resolution in meters per pixel
origin = map_metadata["origin"]  # The bottom-left corner of the map in world coordinates
# Example origin: [-10.0, -10.0, 0.0] means the map starts at (-10m, -10m) in the world frame

# Step 2: Load the PGM map image as a NumPy array
image = Image.open(pgm_file)  # Open the grayscale map image
map_data = np.array(image)  # Convert the image to a NumPy array (2D array of pixel values)

# Step 3: Convert pixel values into an occupancy grid
# ROS maps use the following conventions:
# - 0 (black): Occupied space (walls, obstacles)
# - 255 (white): Free space
# - 205 (gray): Unknown space
occupancy_grid = np.zeros(map_data.shape)  # Initialize an empty grid of the same size as the image

# TODO: Map occupancy values to the occupancy_grid matrix.
# occupancy_grid is an np array with a shape attribute.
# Loop through rows (height) and columns (width).
# Retrieve the grayscale pixel value:
# - Black (0): Fully occupied (walls); ROS OccupancyGrid uses 100 for obstacles.
# - White (255): Free space; ROS OccupancyGrid uses 0 for free space.
# - Gray (205): Unknown; ROS OccupancyGrid uses -1 for unknown areas.


# TODO: Step 4: Compute real-world map size in meters
height_m = 0 # Total map height in meters
width_m = 0  # Total map width in meters

# Step 5: Generate axis values in meters
# The extent defines the real-world coordinate range for the map
x_min = origin[0]  # X-axis minimum (from YAML file)
x_max = origin[0] + width_m  # X-axis maximum (computed from resolution)
y_min = origin[1]  # Y-axis minimum
y_max = origin[1] + height_m  # Y-axis maximum

# Step 7: Plot the map using Matplotlib
plt.figure(figsize=(10, 10))  # Set figure size

# Use `cmap="gray_r"` to correctly display occupancy values
# - "gray_r" (reversed) ensures black=occupied, white=free
plt.imshow(occupancy_grid, cmap="gray_r", extent=[x_min, x_max, y_min, y_max])

# Add labels and a title
plt.title("TurtleBot3 SLAM Map")
plt.xlabel("X (meters)")  # X-axis in meters
plt.ylabel("Y (meters)")  # Y-axis in meters
plt.colorbar(label="Occupancy Value")  # Add a legend for occupancy values

# Show the final plot
plt.show()
