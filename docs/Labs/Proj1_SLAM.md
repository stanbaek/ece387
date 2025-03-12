# 🚀 Proj 1: SLAM

(not ready yet)
## 📌 Objectives
- Students should be able to implement a ROS2 node to detect walls using LiDAR data.

## 📜 Overview

In this project, we will enable our robot to autonomously navigate an unknown maze and build a map of the environment. As in the previous lab, we will use LiDAR to detect the walls of the maze (or obstacles) surrounding the robot. We will be utilizing the Simultaneous Localization and Mapping (SLAM) library provided by ROS2 and TurtleBot3.

SLAM, or Simultaneous Localization and Mapping, is a process used in robotics to enable a robot to build a map of an unknown environment while simultaneously determining its location within that map. It involves combining sensor data, algorithms, and probabilistic methods to perform real-time mapping and localization. SLAM is crucial for autonomous robots to operate effectively in environments where pre-existing maps are not available.

SLAM is one of the fundamental algorithms in robotics and is widely used in applications such as autonomous vehicles, drone navigation, and robotic vacuum cleaners. It enables robots to navigate dynamic and unfamiliar environments without relying on GPS or pre-defined maps, which is essential for many real-world scenarios.

SLAM integrates data from sensors like LiDAR and odometry to construct and update a map while estimating the robot's position. Through statistical methods like Kalman Filters or Particle Filters, SLAM corrects errors in localization and mapping to achieve accurate results. While the underlying mathematics involves advanced topics in statistics and optimization, libraries provided in ROS2 simplify SLAM's implementation, making it accessible for practical applications.

We will use Cartographer in this lab because it provides an efficient and accurate SLAM solution for 2D environments like the maze we’ll be mapping. Its ability to handle LiDAR data and update maps in real time makes it ideal for this project. Furthermore, its compatibility with TurtleBot3 and ROS2 simplifies the setup, allowing us to focus on understanding the SLAM process and its applications.

## 🛠️ Lab Procedures

### 1. **Setting Up TurtleBot3 with SLAM in Gazebo**
Follow these steps to simulate SLAM with TurtleBot3 in the Gazebo environment.

1. Launch the Gazebo world:
   ```bash
   $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

1. Open another terminal and run the SLAM node:
   ```bash
   $ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
   ```

1. Use `gamepad` to manually navigate the robot in Gazebo and build the map:
   ```bash
   $ ros2 launch lab4_gamepad gamepad.launch.py
   ```

1. Once the mapping process is complete, save the generated map:
   ```bash
   $ ros2 run nav2_map_server map_saver_cli -f ~/map
   ```

1. Download [`map_plotter.py`](../files/map_plotter.py) to your `home` directory and make it executable.
   ```bash
   $ chmod +x map_plotter.py
   ```
   Then, verify if the file is now executable using `ls -l`

   ```{important}
   If you are asked to write the command that makes a file executable only for the file owner, you should be able to answer in your GR. 😉
   ```





### 2. **Autonomous Navigation with TurtleBot3**
After building and saving the map, use it for autonomous navigation.

1. Launch the navigation stack:
    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=/path/to/your/saved/map.yaml
    ```

1. Use RViz to set an initial pose for the robot and specify the goal location. Follow these steps:
   - Open RViz with the navigation configuration:
     ```bash
     ros2 launch turtlebot3_navigation2 navigation2_rviz.launch.py
     ```
   - Use the "2D Pose Estimate" tool in RViz to set the robot's initial position.
   - Use the "2D Nav Goal" tool to set the robot's navigation target.

2. The robot will autonomously navigate to the goal using the SLAM-generated map.



### ✍️ Important Notes:
- **SLAM Basics**: Remember, this lab is designed to expose you to SLAM concepts practically. While understanding the underlying mathematical models is ideal, focus on grasping how SLAM operates and its real-world applications.
- **Journal Requirements**: As part of this lab, you are required to document the following:
  - Steps followed during SLAM and navigation.
  - Screenshots of the map generated in RViz.
  - Issues encountered and how you resolved them.
  - Reflections on the challenges and learning outcomes.
- **Support**: If you encounter any issues, use the official TurtleBot3 e-Manual or consult with your peers.



























To run **autonomous SLAM** using **Cartographer** with TurtleBot3 in ROS2, you’ll need to:

1. **Set up Cartographer** to build the map.  
2. **Use Navigation2** to autonomously explore the environment and update the map in real-time.  

### ✅ **Full Step-by-Step Instructions:**

---

### **1. Start Gazebo with TurtleBot3**
Launch the TurtleBot3 in the Gazebo simulation:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

### **2. Start SLAM with Cartographer**
Start Cartographer to perform SLAM:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

- This will start the SLAM process, and Cartographer will begin building the map as you move the robot.

---

### **3. Start Navigation2**
You can now run Navigation2 alongside Cartographer to allow the robot to navigate autonomously using the evolving map:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

- Cartographer will continue updating the map dynamically as the robot navigates.

---

### **4. Start RViz**
Open RViz to visualize the map and set navigation goals:

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

- In RViz:
   - Use **2D Pose Estimate** to set the robot's initial position (if available).
   - Use **2D Nav Goal** to set a navigation target — the robot will explore and update the map.

---

### **5. Save the Map**  
Once you’re satisfied with the map, save it using:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

This creates `map.yaml` and `map.pgm` files in your home directory.

---

### **6. Optional: Run Navigation2 with the Saved Map**  
If you want to use the saved map for localization and navigation later (without running Cartographer), you can launch it like this:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=/path/to/your/saved/map.yaml
```

---

### 🚀 **Summary:**  
✅ Start **Cartographer** → ✅ Start **Navigation2** → ✅ Set Goals in **RViz** → ✅ Save the Map  

---

This setup will let TurtleBot3 explore and map autonomously while using Cartographer for SLAM. 















If you want TurtleBot3 to follow **multiple goals** autonomously using **Nav2**, you can achieve it in a few different ways:

---

## ✅ **Option 1: Use a Python Script with an Action Client**
You can create a Python script to send a sequence of goals to Nav2 using the **`FollowWaypoints`** action.

### **Example Script to Send Multiple Goals:**

1. Create a new Python script:
```bash
mkdir -p ~/master_ws/src/multi_goal_nav
cd ~/master_ws/src/multi_goal_nav
touch multi_goal_nav.py
chmod +x multi_goal_nav.py
```

2. Add the following code to `multi_goal_nav.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import time

class MultiGoalNav(Node):
    def __init__(self):
        super().__init__('multi_goal_nav')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.client.wait_for_server()

    def send_goals(self):
        # Define multiple goals as PoseStamped messages
        goals = []

        goal_1 = PoseStamped()
        goal_1.header.frame_id = 'map'
        goal_1.header.stamp = self.get_clock().now().to_msg()
        goal_1.pose.position.x = 1.0
        goal_1.pose.position.y = 0.5
        goal_1.pose.orientation.w = 1.0
        goals.append(goal_1)

        goal_2 = PoseStamped()
        goal_2.header.frame_id = 'map'
        goal_2.header.stamp = self.get_clock().now().to_msg()
        goal_2.pose.position.x = -0.5
        goal_2.pose.position.y = 1.0
        goal_2.pose.orientation.w = 1.0
        goals.append(goal_2)

        goal_3 = PoseStamped()
        goal_3.header.frame_id = 'map'
        goal_3.header.stamp = self.get_clock().now().to_msg()
        goal_3.pose.position.x = 0.0
        goal_3.pose.position.y = -1.0
        goal_3.pose.orientation.w = 1.0
        goals.append(goal_3)

        self.get_logger().info(f"Sending {len(goals)} goals...")
        
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = goals

        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation complete with {result.missed_waypoints} missed waypoints.')

def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalNav()
    time.sleep(2)  # Ensure connection to server
    node.send_goals()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. **Build and Run:**

```bash
colcon build --packages-select multi_goal_nav
source install/setup.bash
ros2 run multi_goal_nav multi_goal_nav.py
```

---

## ✅ **Option 2: Use RViz Waypoints Plugin**  
You can also use a plugin in RViz to set multiple waypoints:

1. In **RViz**, add the **"Waypoint Follower"** plugin:
   - Open RViz.
   - Click **"Add"** → **"By Topic"** → Select `/goal_pose`.

2. In the RViz toolbar:
   - After adding the plugin, you should see a **"Set Waypoints"** button.
   - Click **"Set Waypoints"** to define multiple goal points on the map.
   - Once all waypoints are set, click **"Follow Waypoints"** to start autonomous navigation.

---

## ✅ **Option 3: Use a YAML File for Goals**  
You can create a list of goals in a YAML file and load it at runtime:

1. Create a YAML file (`multi_goals.yaml`) like this:

```yaml
goals:
  - pose:
      position:
        x: 1.0
        y: 0.5
      orientation:
        w: 1.0
  - pose:
      position:
        x: -0.5
        y: 1.0
      orientation:
        w: 1.0
  - pose:
      position:
        x: 0.0
        y: -1.0
      orientation:
        w: 1.0
```

2. Create a launch file to read the goals and send them to Nav2 using `FollowWaypoints` action.

---

## 🚀 **Recommended Approach:**  
- For flexible automation → Use **Option 1** (Python script).  
- For interactive use → Use **Option 2** (RViz Waypoints Plugin).  
- For repeated runs → Use **Option 3** (YAML file).  







```{image} ./figures/rplidar.png
:width: 300  
:align: center  
```

### 📹 How LiDAR Works
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/EYbhNSUnIdU?si=idFVZOttywfaJ_Ys" title="YouTube video player" frameborder="0" allowfullscreen></iframe>
</center>

### 📹 TurtleBot3 LiDAR Example
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/9oic8aT3wIc?si=nedLzJ4oj7beh2Xk" title="YouTube video player" frameborder="0" allowfullscreen></iframe>
</center>

## 🌱 Pre-Lab: Setting Up and Testing LiDAR

Before we dive into wall detection, we need to ensure that our setup is working correctly.

### Using ROS2 Launch Files

Managing multiple ROS nodes can become overwhelming, especially as our system grows. Instead of running each node in separate terminals, we’ll use ROS2 launch files to simplify the process.

1. **Navigate to Your Package**: Open a terminal and move to the `lab4_gamepad` package.
   ```bash
   cd ~/master_ws/src/ece387_ws/lab4_gamepad
   ```

2. **Create a Launch Directory**: Create a `launch` directory:

3. **Create a Launch File**: Inside the `launch` directory, create a new file named `gamepad.launch.py`:

4. **Open the File in VS Code**: Open the newly created file in your code editor.

5. **Copy and Paste the Following Code**:
   ```python
   import launch
   import launch_ros.actions

   def generate_launch_description():
       """
       Launches the gamepad node from lab4_gamepad
       and the joy_node from the joy package.
       """
       return launch.LaunchDescription([
           launch_ros.actions.Node(
               package='joy',
               executable='joy_node',
               name='joy_node',
               output='screen'
           ),
           launch_ros.actions.Node(
               package='lab4_gamepad',
               executable='gamepad',
               name='gamepad_node',
               output='screen'
           ),
       ])
   ```
6. **Modify `setup.py` to Include the Launch File**: Open `setup.py` and add:
   ```python
   import os
   from glob import glob
   ```
   Then, add this line inside `data_files`:
   ```python
   (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
   ```
7. **Update `package.xml`**: Ensure the dependencies include:
   ```xml
   <depend>launch</depend>
   <depend>launch_ros</depend>
   ```
8. **Build the Package**:
   ```bash
   ccbuild --packages-select lab4_gamepad
   ```
9. **Run the Launch File**:
   ```bash
   ros2 launch lab4_gamepad gamepad.1aunch.py
   ```
10. **Verify Nodes Are Running**:
    ```bash
    ros2 node list
    ```
    You should see `gamepad_node` and `joy_node` listed.

### Install Packages

1. Use the following command to install the `scikit-learn` machine learning package for Python:
    ```bash
    pip install scikit-learn
    ```

1. Download the [`square path Gazebo files`](../files/squarepath_gazebo.tar.xz). Extract the files and move them inside the appropriate directories in `~/master_ws/src/turtlebot3_simulations/turtlebot3_gazebo`. Ensure each new directory is moved to the existing directory with the same name.

1. Return to the ROS2 workspace root and build the package or simply run `ccbuild`.

1. Open the Gazebo simulation:
    ```bash
    ros2 launch turtlebot3_gazebo square_path.launch.py
    ```
    It will open the Gazebo simulation as shown in the figure below.
    ```{image} ./figures/Lab8_SquarePathGazebo.png
    :width: 500  
    :align: center  
    ```  

1. Type the following and observe the command output:
    ```bash
    ros2 topic list
    ros2 topic info /scan
    ros2 topic echo /scan
    ```  

1. Find the message type of the `/scan` topic:
    ```bash
    ros2 topic type /scan
    ```
    Then, examine the details of the message type.

1. Open the RViz visualization tool:
    ```bash
    ros2 launch turtlebot3_bringup rviz.launch.py
    ```
    This should open an RViz window where we can visualize ROS components of our system. In the `Displays` menu on the left, you should see two submenus of interest: `LaserScan` and `RobotModel`. These allow us to depict the TurtleBot3 and LiDAR data. You should see red dots fill the **rviz** map where obstacles exist, as shown below.
    ```{image} ./figures/Lab8_SquarePathRViz.png
    :width: 500  
    :align: center  
    ```  



## 💻 Lab Procedure: LiDAR-Based Wall Detection

Follow the steps below to set up your ROS 2 package, implement the required scripts, and run the simulation or real robot.

### Setting Up Your ROS2 Package

1. **Navigate to Your Workspace:** Open a terminal and move into your ROS 2 workspace directory:
   ```bash
   cd ~/master_ws/src/ece387_ws
   ```

1. **Create a New ROS2 Package:** Create a new package named `lab8_lidar` with the BSD-3 license:

1. **Add Dependencies:** Edit `package.xml` to include the following dependencies:
    ```xml
    <depend>rclpy</depend>
    <depend>sensor_msgs</depend>
    <depend>geometry_msgs</depend>
    <depend>nav2_msgs</depend>
    <depend>tf2_ros</depend>
    <depend>visualization_msgs</depend>
    <depend>numpy</depend>
    <depend>scikit-learn</depend>
    ```

3. **Download Required Scripts**: Download the following scripts and save them in the `lab8_lidar/lab8_lidar` directory:

   - [`wall_detector.py`](../files/wall_detector.py)
   - [`line_follower.py`](../files/line_follower.py)

1. **Modify `setup.py`**: Add your scripts under `entry_points`.

### Implementing the Python Scripts

1. **Complete `wall_detector.py`**  
   Open the `wall_detector.py` script and implement the missing functionality as described in the comments. This script will process LiDAR data to detect walls and publish visualization markers.

2. **Complete `line_follower.py`**  
   Open the `line_follower.py` script and implement the missing functionality as described in the comments. This script will enable the robot to follow the detected center line between walls.

3. **Build the Package**  
   After completing the scripts, build the package:
   ```bash
   ccbuild --packages-select lab8_lidar
   ```

### **Running the Simulation**

1. **Launch Gazebo with TurtleBot3** Start the Gazebo simulation with the TurtleBot3 robot:
   ```bash
   ros2 launch turtlebot3_gazebo square_path.launch.py
   ```

2. **Open RViz for Visualization** Launch RViz to visualize the detected walls and determine the center line:
   ```bash
   ros2 launch turtlebot3_bringup rviz.launch.py
   ```

3. **Run the Wall Detector Node**  Start the wall detection node:
   ```bash
   ros2 run lab8_lidar wall_detector
   ```

4. **Observe the Detected Walls in RViz**  
   - Unselect `TF` and `Odometry` in RViz.  
   - Click the `Add` button in the bottom left of the RViz window, select the `By topic` tab, and add the `Marker` messages under the `/center_line` and `/wall_markers` topics.

5. **Debug If Needed** If you encounter issues, run the wall detector node in debug mode:
   ```bash
   ros2 run lab8lidar wall_detector --ros-args --log-level debug
   ```

6. **Update Parameters** Adjust the parameters for the `DBSCAN` clustering function and the `RANSACRegressor` function to improve wall detection.

7. **Run the Line Follower Node** Once the center line is detected, start the line follower node:
   ```bash
   ros2 run lab8_lidar line_follower
   ```
   Ensure you also launch `gamepad.launch.py` to relinquish control. 

8. **Tune the Controller Gains**  
   Adjust the `kh` (heading gain) and `kd` (distance gain) values in the `line_follower.py` script to optimize the robot's line-following performance.

### Running the Real Robot

1. **Close the Gazebo Simulation** Ensure the Gazebo simulation is closed before running the real robot. Never run the simulation and the real robot simultaneously.

2. **Reopen RViz** Launch RViz to visualize the wall detection on the real robot:
   ```bash
   ros2 launch turtlebot3_bringup rviz.launch.py
   ```

3. **Run the Wall Detector Node** Start the wall detection node:
   ```bash
   ros2 run lab8lidar wall_detector
   ```

4. **Fix QoS Warning** If you see the following warning:
   ```bash
   [WARN] [wall_detector]: New publisher discovered on topic '/scan', offering incompatible QoS. No messages will be received from it. Last incompatible policy: RELIABILITY
   ```
   Investigate the warning and fix it using the `qos_profile` provided in the `wall_detector.py` script. You can use Google or ChatGPT to learn more about ROS 2 QoS (Quality of Service).

5. **Run the Line Follower Node** Start the line follower node:
   ```bash
   ros2 run lab8_lidar line_follower
   ```

6. **Demo the Robot** Demonstrate the robot following the walls in a straight path. **Ensure the robot starts 10 cm from the center line as shown below.**

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/yf1iLbKwU3E?si=f9wdUHjrpu-KqkSU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</center>


## 🚚 Deliverables

1. **[20 Points] Complete the `wall_detector.py` Script**
    - Ensure the script is fully functional and implements all required features.
    - Push your code to GitHub and confirm that it has been successfully uploaded.
    **NOTE:** _If the instructor can't find your code in your repository, you will receive a grade of 0 for the coding part._

2. **[15 Points] Complete the `line_follower.py` Script**
    - Ensure the script is fully functional and implements all required features.
    - Push your code to GitHub and confirm that it has been successfully uploaded.
    **NOTE:** _If the instructor can't find your code in your repository, you will receive a grade of 0 for the coding part._

3. **[15 Points] Demonstration**
    - Show the robot successfully move between two walls.