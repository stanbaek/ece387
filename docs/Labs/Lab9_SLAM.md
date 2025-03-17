# 🔬 Lab9: SLAM


## 📌 Objectives

- Students should be able to implement a ROS2 node to detect walls using LiDAR data.

## 📜 Overview

In this project, we will enable our robot to autonomously navigate an unknown maze and build a map of the environment. As in the previous lab, we will use LiDAR to detect the walls of the maze (or obstacles) surrounding the robot. We will be utilizing the Simultaneous Localization and Mapping (SLAM) library provided by ROS2 and TurtleBot3.

SLAM, or Simultaneous Localization and Mapping, is a process used in robotics to enable a robot to build a map of an unknown environment while simultaneously determining its location within that map. It involves combining sensor data, algorithms, and probabilistic methods to perform real-time mapping and localization. SLAM is crucial for autonomous robots to operate effectively in environments where pre-existing maps are not available.

SLAM is one of the fundamental algorithms in robotics and is widely used in applications such as autonomous vehicles, drone navigation, and robotic vacuum cleaners. It enables robots to navigate dynamic and unfamiliar environments without relying on GPS or pre-defined maps, which is essential for many real-world scenarios.

SLAM integrates data from sensors like LiDAR and odometry to construct and update a map while estimating the robot's position. Through statistical methods like Kalman Filters or Particle Filters, SLAM corrects errors in localization and mapping to achieve accurate results. While the underlying mathematics involves advanced topics in statistics and optimization, libraries provided in ROS2 simplify SLAM's implementation, making it accessible for practical applications.

We will use Cartographer in this lab because it provides an efficient and accurate SLAM solution for 2D environments like the maze we’ll be mapping. Its ability to handle LiDAR data and update maps in real time makes it ideal for this project. Furthermore, its compatibility with TurtleBot3 and ROS2 simplifies the setup, allowing us to focus on understanding the SLAM process and its applications.

## 🌱 Pre-Lab: ROS2 Client Libraries  

The [ROS2 Intermediate Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate.html) on `actions`  are a great starting point for learning about ROS2 action servers and clients.  

```{image} ./figures/Lab9_ROS2_ActionTutorials.png  
:width: 800  
:align: center  
```  
<br>  

Complete the following three tutorials. **Important:** Skip **C++** tutorials and focus only on the **Python** tutorials.  

1. **Managing Dependencies with rosdep**  
   - No need to install anything - your computer already has all the required packages set up.

1. **Creating an action**  
   - Make sure you’re working in the `ros2_ws` workspace. Avoid using the `master_ws` workspace for this one. 

1. **Writing an action server and client (Python)**  
   - As instructed at the end of this tutorial, run the action client.  When the feedback appears on the screen, capture a screenshot and upload it to Gradescope.


## 🛠️ Lab Procedures

### **Setting Up TurtleBot3 with SLAM in Gazebo**

Follow these steps to simulate SLAM with TurtleBot3 in the Gazebo environment.

1. Download the [`maze Gazebo files`](../files/maze.tar.xz). Extract the files and place them in the appropriate directories within `~/master_ws/src/turtlebot3_simulations/turtlebot3_gazebo`. Make sure to merge the new directories with the existing ones.

1. Run the following command to start the Gazebo simulation with the maze world:

    ```bash
    ros2 launch turtlebot3_gazebo maze.launch.py
    ```

    This will launch the Gazebo environment with the maze, as shown below:

    ```{image} ./figures/Proj1_GazeboInit.png
    :width: 400  
    :align: center  
    ```  

1. Open a new terminal and start the Cartographer SLAM process:

    ```bash
    ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
    ```

    Cartographer will begin building a map as you move the robot. The initial map will look like this:

    ```{image} ./figures/Proj1_CartographerInit.png
    :width: 400  
    :align: center  
    ```  

1. Use a gamepad to manually control the robot and explore the maze. Run the following command to start the gamepad controller:

    ```bash
    ros2 launch lab4_gamepad gamepad.launch.py
    ```

    - Black pixels represent obstacles (walls).
    - Gray pixels indicate noise or uncertainty. As you complete multiple laps, the uncertainty decreases, and light gray pixels become darker.

    Once the map is complete, it should look like this:

    ```{image} ./figures/Proj1_CartographerDone.png
    :width: 400  
    :align: center  
    ```  

1. After mapping the maze, save the map using the following command:

    ```bash
    ros2 run nav2_map_server map_saver_cli -f ~/map
    ```

1. Download [`map_plotter.py`](../files/map_plotter.py) to your home directory. Then, make the script executable:

   ```bash
   chmod +x map_plotter.py
   ```

1. Verify the file permissions using `ls -l`.

   ```{important}
   If asked about the command to make a file executable only for the owner, you should know the answer for your GR. 😉
   ```

1. Complete the `TODO` section in `map_plotter.py`, and then run the script to generate the map.
1. Run the script to plot the map:

   ```bash
   ./map_plotter.py
   ```

1. Verify that the map dimensions match the actual maze. Each wall piece is **0.18 meters** long.

### **Autonomous Navigation with SLAM**

Now, let’s set up **autonomous SLAM** using **Cartographer** and **Navigation2** to explore the environment dynamically.

1. Start the TurtleBot3 simulation in the Gazebo world:

    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

    The Gazebo environment will look like this:

    ```{image} ./figures/Lab9_GazeboWorld.png
    :width: 400  
    :align: center  
    ```  

1. Run Cartographer to begin building the map in real-time:

    ```bash
    ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
    ```

    The initial map will appear as shown below:

    ```{image} ./figures/Lab9_CartographerInit.png
    :width: 400  
    :align: center  
    ```  

1. Run Navigation2 alongside Cartographer to allow the robot to navigate autonomously:

    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
    ```

    The RViz2 interface will look like this:

    ```{image} ./figures/Lab9_Nav2WorldInit.png
    :width: 400  
    :align: center  
    ```  

1. Use the **2D Pose Estimate** tool in RViz2 to set the robot’s initial position:
   - Click on the map where the robot is located.
   - Drag the green arrow to match the robot’s orientation.

1. Use the **2D Nav Goal** tool to set navigation targets. The robot will autonomously explore the maze, updating the map dynamically as it moves.

    As the robot explores, the map will evolve, as shown below:

    ```{image} ./figures/Lab9_Nav2WorldInProgress.png
    :width: 400  
    :align: center  
    ```  

    The Gazebo simulation will reflect the robot’s progress:

    ```{image} ./figures/Lab9_GazeboWorldInProgress.png
    :width: 400  
    :align: center  
    ```  

1. Explore the entire world and create a map. Ensure you have dark gray obstacles.

1. Take a screenshot of the cartographer window by right clicking the tileboar.  Submit the screenshot on Gradescope.

## More to come soon


```bash
$ ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$HOME/map.yaml 

$ ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true -p yaml_filename:=$HOME/map.yaml
```

```bash
$ ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```





<!--
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

-->