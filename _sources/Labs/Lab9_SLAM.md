# 🔬 Lab9: SLAM


## 📌 Objectives

- Students should be able to implement a ROS2 node to detect walls using LiDAR data.
- This lab focuses on simulating autonomous exploration and navigation using SLAM in both Gazebo and real-world TurtleBot3 environments. You'll work with a prebuilt map and create new maps using Cartographer.


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
    The use_sim_time:=true parameter ensures proper synchronization with the simulation clock.  Cartographer will begin building a map as you move the robot. The initial map will look like this:

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

1. Confirm that two files, `map.yaml` and `map.pgm`, have been created in your home directory. Then view the contents of `map.yaml` using the `cat` command:

    ```bash
    cat map.yaml
    ```
    Example output:
    ```bash
    image: map.pgm
    mode: trinary
    resolution: 0.05
    origin: [-0.5, -0.7, 0]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.25
    ```
    - resolution: Each pixel represents 0.05 m x 0.05 m in the real world.
    - origin:  [-0.5, -0.7, 0] means the map starts at (-0.5 m, -0.7 m, 0 m) in the world frame
    - occupied_thresh: The occupancy grid values greater than 65 are considered "occupied"
    - free_thresh: The occupancy grid values less than 25 are considered "free space"
    

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

### **Navigation with SLAM**

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

1. Take a screenshot of the cartographer window by right clicking the title bar.  Submit the screenshot on Gradescope.


### **Autonomous Exploration with SLAM in Gazebo**

1. Create a package named `lab9_slam` with the `BSD-3-Clause` license and dependencies:
    - `rclpy`
    - `geometry_msgs`
    - `nav2_msgs`
    - `action_msgs`
    - `numpy`

    _Hint: There’s a way to include all dependencies at the time of package creation._

1. Download the [`explore_maze.py`](../files/explore_maze.py) script and save it in the appropriate folder within your package (You should know where this file should go by now).

1. Update the `setup.py` file by correctly adding the entry point for `explore_maze.py`. This is necessary to ensure that the script runs as a node.

1. Open the `explore_maze.py` script and fill in the `TODO` sections. Pay attention to:
    - Setting the target pose for the robot.
    - Utilizing the Nav2 action server/client.

1. Ensure you build the package correctly to make the script executable.

1. Start the TurtleBot3 simulation in Gazebo.
   ```bash
   ros2 launch turtlebot3_gazebo maze.launch.py
   ```

1. Launch Cartographer to enable SLAM
   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
   ```

1. Start Navigation2 for path planning and exploration.
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
   ```

1. Finally, run the `explore_maze.py` script to let the robot autonomously explore the maze, building and updating a dynamic map.

   ```bash
   ros2 run lab9_slam explore
   ```

### **Autonomous Navigation with Prebuilt Map in Gazebo**

Follow these steps to simulate autonomous navigation with **prebuilt map** in the Gazebo environment.

1. Launch the TurtleBot3 simulation in Gazebo.
   ```bash
   ros2 launch turtlebot3_gazebo maze.launch.py
   ```

1. For this part of the lab, we are working with a prebuilt map. Cartographer, which is used for real-time map creation, is not required here.

1. Start the AMCL (Adaptive Monte Carlo Localization) node to localize the robot within the prebuilt map (map.yaml).
    ```bash
    ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true -p yaml_filename:=$HOME/map.yaml
    ```

1. Publish a Static Transform Between Frames:
    ```bash
    $ ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
    ```
    This command establishes a static relationship between the `map` and `odom` frames, assuming they are aligned without any offset. It's a prerequisite for linking the global (map) frame to the local (odom) frame in a robot's TF (Transform) tree.
    
1. Start the Navigation2 stack with the prebuilt map (`map.yaml`). Confirm that the robot can load and utilize the map effectively.

1. **Run the SLAM Node**
   - Execute the SLAM exploration node. Observe how the robot navigates using the prebuilt map.
    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$HOME/map.yaml 
    ```

1. Run the SLAM exploration node:
    ```bash
    ros2 run lab9_slam explore
    ```

1. **Reflect on Differences**: Compare the robot's performance with a prebuilt map to its performance when generating a map in real-time. Note any improvements or challenges.






<!--
---

### **Part 3: Autonomous Exploration with Cartographer (TurtleBot3, Real Environment)**

1. **Set Up the TurtleBot3**
   - Start the `bringup` process to initialize the robot in a real environment.

2. **Run SLAM**
   - Launch Cartographer to perform SLAM in real time and create a map.

3. **Launch Navigation2**
   - Start the Navigation2 stack to enable autonomous exploration and navigation.

4. **Run the SLAM Node**
   - Execute the SLAM exploration node and observe the robot autonomously explore its surroundings while dynamically updating its map.

---

### **Part 4: Autonomous Navigation with Prebuilt Map (TurtleBot3, Real Environment)**

1. **Set Up the TurtleBot3**
   - Start the `bringup` process for the real TurtleBot3 environment.

2. **Set Up Localization**
   - Use AMCL and the prebuilt map from Part 3 for localization.
   - Set a static transform between `map` and `odom`.

3. **Launch Navigation2**
   - Start the Navigation2 stack with the prebuilt map, ensuring the robot can locate itself within the map.

4. **Run the SLAM Node**
   - Execute the SLAM exploration node. Observe and evaluate how the robot navigates with a prebuilt map.

5. **Analyze Performance**
   - Discuss and note differences in navigation performance between using a real-time generated map and a prebuilt map.

---

### **Deliverables**
1. **Wall Detector Script (20 Points):**
   - Complete the `wall_detector.py` script.
   - Push it to GitHub. _Code not found in your repository will receive a score of 0._

2. **Line Follower Script (15 Points):**
   - Complete the `line_follower.py` script.
   - Push it to GitHub. _Code not found in your repository will receive a score of 0._

3. **Demonstration (15 Points):**
   - Show a working demonstration of the robot successfully navigating between walls.

---

Does this version make it easier for your students to follow? Let me know if you’d like further refinements!








































   

### **Autonomous Navigation with Prebuilt Map in Gazebo**



1. Start the TurtleBot3 simulation in the Gazebo world:
   ```bash
   ros2 launch turtlebot3_gazebo maze.launch.py
   ```

1. Do not run Cartographer. Instead we want to run Navigation2 with the map we already built earlier in this lab. Before starting Navigation2, we need to run the following nodes:

    ```bash

    It starts the AMCL (Adaptive Monte Carlo Localization) node, which is responsible for localizing a robot within a known map using laser scan data and particle filters.

    ```bash
    $ ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
    ```
    This command establishes a static relationship between the `map` and `odom` frames, assuming they are aligned without any offset. It's a prerequisite for linking the global (map) frame to the local (odom) frame in a robot's TF (Transform) tree.

1. Now start Navigation2:
    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$HOME/map.yaml 
    ```
    You should be able to find the prebuilt map. 
    
1. Run the SLAM exploration node:
    ```bash
    ros2 run lab9_slam explore
    ```

1. Can you find any differences in performance when you use a prebuilt map compared to using a map generated in real-time by Cartographer? 


### **Autonomous Exploration with Cartographer with Turtlebot3**

Follow these steps to run autonomous navigation with **Turtlebot3** in real-time *without* the Gazebo environment.

1. Start `bringup`

1. Start Cartographer:
   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py
   ```

1. Start Navigation2:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py
   ```
1. Run the SLAM exploration node:
   ```bash
   ros2 run lab9_slam explore
   ```

### **Autonomous Exploration with Prebuilt Map with Turtlebot3**

Follow these steps to run autonomous navigation with **Turtlebot3** in real-time *without* the Gazebo environment.

1. Start `bringup`

1. Do not run Cartographer. Instead we want to run Navigation2 with the map we already built earlier in this lab. Before starting Navigation2, we need to run the following nodes

    ```bash
    ros2 run nav2_amcl amcl --ros-args -p yaml_filename:=$HOME/map.yaml
    ```
    and
    ```bash
    $ ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
    ```

1. Now start Navigation2:
    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml 
    ```
    You should be able to find the prebuilt map. 
    
1. Run the SLAM exploration node:
    ```bash
    ros2 run lab9_slam explore
    ```

1. Can you find any differences in performance when you use a prebuilt map compared to using a map generated in real-time by Cartographer? 


1. Start Navigation2:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py
   ```
1. Run the SLAM exploration node:
   ```bash
   ros2 run lab9_slam explore
   ```

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


<!--
To determine the origins of the `map` and `odom` frames with respect to the world frame, you'll need to analyze the relationships in your robot's **TF (Transform) tree**. Here’s how you can do it:

---

### 1. **Visualize the TF Tree**
Use the TF visualization tools to observe the transform hierarchy:
   - Run the following in a terminal to visualize the TF tree:
     ```bash
     ros2 run tf2_tools view_frames
     ```
     This will generate a PDF (`frames.pdf`) showing the entire TF tree, including the origins of frames like `map` and `odom`.

   - Alternatively, in RViz2, add the **TF display** plugin to view the transform frames in real time and see how `map` and `odom` are positioned relative to each other and other frames.

---

### 2. **Query Specific Transforms**
To find the exact transform between `map` and `odom` (or any other frames), use the `tf2_echo` command:
   ```bash
   ros2 run tf2_ros tf2_echo map odom
   ```
   This will display the translation (x, y, z) and rotation (quaternion) values of the `odom` frame relative to the `map` frame. If you're looking for the relationship with a world frame, replace `map` or `odom` with the name of your world frame.

   - **Translation:** Gives you the position of the origin (in meters).
   - **Rotation:** Indicates orientation as a quaternion, which you can convert to roll, pitch, and yaw if needed.

---

### 3. **Check Static Transforms**
If static transforms are being used (like in your earlier command), the transformation values might already be predefined. You can inspect the parameters or commands to see the exact translation and rotation settings used when broadcasting those frames.

---

### 4. **Understand Frame Association**
- In most robot setups, the `map` frame is typically aligned with the global reference frame, representing a fixed coordinate system for localization.
- The `odom` frame, on the other hand, is aligned with the odometry system, which can drift over time. The transform between `map` and `odom` is often determined by the robot's localization system (e.g., AMCL, SLAM).

---

Let me know if you'd like guidance on any of these steps! I can also assist with debugging transforms or interpreting the output from `tf2_echo`.
-->