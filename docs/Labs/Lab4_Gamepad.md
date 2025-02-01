# 🔬 Lab4: Gamepad


## 📌 Objectives

- Students should be able to use ROS topics and messages in Python nodes to facilitate communication between publishers and subscribers.
- Students should be able to develop a ROS2 node that subscribes to joystick input and publishes commands to control a robot.
- Students should be able to integrate gamepad input to control a TurtleBot3 in a simulated environment.


## 📜 Overview  

This lab introduces ROS topics and messages, focusing on how nodes communicate by exchanging information. In ROS, a node can **publish** messages to a topic, while other nodes can **subscribe** to that topic to receive data. Each message follows a predefined format that all participating nodes must understand.  

The best way to learn ROS is through hands-on experience, so let's start by experimenting with a virtual **TurtleBot3**! Throughout this course, TurtleBot3 will serve as a key tool for applying and integrating robotics concepts. By the end of the course, you'll be developing complex embedded robotic systems to complete dedicated tasks, such as autonomously navigating the halls of DFEC. But for now, let’s begin with something simpler—getting the robot to drive around.  

In this lab, you will develop a ROS2 node that listens for joystick input (`/joy` topic) and publishes velocity commands (`/cmd_vel` topic) to control the movement of the robot. 🚀

## 🌱 Pre-Lab: ROS2 Client Libraries  

The tutorials at [Beginner: Client Libraries](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html) are a great introduction to ROS2 client libraries.  

```{image} ./figures/Lab2_ROS_Tutorials2.png  
:width: 800  
:align: center  
```  
<br>  

Complete the following four tutorials. **Important:** Skip **C++** tutorials and complete only the **Python** ones.  

1. **Using `colcon` to Build Packages**  
   - Create a new workspace: `~/ros2_ws` (not `~/master_ws`).  
   - Skip the [Setup `colcon_cd`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#setup-colcon-cd) section and below.  

1. **Creating a Workspace**  
   - Use `~/ros2_ws`, not `~/master_ws`.  
   - Select **Linux** in the operating system tab.  

1. **Creating a Package**  
   - Follow the Python tutorial (not the CMake one).  

1. **Writing a Simple Publisher and Subscriber (Python)**  
   - Select **Linux** in the operating system tab.  

1. **Demo the Terminal Outputs**
    - Demo the terminal outputs at the beginning of the lab.
    - One terminal should display messages every 0.5 seconds:  
        ```bash  
        [INFO] [minimal_publisher]: Publishing: "Hello World: 0"  
        [INFO] [minimal_publisher]: Publishing: "Hello World: 1"  
        [INFO] [minimal_publisher]: Publishing: "Hello World: 2"  
        ```  
    - Another terminal should print messages like:  
        ```bash  
        [INFO] [minimal_subscriber]: I heard: "Hello World: 12"  
        [INFO] [minimal_subscriber]: I heard: "Hello World: 13"  
        [INFO] [minimal_subscriber]: I heard: "Hello World: 14"  
        ```  

## 💻 Lab Procedure

### 🔧 Setup  

1. Follow the [Gamepad Setup](../Appendix/GamepadSetup.md) guide to configure the Logitech Gamepad on your **Master** computer.  

1. Open a terminal and navigate to the `src` directory in your workspace:  
   ```bash  
   $ cd ~/master_ws/src  
   ```  

1. Install the TurtleBot3 Simulation Package:  
   ```bash  
   $ git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git  
   ```  
   - Ensure the `turtlebot3_simulations` directory is inside `~/master_ws/src`.  

1. Build the workspace: 
**Important:** Always run `colcon build` from the root of your workspace (`~/master_ws`).  

   ```bash  
   $ cd ~/master_ws  
   $ colcon build --symlink-install  
   ```  

    After running the command, you should see output similar to the following.
    **Note**: If you notice `stderr` messages or `CMake Warning` messages like the one shown below, **you can safely ignore them**—they do not affect the build process.  

    ```bash  
    Finished <<< turtlebot3_fake_node [17.9s]                                       
    --- stderr: turtlebot3_gazebo                                  
    CMake Warning (dev) at /usr/share/cmake-3.22/Modules/FindPackageHandleStandardArgs.cmake:438 (message):  
    The package name passed to `find_package_handle_standard_args` (PkgConfig)  
    does not match the name of the calling package (gazebo). This can lead to  
    problems in calling code that expects `find_package` result variables  
    (e.g., `_FOUND`) to follow a certain pattern.  
    Call Stack (most recent call first):  
    /usr/share/cmake-3.22/Modules/FindPkgConfig.cmake:99 (find_package_handle_standard_args)  
    /usr/lib/x86_64-linux-gnu/cmake/gazebo/gazebo-config.cmake:72 (include)  
    CMakeLists.txt:23 (find_package)  

    This warning is for project developers. Use `-Wno-dev` to suppress it.  

    ---  
    Finished <<< turtlebot3_gazebo [20.9s]  
    Starting >>> turtlebot3_simulations  
    Finished <<< turtlebot3_simulations [0.90s]  
    ```  

### 🚀 Running the TurtleBot3 Simulation

1. Source Your Workspace: Before launching the simulation, make sure your workspace is properly sourced:  
    ```bash  
    $ source ~/master_ws/install/setup.bash  
    ```  

1. Launch the TurtleBot3 Simulation in Gazebo: A **launch file** is used in ROS to start multiple nodes at once. We will learn more about them later. For now, launch the simulation:  
    ```bash  
    $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  
    ```  

1. Open RViz for Visualization: In a **new terminal window or tab**, run the following command to start RViz:  
    ```bash  
    $ ros2 launch turtlebot3_bringup rviz2.launch.py
    ```  
    These launch files will start the necessary nodes to simulate our robot and will open two windows:  

    - **Gazebo:** A simulation environment that provides physics for the robot, allowing us to test algorithms before deploying them in the real world.  
    - **RViz:** A visualization tool that helps us see how ROS topics interact.  

    In **Gazebo**, you should see a TurtleBot3 positioned at (-2.0, -0.5) facing the positive x-direction inside a maze.  

    🔹 **Camera Controls in Gazebo:**  
    - **Left-click + Drag:** Pan the view  
    - **Scroll Wheel:** Zoom in/out  
    - **Hold Scroll Wheel + Drag:** Rotate the camera  

    In **RViz**, you should see a series of red dots outlining obstacles in the maze. These are detected by the TurtleBot3's LIDAR sensor, which publishes data on the **/scan** topic.  

    🔹 **Camera Controls in RViz:**  
    - **Left-click + Drag:** Rotate the view  
    - **Scroll Wheel:** Zoom in/out  
    - **Hold Scroll Wheel + Drag:** Pan the view  

    (Yes, the controls are slightly different in Gazebo and RViz. 😅)  

    📌 *We will explore these tools in more detail later, so don’t worry if you don’t understand everything yet!*  

1. List Active Nodes in Your ROS Network: To check which nodes are currently running in the system, use:  
    ```bash  
    $ ros2 node list  
    ```  
    You should see several active nodes:  
    - **Two nodes for Gazebo** (simulation environment)  
    - **One node for RViz** (robot visualization)  
    - **A default ROS2 communication node** (automatically created)  
    - **A node for handling RViz visualization**  

1. List Active Topics: Let’s see what topics are currently being published:  
    ```bash  
    $ ros2 topic list  
    ```  
    You will notice several topics related to the simulated robot. Some key ones to remember:  
    - **/cmd_vel** → Controls robot movement (velocity commands)  
    - **/imu** → Provides data from the robot’s orientation sensor  
    - **/scan** → LIDAR sensor data (obstacle detection)  

    The remaining topics handle simulation and visualization and can be ignored for now.  

1. Visualizing ROS Connections Using `rqt_graph`: A useful tool for understanding how nodes and topics are connected is **rqt_graph**. Run:  
    ```bash  
    $ rqt_graph  
    ```  
    This will generate a visual representation of the active nodes and their connections.  

    Right now, **Gazebo is publishing position and scan data**, which RViz uses to display the robot.  

    📌 **Close rqt_graph when finished**. Now, let’s add another node to make things more interesting! 🚀  

1. Sending Velocity Commands to the Robot: The **/cmd_vel** topic is used to send movement commands to the robot. These commands use **Twist** messages, which define movement in **linear (x, y, z) and angular (x, y, z) directions**. (Google `ROS twist message` for more information)  

    Our TurtleBot3 moves in **two dimensions**, so we will only use:  
    - **Linear x** → Moves the robot forward/backward  
    - **Angular z** → Rotates the robot left/right  

    To drive our simulated robot, we need a node that can publish *Twist* messages to the **/cmd_vel** topic. Instead of writing our own publisher, we can use an existing ROS2 node, `Teleop_Twist_Keyboard` that sends `Twist` messages using a keyboard.

    Open a **new terminal tab** (`Ctrl + Shift + T`), then run:  
    ```bash  
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard.py  
    ```  

    This launches a keyboard teleoperation node that lets you control the robot!  

    📌 **To drive the robot:**  
    - Press **x** to decrease the linear speed to ~0.25 m/s.  
    - Press **c** to decrease the angular speed to ~0.5 rad/s.  
    - Follow the instructions to move the robot using your keyboard!  

1. Checking ROS Graph After Movement: Once the robot moves, let's see how our ROS network has changed:  
    ```bash  
    $ rqt_graph  
    ```  

    Now, you should see a new node, **teleop_twist_keyboard**, which is publishing **Twist** messages to the **/cmd_vel** topic, allowing the robot to move in Gazebo.  

    📌 **Close rqt_graph when finished**.  

1. Recap  
    - You launched the TurtleBot3 simulation in Gazebo.  
    - You used RViz to visualize sensor data.  
    - You explored active ROS2 nodes and topics.  
    - You used `rqt_graph` to visualize connections in the ROS network.  
    - You controlled the TurtleBot3 using keyboard teleoperation.  

    🚀 *You’re now ready to move on to the next part of the lab!*  


### 📦 Create a New ROS2 Package  

1. Navigate to your local repository inside your workspace:  
   ```bash  
   $ cd ~/master_ws/src/ece387_lastname  
   ```  
1. Create a new ROS2 package: Run the following command to create a new ROS 2 package named `lab4_gamepad`:  

    ```bash  
    $ ros2 pkg create --build-type ament_python lab4_gamepad  
    ```  

    After running this command, your terminal will confirm the package creation and generate the necessary files and folders:  

    ```bash  
    lab4_gamepad/  
        lab4_gamepad/  
        resource/lab4_gamepad  
        package.xml  
        setup.cfg  
        setup.py  
    ```  


### 📝 Write the ROS2 Node  

1. Navigate to the package directory:  
    ```bash  
    $ cd lab4_gamepad/lab4_gamepad  
    ```  
    **Note**: You need to enter the `lab4_gamepad` directory inside the `lab4_gamepad` package folder.

1. Create a Python script for the node:  
    ```bash  
    $ touch gamepad.py  
    $ chmod +x gamepad.py  
    ```  
    The `chmod +x` command makes the `gamepad.py` file executable so it can be run as a script.

1. Open the `gamepad.py` file in VS Code and implement the **TODO** sections in the provided template.  

    ```{hint}  
    Use `ros2 topic info <topic_name>` to inspect topics and `ros2 interface show <message_name>` to see message structures.  
    ```  

    ```python
    #!/usr/bin/env python3
    # The above line is a shebang, which tells the system to run this script using Python 3.

    # Import necessary ROS 2 libraries
    import rclpy  # ROS 2 client library for Python
    from rclpy.node import Node  # Base class for creating ROS 2 nodes

    # Import message types
    from sensor_msgs.msg import Joy  # Message type for joystick (gamepad) inputs
    from geometry_msgs.msg import Twist  # Message type for velocity commands

    class Gamepad(Node):
        """
        A ROS 2 Node that converts joystick (gamepad) inputs into velocity commands
        for a robot. It subscribes to the 'joy' topic and publishes Twist messages
        to the 'cmd_vel' topic.
        """
        
        def __init__(self):
            """
            Constructor: Initializes the gamepad node.
            - Subscribes to the 'joy' topic to receive joystick inputs.
            - Publishes to the 'cmd_vel' topic to send velocity commands.
            """
            # TODO: Initialize the node with the name 'gamepad'


            # TODO: Create a subscriber to the 'joy' topic (gamepad inputs)
            # - This listens for messages of type Joy.
            # - It calls the `joy_callback` function whenever a new message arrives.
            # - Queue size of 10 will buffer up to 10 messages before discarding old ones.


            # TODO: Create a publisher to send velocity commands to the 'cmd_vel' topic.
            # - This sends messages of type Twist.
            # - Queue size of 10 helps manage message buffering.


            # Log a message indicating that the node has started successfully
            self.get_logger().info("Joy to cmd_vel node started!")

        def joy_callback(self, msg):
            """
            Callback function that processes incoming joystick messages.
            - Extracts axis values from the joystick message.
            - Converts these values into a Twist message (velocity commands).
            - Publishes the Twist message to control the robot.

            Args:
                msg (Joy): The incoming joystick message containing axes and button states.
            """
            # TODO: Create a new Twist message for velocity commands.


            # TODO: Map joystick axes to robot velocity:
            # - The left stick (up/down) controls linear speed (forward/backward).
            # - The right stick (left/right) controls angular speed (rotation).
            # Ensure that the joystick axis values are properly scaled to match 
            # the robot's velocity limits:
            # - Max Linear Velocity: 0.22 m/s
            # - Max Angular Velocity: 2.84 rad/s


            # TODO: Publish the velocity command to the '/cmd_vel' topic.



    def main(args=None):
        """
        Main function to start the ROS 2 node.
        - Initializes the ROS 2 system.
        - Creates an instance of the Gamepad node.
        - Keeps the node running using `rclpy.spin()`, which listens for messages.
        - Cleans up resources when the node is shut down.
        """
        rclpy.init(args=args)  # Initialize ROS 2
        gamepad = Gamepad()  # Create an instance of the Gamepad node
        rclpy.spin(gamepad)  # Keep the node running and responsive to joystick input

        # Cleanup when the node is shutting down
        gamepad.destroy_node()  # Properly destroy the node
        rclpy.shutdown()  # Shutdown ROS 2

    # Run the script if executed directly (not imported as a module)
    if __name__ == '__main__':
        main()

    ```

### 🛠 Update `setup.py`  

1. Open the `setup.py` file in your preferred text editor:  
    ```bash  
    $ gedit ~/master_ws/src/ece387_lastname/lab4_gamepad/setup.py  
    ```  

1. Locate the `entry_points` section and modify it to include the `gamepad` node:  
    ```python  
    entry_points={  
        'console_scripts': [  
            'gamepad = lab4_gamepad.gamepad:main',  # Maps the gamepad script to the ROS2 command-line tool
        ],  
    },  
    ```  

1. Save the changes and exit the editor.  


### 🔨 Build the Workspace  

1. Navigate to the root of your workspace:
    ```bash  
    $ cd ~/master_ws  
    ```  

1. Build the workspace:
    ```bash  
    $ colcon build --symlink-install  
    ```  
    - This compiles the package and ensures all dependencies are correctly linked.  
    - If you encounter any errors, check the terminal output for missing dependencies or syntax errors.  

1. Source the package:
    ```bash  
    $ source install/setup.bash  
    ```  
    - This step ensures that the newly built package is recognized in your ROS2 environment.  

### 🚀 Run and Test the Nodes  

1. Start the `joy` node to publish gamepad data:  
   ```bash  
   ros2 run joy joy_node  
   ```  
   - This node reads input from the gamepad and publishes joystick data to the **/joy** topic.  

1. Start your `gamepad` node:  
   ```bash  
   ros2 run lab4_gamepad gamepad  
   ```  
   - This node subscribes to the **/joy** topic and publishes velocity commands to **/cmd_vel**.  

1. Visualize ROS node communication using `rqt_graph`:  
   ```bash  
   ros2 run rqt_graph rqt_graph  
   ```  
   or simply:  
   ```bash  
   rqt_graph  
   ```  

   - This tool provides a graphical representation of the active nodes and their topic connections.  
   - The `joy_node` should be publishing to **/joy**, and your `gamepad` node should be subscribing to it while publishing commands to **/cmd_vel**.  

   **🔹 Note:** You may need to hit the **refresh** button in `rqt_graph` a few times for the updated statistics to appear.  


### 📊 Monitor ROS2 System Using `rqt`  

1. Launch `rqt_gui`:  
   ```bash  
   ros2 run rqt_gui rqt_gui  
   ```  
   or simply:  
   ```bash  
   rqt  
   ```  

1. Enable the Topic Monitor: 
   - If the topic monitor window is not displayed, go to:  
     **`Plugins` → `Topics` → `Topic Monitor`**  
   - Once the topic monitor loads, topics are not monitored by default. Check the box next to each topic to enable monitoring.  
   - To view more details about a topic, click the **▶** icon next to the checkbox.  

1. Monitor the `cmd_vel` and `joy` topics:  
   - Right-click the title bar and select **"Take Screenshot."**  
   - **Submit the screenshot on Gradescope.**  

---

### 📝 **Understanding `rqt` vs. `rqt_graph`**  

> 🔹 `rqt` is a **graphical toolset** that provides a user-friendly interface for monitoring, debugging, and visualizing various aspects of the ROS2 system. It acts as a plugin framework, allowing you to load different tools as needed, such as topic monitors, node viewers, and service callers.  

> 🔹 `rqt_graph` is a **specific plugin** within `rqt`. It **visualizes the ROS graph**, showing all running nodes and their topic connections. It helps you understand **which nodes are publishing and subscribing**, but it does **not** display ROS services.  

📌 **Summary:**  
- **`rqt`** is the **main toolset** with multiple monitoring plugins.  
- **`rqt_graph`** is just **one plugin** within `rqt` that visualizes topic connections.  


### 🎮 Control the TurtleBot3  
- Use the **left stick** to move forward/backward.  
- Use the **right stick** to rotate left/right.  

**Note:** If controls don’t respond correctly, use:  
```bash  
ros2 topic echo /joy  
```  
This will display joystick data, allowing you to adjust mappings in `joy_callback()`.  

---

## 📌 **Deliverables**  

1. **Complete all TODO sections** in `gamepad.py`.  
1. **Demo joystick control** over the TurtleBot3 in simulation.  
1. **Push your code** to your GitHub repository.  
1. **Submit your assignment** on Gradescope.  


