# 🔬 Lab4: Gamepad


## 📌 Objectives

- Students should be able to use ROS topics and messages in Python nodes to facilitate communication between publishers and subscribers.
- Students should be able to develop a ROS2 node that subscribes to joystick input and publishes commands to control a robot.
- Students should be able to integrate gamepad input to control a TurtleBot3 in a simulated environment.


## 📜 Overview  
This lab provides insight into ROS topics and messages and how information is exchanged between nodes. A node can **publish** messages over a topic, and other nodes can **subscribe** to that topic to receive messages. Each message follows a predefined format, which all participating nodes must understand.  

In this lab, you will develop a node that subscribes to joystick input (`/joy` topic) and publishes velocity commands (`/cmd_vel` topic) to control a robot.  

## 💻 Procedure

### 🔧 Pre-Lab: ROS2 Client Libraries  
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

1. **Demo the Terminal Outputs to Your Instructor**  
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



## 🔧 Setup  

1. Follow the [Gamepad Setup](../Appendix/GamepadSetup.md) guide to configure the Logitech Gamepad on your **Master** computer.  

1. Open a terminal and navigate to the `src` directory in your workspace:  
   ```bash  
   cd ~/master_ws/src  
   ```  

1. Install the TurtleBot3 Simulation Package:  
   ```bash  
   git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git  
   ```  
   - Ensure the `turtlebot3_simulations` directory is inside `~/master_ws/src`.  

1. Build the workspace: 
**Important:** Always run `colcon build` from the root of your workspace (`~/master_ws`).  

   ```bash  
   cd ~/master_ws  
   colcon build --symlink-install  
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


## 📦 Create a New ROS2 Package  

1. Navigate to your local repository inside your workspace:  
   ```bash  
   cd ~/master_ws/src/ece387_lastname  
   ```  
1. Create a New ROS2 Package: Run the following command to create a new ROS 2 package named `lab4_gamepad`:  

    ```bash  
    ros2 pkg create --build-type ament_python lab4_gamepad  
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


## 📝 Write the ROS2 Node  

1. Navigate to the package directory:  
    ```bash  
    cd lab4_gamepad/lab4_gamepad  
    ```  
    **Note**: You need to enter the `lab4_gamepad` directory inside the `lab4_gamepad` package folder.

1. Create a Python script for the node:  
   ```bash  
   touch gamepad.py  
   chmod +x gamepad.py  
   ```  
    The `chmod +x` command makes the `gamepad.py` file executable so it can be run as a script.

1. Open the `gamepad.py` file in VS Code and implement the **TODO** sections in the provided template.  

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
            # TODO: Create a new Twist message (velocity command)

            # TODO: Map joystick axes to robot velocity:
            # The left stick up/down controls linear speed (forward/backward)
            # The right stick left/right controls angular speed (rotation)


            # TODO: Publish the velocity command to the 'cmd_vel' topic


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

```{hint}  
Use `ros2 topic info <topic_name>` to inspect topics and `ros2 interface show <message_name>` to see message structures.  
```  



## 🛠 Update `setup.py`  

1. Open the `setup.py` file and modify the `entry_points` section:  
   ```python  
   entry_points={  
       'console_scripts': [  
           'gamepad = lab4_gamepad.gamepad:main',  
       ],  
   },  
   ```  

## 🔨 Build the Workspace  

1. Navigate to the workspace root:  
   ```bash  
   cd ~/master_ws  
   ```  

2. Build the workspace:  
   ```bash  
   colcon build --symlink-install  
   ```  

## 🚀 Run the Node  

1. Source the workspace:  
   ```bash  
   source ~/master_ws/install/setup.bash  
   ```  

2. Launch the TurtleBot3 simulation:  
   ```bash  
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  
   ```  

3. Run the `joy` node to publish gamepad data:  
   ```bash  
   ros2 run joy joy_node  
   ```  

4. Start your `gamepad` node:  
   ```bash  
   ros2 run lab4_gamepad gamepad  
   ```  


## 🎮 Control the TurtleBot3  
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
2. **Verify joystick control** over the TurtleBot3 in simulation.  
3. **Push your code** to your GitHub repository.  
4. **Submit your assignment** on Gradescope.  


