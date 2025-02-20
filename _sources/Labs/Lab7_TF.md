# 🔬 Lab7: Transformation

## 📌 Objectives
- Students should be able to explain the concept of coordinate transformation between global and local frames.
- Students should be able to implement a method to reset local coordinates without restarting the TurtleBot3.
- Students should be able to apply the rotational transformation matrix to convert coordinates between different frames.
- Students should be able to utilize IMU and odometry sensor data to determine the robot’s pose.
- Students should be able to develop and test a ROS2 service to reset the TurtleBot3’s pose programmatically.
- Students should be able to debug and troubleshoot coordinate transformation issues in a real-world robotic system.


## 📜 Overview

In this lab, we will learn how to transform coordinates from the global coordinate system to the local coordinate system. When you power on the TurtleBot3, its initial position and orientation (pose) are automatically set to zero. However, if you want to start a new task after completing a previous one, the initial pose will no longer be zero.

To reset the pose, you could power-cycle the robot or press the reset button on the OpenCR board, but this also requires restarting `bringup`, which is not always convenient. Instead, we will implement a method to reset the local coordinates without having to restart the board.

```{image} ./figures/Lab7_CoordTransformation.png
:width: 300
:align: center
```

As discussed in the lecture, a coordinate transformation allows us to express a point’s position in one frame relative to another. The rotational matrix that rotates Frame {B} with respect to Frame {A} by an angle $\theta$ in a 2D space is given by:

$${}^{A}R_B = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} $$

Using this transformation, a position $\mathbf{p}$ in the figure above can be represented in both Frame {A} (the global frame) and Frame {B} (the local frame). The coordinate transformation equation is:

$$\begin{bmatrix} {}^{A}x \\ {}^{A}y \end{bmatrix} = {}^{A}R_B  \begin{bmatrix} {}^{B}x \\ {}^{B}y \end{bmatrix} - \begin{bmatrix} x \\ y \end{bmatrix} $$

where:
- $[{}^{A}x \quad {}^{A}y]^\top$ represents the position in the global frame, measured by the TurtleBot's IMU and odometry sensors.
- $[{}^{B}x \quad {}^{B}y]^\top$ represents the position in the local frame, which we aim to determine.
- $\theta$ is the orientation of Frame {B} relative to Frame {A}.
- $[x \quad y]^\top$ represents the displacement between Frame {A} and Frame {B}.

To find the local coordinates $[{}^{B}x \quad {}^{B}y]^\top$, we rearrange the equation:

$$\begin{bmatrix} {}^{B}x \\ {}^{B}y \end{bmatrix} = R^{-1} \left(\begin{bmatrix} x \\ y \end{bmatrix} - \begin{bmatrix} {}^{A}x \\ {}^{A}y \end{bmatrix}\right)$$

This transformation allows us to reset the local coordinates programmatically rather than manually restarting the TurtleBot3, making our system more efficient and flexible.


## 🌱 Pre-Lab: Testing the IMU  

### Non-Interactive SSH Sessions

Sometimes, we need to execute commands remotely without logging into the remote host via SSH. Here's how you can run commands remotely from your local computer:

1. Run the following command on your master computer:
    ```bash
    $ ssh pi@robotX 'ls -la'
    ```
    Here, X is the number assigned to your robot. This command will display the files and directories in the `pi` user's home directory, allowing us to execute commands on the remote computer without actually logging into it.

2. Try running this command next:
    ```bash
    $ ssh pi@robotX 'echo $ROS_DOMAIN_ID'
    ```
    You may notice that it returns an empty string, even though we have already defined `ROS_DOMAIN_ID` inside the `.bashrc` file. To verify it’s defined correctly, run:
    ```bash
    $ ssh pi@robotX 'cat ~/.bashrc'
    ```
    Look for the line:
    ```sh 
    export ROS_DOMAIN_ID=99
    ```
    It confirms that `ROS_DOMAIN_ID` is defined correctly in the `.bashrc` file.

3. At the beginning of the `.bashrc` file, you will find the following lines:
    ```sh
    # If not running interactively, don't do anything
    case $- in
        *i*) ;;
          *) return;;
    esac
    ```
    This code prevents the `.bashrc` file from executing if the shell is not in interactive mode. Since our `ssh` command runs in non-interactive mode, this code skips the rest of the `.bashrc` file. When you log into the host computer via SSH, the shell is in interactive mode, and all commands are executed normally.

4. Open Visual Studio Code (`VS Code`) and click on the `Extensions` icon on the left sidebar or press `Ctrl+Shift+X`. Type `remote` and select `Remote-SSH`. Install it if it’s not already installed.

5. Open the Command Palette by pressing `Ctrl+Shift+P` or navigating to `View > Command Palette`. Type the first few letters of `Remote-SSH` and select `Remote-SSH: Connect to Host`. Enter `pi@robotX` and hit Enter. Type the password if prompted, though it should be password-free if you’ve set up an SSH key. It may take a couple of minutes to set up the `vscode server` on the remote computer for the first time.

6. Click `Explorer` on the left sidebar, choose `/home/pi` for the `Open Folder` option, and hit OK. You should now see all the files in the home directory. Click on `.bashrc` to open it. Replace the following lines:
    ```sh
    # If not running interactively, don't do anything
    case $- in
        *i*) ;;
          *) return;;
    esac
    ```
    with:
    ```bash
    # If not running interactively, set the following environment variables
    if [[ $- != *i* ]]; then
        source /opt/ros/humble/setup.bash
        source ~/robot_ws/install/setup.bash
        export TURTLEBOT3_MODEL=burger
        export ROS_DOMAIN_ID=98     # TURTLEBOT3
        export LDS_MODEL=LDS-02
        return
    fi
    ```
    Ensure that `ROS_DOMAIN_ID=98` is set with your domain ID. Save the file.

7. On a terminal, try:
    ```bash
    $ ssh pi@robotX 'echo $ROS_DOMAIN_ID'
    ```
    It should now return your domain ID.

### Running `robot.launch.py` without SSH into the Robot

Previously, to drive a physical TurtleBot3, we had to log into the remote host using SSH and run `ros2 launch turtlebot3_bringup robot.launch.py`. Here, we will implement a more convenient method to accomplish the same task without logging into the remote computer:

1. On a terminal, run:
    ```bash
    $ ssh pi@robotX 'ros2 launch turtlebot3_bringup robot.launch.py'
    ```
    This command remotely starts the `robot.launch.py` script on the TurtleBot3.

1. To simplify this process, add the following alias to the `.bashrc` file on your `master` computer:
    ```bash
    alias bringup='ssh pi@robotX '\''ros2 launch turtlebot3_bringup robot.launch.py'\'
    ```

1. Apply the changes by sourcing the `.bashrc` file or restarting the terminal. Now, simply run:
    ```bash
    $ bringup
    ```
    This alias will allow you to run the `robot.launch.py` script on your TurtleBot3 without needing to log into the remote computer each time.

1. To verify that the launch file is running correctly, check the active ROS nodes on the `master` computer:
    ```bash
    $ ros2 node list
    ```
    If the expected nodes are running, your setup is correct.

<!--
1. (Optional) If you need to stop the launch file remotely, you can use:
    ```bash
    $ ssh pi@robotX 'pkill -f robot.launch.py'
    ```
    This will terminate the running launch process on the TurtleBot3.
-->

Following these steps will ensure a seamless and efficient workflow for launching ROS2 on your TurtleBot3 without needing manual SSH logins each time.


(Not ready yet!)

## 💻 Lab Procedure

### **Create a New ROS2 Package**

1. Open a terminal and navigate to the `ece387_ws` directory of your workspace:
   ```bash
   $ cd ~/master_ws/src/ece387_ws
   ```

1. Use the following command to create a new ROS2 package named `lab7_tf` with the BSD-3 license:
   ```bash
   $ ros2 pkg create --build-type ament_python --license BSD-3-Clause lab7_tf
   ```

1. Copy the `move2goal.py` file from Lab6 to `ece387_ws/lab7_tf/lab7_tf/` and rename it to `move2goal_tf.py`

1. Add the following lines as a dependency in your `package.xlm`. 
    ```sh
    <depend>rclpy</depend>
    <depend>geometry_msgs</depend>
    <depend>nav_msgs</depend>
    <depend>sensor_msgs</depend>
    <depend>std_srvs</depend>
    ```

1. Open the `setup.py` file and modify the `entry_points` section to include the `move2goal_tf` script:
   ```python
   entry_points={
       'console_scripts': [
           'move2goal_tf = lab7_tf.move2goal_tf:main',
       ],
   },
   ```

1. In `setup.py`, add `'std_srvs'` to the `install_requires` list:
    ```python
    'std_srvs',
    ```

### **Implement the `reset_pose` Service**

1. Import the required ROS2 service type:
   ```python
   from std_srvs.srv import Empty
   ```

1. Add a service server in the `__init__` method:
   ```python
    # TODO: Create a service server that will handle 'reset_pose' service requests
    # - Service type: 'Empty' (no data is exchanged)
    # - Service name: 'reset_pose'
    # - Callback function: 'self.reset_pose_callback' to execute when the service is called
    self.reset_service = 0 # Update this line.
   ```
1. Replace the following lines in the `__init__` method
    ```python
    # Variables to store the robot's current position and orientation
    self.x = 0  # Current x-coordinate
    self.y = 0  # Current y-coordinate
    self.yaw = 0  # Current orientation (yaw angle in radians)
    ```
    with

    ```python
    # Local Position Variables (Start at 0,0,0)
    self.local_x = 0.0
    self.local_y = 0.0
    self.local_yaw = 0.0

    # Variables for previous position to compute displacement
    self.initial_global_x = None
    self.initial_global_y = None
    self.initial_global_yaw = None
    ```

1. Define the callback function for the service to reset local pose and initial global pose:
    ```python
    def reset_pose_callback(self, request: Empty, response: Empty) -> Empty:
        """
        Resets the local position coordinates to zero and clears the previous position coordinates.
        """

        # Reset the local position coordinates to zero
        # This sets the current local position to the origin (0, 0, 0)
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_yaw = 0.0

        # Clear the previous position coordinates
        # This ensures that any previous position data is discarded
        self.initial_global_x = None
        self.initial_global_y = None
        self.initial_global_yaw = None

        # Log a message to indicate the local position has been reset
        self.get_logger().info("Local pose reset to zero.")

        # Return the response to the service caller
        return response
    ```

1. Test the service
    Run this node:
    ```bash
    $ ros2 run lab7_tf move_to_goal
    ```
    Then, in another terminal, call the service:
    ```bash
    $ ros2 service call /reset_pose std_srvs/srv/Empty
    ```
    This should reset `local_x`, `local_y`, and `local_yaw` to zero, and you should see a log message confirming the reset.


### **Implement Coordinate Transformation**

1. Modify the `odom_callback` method:
    ```python
    def imu_callback(self, imu_msg: Imu) -> None:
        """
        Callback function for handling control relinquishment messages.
        Updates local yaw relative to the starting orientation.
        """

        q = imu_msg.orientation
        _, _, global_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.initial_global_yaw is None:
            # Store the initial yaw as a reference
            self.initial_global_yaw = global_yaw
            return  # Skip first iteration

        # TODOL Compute local yaw relative to the initial global yaw
        self.local_yaw = 0
    ```

1. Modify the `odom_callback` method:

    ```python
    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback function for handling odometry messages.
        Updates the local x and y coordinates relative to the starting position.
        """
        # Extract global x and y coordinates from the odometry message
        global_x = msg.pose.pose.position.x
        global_y = msg.pose.pose.position.y

        # Check if this is the first iteration (initial global position not set)
        if self.initial_global_x is None:
            # Store the initial global position as the reference point
            self.initial_global_x = global_x
            self.initial_global_y = global_y
            # Skip processing for the first iteration
            return

        # TODO: Compute the displacement of the global position from the initial global position
        dx = 0
        dy = 0

        # TODO: Rotate displacement to align with the initial local frame
        # This step is necessary to ensure the local coordinates are relative to the starting orientation
        self.local_x = 0
        self.local_y = 0
    ```

1. Make necessary changes in the `control_loop` method.








### 🚚 Deliverables


In this lab, you will create a ROS2 Python package that enables the TurtleBot3 to navigate to a desired **location and orientation** using data from the **IMU (`/imu`)** and **ODOM (`/odom`)** topics.

















## Purpose
Large applications in robotics typically involve several interconnected ROS nodes, each of which have many parameters. Your current setup is a good example: as you experienced in the IMU lab, you had to open 3 different terminals to run all of the nodes necessary for our system to that point:



This problem is only going to get more complex as we add additional functionality to our robot. As it stands right now, every node requires a separate terminal window and the associated command to run it. Using the *roslaunch* tool, we can eliminate that administrivia of running each node separately. We will create/edit two launch files to bring up the nodes on the master and robot.

## [roslaunch](http://wiki.ros.org/roslaunch)
The *roslaunch* tool is used to launch multiple ROS nodes locally and remotely via SSH. We can run nodes that we have created, nodes from pre-built packages, and other launch files. The roslaunch tool takes in one or more XML configuration files (with the .launch extension) that specify the parameters to set and nodes to launch.

A launch file is an XML document which specifies:
- which nodes to execute
- their parameters
- what other files to include

An XML file stands for Extensible Markup Language (XML). This is a markup language that defines a set of rules for encoding documents in a format that is both human-readable and machine-readable. That isn't necessarily important for this class, but you can read about XML on Wikipedia if you are interested.

We will then use a tool embedded within ROS called *roslaunch* to easily launch multiple nodes or even other launch files.

By convention, we will give our launch files the *.launch* extension and store them in a *launch* folder within our package. This isn't required, but it is the common convention.

## Current State
In this section we will first use the conventional technique to bring up all of the nodes required for **Lab 2** using the currently understood techniques.


Notice, running **roscore** now monopolized that terminal and you can no longer use it for anything else.

Open a new terminal or tab on your **Master** and run the **controller.py** node:

```bash
rosrun lab2 turtlebot_controller.py
```



