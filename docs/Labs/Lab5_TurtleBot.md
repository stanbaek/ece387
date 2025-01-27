# 🔬 Lab4: Driving the Robot


## Lesson Objectives:
1. Gain additional familiarity with simulation environment
1. Gain familiarity with Turtlebot 3 robotics platform
1. Practice with ROS diagnostic tools

## Agenda:
1. Use Linux terminals to launch and control Turtlebot 3 in simulation environment.
1. Use Linux terminals to launch and control actual Turtlebot 3.















Sure, I can help with that! Here are the step-by-step instructions to use your Logitech F310 with ROS2:

1. **Plug in your Logitech F310 controller** to your computer via USB.
2. **Install the necessary dependencies** for ROS2 and joystick support. You can do this by running:
   ```bash
   sudo apt-get update
   sudo apt-get install ros-<ros_version>-joy
   ```
   Replace `<ros_version>` with your ROS2 version (e.g., `foxy`, `galactic`).
3. **Set up the udev rules** for your controller. Create a file named `99-joystick.rules` in the `/etc/udev/rules.d/` directory with the following content:
   ```bash
   SUBSYSTEM=="input", ATTRS{name}=="Logitech USB Gamepad", MODE="0666"
   ```
4. **Reload the udev rules** to apply the changes:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
5. **Launch the teleop_f310 node** in ROS2:
   ```bash
   roslaunch teleop_f310 teleop_f310.launch
   ```
   This will start the node that allows you to control your robot using the Logitech F310 joystick.

6. **Verify the joystick input** by echoing the `joy` topic:
   ```bash
   rostopic echo joy
   ```
   You should see the joystick inputs being published if everything is set up correctly.

7. **Adjust the joystick configuration** if needed using the dynamic reconfigure plugin in `rqt`. This allows you to change the joystick mappings on-the-fly.

That's it! You should now be able to use your Logitech F310 with ROS2. If you encounter any issues, feel free to ask for more help!











## Running the TurtleBot3 simulation

The best way to learn about ROS is through implementation, however, let's start off by playing around with a virtual TurtleBot3! The TurtleBot3 is a tool we will utilize throughout this course to apply and integrate robotics systems. Ultimately you will create a complex embedded robotics system to perform a specific dedicated task, such as navigating the halls of DFEC. But let's see if we can get the robot to drive around first.

1. In a terminal (**Pro tip:** ctrl+alt+t opens a new terminal window, while ctrl+shift+t opens a new terminal tab in an existing window) and initialize the ROS network:

    `roscore`

1. That terminal is now occupied. Open a new terminal tab/window and launch the TurtleBot3 gazebo launch file (A launch file is a way to run one or more nodes at once, we will learn about launch files later):

    `roslaunch turtlebot3_gazebo turtlebot3_world.launch`

> ⌨️ **Syntax:** `roslaunch <package> <launchfile>`

 
3. Open another terminal tab/window and launch the following:

    `roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch`

The launch files will run the nodes necessary to simulate our robot and opens two windows: Gazebo and rviz. Gazebo is a simulation tool that provides physics for our robot and enables the testing of algorithms prior to real-world implementation. You should see a TurtleBot3 sitting at (-2.0, -0.5) facing the positive x direction surrounded by a maze. Using the mouse, left-click and holding will pan, holding the scroll wheel will change the orientation of the camera, and scrolling will zoom in and out.

The second window is rviz, a tool that visualizes topics interacting in our system. You should see a number of red dots outlining the location of the obstacles in our simulation. These are objects detected by our LIDAR which is communicating over a scan topic. Using the mouse, left-click will change the orientation of the camera, holding the scroll wheel will pan, and scrolling will zoom in and out (would be nice if they were the same).

Don't worry! We will learn more about both of these tools at a later time.

Let's go ahead and take a look at what nodes and topics currently running in our system. 

The "!" character in front of the following commands allows us to run bash commands from the Jupyter NB and would **NOT** be used in the command line.


```python
rosnode list
```

There are five nodes running, the first two enable the Gazebo simulation, the third allows for the visualization of the simulated robot, the fourth is created every time *roscore* is ran and helps with communication in our network, and the last enables rviz.

Not too exciting yet, so lets see what topics are active.


```python
rostopic list
```

There are a lot of topics that are utilized to simulate our robot. Our real-world robot will eventually have most of these, such as **/cmd_vel**, **/imu**, and **/scan**. These are topics that allow us to communicate with some of the simulated hardware, such as the orientation sensor (/imu), LIDAR (/scan), and driving the robot (/cmd_vel). The rest of the topics enable visualization and movement within the simulation and can be ignored for now.

Another useful tool for visualizing nodes and topics is called *rqt_graph*:


```python
rqt_graph
```

All that is going on right now is Gazebo is publishing the position and scan data which rviz uses to visualize the robot. **Close your rqt_graph** and let's add another node to make things a bit more interesting. 

Earlier we saw the topic **/cmd_vel**. This is a topic used to send *Twist* messages to a robot. A *Twist* message includes linear and angular x, y, z values to move a robot in a 3-dimensional space (google `ROS twist message` for more information). Our robot only drives in 2-dimensions and the wheels can only move forward and backward, so we will only use the linear x (forward and backward) and angular z (turning) attributes of the *Twist* message. To drive our simulated robot, we need a node that can publish *Twist* messages to the **/cmd_vel** topic. Luckily, there is a pre-built node called Teleop_Twist_Keyboard that sends *Twist* messages using a keyboard!

Open a new terminal tab (select the terminal and press ctrl+shift+t) and run the following (**Pro tip**: in a Linux terminal if you type the first couple letters of a command and then hit tab it will autocomplete the command for you):

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

> ⌨️ **Syntax:** `rosrun <package> <executable>`

To drive the robot use the 'x' key to decrease linear x speed to about .25 m/s and use the 'c' key to decrease angular z speed to about .5 rad/s. Now follow the directions to utilize the keyboard to drive the robot! You should see the robot move in the simulation.

Let's take a look at our rqt_graph to see if anything has changed.

```python
rqt_graph
```

You should now see the teleop_twist_keyboard node which sends messages over **/cmd_vel** topic to Gazebo. **Close the rqt_graph window**. Let's run through a number of commands that will provide you more information about your ROS network. You will use these throughout the course to determine what is going on in your ROS network.

## Common ROS commands

The `rosnode` command allows us to interact with nodes. Typing any ROS command followd by `--help` will provide information about that command:


```python
rosnode --help
```

Let's get some information about our new node, teleop_twist_keyboard:


```python
rosnode info /teleop_twist_keyboard
```

The output of the command lists the topics the node is publishing and subscribing to (here is where we can see it publishes on **/cmd_vel**).

The `rostopic` command interacts with topics.


```python
rostopic --help
```

Some of the common rostopic commands we will use in this course are `echo`, `hz`, `info`,  `type`, and `list`


```python
rostopic list
```

Let's get some information about the **/cmd_vel** topic.


```python
rostopic info /cmd_vel
```

From the output we can see what nodes are publishing and subscribing to the **/cmd_vel** topic.

Echoing the topic will allow us to see what messages are sent over the topic. After running the below command, browse back to your teleop_twist_keyboard node and drive the robot. You should see the twist messages sent to Gazebo.


```python
rostopic echo /cmd_vel
```

```{note}
When moving forward and backward ('i' and ',' keys) only a linear x value is sent, when turning left or right ('j' and 'l' keys) only an angular z value is sent, and when arcing ('u', 'o', 'm', and '.' keys) both a linear x and angular z value are sent.
```

```{note}
The previous command still has an `*` character to the left. This means this command is waiting for inputs and will block all future commands. To kill the command and restart the kernel in the Jupyter Notebook at the top menu bar select "Kernel" and "Restart & Clear Output". This will allow future commands to run.
```

To learn more about the messages sent over the **/cmd_vel** topic we can use the `type` command and the `rosmsg` tool.


```python
rostopic type /cmd_vel
```


```python
rosmsg show geometry_msgs/Twist
```

Or in one combined command:


```python
rostopic type /cmd_vel | rosmsg show
```








## Gain Additional Familiarity with Simulation Environment.
At this point we are nearly 20% complete with the course, and we have the foundational knowledge required about the ecosystem we will be working in. In short we will be using the Linux operating system to host ROS. We will use ROS to execute python code to control and interact with the various subsystems on our robotics platform.

Before we move on to controlling our actual robot, we want to stress the importance and capabilities of our simulation environment. There will be times, where it may be inconvenient to work with the actual robot. Additionally, with all hardware systems, there can occasionally be inconsistent behaviors. For both of these reasons (and many others), having an effective simulator may be very useful. There are two tools that we will find very useful for simulation (RViz and Gazebo).

- RViz - (short for “ROS visualization”) is a 3D visualization software tool for robots, sensors, and
algorithms. It enables you to see the robot’s perception of its world (real or simulated).
- Gazebo - Gazebo is a 3D robot simulator. Its objective is to simulate a robot, giving you a close
substitute to how your robot would behave in a real-world physical environment. It can compute the
impact of forces (such as gravity).

The difference between the two can be summed up in the following excerpt from Morgan Quigley (one of
the original developers of ROS) in his book Programming Robots with ROS:

```
RViz shows you what the robot thinks is happening, while Gazebo shows you what is really happening.
```

For future research or development efforts, you may need to build your own simulation environment. However,
for this course, the developers of the Turtlebot3, Robotis, have already done this for us.


- In a free terminal window, bring up roscore
- In a second free terminal window, we are going to use the roslaunch command to bring up the Gazebo
environment within a maze. Tip: There are numerous virtual environments, but this will work for
now
```bash
dfec@master:∼$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
- In a third terminal window, we are now going to bring up the RViz environment.
```bash
dfec@master:∼$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```
- In a fourth terminal window, we are going to bring up a valuable tool that will allow us to control the
robot via the keyboard. This tool is called teleop_twist.
```bash
dfec@master:∼$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

```{tip} 
I strongly recommend that you commit the above sequence of commands to memory, or at a minimum
have them in a place that you can quickly recall them. There is nothing until Module 9 that absolutely requires the real robot, as everything else can be simulated.
```

## Gain Familiarity with Turtlebot3 Robotics Platform.
The Module04 Jupyter Notebook will guide you through the process of connecting to and activating your
robot for the first time.

1. On the master, open the Jupyter Notebook server (if it is not already open):
```bash
$ cd ~/master_ws/src/ece387_lastname/Module04_DrivingTheRobot
```

2. Open ICE4: Driving the Robot and follow the instructions.

## Assignments.
- Complete ICE4 if not accomplished during class.
- Push screen captures into your repo/master/module04 on github

## Next time.
- Lesson 10: Module 5 - Custom Messages








# ICE4: Driving the Robot

## Purpose
This In-Class Exercise will introduce you to utilizing pre-built ROS packages to accomplish a task. It will also provide you experience interacting with someone else's source code (.py files) to learn how that component works. You will use ROS to run two nodes, **turtlebot3_core** and **teleop_twist_keyboard**, to drive the Turtlebot3 with a keyboard. You will continue to practice using ROS tools to observe how these components communicate.

## Code used to drive the robot

1. On the Master, open a terminal and run **roscore**.

1. Open a new terminal on the Master and create a secure shell into the Turtlebot3 using the SSH command you learned during Module 2. This will allow you to run commands as if you were on the Turtlebot3.

1. Using the secure shell, open the source code for the **turtlebot3_core** launch using the nano command line editor tool through the rosed command:

    ```bash
    $ rosed turtlebot3_bringup turtlebot3_core.launch
    ```

    > ⌨️ **Syntax:**  `rosed <package> <filename>`

    ```{note} 
    You may remember when we set up our *.bashrc* file we set the system variable **EDITOR** to `nano -w`. This enables the `rosed` command to utilize the nano editor.
   ```
   
    We will learn more about launch files in a few modules, but just understand that a launch file is used to launch one or more ROS nodes. This particular launch file only launches one node, **serial_node.py**. This node will connect to the OpenCR controller on the Turtlebot3 using the port and baud rate parameters. This connection will enable us to send *Twist* messages over the **/cmd_vel** topic to drive the Turtlebot3 using the keyboard.

1. Close the editor by hitting `ctrl+x`.

1. It is always a good idea to check that the Turtlebot3 is communicating with the Master. To do this, we can list the active topics the Turtlebot3 sees. Run the following within your secure shell:

    ```bash
    $ rostopic list`
    ```
    
    If all is well, then there should be two topics provided by **roscore** running on the Master: **/rosout** and **/rosout_agg**. We will typically ignore these topics.

1. Run the **turtlebot3_core.launch** file using the `roslaunch` command:

    ```bash
    $ roslaunch turtlebot3_bringup turtlebot3_core.launch`
    ```

    > ⌨️ **Syntax:** `roslaunch <package> <launchfile>`
    
    Your Turtlebot3 is now ready to drive and should be listening for *Twist* messages to be sent over the **/cmd_vel** topic.

## Driving the robot
1. Open a new terminal on the Master and observe the nodes currently running:

    `rosrun rqt_graph rqt_graph`
    
    You should only see one node running right now, **turtlebot3_core**, with no connections.
    
1. Open a new terminal tab and list the active topics. There should be one active topic other than the ones created by **roscore**: **/cmd_vel**.

1. We used the **/cmd_vel** topic when driving the simulated Turtlebot3, but let's refresh our memory about the topic:

    `rostopic info cmd_vel`
    
    As you can see the **/cmd_vel** topic is currently subscribed to by the **turtlebot3_core** with no publishers (just as we would expect after seeing the rqt_graph). We also note that topic utilizes the *Twist* message type. The following will show information about the fields within the *Twist* message sent over the **/cmd_vel** topic:
    
    `rostopic type cmd_vel | rosmsg show`

1. You can find information about pre-built packages by googling the package name along with the ROS distribution. Open up your favorite browser and google "teleop twist keyboard noetic". The first result should be from the ROS wiki page.

1. Ensure the ROS package **teleop_twist_keyboard** is installed on your Master:

    `rospack find teleop_twist_keyboard`
    
    If installed, the command should return the absolute path to the package, similar to `/opt/ros/noetic/share/teleop_twist_keyboard`
    
    If the command instead returns an error, then you need to install the package using apt:
    
    `sudo apt install ros-noetic-teleop-twist-keyboard`
    
    ```{tip}
    All packages built for Noetic can be downloaded in the above manner (ros-noetic-desired-pkg with underscores in the package name replaced by dashes). Some packages were only built for previous ROS distribution and will have to be built from source (we will demonstrate this at a future time).
    ```
    
1. Run the **teleop_twist_keyboard** node on the Master:

    ```{tip}
    Don't forget your tab completion! You can start typing a package name or node and then hit tab for it to complete the command for you!
    ```
    
    `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
    
1. Before we get too excited and drive the Turtlebot3 off a cliff, observe how the nodes communicate using the **rqt_graph** tool in a new terminal (if you still have the previous rqt_graph running, you can hit the refresh button in the top left corner).

1. Select the terminal that has the **teleop_twist_keyboard** node running and observe the instructions for sending *Twist* messages. These are the same as when driving the simulated Turtlebot3.

1. The Turtlebot3 operates best with a linear velocity between 0.2 m/s and 0.5 m/s. It turns best with an angular velocity between 0.5 rad/s and 1.5 rad/s. Drive the Turtlebot3 using these parameters.

## ROS

In labs throughout this course we will request information about the topics, nodes, and messages within your system. Accomplish the following in a new terminal on your Master (you can ignore all nodes/topics that result from **roscore**).

1. List all running nodes.

1. Determine what topics the nodes subscribe and publish to (repeat for each node).

1. Display running nodes and communication between them.

1. List the active topics.

1. Determine the type of messages sent over the topics (repeat for each topic).

1. Determine the fields of the messages.

1. Observe the information sent over a topic (repeat for each topic).

## Checkpoint
Once complete, push screenshots showing the output of each of the above to your student repo on github in a /master/module04 folder.

## Summary
In this exercise you examined and used pre-built packages and source code to drive the Turtlebot3 and understand how the system worked. You then were able to analyze the topics, nodes, and messages within the ROS system to better understand the flow of information and control. The **pro-tips** presented throughout this exercise will make you a better user of Linux and ROS.

## Cleanup
In each terminal window, close the node by typing `ctrl+c`. Exit any SSH connections. 