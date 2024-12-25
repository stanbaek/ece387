# Lab4: Driving the Robot


## Lesson Objectives:
1. Gain additional familiarity with simulation environment
1. Gain familiarity with Turtlebot 3 robotics platform
1. Practice with ROS diagnostic tools

## Agenda:
1. Use Linux terminals to launch and control Turtlebot 3 in simulation environment.
1. Use Linux terminals to launch and control actual Turtlebot 3.


## Gain Additional Familiarity with Simulation Environment.
At this point we are nearly 20% complete with the course, and we have the foundational knowledge required
about the ecosystem we will be working in. In short we will be using the Linux operating system to host
ROS. We will use ROS to execute python code to control and interact with the various subsystems on our
robotics platform.

Before we move on to controlling our actual robot, we want to stress the importance and capabilities of our simulation environment. There will be times, where it may be inconvenient to work with the actual robot. Additionally, with all hardware systems, there can occasionally be inconsistent behaviors. For both of these  reasons (and many others), having an effective simulator may be very useful.
There are two tools that we will find very useful for simulation (RViz and Gazebo).

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
have them in a place that you can quickly recall them. There is nothing until Module 9 that absolutely
requires the real robot, as everything else can be simulated.
```

## Gain Familiarity with Turtlebot3 Robotics Platform.
The Module04 Jupyter Notebook will guide you through the process of connecting to and activating your
robot for the first time.

1. On the master, open the Jupyter Notebook server (if it is not already open):
```bash
dfec@master:∼$ roscd ece387_curriculum/Module04_DrivingTheRobot
dfec@master:∼$ jupyter−lab
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