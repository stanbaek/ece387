# Driving the Robot


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

2. Open [ICE4: Driving the Robot](ICE4_DrivingTheRobot.md) and follow the instructions.

## Assignments.
- Complete ICE4 if not accomplished during class.
- Push screen captures into your repo/master/module04 on github

## Next time.
- Lesson 10: Module 5 - Custom Messages