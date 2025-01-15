# 🔬 Lab2: ROS

## Purpose
This lecture accompanies the introduction to ROS used in class. We will apply the knowledge you learned by interacting with a simulated TurlteBot3 Burger.

## ROS Introduction.

The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS is sometimes called a meta operating system because it performs many functions of an operating system, but it requires a computer's operating system such as Linux.

Why? Because creating truly robust, general-purpose robot software is hard. From the robot's perspective, problems that seem trivial to humans often vary wildly between instances of tasks and environments. Dealing with these variations is so hard that no single individual, laboratory, or institution can hope to do it on their own.

As a result, ROS was built from the ground up to encourage collaborative robotics software development. For example, one laboratory might have experts in mapping indoor environments, and could contribute a world-class system for producing maps. Another group might have experts at using maps to navigate, and yet another group might have discovered a computer vision approach that works well for recognizing small objects in clutter. ROS was designed specifically for groups like these to collaborate and build upon each other's work, as is described throughout this site.

ROS2 Humble Hawksbill: https://docs.ros.org/en/humble/

## ROS Command-line tools

The tutorials at [Beginner: CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html) offer a great introduction to ROS (Robot Operating System) command-line tools for beginners. These tutorials are designed to help you get started with ROS and learn how to use its command-line tools effectively.  

For the first tutorial, `Configuring Environment`, your environment has already been configured. Simply read through the page but do not run any commands. Instead, display the contents of your `.bashrc` file by running the following commands:

```bash
cd
cat .bashrc
```

You should see the following lines at the bottom of the `.bashrc` file. 
```bash
source /opt/ros/humble/setup.bash
source ~/master_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
source /usr/share/colcon_cd/function/colcon_cd.sh
export ROS_DOMAIN_ID=0  # For master0 and robot0
export _colcon_cd_root=/opt/ros/humble/
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01 # replace with LDS-02 if using new LIDAR
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
Please note that your `ROS_DOMAIN_ID=XX` should match your computer ID, where `XX` corresponds to the `XX` in `MasterXX`.


Visit [Beginner: CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html) and complete all 10 tutorials. The entire set should take approximately 2 hours to finish.  



## Deliverables (NOT READY YET)

Below you will run ROS commands. With the client and server nodes running execute the below commands.

List all running nodes:


```python

```

List the active topics:


```python
 
```

Display running nodes and communication between them:


```python
 
```

Exit the rqt_graph.

Show information about a the **/client** topic such as what type of messages are sent over the topic and publishing and subscribing nodes.


```python
 
```

Display information about the message that is sent over the **/client** topic.


```python

```

Display messages sent over the **/client** topic:


```python

```

In the ICE3_Client notebook when you send a message you should see the message echoed above.

## Checkpoint
Once complete, take the necessary screen shots of all the items above functioning. Upload those screenshots to your repo in a Module03 folder within the master folder of your repo.




## ROS
Below you will see the ROS commands you will use throughout this course to investigate your ROS system and write your lab reports. The "!" character in the front allows us to run bash commands from Jupyter and would **NOT** be used in the command line.

With the talker and listener nodes running execute the below commands.

List all running nodes:


```
$ rosnode list
```

You should see the listener and talker nodes. The */rosout* node is created when running `roscore` and facilitates communication in the network. You can ignore this node in your lab reports.

Get more information about the */listener* node:


```
$ rosnode info /listener
```

You can see what topics the node is publishing and subscribing to. It publishes to the ROS log file (for debugging) and subscribes to the **/chat** topic.

List the active topics:


```
$ rostopic list
```

The first topic is the one we created. The last two are created by `roscore` and can be ignored.

Show information about the **/chat** topic such as what type of messages are sent over the topic and publishing and subscribing nodes.




```
$ rostopic info /chat
```

As expected, the *talker* node is publishing to the **/chat** topic while the *listener* node subscribes.

Display running nodes and communication between them:


```
$ rqt_graph
```

Close the rqt_graph.

Display information about the message that is sent over the **/chat** topic.


```
$ rostopic type /chat | rosmsg show
```

The output of the command is the same as the information we saw from the ROS documentation. Again, to access the message we have to use the `data` attribute.

Display messages sent over the **/chat** topic:


```
$ rostopic echo /chat
```

In the ICE1_Talker notebook send a message to the listener. You should see that message show up both here and at the listener. This echo tool is useful to ensure your nodes are sending the messages as expected.

## Checkpoint
Once complete, get checked off by an instructor showing the output of each of the above.

## Cleanup
In each of the notebooks reset the Jupyter kernel and clear output (at the top menu bar select "Kernel" and "Restart & Clear Output"). Close each notebook. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.


