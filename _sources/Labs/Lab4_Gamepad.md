# 🔬 Lab4: Gamepad


## **THIS LAB IS NOT READY**

## 📜 Overview
This Lab will provide you more insight into ROS topics and messages and how information is passed between nodes. A node can publish specific messages over a topic and other nodes are able to subscribe to that topic to receive the message. The format of these messages must be pre-defined and each node needs to know the format of the message. In this lab you will develop a node that subscribes to a joystick topic and publishes a topic that is used to enable a controller to drive the robot.



## 💻 Procedure

1. Go to the [Gamepad Setup](../Appendix/GamepadSetup.md) page to configure Logitech Gamepad as an input device to your `Master` computer.

1. Open a Terminal and navigate to the `src` directory in your workspace:
    ```bash
    $ cd  ~/master_ws/src
    ```

1. Install the TurtleBot3 Simulation Package by running
    ```bash
    $ git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    ```
    Ensure the `turtlebot3_simulations` directory is in the `src` directory.

1. Run `colcon build`
    ```bash
    $ cd ~/master_ws
    $ colcon build --symlink-install
    ```
    Note that you must always run `colcon build` in the root of your workspace, i.e., `~/master_ws`.

    You should see the following output. If you see the `stderr` and `CMake Warning` as shown below, don't worry and you can ignore them. 

    ```bash
    Finished <<< turtlebot3_fake_node [17.9s]                                       
    --- stderr: turtlebot3_gazebo                                
    CMake Warning (dev) at /usr/share/cmake-3.22/Modules/FindPackageHandleStandardArgs.cmake:438 (message):
    The package name passed to `find_package_handle_standard_args` (PkgConfig)
    does not match the name of the calling package (gazebo).  This can lead to
    problems in calling code that expects `find_package` result variables
    (e.g., `_FOUND`) to follow a certain pattern.
    Call Stack (most recent call first):
    /usr/share/cmake-3.22/Modules/FindPkgConfig.cmake:99 (find_package_handle_standard_args)
    /usr/lib/x86_64-linux-gnu/cmake/gazebo/gazebo-config.cmake:72 (include)
    CMakeLists.txt:23 (find_package)
    This warning is for project developers.  Use -Wno-dev to suppress it.

    ---
    Finished <<< turtlebot3_gazebo [20.9s]
    Starting >>> turtlebot3_simulations
    Finished <<< turtlebot3_simulations [0.90s]                

    ```




```bash
$ ros2 pkg create --build-type ament_python lab4_gamepad
```

Your terminal will return a message verifying the creation of your package py_pubsub and all its necessary files and folders.









Creating a ROS 2 package that subscribes to the `joy` topic (from a joystick) and publishes `cmd_vel` to control a TurtleBot3 is a great project! Below are step-by-step instructions to achieve this.

---

### Step 2: Create a New ROS 2 Package
1. Navigate to the `src` directory of your workspace:
   ```bash
   cd ~/ros2_ws/src
   ```
2. Create a new ROS 2 package:
   ```bash
   ros2 pkg create --build-type ament_python joy_to_cmd_vel
   ```
   This creates a package named `joy_to_cmd_vel`.

---

### Step 3: Write the ROS 2 Node
1. Navigate to the package directory:
   ```bash
   cd joy_to_cmd_vel/joy_to_cmd_vel
   ```
2. Create a Python script for the node:
   ```bash
   touch joy_to_cmd_vel_node.py
   chmod +x joy_to_cmd_vel_node.py
   ```
3. Open the script in a text editor and add the following code:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Joy

---

### Step 4: Update `setup.py`
1. Open the `setup.py` file in the package directory:
   ```bash
   cd ~/ros2_ws/src/joy_to_cmd_vel
   nano setup.py
   ```
2. Add the following entry point under `console_scripts`:
   ```python
   entry_points={
       'console_scripts': [
           'joy_to_cmd_vel_node = joy_to_cmd_vel.joy_to_cmd_vel_node:main',
       ],
   },
   ```

---

### Step 5: Build the Workspace
1. Navigate to the root of your workspace:
   ```bash
   cd ~/ros2_ws
   ```
2. Build the workspace:
   ```bash
   colcon build
   ```

---

### Step 6: Run the Node
1. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
2. Launch the TurtleBot3 simulation (optional):
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
3. Run the `joy` node to publish joystick data:
   ```bash
   ros2 run joy joy_node
   ```
4. Run your `joy_to_cmd_vel` node:
   ```bash
   ros2 run joy_to_cmd_vel joy_to_cmd_vel_node
   ```

---

### Step 7: Control the TurtleBot3
- Use your joystick to control the TurtleBot3. The left stick should move the robot forward/backward and rotate it left/right.

---

### Notes
- **Joystick Mapping**: The axes and buttons on your joystick may vary. Use `ros2 topic echo /joy` to see the joystick data and adjust the mapping in the `joy_callback` function if needed.
- **TurtleBot3 Model**: Ensure the `TURTLEBOT3_MODEL` environment variable is set correctly (e.g., `waffle` or `burger`).

---

Let me know if you need further assistance!













## Implementing the chat subscriber

### Import modules


```python
# import required modules
import rospy
from std_msgs.msg import String
```

## Listener
This function will create the subscriber ("listener") used to receive chat messages from the publisher ("talker").


```python
def listener():
    rospy.Subscriber('chat', String, callback_chat)
```

The above function creates the subscriber to the **/chat** topic. Every time a *String* message is sent over the topic the `callback_chat()` function is called. This is an interrupt that spins a new thread to call that function.

### Callback function
The callback function will log and display what the chat listener sent.


```python
def callback_chat(message):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", message.data)
```

The callback function receives the *String* message as an input (you can name this parameter anything, but it is helpful if it is a meaningful variable name). To access the actual message, we need to utilize the data attribute of the *String* message. If you browse to the documentation for the [String Mesage](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html), you will note that the message attribute is called *data* and it is of type *string*. This is why we use the command `message.data`.

### Main


```python
def main():
    rospy.init_node('listener')
    try:
        listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

The above is similar to the talker, but adds the `rospy.spin()` function call to create an infinite loop to allow the subscriber to operate in the background.

In an actual Python script we will replace 
```python
def main()
``` 
with 
```python
if __name__ == "__main__":
```
This allows our python files to be imported into other python files that might also have a main() function.

### Run the listener


```python
main()
```

    [INFO] [1666756858.595079, 308.564000]: /listener I heard hello
    

At this point, the subscriber is waiting for the publisher to send a message. Browse back to your talker and type a message! You should see that message show up above after hitting `enter` in the talker Notebook.

## Talker

### A note on this document
This document is known as a Jupyter Notebook; it is used in academia and industry to allow text and executable code to coexist in a very easy to read format. Blocks can contain text or code, and for blocks containing code, press `Shift + Enter` to run the code. Earlier blocks of code need to be run for the later blocks of code to work.

### Purpose
This Jupyter Notebook will delve a little deeper into ROS implementation. You will create a basic one way chat server that allows a user to send messages to another (both users will be on the same computer at this point in time).

### Initialize ROS:
The first step when utilizing  ROS is to initialize *roscore*. There are two methods to accomplish this: first, by explicitly running *roscore* and second, by running a launch file (which will initialize *roscore* if it is not already running). During the first portion of this course we will explicitly run *roscore* and then take advantage of launch files later in the course.

Copy the following code and run it in a new terminal (use the shortcut `ctrl+alt+t` to open a new terminal window or select an open terminal and hit `ctrl+shift+t` to open a new terminal tab):

`roscore`

### Implementing the chat publisher
> 📝️ **Note:** The following is Python code that will be implemented within the Jupyter Notebook. Again, the Notebook is just a way to allow for code and text to coexist to help guide you through the ICE. You could take all of the code in this Notebook and put it within a Python file and it would work the same as it does here. The focus of this ICE is the ROS implementation. It is assumed you have a working knowledge of Python at this time so this Notebook will not go into a lot of background regarding the Python code. Module 3 will provide a Python refresher.

### Import modules

> ⚠️ **Important:** Importing classes may take some time. You will know the code block is still executing as the bracket on the left of the cell will change to a `*` character. Do not move to the next step until the `*` is gone.


```python
# import required modules
import rospy
from std_msgs.msg import String
```

Here, we have two modules, rospy and a message. The rospy module provides all ROS implementation to create nodes and publish messages on topics. The next line imports the String message from the std_msgs package which we will use to send our chat messages. The Standard ROS Messages include a number of common message types. You can find more information about these messages on the [ROS wiki](http://wiki.ros.org/std_msgs).

### Talker Function
One method to communicate in ROS is using a publisher/subscriber model. One node publishes messages over a topic and any other nodes can subscribe to this topic to receive the messages.

This function will create the publisher ("talker") used to send chat messages to the subscriber ("listener"). It will read user input and publish the message.


```python
def talker():
    chat_pub = rospy.Publisher('chat', String, queue_size = 1)
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        chat_str = input("Message: ")
        rospy.loginfo("I sent %s", chat_str)
        chat_pub.publish(chat_str)
        rate.sleep()
```

Line 2 creates the publisher. The publisher will publish *String* messages over the **/chat** topic. The `queue_size` parameter determines how many messages the ROS network will hold before dropping old messages. If the publisher publishes faster than the network can handle, then messages will start getting dropped.

Line 3 determines the rate at which the following loop should run. With an input of 10, the loop should run 10 times per second (10 Hz). Line 4 creates a loop that runs until `ctrl+c` is pressed in the terminal. Line 5 gets user input while line 6 logs the message into a log file (and prints it to the screen). Line 7 publishes the chat message using the previously created publisher. Lastly, line 8 sleeps so the loop runs at the desired rate.

> 📝️ **Note:** Waiting for user input will cause the loop to not run at 10 Hz as line 5 will block until the user hits enter.

### Main
The main function calls our talker function.


```python
def main():
    rospy.init_node('talker')
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

Line 2 of the above code initializes our talker node. The rest creates a try-except statement that calls our talker function. All the try/except really does is ensures we exit cleanly when ctrl+c is pressed in a terminal.

In an actual Python script we will replace 
```python
def main()
``` 
with 
```python
if __name__ == "__main__":
```
This allows our python files to be imported into other python files that might also have a main() function.

### Run the talker


```python
main()
```

### Create the listener
At this point the talker is waiting for user input. Don't start typing yet, though! We need to implement and run our listener. 

### ROS commands

Note that the Jupyter code block for the `main()` function call on both the talker and listener has an `*` on the left side. That is due to the infinite loops in the talker and main functions. This means that those functions are blocking and no other Jupyter code blocks will run in these two notebooks. We have to open a new notebook to run the ROS commands we would use to investigate the state of our ROS system. This would be equivalent to opening a new terminal on the Linux computer. 



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

