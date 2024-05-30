# ICE1: Talker and Listener

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

Note that the Jupyter code block for the `main()` function call on both the talker and listener has an `*` on the left side. That is due to the infinite loops in the talker and main functions. This means that those functions are blocking and no other Jupyter code blocks will run in these two notebooks. We have to open a new notebook to run the ROS commands we would use to investigate the state of our ROS system. This would be equivalent to opening a new terminal on the Linux computer. Open the [ROS](ROS.md) notebook and follow the instructions.



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
