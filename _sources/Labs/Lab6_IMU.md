# 🔬 Lab6: IMU


## Purpose
In practice, an inertial measurement unit (IMU) device provides orientation, angular velocity, and linear acceleration. The [ICM-20648 6-Axis MEMS MotionTracking Device](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-20648/) from TDK includes a 3-axis gyroscope, 3-axis accelerometer, and a Digital Motion Processor (DMP). This IMU is integrated on the OpenCR1.0 board, an open source robot controller embedded with an ARM Cortex-M7 processor. The OpenCR combines sensor data using an EKF to generate IMU estimates 170 times a second.

```{image} ./figures/IMU.jpg
:width: 380
:align: center
```
<br>

The IMU provides values that are part of the robot state and allow the robot to navigate more accurately. Combined with data from the tachometers these values provide the odometry of the robot to estimate change in position over time. We will primarily use the IMU to perform 90 and 180 degree turns.

```{image} ./figures/TurtleBot3_Coordinates.png
:width: 280
:align: center
```

## Calibrating the IMU
As described above, there are a number of different sensors that work together to provide the attitude and heading estimates for the TurtleBot3. These sensors are sensitive to magnetic fields which are unique to locale and device. As you will learn in future ECE classes, all electronic devices create small magnetic fields. Even electrons traveling over a wire create magnetic fields. The OpenCR board and IMU are strategically placed in the center of the robot for best attitude and heading performance, however, this location is also in the center of a number of magnetic fields. Luckily for us, the creators of the Turtlebot3 were aware of these issues and whenever you run the serial node to connect to the robot the IMU is calibrated.

## Setup
The ICM-20648 is already integrated into the Turtlebot3 robot, therefore, there is no setup required. Whenever the serial node is ran to connect to the OpenCR board, the IMU is initialized and will start publishing data.

## Test the IMU
Open a new terminal on the master and run roscore and setup for statistics:

```bash
roscore
rosparam set enable_statistics true
```

Create a secure shell connection to your **Robot** and launch the Turtlebot3 core launchfile.

```bash
roslaunch turtlebot3_bringup turtlebot3_core.launch
```

Open a new terminal on your **Master** and observe what topics are running.

You should note two topics of interest: **/imu** and **/odom**.

Echo the output of each of the topics and rotate the **Robot** to see the values change.

The **/imu** topic combines information from the gyroscope and accelerometer to provide orientation, angular velocity, and linear acceleration. The **/odom** topic combines the information from the **/imu** topic and tachometers to estimate position, orientation, and linear and angular velocities.

Both of these topics provide the orientation of the robot using a quaternion representation. While quaternions can make computation easier, they are not very human readable, so we will convert to Euler angles. To do this we will use a Python library called [squaternion](https://pypi.org/project/squaternion/).

The two main functions we will use from the squaternion library:

First we will need to create a Quaternion object:
```
q = Quaternion(w, x, y, z)
```

Then we will conver that Quaternion to an Euler:
```
e = q.to_euler(degrees=True)
```

Now we have `e`, an array representing our Euler angles, `e[roll, pitch, yaw]`.

You can keep the node running for the next portion of the lesson.

## Write the Subscriber
1. In a new terminal on the **Master**, create an **ice6** package which depends on the *geometry_msgs*, *rospy*, and *turtlebot3_bringup* packages, compile and source the workspace:

    ```bash
    cd ~/master_ws/src/ece387_master_sp2X-USERNAME/master
    catkin_create_pkg ice6 std_msgs rospy turtlebot3_bringup
    cd ~/master_ws
    catkin_make
    source ~/.bashrc
    ```

1. Create an IMU node:

    ```bash
    roscd ice6/src
    touch imu_sub.py
    ```
    
1. Copy and complete the below code using a GUI editor tool, such as **Sublime** or **VS Code**. Browse to the subscriber you just created and double-click. This will open the file in **Sublime** or **VS Code** (if it is open in any other editor, stop, raise your hand, and get help from an instructor)
> 💡️ **Tip:** Look for the **"TODO"** tag which indicates where you should insert your own code.

```python
#!/usr/bin/env python3
import rospy
from squaternion import Quaternion
# TODO: import message type sent over imu topic


class IMU:
    """Class to read orientation data from Turtlebot3 IMU"""
    def __init__(self):        
        # TODO: subscribe to the imu topic that is published by the
        # Turtlebot3 and provides the robot orientation


        # nicely handle shutdown (Ctrl+c)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    # The IMU provides yaw from -180 to 180. This function
    # converts the yaw (in degrees) to 0 to 360
    def convert_yaw (self, yaw):
        return 360 + yaw if yaw < 0 else yaw		

    # Print the current Yaw
    def callback_imu(self, imu):
        if not self.ctrl_c:
            # TODO: create a quaternion using the x, y, z, and w values
            # from the correct imu message
            
            
            # TODO: convert the quaternion to euler in degrees
            
            
            # TODO: get the yaw component of the euler
            yaw = 

            # convert yaw from -180 to 180 to 0 to 360
            yaw = self.convert_yaw(yaw)
            print("Current heading is %f degrees." % (yaw))

    # clean shutdown
    def shutdownhook(self):
        print("Shutting down the IMU subscriber")
        self.ctrl_c = True

if __name__ == '__main__':
    rospy.init_node('imu_sub')
    IMU()
    rospy.spin()
```

4. Save, exit, and make the node executable.

4. Open a new terminal on the **Master** and run the **imu_sub.py** node.

4. Rotate the **Robot** and observe the output.

## Checkpoint
Once complete, get checked off by an instructor showing the output of your **imu_sub** and **rqt_graph** node.  Push your ice6 package to your repo for credit

## Summary
In this lesson you learned how to utilize the on-board IMU and determine the orientation of the Turtlebot3. In the lab that corresponds to this lesson you will apply this knowledge to turn the robot in 90 and 180 degree turns.ROS

## Cleanup
In each terminal window, close the node by typing `ctrl+c`. Exit any SSH connections. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-lab` in. Select 'y'.

**Ensure roscore is terminated before moving on to the lab.**

## Purpose
This lab will integrate the on-board IMU with the Turtlebot3 controller to turn the robot 90 degrees left or right.  Do not disable any of the mouse controller functionality in the turtlebot_controller.py code.  The IMU and mouse control functionality should be able to coexist seemlessly.

## Master
### Setup:
In the `/master_ws/src/ece387_master_sp2X-USERNAME/master` folder, create a **lab2** package which depends on **std_msgs**, **rospy**, **geometry_msgs**, and **turtlebot3_bringup**.

### controller.py
1. Copy the turtlebot_controller.py file from lab1 into the lab2 package.

1. Open the turtlebot_controller.py file from lab2 using an editor.

1. Import the squaternion library and Imu message used in ICE6.

1. Add the following Class variables within the class **above** the `__init__()` function:

    1. `K_HDG = 0.1 # rotation controller constant`
    1. `HDG_TOL = 10 # heading tolerance +/- degrees`
    1. `MIN_ANG_Z = 0.5 # limit rad/s values sent to Turtlebot3`
    1. `MAX_ANG_Z = 1.5 # limit rad/s values sent to Turtlebot3`
    
1. Add the following to the `__init__()` function:

    1. Instance variable, `self.curr_yaw`, initialized to 0 to store the current orientation of the robot
    1. Instance variable, `self.goal_yaw`, initialized to 0 to store the goal orientation of the robot
    1. Instance variable, `self.turning`, initialized to `False` to store if the robot is currently turning
    1. A subscriber to the IMU topic of interest with a callback to the callback_imu() function
    
1. Add the `convert_yaw()` function from ICE6.
    
1. Add the `callback_imu()` function from ICE6, removing print statements and setting the instance variable, `self.curr_yaw`.

1. Edit the `callback_controller()` function so it turns the robot 90 degrees in the direction inputed by the user (left or right). Below is some pseudo-code to help you code the controller function 

> ⚠️ **WARNING:** Pseudo-code is not actual code and cannot be copied and expected to work!

```python
def callback_controller(self, event):
    # local variables do not need the self
	yaw_err = 0
	ang_z = 0
    # not turning, so get user input
    if not turning:
        read from user and set value to instance variable, self.goal_yaw
        input("Input l or r to turn 90 deg")
        
        # check input and determine goal yaw
        if input is equal to "l" 
            set goal yaw to curr yaw plus/minus 90
            turning equals True
        else if input is equal to "r"
           	set goal yaw to curr yaw plus/minus 90
            turning equals True
        else 
        	print error and tell user valid inputs
            
        # check bounds
        if goal_yaw is less than 0 then add 360
        else if goal_yaw is greater than 360 then subtract 360
    
    # turn until goal is reached
    elif turning:
        yaw_err = self.goal_yaw - self.curr_yaw
        
        # determine if robot should turn clockwise or counterclockwise
        if yaw_err > 180:
            yaw_err = yaw_err - 360
        elif yaw_err < -180:
            yaw_err = yaw_err + 360
            
        # proportional controller that turns the robot until goal 
        # yaw is reached
        ang_z = self.K_HDG * yaw_err
        
        if ang_z < self.MIN: ang_z = self.MIN		# need to add negative test as well!
        elif ang_Z > self.MAX: ang_z = self.MAX	# need to add negative test as well!
        
        # check goal orientation
        if abs(yaw_err) < self.HDG_TOL
            turning equals False
            ang_z = 0
            
   # set twist message and publish
   self.cmd.linear.x = 0
   self.cmd.angular.z = ang_z
   publish message
```

## Run your nodes
1. On the **Master** open a terminal and run **roscore**.
1. Open another terminal and enable statistics for **rqt_graph**.
1. Run the controller node.
1. Open secure shell into the **Robot** and run the **turtlebot3_core** launch file.
1. Type "l" or "r" to turn the robot 90 degrees.

## Report
Complete a short 2-3 page report that utilizes the format and answers the questions within the report template. The report template and an example report can be found within the Team under `Resources/Lab Template`.

## Turn-in Requirements
**[25 points]** Demonstration of keyboard control of Turtlebot3 (preferably in person, but can be recorded and posted to Teams under the Lab 2 channel).

**[50 points]** Report via Gradescope.

**[25 points]** Code: push your code to your repository. Also, include a screen shot of the **turtlebot_controller.py** file at the end of your report.



