# Lab 2: Inertial Measurement Unit

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
