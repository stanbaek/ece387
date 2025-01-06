# 🔬 Lab8: LIDAR

## Purpose
This lab will integrate the Turtlebot3 LIDAR with the existing controller to drive the robot forward and turn 90 degrees when there is an obstacle.

## Master
### Setup:
In the `/master_ws/src/ece387_master_spring202X-USERNAME/` folder, create a **lab3** package which depends on **rospy**, **std_msgs**, **sensor_msgs**, **geometry_msgs**, and **turtlebot3_bringup**.

Make and source your workspace.

### controller.py
1. Copy the controller.py file from lab2 into the lab3 package.

1. Open the controller.py file from lab3 using the **Atom** editor.

1. Import the laser message used in ICE8.

1. Copy the 2 lambda functions from ICE8 (RAD2DEG & DEG_CONV).

1. Add the following Class variables within the class above the `__init__()` function:

    1. `DISTANCE = 0.4 # distance from the wall to stop`
    1. `K_POS = 100 # proportional constant for slowly stopping as you get closer to the wall`
    1. `MIN_LIN_X = 0.05 # limit m/s values sent to Turtlebot3`
    1. `MAX_LIN_X = 0.2 # limit m/s values sent to Turtlebot3`
    
1. Add the following to the `__init__()` function:

    1. Instance variable, `self.avg_dist`, initialized to 0 to store the average dist off the nose.
    1. Instance variable, `self.got_avg`, initialized to False to store when an average is calculated.
    1. A subscriber to the LIDAR topic of interest with a callback to the callback_lidar() function.

1. Add the `callback_lidar()` function from ICE8, removing print statements and setting the instance variables, `self.avg_dist` and `self.got_avg`.

1. Edit the `callback_controller()` to accomplish the following:

    1. Remove user input.
    1. When not turning and you have an average LIDAR reading, calculate the distance error (`actual dist` - `desired dist`) and use that to drive your robot straight at a proportional rate (very similar to how we calculated the turn rate in lab 2).
    1. Limit the linear speed of the robot to `MIN_LIN_X` and `MAX_LIN_X`.
    1. If within `DISTANCE` of a wall, then stop and start turning (left or right, you decide).
    
    > 💡️ **Tip:** You should be able to reuse a lot of code for this step!
    
    1. Save the linear x and angular z values to the `Twist` message and publish.
    
1. Save, exit, and make executable if necessary.

## Create a launch file
1. Create a launch directory in your lab3 folder.
1. Copy the launch file from lab2 to lab3.
1. Open the **turtlebot3_lidar.launch** file from the *turtlebot3_bringup* package and copy the arguments and nodes to your lab3 launch file.
1. Add the machine tag to the lidar node.

## Run your nodes
1. On the **Master**, open a terminal and run the **lab3.launch** file

## Report
Complete a short 2-3 page report that utilizes the format and answers the questions within the report template. The report template and an example report can be found within the Team under `Resources/Lab Template`.

> 📝️ **NOTE:** We will be primarily grading sections 3.3 System level design and 3.4 Testing for this lab, but do include the entire lab as you will need other components for the final project report.

## Turn-in Requirements
**[25 points]** Demonstration of Turtlebot3 driving and not hitting a wall (preferably in person, but can be recorded and posted to Teams under the Lab3 channel).

**[50 points]** Report via Gradescope.

**[25 points]** Code: push your code to your repository. Also, include a screen shot of the **controller.py** file at the end of your report.





## ICE8: LIDAR

## Purpose
In this lesson we will enable the robot to avoid obstacles. Many sensors provide obstacle avoidance capabilities: camera, sonar, infrared, LIDAR, etc. All of these will work to enable the robot to avoid obstacles, but we will use LIDAR as it is an affordable, but very capable solution.

## LIDAR
[Robotis's LDS-01](https://www.robotis.us/360-laser-distance-sensor-lds-01-lidar/) is a 360 deg Laser Distance Sensor (LDS). It is based on laser triangulation ranging principles and uses high-speed vision acquisition and processing hardware. It measures distance data in more than 1800 times per second. It has a detection range between .12 m and 3.5 m and an angular resolution of 1 degree. The distance accuracy is .015 m between .12 m and .499 m then +/- 5% up to 3.5 m.

![logo](Figures/rplidar.png)

### Videos:
[Airborne LiDAR](https://www.youtube.com/watch?v=EYbhNSUnIdU)

[Turtlebot3 LDS](https://www.youtube.com/watch?v=9oic8aT3wIc&t)

## Quick Check on LIDAR Variant
The robots for our class have two different LIDAR variants.  The older bots have the LDS-01 which is exactly what is pictured above.  The newer bots likely have the LDS-02 (pictured below).  If you have the LDS-02, you will need to go into the .bashrc file and change the last line in the file to indicate the proper variant.  

![logo](Figures/LDS02.jpeg)

```bash
sudo nano ~/.bashrc
```
You are looking for a line that looks like this:

```bash
export LDS_MODEL=LDS-01 # replace with LDS-02 if using new LIDAR
```
You will need to change that line to LDS-02.  **You will need to accomplish this on both the master and the robot**

## Setup
The [hls_lfcd_lds_driver](http://wiki.ros.org/hls_lfcd_lds_driver) package enables data to be received from the LIDAR over the **/scan** topic. The package is pre-installed on your **Robot**, but as always, trust, but verify. Open a new secure shell into your **Robot** and run the following:

```bash
rospack find hls_lfcd_lds_driver
```

If installed, the command should return the absolute path to the package, similar to:

```bash
/opt/ros/noetic/share/hls_lfcd_lds_driver
```

If the command instead returns an error, then you need to install the package.

```bash
sudo apt install ros-noetic-hls-lfcd-lds-driver
```

## Testing LIDAR
Open a new terminal on the master and run roscore and setup for statistics:

```bash
roscore
rosparam set enable_statistics true
```

Select the terminal with the secure shell connection to your **Robot** and open the `turtlebot3_lidar.launch` file:

```
rosed turtlebot3_bringup turtlebot3_lidar.launch
```

We can see that this launch file is pretty simple and only launches the **hls_laser_publisher** node.

Run the launch file on the **Robot**:

```bash
roslaunch turtlebot3_bringup turtlebot3_lidar.launch
```

In a new terminal on the **Master**, we can visualize the Turtlebot3 and LIDAR data using another launch file from the Turtlebot3:

```bash
roslaunch turtlebot3_bringup turtlebot3_model.launch
```

This should open an RVIZ window where we can visualize ROS components of our system. In the "Displays" menu on the left you should see two submenus of interest: "LaserScan" and "RobotModel". These allow us to depict the Turtlebot3 and LIDAR data.

You should see red dots fill the **rviz** map where obstacles exist as shown below.

![logo](Figures/lidarscan_example.png)


Investigate what data the **hls_laser_publisher** is sending. Type the following and observe the command output:

```bash
rostopic list
rostopic info /scan
rostopic type /scan
rostopic type /scan | rosmsg show
rostopic echo /scan
```

At this point you can kill all nodes on the master, but keep the **turtlebot3_lidar** launch file running on the **Robot**.

## LIDAR Subscriber
In this section we will build a subscriber that will print the range data from the Turtlebot3 LIDAR.

1. Browse to a terminal on the **Master** and create an `ice8` package:
    ```bash
    cd ~/master_ws/src/ece387_master_sp2X-USERNAME/master
    catkin_create_pkg ice8 rospy sensor_msgs geometry_msgs turtlebot3_bringup
    cd ~/master_ws
    catkin_make
    source ~/.bashrc
    ```

1. Create an lidar node:

    ```bash
    roscd ice8/src
    touch lidar_sub.py
    ```
    
1. Copy and complete the below code using the GUI editor tool, **Atom**. Browse to the subscriber you just created and double-click. This will open the file in **Atom** (if it is open in any other editor, stop, raise your hand, and get help from an instructor)
> 💡️ **Tip:** Look for the **"TODO"** tag which indicates where you should insert your own code.

The code should obtain the list of range data from the LIDAR launch file running on the robot, convert the angles from 0 to 180 degrees and 0 to -180 degrees to 0 to 360 degrees. Lastly, the subscriber will print the average distance of obstacles 30 degrees off the nose of the robot.

```python
#!/usr/bin/env python3
import rospy, math
# TODO: import correct message


# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180./math.pi)
# convert LaserScan degree from -180 - 180 degs to 0 - 360 degs
DEG_CONV = lambda deg: deg + 360 if deg < 0 else deg

class LIDAR:    
    """Class to read lidar data from the Turtlebot3 LIDAR"""
    def __init__(self):
        # TODO: create a subscriber to the scan topic published by the lidar launch file

        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def callback_lidar(self, scan):
    	if not self.ctrl_c:
	    	degrees = []
	    	ranges = []
	    	
	    	# determine how many scans were taken during rotation
	        count = len(scan.ranges)

	        for i in range(count):
	            # using min angle and incr data determine curr angle, 
	            # convert to degrees, convert to 360 scale
	            degrees.append(int(DEG_CONV(RAD2DEG(scan.angle_min + scan.angle_increment*i))))
	            rng = scan.ranges[i]
	            
	            # ensure range values are valid; set to 0 if not
	            if rng < scan.range_min or rng > scan.range_max:
	                ranges.append(0.0)
	            else:
	            	ranges.append(rng)
	        
	        # python way to iterate two lists at once!
	        for deg, rng in zip(degrees, ranges):
	        	# TODO: sum and count the ranges 30 degrees off the nose of the robot
                
                
            # TODO: ensure you don't divide by 0 and print average off the nose
	        	
            
	def shutdownhook(self):
		print("Shutting down lidar subscriber")
		self.ctrl_c = True
        
if __name__ == '__main__':
    rospy.init_node('lidar_sub')
    LIDAR()
    rospy.spin()
```

4. Save, exit, and make the node executable.

4. Open a new terminal on the **Master** and run the **lidar_sub.py** node.

4. Rotate the **Robot** and observe the distance off the nose.

## Checkpoint
Once complete, get checked off by an instructor showing the output of your **lidar_sub** and **rqt_graph** node.

## Summary
In this lesson you learned how to integrate the LIDAR and get the distance of objects off the nose of the robot using the pre-built LIDAR package. In the lab that corresponds to this lesson you will apply this knowledge to stop the robot a specified distance from an obstacle and turn.

## Cleanup
In each terminal window, close the node by typing `ctrl+c`. Exit any SSH connections. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.

**Ensure roscore is terminated before moving on to the lab.**
