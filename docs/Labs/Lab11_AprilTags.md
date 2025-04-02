# 🔬 Lab11: AprilTags


### Calibrate USB Camera

A camera must first be calibrated to utilize computer vision based tasks. Otherwise, there is no reference for how large objects are in regards to the camera frame. The [ROS Calibration Tool](http://wiki.ros.org/camera_calibration) creates a calibration file that is then used by other ROS packages to enable size and distance calculations. The **camera_calibration** package utilizes OpenCV camera calibration to allow easy calibration of monocular or stereo cameras using a checkerboard calibration target. The complete guide can be found on the [Camera Calibration Tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).

Connect to the camera using the **usb_cam** node:

```bash
roslaunch lab4 lab4.launch
```

Run the camera calibrate package with the correct parameters (even though the checkerboard says it is a 9x6 board with 3.0 cm squares it is actually a 8x5 board with 2.7 cm squares - the size the calibration tool uses is actually the interior vertex points, not the squares).

Open a new terminal on the **Master** and run the folowing:

```bash
rosrun camera_calibration cameracalibrator.py --size 8x5 --square 0.027 image:=/usb_cam/image_raw camera:=/usb_cam
```
You should see a window open that looks like this:
![logo](Figures/callibration.png)

In order to get a good calibration you will need to move the checkerboard around in the camera frame such that:

- checkerboard on the camera's left, right, top and bottom of field of view
    - X bar - left/right in field of view
    - Y bar - top/bottom in field of view
    - Size bar - toward/away and tilt from the 

- checkerboard filling the whole field of view
- checkerboard tilted to the left, right, top and bottom (Skew)


As you move the checkerboard around you will see four bars on the calibration sidebar increase in length. 

When the CALIBRATE button lights, you have enough data for calibration and can click CALIBRATE to see the results. Calibration can take a couple minutes. The windows might be greyed out but just wait, it is working.

![logo](Figures/callibrate.png)

When complete, select the save button and then commit.

Browse to the location of the calibration data, extract, and move to the appropriate ROS folder on the robot:

```bash
cd /tmp
tar xf calibrationdata.tar.gz
scp ost.yaml pi@robotX:/home/pi/.ros/camera_info/head_camera.yaml
```

Kill the `lab4.launch`. 

Create a secure shell to the robot and edit the calibration data and replace "narrow\_stero" with "head\_camera":

```bash
ssh pi@robotX
nano /home/pi/.ros/camera_info/head_camera.yaml
```

Rerun the `lab4.launch` file on the robot. You should see the camera feed reopen and see no errors in the command line (you may need to unplug and plug your camera back in).

### Checkpoint
Show an instructor the working camera feed and that the **usb_cam** node was able to open the camera calibration file.

### Summary
You now are able to connect to a USB camera using ROS, display the image provided by the node, and have a calibration file that ROS can use to identify the size of objects in the frame.

### Cleanup
Kill all rosnodes and roscore!

## Part 6: Fiducial Markers

In this lesson we will learn how fiducial markers are used in image processing. Specifically, we will utilize ROS tools to identify different [April Tags](https://april.eecs.umich.edu/papers/details.php?name=olson2011tags) and use the 3D position and orientation to determine the robot's distance from an object.

A fiducial marker is an artificial feature used in creating controllable experiments, ground truthing, and in simplifying the development of systems where perception is not the central objective. A few examples of fiducial markers include ArUco Markers, AprilTags, and QR codes. Each of these different tags hold information such as an ID or, in the case of QR codes, websites, messages, and etc. We will primarily be focusing on AprilTags as there is a very robust ROS package already built. This library identifies AprilTags and will provide information about the tags size, distance, and orientation.

### AprilTag ROS
Browse to the AprilTag_ROS package on the **Master** and edit the config file:

```bash
roscd apriltag_ros/config
sudo nano tags.yaml
```

This is where you provide the package with information about the tags it should identify. You should have gotten tags 0-3. Each of these tags is $.165 m$ wide and should have a corresponding name: "tag_0" (in the final project, you might want to change these names as we will be providing you commands that correspond to each tag). In the `tags.yaml` file, add a line for each tag under "standalone tags" (replace ... with last two tags):

```python
standalone_tags:
  [
  	{id: 0, size: .165, name: tag_0},
  	{id: 1, size: .165, name: tag_1},
  	...
  ]
```

Repeat these steps on the **Robot**.

### Launch File - Apriltag_Ros

Edit the `lab4.launch` file on the **Master**, calling the `continuous_detection` node provided by the **apriltag_ros** package. We need to set the arguments to the values provided by the `usb_cam` node:

Add the following arguments and parameters to the top of the launch file:

```xml
<arg name="launch_prefix" default="" />
<arg name="node_namespace" default="apriltag_ros_continuous_node" />
<arg name="camera_name" default="/usb_cam" />
<arg name="image_topic" default="image_raw" />

<!-- Set parameters -->
<rosparam command="load" file="$(find apriltag_ros)/config/settings.yaml" ns="$(arg node_namespace)" />
<rosparam command="load" file="$(find apriltag_ros)/config/tags.yaml" ns="$(arg node_namespace)" />
```

Add the apriltag node in the remote section:


```xml
<!-- apriltag_ros -->
<node machine="robot0" pkg="apriltag_ros" type="apriltag_ros_continuous_node" name="$(arg node_namespace)" clear_params="true" output="screen" launch-prefix="$(arg launch_prefix)" >
<!-- Remap topics from those used in code to those on the ROS network -->
<remap from="image_rect" to="$(arg camera_name)/$(arg image_topic)" />
<remap from="camera_info" to="$(arg camera_name)/camera_info" />

<param name="publish_tag_detections_image" type="bool" value="true" />      <!-- default: false -->
</node>

```

Save and exit.

Launch the `lab4.launch` file. 

In a terminal on the master open the **rqt_image_view** node (`rosrun rqt_image_view rqt_image_view`) and select the *tag_detections_image* topic. If you hold up each tag, you should see a yellow box highlight the tag with an id in the middle of the tag.

In another terminal on the master echo the topic `tag_detections`. What information do you see? Will the apriltag_ros node identify only one tag at a time? Which value do you think we would use to determine distance from the tag? What kind of message is this? What package does this message come from?

### Checkpoint
Show an instructor that the **apriltag_ros** can identify tags and provides position data.

### Summary
You now have the ability to identify AprilTags and because you have a calibrated camera, you can detect the size, orientation, and distance of a tag.

### Cleanup
Kill all rosnodes and roscore!
