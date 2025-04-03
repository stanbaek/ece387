# 🔬 Lab10: CV


## 📌 Objectives

- Students should be able to explain the concept of Histogram of Oriented Gradients (HOG) and its role in object detection.
- Students should be able to train a custom object detector using Dlib and analyze its performance.
- Students should be able to implement and test a trained object detector on new images.
- Students should be able to create a ROS 2 package for computer vision tasks and integrate OpenCV for image processing.


## 📜 Overview

In this lab, we explore how the Histogram of Oriented Gradients (HOG) features, combined with a Support Vector Machine (SVM), enable object detection. By now, we’re all familiar with histograms, but in this context, they help simplify an image so a computer can quickly and accurately identify objects within it.

Rather than analyzing the gradient direction of every single pixel, HOG groups pixels into small cells. Within each cell, the gradient directions are computed and categorized into orientation bins, with stronger gradients carrying more weight. This approach helps reduce the influence of random noise and provides a structured representation of the image. HOG features maintain the distinct shape of an object while allowing for slight variations. For instance, consider an object detector designed to recognize a car:

```{image} ./figures/Lab10_HOG_Features.jpg  
:width: 600  
:align: center  
```  
<br>

Comparing each individual pixel of this training image with another test image would not only be time consuming, but it would also be highly subject to noise.  As previously mentioned, the HOG feature will consider a block of pixels.  The size of this block is variable and will naturally impact both accuracy and speed of execution for the algorithm.  Once the block size is determined, the gradient for each pixel within the block is computed.  Once the gradients are computed for a block, the entire cell can then be represented by this histogram.  Not only does this reduce the amount of data to compare with a test image, but it also reduces the impacts of noise in the image and measurements.  

```{image} ./figures/Lab10_HOG_Histogram.jpg
:width: 500  
:align: center  
```  
<br>

Now that we understand HOG features, let’s leverage OpenCV and Dlib to build a stop sign detector. First, we need to download a repository containing pre-made test and training data. Keep in mind that we won’t evaluate the algorithm’s effectiveness using the training data—it’s expected to perform well there. Instead, our goal is to use a diverse test set to develop a detector robust enough to recognize stop signs in new images.  

## 🌱 Pre-Lab: ROS2 Client Libraries  

1. Create a package named `lab10_cv` with the `BSD-3-Clause` license and dependencies:
    - `rclpy`
    - `cv_bridge`
    - `sensor_msgs`
    - `std_msgs`
    - `opencv2`

    _Hint: There’s a way to include all dependencies at the time of package creation._

1. Download the [`lab10_prelab.tar.xz`](../files/lab10_prelab.tar.xz). Extract the files and move them to `~/master_ws/src/ece387_ws/lab10_cv/test`. 

1. Open the Jupyter Notebook file, `lab10_prelab.ipynb` with vscode. 

1. Click `Select Kernel` in the top right corner, choose `Python Environments`, and select `/usr/bin/python3`.  

1. As you read through the notebook, run the python code by clicking the arrow button in the top left corner. 

1. If the following message pops up, choose `Install`

    ```{image} ./figures/Lab10_ipykernel.png
    :width: 450  
    :align: center  
    ```  
    <br>
1. Take a screenshot of the gradient image and submit it on Gradescope.

## 🛠️ Lab Procedures

### **1. Building a detector using HOG features**

1. Download the example demo.

    ```bash
    cd ~/master_ws/src
    git clone git@github.com:ECE495/HOG_Demo.git
    cd HOG_Demo
    ```

1. Take a look at what is contained within the repo.  Essentially you have both a training data folder and a test folder.  We will now use a tool called **imglab** to annotate the images for building our detector.

1. Browse to the [imglab tool](https://solothought.com/imglab/#) and select **"UMM, MAYBE NEXT TIME!"**.

1. In the bottom left of the site, click on the `load` button, select the `training` folder, and click the `upload` button. It will upload 19 files.

```{image} ./figures/Lab10_Load.png
:width: 150  
:align: center  
```  
<br>

1. Select the first stop sign and the **"Rectangle"** tool. 

    ```{image} ./figures/Lab10_Rectangle.png
    :width: 50  
    :align: center  
    ```  
    <br>

1. Highlight the border of the stop sign: drag-and-draw a bounding rectangle, ensuring to **only** select the stop sign and to select all examples of the object in the image.

    > 📝️ **NOTE:** It is important to label all examples of objects in an image; otherwise, Dlib will implicitly assume that regions not labeled are regions that should not be detected (i.e., hard-negative mining applied during extraction time).

1. You can select a bounding box and hit the delete key to remove it.

1. If you press `alt+left/right arrow` you can navigate through images in the slider and repeat highlighting the objects.

1. Once all stop signs are complete hit `ctrl+e` to save the annotations (bounding box information) as a **"Dlib XML"** file within the `training` folder using a descriptive name such as `stop_annotations.xml`.

    ```{image} ./figures/Lab10_Dlib.png
    :width: 200  
    :align: center  
    ```  

1. We now need to create the code to build the detector based on our annotated training data.

    ```bash
    cd ~/master_ws/src/HOG_Demo
    touch trainDetector.py
    ```

1. Now open this in your favorite editor to add the following code.  I have built into the code the ability to provide command line arguments.  This will make the code a bit more flexible such that you don't need to recreate it in the future if you want to reuse if for another project.  You will provide two arguments at runtime.  First you need to tell the program where the .xml file is.  Second, you will state where you want to put the detector that you create... the detector should have a .svm extension.

    ```python
    # import the necessary packages
    from __future__ import print_function
    import argparse
    import dlib

    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-x", "--xml", required=True, help="path to input XML file")
    ap.add_argument("-d", "--detector", required=True, help="path to output detector")
    args = vars(ap.parse_args())

    # grab the default training options for the HOG + Linear SVM detector, then
    # train the detector -- in practice, the `C` parameter can be adjusted...
    # feel free to research and see if you can improve
    print("[INFO] training detector...")
    options = dlib.simple_object_detector_training_options()
    options.C = 1.0
    options.num_threads = 4
    options.be_verbose = True
    dlib.train_simple_object_detector(args["xml"], args["detector"], options)

    # show the training accuracy
    print("[INFO] training accuracy: {}".format(
        dlib.test_simple_object_detector(args["xml"], args["detector"])))
        
    # load the detector and visualize the HOG filter
    detector = dlib.simple_object_detector(args["detector"])
    win = dlib.image_window()
    win.set_image(detector)
    dlib.hit_enter_to_continue()
    ```

1. Once you have the code entered, you can run it with the following command.  Remember, you need to provide two command line arguments:

    ```bash
    cd ~/master_ws/src/HOG_Demo
    python3 trainDetector.py --xml training/stop_annotations.xml --detector training/stop_detector.svm
    ```

1. You may get a few errors pop up during execution based on your choice for bounding boxes.  Make sure you address those errors before continuing.  If everything executed correctly, you should ultimately see a picture of the HOG feature you designed.  

### **2. Testing a detector**
1. Now it is time to build our code to test the detector.  The following code will make use of the imutils library as well.

1. You may get a few errors pop up during execution based on your choice for bounding boxes.  Make sure you address those errors before continuing.  If everything executed correctly, you should ultimately see a picture of the HOG feature you designed.  

1. Now it is time to build our code to test the detector.

    ```bash
    cd ~/master_ws/src/HOG_Demo
    touch testDetector.py
    ```

1. Enter the code below: 

    ```python
    # import the necessary packages
    from imutils import paths
    import argparse
    import dlib
    import cv2

    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--detector", required=True, help="Path to trained object detector")
    ap.add_argument("-t", "--testing", required=True, help="Path to directory of testing images")
    args = vars(ap.parse_args())

    # load the detector
    detector = dlib.simple_object_detector(args["detector"])

    # loop over the testing images
    for testingPath in paths.list_images(args["testing"]):
        # load the image and make predictions
        image = cv2.imread(testingPath)
        boxes = detector(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        
        # loop over the bounding boxes and draw them
        for b in boxes:
            (x, y, w, h) = (b.left(), b.top(), b.right(), b.bottom())
            cv2.rectangle(image, (x, y), (w, h), (0, 255, 0), 2)
            
        # show the image
        cv2.imshow("Image", image)
        cv2.waitKey(0)
    ```

1. Run the test detector:

    ```bash
    cd ~/master_ws/src/HOG_Demo
    python3 testDetector.py --detector training/stop_detector.svm --testing test
    ```

1. OK, so how did you do? What surprises did you have? What might you consider to improve the detector?


## Not Ready Yet


sudo apt install python3-pip
sudo apt install ros-humble-usb-cam
pip3 install pydantic
pip3 install "pydantic<2"

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480




ubuntu@robot99:~
$ ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480
[INFO] [1743632020.387985164] [usb_cam]: camera_name value: default_cam
[WARN] [1743632020.388376164] [usb_cam]: framerate: 30.000000
[INFO] [1743632020.392737645] [usb_cam]: using default calibration URL
[INFO] [1743632020.392877460] [usb_cam]: camera calibration URL: file:///home/ubuntu/.ros/camera_info/default_cam.yaml
[ERROR] [1743632020.393096812] [camera_calibration_parsers]: Unable to open camera calibration file [/home/ubuntu/.ros/camera_info/default_cam.yaml]
[WARN] [1743632020.393148516] [usb_cam]: Camera calibration file /home/ubuntu/.ros/camera_info/default_cam.yaml not found
[INFO] [1743632025.477989457] [usb_cam]: Starting 'default_cam' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS
[swscaler @ 0xaaaab3267a90] No accelerated colorspace conversion found from yuv422p to rgb24.
This device supports the following formats:
	Motion-JPEG 1280 x 720 (30 Hz)
	Motion-JPEG 1280 x 720 (20 Hz)
	Motion-JPEG 1280 x 720 (15 Hz)
	Motion-JPEG 1280 x 720 (10 Hz)
	Motion-JPEG 1280 x 720 (5 Hz)
	Motion-JPEG 1280 x 720 (30 Hz)
	Motion-JPEG 1280 x 720 (20 Hz)
	Motion-JPEG 1280 x 720 (15 Hz)
	Motion-JPEG 1280 x 720 (10 Hz)
	Motion-JPEG 1280 x 720 (5 Hz)
	Motion-JPEG 1920 x 1080 (30 Hz)
	Motion-JPEG 1920 x 1080 (20 Hz)
	Motion-JPEG 1920 x 1080 (15 Hz)
	Motion-JPEG 1920 x 1080 (10 Hz)
	Motion-JPEG 1920 x 1080 (5 Hz)
	Motion-JPEG 960 x 540 (30 Hz)
	Motion-JPEG 960 x 540 (20 Hz)
	Motion-JPEG 960 x 540 (15 Hz)
	Motion-JPEG 960 x 540 (10 Hz)
	Motion-JPEG 960 x 540 (5 Hz)
	Motion-JPEG 800 x 600 (30 Hz)
	Motion-JPEG 800 x 600 (20 Hz)
	Motion-JPEG 800 x 600 (15 Hz)
	Motion-JPEG 800 x 600 (10 Hz)
	Motion-JPEG 800 x 600 (5 Hz)
	Motion-JPEG 640 x 480 (30 Hz)
	Motion-JPEG 640 x 480 (20 Hz)
	Motion-JPEG 640 x 480 (15 Hz)
	Motion-JPEG 640 x 480 (10 Hz)
	Motion-JPEG 640 x 480 (5 Hz)
	Motion-JPEG 320 x 240 (30 Hz)
	Motion-JPEG 320 x 240 (20 Hz)
	Motion-JPEG 320 x 240 (15 Hz)
	Motion-JPEG 320 x 240 (10 Hz)
	Motion-JPEG 320 x 240 (5 Hz)
	Motion-JPEG 320 x 180 (30 Hz)
	Motion-JPEG 320 x 180 (20 Hz)
	Motion-JPEG 320 x 180 (15 Hz)
	Motion-JPEG 320 x 180 (10 Hz)
	Motion-JPEG 320 x 180 (5 Hz)
	Motion-JPEG 176 x 144 (30 Hz)
	Motion-JPEG 176 x 144 (20 Hz)
	Motion-JPEG 176 x 144 (15 Hz)
	Motion-JPEG 176 x 144 (10 Hz)
	Motion-JPEG 176 x 144 (5 Hz)
	Motion-JPEG 160 x 120 (30 Hz)
	Motion-JPEG 160 x 120 (20 Hz)
	Motion-JPEG 160 x 120 (15 Hz)
	Motion-JPEG 160 x 120 (10 Hz)
	Motion-JPEG 160 x 120 (5 Hz)
	Motion-JPEG 1280 x 720 (30 Hz)
	Motion-JPEG 1280 x 720 (20 Hz)
	Motion-JPEG 1280 x 720 (15 Hz)
	Motion-JPEG 1280 x 720 (10 Hz)
	Motion-JPEG 1280 x 720 (5 Hz)
	Motion-JPEG 1280 x 720 (30 Hz)
	Motion-JPEG 1280 x 720 (20 Hz)
	Motion-JPEG 1280 x 720 (15 Hz)
	Motion-JPEG 1280 x 720 (10 Hz)
	Motion-JPEG 1280 x 720 (5 Hz)
	YUYV 4:2:2 1920 x 1080 (5 Hz)
	YUYV 4:2:2 1920 x 1080 (5 Hz)
	YUYV 4:2:2 1280 x 720 (5 Hz)
	YUYV 4:2:2 640 x 480 (30 Hz)
	YUYV 4:2:2 640 x 480 (20 Hz)
	YUYV 4:2:2 640 x 480 (15 Hz)
	YUYV 4:2:2 640 x 480 (10 Hz)
	YUYV 4:2:2 640 x 480 (5 Hz)
	YUYV 4:2:2 320 x 240 (30 Hz)
	YUYV 4:2:2 320 x 240 (20 Hz)
	YUYV 4:2:2 320 x 240 (15 Hz)
	YUYV 4:2:2 320 x 240 (10 Hz)
	YUYV 4:2:2 320 x 240 (5 Hz)
	YUYV 4:2:2 1920 x 1080 (5 Hz)
	YUYV 4:2:2 1920 x 1080 (5 Hz)
terminate called after throwing an instance of 'char*'
[ros2run]: Aborted
ubuntu@robot99:~
$ ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480
[INFO] [1743632090.756654622] [usb_cam]: camera_name value: default_cam
[WARN] [1743632090.756920715] [usb_cam]: framerate: 30.000000
[INFO] [1743632090.763897770] [usb_cam]: using default calibration URL
[INFO] [1743632090.764034159] [usb_cam]: camera calibration URL: file:///home/ubuntu/.ros/camera_info/default_cam.yaml
[ERROR] [1743632090.764264585] [camera_calibration_parsers]: Unable to open camera calibration file [/home/ubuntu/.ros/camera_info/default_cam.yaml]
[WARN] [1743632090.764336696] [usb_cam]: Camera calibration file /home/ubuntu/.ros/camera_info/default_cam.yaml not found
[INFO] [1743632137.936049538] [usb_cam]: Starting 'default_cam' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS
[swscaler @ 0xaaaaf2569080] No accelerated colorspace conversion found from yuv422p to rgb24.
This device supports the following formats:
	Motion-JPEG 1280 x 720 (30 Hz)
	Motion-JPEG 1280 x 720 (20 Hz)
	Motion-JPEG 1280 x 720 (15 Hz)
	Motion-JPEG 1280 x 720 (10 Hz)
	Motion-JPEG 1280 x 720 (5 Hz)
	Motion-JPEG 1280 x 720 (30 Hz)
	Motion-JPEG 1280 x 720 (20 Hz)
	Motion-JPEG 1280 x 720 (15 Hz)
	Motion-JPEG 1280 x 720 (10 Hz)
	Motion-JPEG 1280 x 720 (5 Hz)
	Motion-JPEG 1920 x 1080 (30 Hz)
	Motion-JPEG 1920 x 1080 (20 Hz)
	Motion-JPEG 1920 x 1080 (15 Hz)
	Motion-JPEG 1920 x 1080 (10 Hz)
	Motion-JPEG 1920 x 1080 (5 Hz)
	Motion-JPEG 960 x 540 (30 Hz)
	Motion-JPEG 960 x 540 (20 Hz)
	Motion-JPEG 960 x 540 (15 Hz)
	Motion-JPEG 960 x 540 (10 Hz)
	Motion-JPEG 960 x 540 (5 Hz)
	Motion-JPEG 800 x 600 (30 Hz)
	Motion-JPEG 800 x 600 (20 Hz)
	Motion-JPEG 800 x 600 (15 Hz)
	Motion-JPEG 800 x 600 (10 Hz)
	Motion-JPEG 800 x 600 (5 Hz)
	Motion-JPEG 640 x 480 (30 Hz)
	Motion-JPEG 640 x 480 (20 Hz)
	Motion-JPEG 640 x 480 (15 Hz)
	Motion-JPEG 640 x 480 (10 Hz)
	Motion-JPEG 640 x 480 (5 Hz)
	Motion-JPEG 320 x 240 (30 Hz)
	Motion-JPEG 320 x 240 (20 Hz)
	Motion-JPEG 320 x 240 (15 Hz)
	Motion-JPEG 320 x 240 (10 Hz)
	Motion-JPEG 320 x 240 (5 Hz)
	Motion-JPEG 320 x 180 (30 Hz)
	Motion-JPEG 320 x 180 (20 Hz)
	Motion-JPEG 320 x 180 (15 Hz)
	Motion-JPEG 320 x 180 (10 Hz)
	Motion-JPEG 320 x 180 (5 Hz)
	Motion-JPEG 176 x 144 (30 Hz)
	Motion-JPEG 176 x 144 (20 Hz)
	Motion-JPEG 176 x 144 (15 Hz)
	Motion-JPEG 176 x 144 (10 Hz)
	Motion-JPEG 176 x 144 (5 Hz)
	Motion-JPEG 160 x 120 (30 Hz)
	Motion-JPEG 160 x 120 (20 Hz)
	Motion-JPEG 160 x 120 (15 Hz)
	Motion-JPEG 160 x 120 (10 Hz)
	Motion-JPEG 160 x 120 (5 Hz)
	Motion-JPEG 1280 x 720 (30 Hz)
	Motion-JPEG 1280 x 720 (20 Hz)
	Motion-JPEG 1280 x 720 (15 Hz)
	Motion-JPEG 1280 x 720 (10 Hz)
	Motion-JPEG 1280 x 720 (5 Hz)
	Motion-JPEG 1280 x 720 (30 Hz)
	Motion-JPEG 1280 x 720 (20 Hz)
	Motion-JPEG 1280 x 720 (15 Hz)
	Motion-JPEG 1280 x 720 (10 Hz)
	Motion-JPEG 1280 x 720 (5 Hz)
	YUYV 4:2:2 1920 x 1080 (5 Hz)
	YUYV 4:2:2 1920 x 1080 (5 Hz)
	YUYV 4:2:2 1280 x 720 (5 Hz)
	YUYV 4:2:2 640 x 480 (30 Hz)
	YUYV 4:2:2 640 x 480 (20 Hz)
	YUYV 4:2:2 640 x 480 (15 Hz)
	YUYV 4:2:2 640 x 480 (10 Hz)
	YUYV 4:2:2 640 x 480 (5 Hz)
	YUYV 4:2:2 320 x 240 (30 Hz)
	YUYV 4:2:2 320 x 240 (20 Hz)
	YUYV 4:2:2 320 x 240 (15 Hz)
	YUYV 4:2:2 320 x 240 (10 Hz)
	YUYV 4:2:2 320 x 240 (5 Hz)
	YUYV 4:2:2 1920 x 1080 (5 Hz)
	YUYV 4:2:2 1920 x 1080 (5 Hz)
[INFO] [1743632137.963216446] [usb_cam]: Setting 'brightness' to 50
unknown control 'white_balance_temperature_auto'

[INFO] [1743632137.996255020] [usb_cam]: Setting 'white_balance_temperature_auto' to 1
[INFO] [1743632137.996413390] [usb_cam]: Setting 'exposure_auto' to 3
unknown control 'exposure_auto'

[INFO] [1743632138.010602075] [usb_cam]: Setting 'focus_auto' to 0
unknown control 'focus_auto'

[INFO] [1743632138.234247168] [usb_cam]: Timer triggering every 33 ms






<!--
### **3. ROS and Image Capture**
ROS provides a number of tools to interact with a commercial-off-the-shelf camera such as the USB camera connected to your robot. The primary tool we will use is the [usb_cam](http://wiki.ros.org/usb_cam) package which is already installed on your robot.

Let's create a **lab4** package on the **Master** we can use to start developing a launch file to run our computer vision tools.

In a terminal create a **lab4** package, `launch` folder, and `lab4.launch` file:

```bash
cd ~/master_ws/src/ece387_ws/lab10_cv/
catkin_create_pkg lab4 rospy sensor_msgs std_msgs cv_bridge apriltag_ros
cd lab4
mkdir launch
cd launch
touch lab4.launch
```

Make and source your workspace.

### Launch File - USB Cam

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'usb_cam',
                'io_method': 'mmap'
            }]
        )
    ])
```

### **1. Create a New ROS 2 Package**
Navigate to your ROS 2 workspace and create a package for the launch file:

```bash
cd ~/ros2_ws/src
ros2 pkg create usb_cam_launch --build-type ament_python
```

### **2. Move the Launch File into the Package**
Place your `usb_cam_launch.py` inside the `launch` directory of the package:

```bash
mkdir -p ~/ros2_ws/src/usb_cam_launch/launch
mv usb_cam_launch.py ~/ros2_ws/src/usb_cam_launch/launch/
```

### **3. Modify `package.xml` and `setup.py`**
Ensure `package.xml` includes dependencies for `launch_ros`. Also, update `setup.py` to register the launch file:

Modify `setup.py`:
```python
import os
from glob import glob
from setuptools import setup

package_name = 'usb_cam_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email',
    description='Launch file for USB camera in ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [],
    },
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
)
```


```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
```


### **4. Build and Source the Package**
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### **5. Run the Launch File**
Now you can launch the file using:

```bash
ros2 launch usb_cam_launch usb_cam_launch.py
```

-->




<!--

Edit the `lab4.launch` file to call the **usb_cam_node** on the robot which will automatically connect to the camera and publish the video over a topic.

```xml
<launch>

    <node machine="robot0" name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
        <param name="video_device" value="/dev/video0" />
        <param name="image_width" value="640" />
        <param name="image_height" value="480" />
        <param name="pixel_format" value="yuyv" />
        <param name="camera_frame_id" value="usb_cam" />
        <param name="io_method" value="mmap"/>
  </node>
    
</launch>
```

Save and exit.

Ensure **roscore** is running on the **Master**.

Run the **usb_cam** node on the **Robot** using the **lab4** launch file.

Open a terminal on the **Master** and view the topics created by the node.

The primary topic we will look at is */usb_cam/image_raw*. What type of message is sent over this topic? Take note as we will use this in the lab!

Let's display the video using the **image_view** tool on the **Master**.

```bash
rostopic list
rosrun rqt_image_view rqt_image_view
```
Ensure the `/usb_cam/image_raw` topic is selected.


<!--
# Lab 4: Computer Vision


## Purpose
This lab will integrate a USB Camera with the Robot. You will use a Python script to take pictures of the stop sign and build a stop sign detector then test it using a live video feed. You will then use the detector and known size of the stop sign to estimate how far the stop sign is from the camera. Lastly, you will create a node to identify and determine how far an April Tag is from the robot.

## Setup packages
Open a terminal on the **Robot** and create a lab4 package:

```bash
cd ~/master_ws/src/ece387_robot_spring202X-USERNAME/
catkin_create_pkg lab4 rospy sensor_msgs std_msgs cv_bridge apriltag_ros
```

Make and source your workspace.


## Create a ROS node to save images
Browse to your lab4 source folder on the **Master** and create a node called **image_capture.py**.

```python
#!/usr/bin/env python3
import rospy, cv2, argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class SavingImage(object):
    def __init__(self, img_dest):
        self.img_dest = img_dest
        self.ctrl_c = False
        self.count = 0
        
        # subscribe to the topic created by the usb_cam node
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback)
        
        # CV bridge converts between ROS Image messages and OpenCV images
        self.bridge_object = CvBridge()
        
        # callback to save images when user presses button
        rospy.Timer(rospy.Duration(.1), self.callback_save)
        
        rospy.on_shutdown(self.shutdownhook)

    def camera_callback(self, img):
        if not self.ctrl_c:
            try:
                # convert ROS image to OpenCV image
                self.cv_image = self.bridge_object.imgmsg_to_cv2(img, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)
            
            # show the image (waitKey(1) allows for automatic refressh creating video)
            cv2.imshow('image', self.cv_image)
            cv2.waitKey(1)
        
    def callback_save(self, event):
        # when user is ready to take picture press button
        _ = input("Press enter to save the image.")
        dest = self.img_dest + "img" + str(self.count) + ".jpg"
        self.count += 1
        print(dest)
        try:
            # write to file
            cv2.imwrite(dest, self.cv_image)
        except:
            print("Not valid image name. Try restarting with valid path.")
            
    def shutdownhook(self):
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('image_saver', anonymous=True)
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", required=True, help="path to output img")
    args = vars(ap.parse_args())
    saving_image_object = SavingImage(args["output"])
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
```

Save, exit, and make executable.

## Train your stop detector

- Create a new folder in your **lab4** package called **training_images**.
- Run the `image_capture.py` node on the **Master** using the following command:

```{note}
You must have the `lab4.launch` file running.
```

```bash
rosrun lab4 image_capture.py -o /home/dfec/master_ws/src/ece387_master_spring202X-NAME/lab4/training_images/
```

Store images of the stop sign by pressing `enter` when prompted. You decide how many and at what orientations to properly train your detector. When complete, hit `ctrl+c` to exit.

Utilize the steps from Module 9: [Building a detector using HOG features](CV:HOG) to label your images and train your object detector using the new images, saving the `stop_detector.svm` file within the **training_images** folder.

## Test your stop detector
Create a node in the **lab4** package on the **Master** called `stop_detector.py` and copy the below into it:

```python
#!/usr/bin/env python3
import rospy, cv2, dlib
from cv_bridge import CvBridge, CvBridgeError

# TODO: import usb_cam message type


class StopDetector(object):

    def __init__(self, detectorLoc):
        self.ctrl_c = False
        
        #TODO: create subscriber to usb_cam image topic

        
        self.bridge_object = CvBridge()
        self.detector = dlib.simple_object_detector(detectorLoc)
        
        rospy.on_shutdown(self.shutdownhook)
        
    def camera_callback(self,data):
        if not self.ctrl_c:
            #TODO: write code to get ROS image, convert to OpenCV image,
            # apply detector, add boxes to image, and display image
            

    def shutdownhook(self):
        print("Shutting down")
        self.ctrl_c = True
        cv2.destroyAllWindows()
        
if __name__ == '__main__':
    rospy.init_node('stop_detector')
    detector = rospy.get_param("/stop_detector/detector")
    stop_detector = StopDetector(detector)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
```

Edit the `stop_detector.py` node so it utilizes the `camera_callback()` function we used above to get images from the camera.

After getting the `cv_image` within the `camera_callback()`, apply the detector in a similar method as Module 9: [Testing a detector](CV:HOG) creating boxes around all detected stop signs. Using a `waitKey(1)` will allow for the image to refresh automatically without user input and display the video.

## Checkpoint 1
Demonstrate the stop detector on the **Master** detecting a stop sign from the **Robot's** camera.

```bash
rosrun lab4 stop_detector.py _detector:=/home/dfec/master_ws/src/ece387_master_spring202X-NAME/lab4/training_images/stop_detector.svm
```

```{note}
You must have the `lab4.launch` file running.
```

## Move detector to robot
Copy the detector and node to the robot:

```bash
roscd lab4/training_images
scp stop_detector.svm pi@robotX:/home/pi/robot_ws/src/ece387_robot_spring202X-NAME/lab4/training_images/stop_detector.svm
roscd lab4/src
scp stop_detector.py pi@robotX:/home/pi/robot_ws/src/ece387_robot_spring202X-NAME/lab4/src/stop_detector.py
```

Remove the lines that display the video and instead print "Stop detected" if `boxes` is not empty.

Do you note a difference in processing speed?

## Launch file
Edit the `lab4.launch` file so it will run the stop detector node with the `detector` param set to the location of the detector. For example:

```xml
<node machine="robotX" name="stop_detector" pkg="lab4" type="stop_detector.py" output="screen">
    <param name="detector" value="/home/pi/robot_ws/src/ece387_robot_spring202X-Name/robot/lab4/training_images/stop_detector.svm"/>
</node>
```

## Checkpoint 2
Demonstrate the stop detector on the **Robot** detecting a stop sign.

## Determine distance from stop sign

### Edit `stop_detector.py`

You will edit your stop sign detector on the **Robot** to calculate an estimated distance between the camera and the stop sign using triangle similarity. 

Given a stop sign with a known width, $W$, we can place the stop sign at a known distance, $D$, from our camera. The detector will then detect the stop sign and provide a perceived width in pixels, $P$. Using these values we can calculate the focal length, $F$ of our camera:

$F = \frac{(P\times D)}{W}$

We can then use the calculated focal length, $F$, known width, $W$, and perceived width in pixels, $P$ to calculate the distance from the camera:

$D' = \frac{(W\times F)}{P}$

Use the above information and create two class variables, `FOCAL` and `STOP_WIDTH`, and a class function to calculate distance given a known `FOCAL` length and a known width of the stop sign, `STOP_WIDTH`. You will need to print the perceived width of the stop sign to determine the $P$ value used in the calculation to find the focal length.

> 💡️ **Tip:** Pay attention to what the `x` and `w` variables of the `box` actually represent!

Create a new publisher that will publish the distance using **Float32** *std_msgs* messages over the */stop_dist* topic.

Publish the distance of each object seen in the image. 

Remove any print statements after troubleshooting!

## Checkpoint 3
Demonstrate the **stop_detector** node publishing distance from the stop sign.

## Printing April Tag information

Create a node on the master in lab4 called `apriltag_dist.py`. Import the appropriate AprilTag message. Subscribe to the `tag_detections` topic. Print the identified AprilTag ID and distance. If the camera sees multiple tags, it should print the information for each tag.

In your callback function you will want to create a for loop such as:

```python
for tag in data.detections:
```

Use print statements to determine the characteristics of the message (you can also google the message).

Add the `apriltag_dist` node to the **lab4** launch file.

## Checkpoint 4

Demonstrate the `apriltag_dist` node printing the ID and distance of each April Tag.

## Report
Complete a short 2-3 page report that utilizes the format and answers the questions within the report template. The report template and an example report can be found within the Team under `Resources/Lab Template`.

> 📝️ **Note:** We will be primarily grading sections 3.1, 3.2, and 3.3 for this lab, but do include the entire lab as you will need other components for the final project report.

## Turn-in Requirements
**[25 points]** All checkpoints marked off.

**[50 points]** Report via Gradescope.

**[25 points]** Code: push your code to your repository. Also, include a screen shot of the **apriltag_dist.py** and **stop_detector.py** files at the end of your report.

-->