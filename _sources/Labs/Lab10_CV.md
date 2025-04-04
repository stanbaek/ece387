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

---

## 🧪 Lab Instructions: Object Detection Using HOG and ROS

In this lab, you’ll build a stop sign detector using HOG (Histogram of Oriented Gradients) features, test it, and integrate it with a ROS-based camera system.

(CV:HOG)=
### **1. Build a Detector Using HOG Features**

1. Open a terminal and download the demo repository:

    ```bash
    cd ~/master_ws/src
    git clone git@github.com:ECE495/HOG_Demo.git
    cd HOG_Demo
    ```

1. Inside the repo, you'll find folders for training and test images. We'll use a tool called `imglab` to annotate the training images.

1. Go to [imglab](https://solothought.com/imglab/#) in your browser.

1. When prompted, click **"UMM, MAYBE NEXT TIME!"** to skip the sign-in.

1. In the bottom left corner, click `load`, select the `training` folder from your local files, and then click `upload`. This should load 19 images.

    ```{image} ./figures/Lab10_Load.png
    :width: 150  
    :align: center  
    ```  

1. Select the **Rectangle** tool and begin labeling stop signs:

    ```{image} ./figures/Lab10_Rectangle.png
    :width: 50  
    :align: center  
    ```  
    <br>

   - Click and drag to draw a bounding box **only around each stop sign**.
   - If an image contains multiple stop signs, draw a box around each.
   - If you make a mistake, select the box and press `delete`.

    ```{note} 
    It is important to label all examples of objects in an image; otherwise, Dlib will implicitly assume that regions not labeled are regions that should not be detected (i.e., hard-negative mining applied during extraction time).
    ```

    ```{tip}
    Use `Alt + ←/→` to switch between images.
    ```

1. Once you've labeled all images, press `Ctrl + e` to export your annotations.
   - Save the file as `stop_annotations.xml` in the `training` folder.

    ```{image} ./figures/Lab10_Dlib.png
    :width: 200  
    :align: center  
    ```  
1. Create a Python script:

    ```bash
    cd ~/master_ws/src/HOG_Demo
    touch trainDetector.py
    ```

1. Open it in your preferred code editor and paste in the following code:

    ```python
    # Import required libraries
    import argparse  # For parsing command-line arguments
    import dlib      # For training and testing the object detector

    # Set up command-line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-x", "--xml", required=True, help="Path to input XML file")          # Path to the labeled training dataset in XML format
    ap.add_argument("-d", "--detector", required=True, help="Path to output detector (.svm)")  # Path where the trained .svm model will be saved
    args = vars(ap.parse_args())  # Parse the arguments into a dictionary

    # Inform the user that training is starting
    print("[INFO] Training detector...")

    # Set training options for the object detector
    options = dlib.simple_object_detector_training_options()
    options.C = 1.0               # Regularization parameter; higher values = lower bias, higher variance
    options.num_threads = 4      # Number of CPU threads to use for training
    options.be_verbose = True    # Print progress and training status to the console

    # Train the detector using the specified XML file and save the model
    dlib.train_simple_object_detector(args["xml"], args["detector"], options)

    # After training, test the detector on the training dataset to evaluate performance
    print("[INFO] Training accuracy: {}".format(
        dlib.test_simple_object_detector(args["xml"], args["detector"])))

    # Load the trained detector from disk
    detector = dlib.simple_object_detector(args["detector"])

    # Create a window to visualize the learned detector's HOG features
    win = dlib.image_window()
    win.set_image(detector)  # Show the detector as a HOG filter visualization

    # Wait for the user to hit Enter before closing
    dlib.hit_enter_to_continue()

    ```

1. Run the script to train your detector:

    ```bash
    python3 trainDetector.py --xml training/stop_annotations.xml --detector training/stop_detector.svm
    ```

    You may get a few errors pop up during execution based on your choice for bounding boxes.  Make sure you address those errors before continuing. If everything runs correctly, you’ll see a visualization of the trained HOG filter. If you get an error, double-check your annotations and fix any issues.

---

### **2. Test the Detector**

Now it is time to build our code to test the detector.  

1. Create a new Python script:

    ```bash
    cd ~/master_ws/src/HOG_Demo
    touch testDetector.py
    ```

1. Add the following code:

    ```python
    # Import the necessary packages
    from imutils import paths   # Utility to easily grab file paths from a directory
    import argparse             # Used to handle command-line arguments
    import dlib                 # Library for machine learning tools including object detection
    import cv2                  # OpenCV for image processing

    # Set up command-line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--detector", required=True, help="Path to trained detector (.svm)")
    ap.add_argument("-t", "--testing", required=True, help="Path to testing images folder")
    args = vars(ap.parse_args())  # Parse the arguments into a dictionary

    # Load the trained detector using the path provided
    # The .svm file contains the learned HOG + SVM model
    detector = dlib.simple_object_detector(args["detector"])

    # Loop through each image file in the testing directory
    for imagePath in paths.list_images(args["testing"]):

        # Read the image from disk using OpenCV
        image = cv2.imread(imagePath)

        # Convert the image from BGR (OpenCV default) to RGB (required by dlib)
        # Then, pass it to the detector which returns a list of bounding boxes
        boxes = detector(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        # Loop through each bounding box returned by the detector
        for b in boxes:
            # Get the coordinates of the bounding box
            (x, y, w, h) = (b.left(), b.top(), b.right(), b.bottom())

            # Draw the bounding box on the image using a green rectangle
            # (x, y) is the top-left, (w, h) is the bottom-right corner
            cv2.rectangle(image, (x, y), (w, h), (0, 255, 0), 2)

        # Display the image with the detections in a pop-up window
        cv2.imshow("Detection", image)

        # Wait for a key press before moving to the next image
        # (0 means wait indefinitely until a key is pressed)
        cv2.waitKey(0)
    ```

3. Run your test:

    ```bash
    python3 testDetector.py --detector training/stop_detector.svm --testing test
    ```

Look at your results. Did the detector work well? Were there any false positives or missed signs?

---

### **3. ROS: Live Camera Streaming**

ROS includes several tools for working with commercial off-the-shelf cameras, like the USB camera on your robot. The main one we'll use is the [usb_cam](https://docs.ros.org/en/humble/p/usb_cam/) package, which is already installed. We'll now use ROS 2 and `usb_cam` to stream live video from the camera.

1. SSH into your robot and run:

    ```bash
    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
    ```

Ignore any calibration error messages — we’ll handle that later.

1. On your Master PC, Run:

    ```bash
    ros2 topic list
    ```

    You should see topics like `/image_raw`, `/camera_info`, etc.

1. Check topic bandwidth:

    ```bash
    ros2 topic bw /image_raw
    ros2 topic bw /image_raw/compressed
    ```

1. Check image publishing rate:

    ```bash
    ros2 topic hz /image_raw
    ros2 topic bw /image_raw/compressed
    ```

1. View the camera feed:

    ```bash
    ros2 run rqt_image_view rqt_image_view
    ```

    Select `/image_raw` to see the feed. Try waving your hand in front of the camera to check latency.

1. You may have noticed that streaming images over WiFi is quite slow. While using compressed images can help, it also increases the processing load on both the Raspberry Pi and the master computer. To avoid this during development, we'll run camera-related code directly on the master computer. Once everything is working, we can move the code back to the robot.

1. Disconnect the camera from the robot and plug it into the master computer. You can also unplug the gamepad—it's not needed for now.

1. On the master computer, run the following command to start the camera node:

    ```bash
    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
    ```
1. Check topic bandwidth and image publishing rate:

    ```bash
    ros2 topic bw /image_raw
    ros2 topic hz /image_raw
    ```

1. To verify the image stream, launch the image viewer:

    ```bash
    ros2 run rqt_image_view rqt_image_view
    ```

### **4. Capture Training Images with ROS**

You’ll now use a script to save training images of stop signs from your live feed.

1. Download the [`image_capture.py`](../files/image_capture.py) script and place it in your package’s script folder.

1. Update your `setup.py` to include the script as an entry point. This is necessary to ensure that the script runs as a node.

1. Open the `image_capture.py` script and read through the code carefully. It may be unfamiliar, but take your time to understand what each part is doing.

1. Rebuild your package:

    ```bash
    ccbuild --packages-selelct labl0-cv
    ```

4. Run the USB camera node on Master:

    ```bash
    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
    ```

5. Open a new terminal, navigate to the `Documents` directory, and run

    ```bash
    ros2 run lab10_cv image_capture -o ./training_images/
    ```

    Press `s` and `Enter` to save an image. Take multiple images from different angles and distances. Press `Ctrl + C` when done.

6. Use `imglab` again to annotate your new images and save the XML file as before. Then re-train your detector using the updated dataset.

---

## Additional lab exercises will be introduced later.

<!--

## 5. Use Your Detector in a ROS Node

Create a ROS node to run your detector in real-time.

1. Inside the `lab4` package, create a new script:

    ```bash
    touch stop_detector.py
    ```

2. Use this starter code:

    ```python
    import rospy, cv2, dlib
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    class StopDetector:
        def __init__(self, detector_path):
            self.bridge = CvBridge()
            self.detector = dlib.simple_object_detector(detector_path)
            self.image_sub = rospy.Subscriber("/image_raw", Image, self.camera_callback)
            rospy.on_shutdown(self.shutdownhook)
            self.ctrl_c = False

        def camera_callback(self, data):
            if self.ctrl_c:
                return
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                boxes = self.detector(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
                for b in boxes:
                    cv2.rectangle(cv_image, (b.left(), b.top()), (b.right(), b.bottom()), (0, 255, 0), 2)
                cv2.imshow("Stop Detector", cv_image)
                cv2.waitKey(1)
            except Exception as e:
                rospy.logerr(f"Error processing image: {e}")

        def shutdownhook(self):
            self.ctrl_c = True
            cv2.destroyAllWindows()

    if __name__ == "__main__":
        rospy.init_node('stop_detector')
        detector_path = rospy.get_param("/stop_detector/detector")
        StopDetector(detector_path)
        rospy.spin()
    ```

3. Update `setup.py` again and rebuild your package.

Now, your detector is running live on video, using ROS and your trained HOG model.




## **5.Test your stop detector**

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




You will then use the detector and known size of the stop sign to estimate how far the stop sign is from the camera. Lastly, you will create a node to identify and determine how far an April Tag is from the robot.

-->
<!--
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



<!--
Make and source your workspace.










1. Now, On the drop down menu, select `image_raw/compressed`.  Nothing will be displayed. Instead, you will be able to find error messages on the terminal.

1. Open another teminal and log in to the robot using SSH. Then run the following command

ros2 run image_transport republish raw compressed   --ros-args   -r in:=/camera1/image_raw   -r out:=/camera1/image_compressed   -p jpeg_quality:=70  # Adjust for quality/bandwidth tradeoff


    ros2 run image_transport republish raw compressed --ros-args --remap in:=/image_raw --remap out/compressed:=/image_raw/compressed -p compressed.format:=jpeg -p compressed.jpeg_quality:=50  # Lower = less CPU, worse quality




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
-->


