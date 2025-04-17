# 🚀 Final Project


## NOT READY YET



```{attention}
Read this page thoroughly before you start working on this final project.   
```


## 📌 Objectives  

- Students will integrate concepts from earlier modules to design and implement a complete robotic system capable of handling complex tasks using ROS2.  
- Students will demonstrate proficiency in robotic systems by building task-specific machines with onboard computing.  
- Students will apply knowledge from across the course to develop, test, and deliver a functional final project.


## 📜 Agenda

The final project brings together everything you've learned throughout the course. It challenges you to design and build a complete robotic system using ROS2 that can solve a real-world task with multiple components working together. This project emphasizes embedded, task-specific robotics and is where you apply your accumulated knowledge in a hands-on, functional way.

Your robot will operate in the maze in the lab, using a combination of technologies from previous modules. LIDAR will help it stay centered between walls, the IMU will handle precise turns (90°, 180°, 360°), and OpenCV will detect and respond to randomly placed stop signs. Additionally, you'll use AprilTags to guide the robot through the maze —navigating based on tag ID and distance to determine whether it should turn left, right, around, or stop at its goal.

```{important}
If you notice any discrepancies in the project description or grading criteria, the correct information will follow this order of priority: (1) Instructor messages via Teams, (2) this Course Web, (3) Gradescope, and (4) the project overview slides. The Course Web will always be updated immediately to reflect any changes, and outdated instructions will be clearly crossed out.
```

##  Synopsis

🏴‍☠️ Ahoy, crew! The infamous pirate captain, Dr. Baek, has discovered an ancient scroll that reveals the location of a hidden treasure island. But there’s a catch—the treasure is locked deep inside a deadly maze! 

According to the scroll, the maze is filled with mysterious symbols called AprilTags. These symbols are the only clues that lead to the treasure. Miss one, or read it wrong, and... well, let’s just say you won’t be making it back.

But instead of risking life and limb, Captain Baek wants to use some good ol' 21st-century tech. That’s where you come in. Your mission: build a robotic system that can explore the maze on its own, follow the AprilTags, and find the treasure.

Here is what the scroll says
    - You must follow the walls in the maze. 
    - Do not stop at windows or look out of the windows as they are so deadly.
    - At AprilTag ID 0, you must turn left. Make sure you make a 270$^\circ$ clockwise turn. 
    - At AprilTag ID 1, you must stop and pause for 5 seconds then make a 90$^\circ$ left turn.
    - At AprilTag ID 2, you must turn right. Make sure you make a 90$^\circ$ clockwise turn. 
    - At AprilTag ID 3, you must turn left. Make sure you make a 90$^\circ$ counterclockwise turn. 
    - At a stop sign, you must stop from 0.3 meters from the stop sign, then make a left turn. The treasure chest is located around there.  Now you should look around to find the exact location of it. The treasure chest is buried right under AprilTag ID 4.
    

## 🎮 Final Project Gamesmanship

1. Demo and coding (40 points)
    - Read the presentation and final report requirements before you start coding.
    - Start early to earn early-bird bonus points; delay may jeopardize timely completion.
    - Use the code from Labs 10 and 11 as foundations, but avoid implementing your final project in these lab files. Instructors will review the code inside the `final_project` folder. 
    - Employ `rqt`, `ros2 topic`, and `rviz` extensively for debugging high-level behaviors. 
    - **Cease work for the final report**
        - Your analysis in the report is much more important than completing the maze. 
        - Balance time spent on coding and demo; don't sacrifice report quality for demos. Don't lose 30 points on the report to earn 15 demo points.
        - If your robot reaches only halfway to the Level 1 goal point, the deduction will be approximately 5-10 points.

1. Presentation (20 points)
    - Utilize visual aids a lot! Figures, tables, and graphs are more helpful than words.
    - Ensure you **discuss everything** in the presentation section.
    - Adhere to the 6-minute time limit; practice for effective delivery. Your talk will be stopped at the 7-minute mark, and credit will not be given for parts not discussed. Adhering to the time limit for presentations is a basic etiquette. Even at professional conferences, talks can be cut short if they exceed the allotted time.
    - You are strongly recommended to practice your talk. Students who gave very short presentations to avoid exceeding the time limit often did not discuss enough details and lost even more points. So, practice! Even experienced engineers practice for their conference presentations.

1. Report (40 points)
    - Thoroughly Read the template and **do not miss anything in the template**.
    - Use figures and tables to support your analysis and results.
    - While in-person demos are accepted, ensure submission of video demos aligned with the plots in your report.

## 🛠️ Requirements

### Timeline

1. L36 0700: Design Presentation slides (Gradescope and Instructors)
    - Submit your `MS PowerPoint pptx` file to your instructor NLT L36 0700. Your slides will be played on your instructor's PC for smooth transitions between speakers. Make sure to send a pptx file and not a Keynote file or Google Slides unless your instructor has approved it.
    - Additionally, submit the PDF version of your presentation file on Gradescope no later than L36 0700. 
    - **_No grace days_** can be used for the PowerPoint slides.
1. L36: Design Presentations
1. L39 0700: Live demo Due   
    - Late Demos: You can use grace days, but all products must be submitted NLT T40 2359 (by the Dean's policy). 
1. T40 2359: Final report & Code
    - No grace days can be used. All products must be submitted by midnight on T40. 


### 🎬 Demonstrations 

Demonstrations will be accomplished on lesson 39 in the maze. Points will be deducted for failed checkpoints (e.g., does not stop and turn within approximately 2 meters of AprilTag 0). The final rubric is below and a total of **40 points** assigned to the demonstration:

- Wall following [10 points]: The robot should follow the walls without hitting them. A deduction of 1 point for each wall collision will be applied. No more than 3 points will be deducted if the robot reaches the end goal ($\pm$30 cm from the yellow line).  
- Stop sign [10 points]: The robot should stop at the yellow line - Any part of the the bottom plate of the robot should . Stop within 0.3 meters from a stop sign, wait until the stop sign is removed.
- AprilTag ID 0: Stop within 0.27 meters from the AprilTag and then turn left. 
- AprilTag ID 1: Stop within 0.27 meters from the AprilTag and then turn right. 
- AprilTag ID 2: Stop within 0.27 meters from the AprilTag, and then make a 270$^\circ$ left turn.. 



    - You must follow the walls in the maze. 
    - Do not stop at windows or look out of the windows as they are so deadly.
    - At AprilTag ID 0, you must turn left. Make sure you make a 270$^\circ$ clockwise turn. 
    - At AprilTag ID 1, you must stop and pause for 5 seconds then make a 90$^\circ$ left turn.
    - At AprilTag ID 2, you must turn right. Make sure you make a 90$^\circ$ clockwise turn. 
    - At AprilTag ID 3, you must turn left. Make sure you make a 90$^\circ$ counterclockwise turn. 
    - At a stop sign, you must stop from 0.3 meters from the stop sign, then make a left turn. The treasure chest is located around there.  Now you should look around to find the exact location of it. The treasure chest is buried right under AprilTag ID 4.
    




## 🛠️ Procedures

### ✅ Part 1: Calibrate the USB Camera

For a camera to effectively perform computer vision tasks, it must first be calibrated. Without calibration, there's no reliable reference for determining the size of objects within the camera's frame. The [ROS Calibration Tool](http://wiki.ros.org/camera_calibration) generates a calibration file that other ROS packages can use to calculate object size and distance. The **camera_calibration** package leverages OpenCV's calibration techniques to simplify the process of calibrating monocular or stereo cameras, typically using a checkerboard pattern as the target. For detailed instructions, refer to the [Camera Calibration Tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).

1. Disconnect the camera from the robot and plug it into the **Master computer**. Then run:

    ```bash
    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
    ```

1. Check if topics are being published:
    ```bash
    ros2 topic list
    ```

    Make sure you see `/image_raw` and `/camera_info`. Then check the message type:

    ```bash
    ros2 topic type /camera_info
    ros2 interface show <message>
    ```

1. Echo the camera info:

    ```bash
    ros2 topic echo /camera_info
    ```

    Look for values under `K`, `R`, and `P`. Initially, they’ll be all zeros—this means the camera is not yet calibrated.

    > 💡 You should be comfortable using these ROS2 commands. Expect to answer related questions during graded reviews (GRs).

1. Run the calibration tool. Use the checkerboard (9x6 internal corners, 2.5 cm spacing). Run:
    ```bash
    ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.025 --camera_name default_cam --no-service-check --ros-args -r image:=/image_raw
    ```
    This launches a calibration window.

    ```{image} ./figures/Lab11_Calibration.png
    :width: 500  
    :align: center  
    ```  

1. To collect calibration data, move the checkerboard in the camera view in various ways:
    - Left/right/top/bottom
    - Closer/farther
    - Tilted at angles
    - Fill the entire frame

    Four progress bars will fill up. When the "CALIBRATE" button activates, click it.

    ```{image} ./figures/Lab11_Calibrate.png
    :width: 500  
    :align: center  
    ```  

    > 🕓 Calibration may take a few minutes. The windows might be greyed out but just wait, it is working.

    When complete, it will display something similar to the following output:

    ```bash
    **** Calibrating ****
    mono pinhole calibration...
    D = [-0.027598200869062808, -0.0273210932029884, 0.002259050714452529, 0.004238057516107571, 0.0]
    K = [517.6781776101559, 0.0, 330.5433124974498, 0.0, 522.269778659698, 247.60730067305022, 0.0, 0.0, 1.0]
    R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P = [508.604248046875, 0.0, 333.1752989083179, 0.0, 0.0, 517.5308227539062, 248.01611962861898, 0.0, 0.0, 0.0, 1.0, 0.0]
    None
    # oST version 5.0 parameters


    [image]

    width
    640

    height
    480

    [default_cam]

    camera matrix
    517.678178 0.000000 330.543312
    0.000000 522.269779 247.607301
    0.000000 0.000000 1.000000

    distortion
    -0.027598 -0.027321 0.002259 0.004238 0.000000

    rectification
    1.000000 0.000000 0.000000
    0.000000 1.000000 0.000000
    0.000000 0.000000 1.000000

    projection
    508.604248 0.000000 333.175299 0.000000
    0.000000 517.530823 248.016120 0.000000
    0.000000 0.000000 1.000000 0.000000
    ```


1. Save and extract calibration data. Click **Save**, then **Commit**. Browse to the location of the calibration data and extract
    ```bash
    cd /tmp
    tar xf calibrationdata.tar.gz
    ```

    Move the `ost.yaml` file to the correct ROS folder:

    ```bash
    cd ~/.ros
    mkdir camera_info
    mv /tmp/ost.yaml ./camera_info/default_cam.yaml
    ```

1. Relaunch the camera node:

    ```bash
    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
    ```

1. Take another look at the `camera_info` topic. Do you notice any differences compared to the message published by the `usb_cam` node when calibration data is not included? Keep in mind that the `usb_cam` node does not use the `CameraInfo` message to produce rectified images from the raw data (`\image_raw`). Instead, it publishes the `CameraInfo` message so that you can use it to create rectified images if needed.

1. Take a screenshot of the calibrated `camera_info` output. This is one of your deliverables.

---

### ✅ Part 2: AprilTag Detection with ROS2

In this section, you will explore how to detect AprilTags in ROS 2. There are several methods to achieve this, one of which involves using multiple existing ROS nodes and connecting them via ROS topics. While this approach may seem straightforward, managing the interactions between different nodes can become complex and inefficient. To simplify this process, we will develop our own custom node to handle all the required functionality.

1. Create a package named `lab11_apriltag` with the dependencies:
- `rclpy`
- `cv_bridge`
- `sensor_msgs`
- `std_msgs`
- `opencv2`

1. Download the provided [`apriltag.py`](../files/apriltag.py) and place it in your package’s scripts directory.

1. Update your `setup.py` to include the script as an entry point. This is necessary to ensure that the script runs as a node.

1. Inside your package (not the scripts folder), create a new directory called `config`. Next, copy the `default_cam.yaml` file into this directory. The code in `apriltag.py` is already set up to load this default calibration file if no calibration file path is provided by the user when the node is executed. Your directory structure should look like this:

    ```bash
    lab11_apriltag/
    ├── config/
    │   └── default_camera_info.yaml
    ├── lab11_apriltag/
    ├── resources/
    └── ...
    ```

1. Add this path to your `setup.py` under `data_files`:

    ```python
    data_files=[
        ('share/' + package_name + '/config', ['config/default_camera_info.yaml']),
        ...
    ]
    ```
    And in `package.xml`, include:
    ```xml
    <buildtool_depend>ament_cmake</buildtool_depend>
    <exec_depend>ament_index_python</exec_depend>
    ```

1. Open the `apriltag.py` script and examine the constructor to understand how it handles loading files via command line arguments. If no argument is provided, it defaults to a pre-configured file. Pay close attention to the `load_camera_info` method—it demonstrates how to extract data from a `yaml` file effectively. While the same functionality could be achieved by subscribing to the `camera_info` topic published by the `usb_cam` node, using a calibration file is more efficient since the `camera_info` topic repeatedly publishes identical data over time.

2. Launch the `usb_cam` node with a frame rate set to **15 Hz**:
   ```bash
   ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p framerate:=15.0
   ```

3. Start the `apriltag_ros` node to detect AprilTags.

4. Open another terminal and echo the topic `/apriltag_pose` on the master. Observe the output. Does the `apriltag_ros` node detect more than one tag simultaneously? Consider which value might be used to calculate the distance to a tag, and note the type of message being published. Identify the package this message originates from.

5. Demonstrate to the instructor that the **apriltag_ros** node is successfully detecting tags and publishing their position data.

With a properly calibrated camera, you are now equipped to identify AprilTags along with their size, orientation, and distance.


## 🚚 Deliverables

1. **[20 Points] Complete the `apriltag.py` Script**
    - Ensure the script is fully functional and implements all required features.
    - Push your code to GitHub and confirm that it has been successfully uploaded.
    **NOTE:** _If the instructor can't find your code in your repository, you will receive a grade of 0 for the coding part._

1. **[10 Points] Submit Screenshots**
    - Submit a screenshot of the `camera_info` topic.

1. **[20 Points] Demonstration**
    - Demonstrate the `apriltag_ros` node printing the ID and distance of each April Tag.
    - Demonstrate the `apriltag_ros` node publishes the `apriltag_pose` topic.
