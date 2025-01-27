
# 🔧 Gamepad Setup


## Purpose
Setup Logitech and XBox One Gamepads on the **Master**.

To use both a Logitech Gamepad and an Xbox One Gamepad with ROS2 Humble on Ubuntu, you'll need to install several dependencies, configure the input devices, and then use them in your ROS2 workspace. Here's a step-by-step guide:


(GamepadSetup)=
### Create a Repository within the GitHub Classroom

1. **Install `joy` package**: (_This package is already installed on the `Master` computer._) The `joy` package in ROS2 provides the necessary interface to read inputs from gamepads. 
   ```
   sudo apt update
   sudo apt install ros-humble-joy
   ```

1. **Install `teleop_twist_joy` package**: (_This package is already installed on the `Master` computer._)If you want to control a robot using the gamepads, you'll need the `teleop_twist_joy` package.  Install it by running:
   ```
   sudo apt install ros-humble-teleop-twist-joy
   ```

1. **Plug in your gamepads**: Connect your Logitech Gamepad and Xbox One Gamepad to your PC via USB.
   
1. **Verify gamepad detection**:
   You can verify that your gamepads are detected by checking for the input devices. Run the following command:
   ```bash
   $ ls /dev/input/js*
   ```
   You should see something like `js0`, `js1` corresponding to your gamepads.
   

1. **Install Joystic GUI Calibration Tool** and run it.
    ```bash
    $ sudo apt-get install jstest-gtk
    $ jstest-gtk
    ```
    Once it starts, double click the joystick. Move around the axes and buttons and identify the axis and numbers. The joystick values may not read a zero when the sticks are in the neutral position. Click the `Calibration` button and follow the procedure to calibrate the joystick.



3. **Set up the udev rules** for your controller. Create a file named `99-joystick.rules` in the `/etc/udev/rules.d/` directory with the following content:
   ```bash
   SUBSYSTEM=="input", ATTRS{name}=="Logitech USB Gamepad", MODE="0666"
   ```
4. **Reload the udev rules** to apply the changes:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
5. **Launch the teleop_f310 node** in ROS2:
   ```bash
   roslaunch teleop_f310 teleop_f310.launch
   ```
   This will start the node that allows you to control your robot using the Logitech F310 joystick.

6. **Verify the joystick input** by echoing the `joy` topic:
   ```bash
   rostopic echo joy
   ```
   You should see the joystick inputs being published if everything is set up correctly.

7. **Adjust the joystick configuration** if needed using the dynamic reconfigure plugin in `rqt`. This allows you to change the joystick mappings on-the-fly.

==========================================================================================






### Step 3: Configure the `joy` Node
1. **Launch the `joy` node**:
   The `joy` node reads the input from your gamepads. Run it with:
   ```
   ros2 run joy joy_node
   ```
   This will start a node that listens to the gamepads and publishes joystick data on `/joy` topic.

2. **Test joystick output**:
   You can verify that the gamepad is working by echoing the `/joy` topic:
   ```
   ros2 topic echo /joy
   ```
   Move the joysticks or press buttons on your gamepad to see data being published.

### Step 4: Configure Teleoperation (Optional)
If you want to use the gamepad to control a robot using teleoperation, follow these steps.

1. **Configure `teleop_twist_joy`**:
   Create a configuration file for `teleop_twist_joy`. The default configuration should be good, but you can adjust it to your needs. The configuration file typically looks like this:
   
   Create the file: `~/.ros/teleop_twist_joy.yaml` with the following contents:
   ```yaml
   joy_node:
     ros__parameters:
       device: "/dev/input/js0"  # Adjust the device based on your gamepad (e.g., js0 for the first gamepad)
   teleop_twist_joy:
     ros__parameters:
       axis_linear: 1  # Axis for linear movement (usually left joystick)
       axis_angular: 3  # Axis for angular movement (usually right joystick)
       button_forward: 4  # Button for moving forward
       button_backward: 5  # Button for moving backward
       button_left: 6  # Button for turning left
       button_right: 7  # Button for turning right
   ```

2. **Run the teleop node**:
   Launch the `teleop_twist_joy` node to control your robot via the gamepad:
   ```
   ros2 run teleop_twist_joy teleop_twist_joy_node --ros-args -p joy_node.device:=/dev/input/js0
   ```
   Replace `/dev/input/js0` with the correct device file if necessary.

### Step 5: Using Both Gamepads Simultaneously
- If you want to use both gamepads simultaneously, you'll need to launch two `joy_node` instances: one for each gamepad.

1. **Launch two joy nodes**:
   You can launch two `joy_node` instances like this:
   ```bash
   ros2 run joy joy_node --ros-args -p joy_node.device:="/dev/input/js0" &
   ros2 run joy joy_node --ros-args -p joy_node.device:="/dev/input/js1" &
   ```

2. **Verify both devices are working**:
   You can echo both `/joy` topics to see if both devices are sending data:
   ```bash
   ros2 topic echo /joy
   ```

### Step 6: Test Everything
1. **Control the robot (if applicable)**:
   If you're using `teleop_twist_joy` to control a robot, the gamepads should now send commands to move the robot.

2. **Check input from both gamepads**:
   If you’re using both gamepads to control different aspects of the robot, make sure both are publishing data correctly by echoing the topics.

### Troubleshooting:
- **Device not found**: If your gamepad isn't detected, check if the device file (`/dev/input/js0`, etc.) exists.
- **Buttons or axes not responding**: Adjust the button/axis mappings in the configuration YAML file (`teleop_twist_joy.yaml`).

### Conclusion:
You should now have both the Logitech and Xbox One gamepads working with ROS2 Humble on Ubuntu! You can either use them for teleoperation or just gather joystick data for further processing in your ROS2 nodes.

Let me know if you run into any issues!