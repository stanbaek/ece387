# 🔬 Lab5: Driving the Robot

## 📌 Objectives

- Students should be able to remotely access and control the TurtleBot3 robot using SSH.
- Students should be able to configure and verify ROS communication between the Master computer and the TurtleBot3.
- Students should be able to launch and analyze ROS nodes and topics using ROS diagnostic tools.
- Students should be able to use pre-built ROS packages to drive the TurtleBot3.

## 📝 Overview

In this lab, you will learn how to interact with a remote robot via SSH and use ROS tools to communicate with and control a TurtleBot3. You will work with pre-built ROS packages, such as **turtlebot3_bringup** and **turtlebot3_teleop**, to establish and analyze communication between different ROS components. Additionally, you will use the **gamepad** node from the previous lab to control the robot, reinforcing your understanding of ROS topics, nodes, and message exchange.

## 💻 Lab Procedure

### Working with a Remote Machine

The Raspberry Pi on your robot acts as a Wi-Fi access point (AP), allowing direct communication between your Master computer and the robot without requiring an external network. Since the TurtleBot3 does not have a dedicated monitor and keyboard, you will connect to it remotely using SSH. This will allow you to run ROS nodes and send commands from your Master computer.

(AccessPoint)=
1. **Connect to the Robot's Wi-Fi Network**
   - Click the **system menu** (top-right corner of the screen).  
   - Select **Wi-Fi Networks** and choose `RobotXX`, where `XX` is your assigned robot number.  
   
1. **Check Connectivity**  

   - Open a terminal on your Master computer and run:  
     ```bash
     $ ping 192.168.4.1
     ```
   - If the connection is successful, you will see responses from the robot’s IP address.  
   - If you don’t receive a response, check that you are connected to the correct Wi-Fi network.  

1. **Establish an SSH Connection**  

   - Access the robot remotely by running:  
     ```bash
     $ ssh pi@192.168.4.1
     ```
     > ⌨️ **Syntax:** `ssh <username>@<hostname/IP address>`
   - When prompted, enter the default password (provided by your instructor).  
   - Once connected, any commands entered in this terminal will execute on the robot.  

1. **Change Your Password (First-Time Setup)**  

   - To secure your connection, update the password:  
        ```bash
        $ passwd
        ```
   - Follow the prompts to enter and confirm your new password. If successful, you will see the message:
        ```
        password updated successfully
        ```
   - Test the new password by opening a **new terminal** and reconnecting to the robot via SSH.  

1. **Edit the `.bashrc` File**  

   - Open the file for editing:  
     ```bash
     $ nano ~/.bashrc
     ```
   - Ensure the following lines are at the bottom of the file:  
      ```bash
      source /opt/ros/humble/setup.bash
      source ~/robot_ws/install/setup.bash
      source /usr/share/colcon_cd/function/colcon_cd.sh
      export ROS_DOMAIN_ID=0  # For master0 and robot0
      export _colcon_cd_root=/opt/ros/humble/
      export TURTLEBOT3_MODEL=burger
      export LDS_MODEL=LDS-01  # Replace with LDS-02 if using new LIDAR
      source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
      ```

   - Ensure your `ROS_DOMAIN_ID=XX` matches your computer ID, where `XX` aligns with the `XX` in `MasterXX`.
   - Our class robots have two different LIDAR variants: LDS-01 and LDS-02 (shown below).
      ```{image} ./figures/Lab5_LDS.png
      :width: 480
      :align: center
      ```
      <br>
   - If you have the LDS-02, change `export LDS_MODEL=LDS-01` to `export LDS_MODEL=LDS-02` in your `.bashrc` file.
   - Save your changes and exit the editor.

1. **Close the SSH Connection**  

   - To disconnect from the robot, type:  
     ```bash
     $ exit
     ```  
   - This will return you to your Master computer’s terminal.  


### Updating the Hosts File on **Master**

To make it easier to remember and use the hostname instead of the IP address, let's modify the `hosts` file so that the master computer recognizes the hostname of the Raspberry Pi.

```{warning}
Ensure you execute the commands in this section on the **Master** computer. Do not execute them on the Raspberry Pi.
```

1. **Update the Hosts File on the Master Computer**: To add the robot's IP address to the hosts file, follow these steps on the **`Master`** computer:
   ```sh
   $ sudo gedit /etc/hosts
   ```
   Add the following line to the file:
   ```sh
   192.168.4.1    robotXX
   ```
   Replace `robotXX` with your specific robot number.

1. **Check Connectivity Using Hostname**
   ```bash
   $ ping robotXX
   ```

1. **Reconnect Using Hostname Instead of IP**
   ```bash
   $ ssh pi@robotXX
   ```
   > ⌨️ **Syntax:** `ssh <username>@<hostname>`

### Setting Up Password-Free SSH Authentication
Using password-free SSH authentication improves both security and convenience. Instead of manually entering a password each time, SSH keys provide a more secure and automated way to log in. This also makes remote access faster and more efficient.

1. **Generate an SSH Key Pair on the Client**: 
   - Open a terminal on the client (`Master`) machine and run the following command to generate an RSA key pair:
   ```bash
   $ ssh-keygen -t rsa -b 4096
   ```
   - When prompted to enter a file to save the key, press **Enter** to accept the default location (`~/.ssh/id_rsa`).
   - If prompted for a passphrase, leave it **empty** (just press Enter) to enable password-free login.

1. **Copy the Public Key to the Remote Server**: 
   - Transfer your public key to the server by running the following command, replacing `username` and `hostname` with your actual credentials:
   ```bash
   $ ssh-copy-id username@hostname
   ```
   - If prompted, enter your password for the remote machine. After this, the key will be added to the server's authorized keys.

1. **Test the Connection**: Now, try logging into the remote machine without a password:
    ```bash
    $ ssh username@hostname
    ```
    If everything is set up correctly, you should log in without being prompted for a password.

### Driving the Robot

1. **Using SSH, launch the `robot.launch.py` file on the robot**:

    ```{tip}
    Use tab completion to make your life easier! Start typing a package name or node, then press **Tab** to auto-complete the command.
    ```
    **Note:** The following command contains a deliberate typo to prevent copying and pasting. Be sure to type it out manually or use **Tab** for auto-completion:
    ```bash
    $ ros2 launch turt1ebot3_bringup robot.launch
    ```
    > ⌨️ **Syntax:** `ros2 launch <package> <launchfile>`

    You should see something like this:
    ```bash
    [turtlebot3_ros-3] [INFO] [1738299487.825470539] [turtlebot3_node]: Succeeded to create battery state publisher
    [turtlebot3_ros-3] [INFO] [1738299487.829476168] [turtlebot3_node]: Succeeded to create imu publisher
    [turtlebot3_ros-3] [INFO] [1738299487.841928335] [turtlebot3_node]: Succeeded to create sensor state publisher
    [turtlebot3_ros-3] [INFO] [1738299487.844016446] [turtlebot3_node]: Succeeded to create joint state publisher
    [turtlebot3_ros-3] [INFO] [1738299487.844149094] [turtlebot3_node]: Add Devices
    [turtlebot3_ros-3] [INFO] [1738299487.844204353] [turtlebot3_node]: Succeeded to create motor power server
    [turtlebot3_ros-3] [INFO] [1738299487.849349150] [turtlebot3_node]: Succeeded to create reset server
    [turtlebot3_ros-3] [INFO] [1738299487.851512798] [turtlebot3_node]: Succeeded to create sound server
    [turtlebot3_ros-3] [INFO] [1738299487.853739761] [turtlebot3_node]: Run!
    [turtlebot3_ros-3] [INFO] [1738299487.890749557] [diff_drive_controller]: Init Odometry
    [turtlebot3_ros-3] [INFO] [1738299487.909780816] [diff_drive_controller]: Run!
    ```

    We will delve deeper into launch files in future lessons, but for now, understand that a launch file is used to initiate one or more ROS nodes.

    Your Turtlebot3 is now ready to drive and should be listening for *Twist* messages sent over the **/cmd_vel** topic.

1. **Verify Turtlebot3's Communication with the Master**:
   - It's always a good idea to check that the Turtlebot3 is communicating with the Master. To do this, list the active topics that the Turtlebot3 is publishing. Run the following command on your **Master**:
   ```bash
   $ ros2 topic list
   ```

   If everything is working correctly, you should see something like this:
   ```bash
   /battery_state
   /cmd_vel
   /imu
   /joint_states
   /magnetic_field
   /odom   
   /tf_static
   ```

1. **Observe the Nodes Running on the Master**:
    - Open a new terminal on the Master and run the following command to observe the currently running nodes:
    ```bash
    $ rqt_graph
    ```
    - You should see `/turtlebot3_node` subscribing to the **/cmd_vel** topic and publishing multiple topics including **/imu**.

1. **Get Information About `/turtlebot3_node`**:
    - Open another terminal and run the following command to get information about the `/turtlebot3_node`:
    ```bash
    $ ros2 node info /turtlebot13_node
    ```
    - This will show the topics `/turtlebot3_node` is publishing and subscribing to.

1. **Review the **/cmd_vel** Topic**:
    - We used the **/cmd_vel** topic when driving the simulated Turtlebot3. To refresh our memory, let's check the topic information:
    ```bash
    $ ros2 topic info /cmd_vel
    ```
    - This shows that the **/cmd_vel** topic uses the *Twist* message type. We can verify this by running:
    ```bash
    $ ros2 topic type /cmd_vel
    ```

1. **Show Information About the *Twist* Message**:
    - To see information about the fields within the *Twist* message sent over the **/cmd_vel** topic, run the following command:
    ```bash
    $ ros2 interface show geometry_msg/msg/Twist
    ```

1. **Run the `teleop_keyboard` Node on the Master**:
    - Open a terminal on the Master and run the following command:
    ```bash
    $ ros2 run turtlebot3_teleop te1eop_keyboard
    ```

    ```{warning}
    If you run `ros2 run teleop_twist_keyboard teleop_twist_keyboard`, the minimum linear x speed of the `cmd_vel` published by the `teleop_twist_keyboard` node is 0.5 m/s, which is greater than the maximum speed of TurtleBot3, so TurtleBot3 will ignore the topic.
    ```

2. **Observe Node Communication with `rqt_graph`**:
    - Before driving the Turtlebot3, observe how the nodes communicate using the `rqt_graph` tool. Open a new terminal (or refresh the existing `rqt_graph` window if it's still open) to see the interactions.

3. **Operate Turtlebot3 with Optimal Parameters**:
    - The Turtlebot3 operates best with a linear velocity between 0.1 m/s and 0.2 m/s. It turns best with an angular velocity between 0.5 rad/s and 1.5 rad/s. Use these parameters to drive the TurtleBot3 safely.


9. **Drive the TurtleBot3**

   - Use arrow keys to navigate.
   - Adjust linear velocity (0.1-0.2 m/s) and angular velocity (0.5-1.5 rad/s) for best control.

## 🛠️ ROS Diagnostics

1. **List all running nodes**

   ```bash
   ros2 node list
   ```

2. **Display ROS graph**

   ```bash
   rqt_graph
   ```

3. **List active topics**

   ```bash
   ros2 topic list
   ```

4. **Check message types for topics**

   ```bash
   ros2 topic type <topic_name>
   ```

5. **View topic messages in real-time**

   ```bash
   ros2 topic echo <topic_name>
   ```

## 🚚 Deliverables

- Push screenshots showing output from ROS commands to your student repository under `/master/module04/`.
- Demonstrate that you can successfully drive the TurtleBot3 using keyboard teleoperation.
- Submit a summary of your findings on how the ROS nodes and topics interact.

## 📄 Summary

In this lab, you learned how to:

- Connect to a remote robot using SSH.
- Use ROS diagnostic tools to examine nodes, topics, and messages.
- Launch pre-built ROS packages to drive the TurtleBot3.
- Observe real-time system communication using `rqt_graph`.

Mastering these skills will be essential for future labs, especially as you begin developing your own ROS nodes for autonomous naation!












    



## ROS

In labs throughout this course we will request information about the topics, nodes, and messages within your system. Accomplish the following in a new terminal on your Master.

1. List all running nodes.

1. Display running nodes and communication between them.

1. List the active topics.

1. Determine the type of messages sent over the topics (repeat for each topic).

1. Determine the fields of the messages.

1. Observe the information sent over a topic (repeat for each topic).

## Checkpoint
Once complete, push screenshots showing the output of each of the above to your student repo on github in a /master/module04 folder.

## Summary
In this exercise you examined and used pre-built packages and source code to drive the Turtlebot3 and understand how the system worked. You then were able to analyze the topics, nodes, and messages within the ROS system to better understand the flow of information and control. The **pro-tips** presented throughout this exercise will make you a better user of Linux and ROS.


```{tip} 
I strongly recommend that you commit the above sequence of commands to memory, or at a minimum
have them in a place that you can quickly recall them. There is nothing until Module 9 that absolutely requires the real robot, as everything else can be simulated.
```






