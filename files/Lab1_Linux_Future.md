# 🔬 Lab1: Linux

## Objectives
- Students should be able to

## Linux Commands
During class, we covered several basic Linux commands. In this lab, we will get hands-on practice with those commands.

1. To log in the Master computer, use `m3` if you're in the M3 section and `t5` if you're in the T5 section.
1. Click the Terminal icon on the Dock to open a terminal and practice commands.

    ```{image} ./figures/Lab1_Terminal.png
    :width: 300
    :align: center
    ```
    <br>

1. You can select an open terminal and use `Ctrl + Shift + n` to open a new terminal window or `Ctrl + Shift + t` to add a new tab to the current terminal.

1. When observing the terminal (or Shell) you will notice the following
    - Syntax: `username@hostname:`(e.g., on the master: `m3@masterX:`, on the robot: `pi@robotX`)
    - Current working directory: Represented as `~`, which refers to the user's home directory. 
    - Prompt: The `$` character followed by a blinking cursor, which indicates the terminal is active and ready for commands.

### Using Filesystem

To master Linux, it’s crucial to understand the filesystem and become comfortable with the Terminal (or shell). While the command line may feel intimidating at first, regular practice will make it second nature.


1. **View Directory Contents**: Enter `ls` to see the contents of the current directory. You'll see directories like Desktop, Documents, and Downloads. Color coding helps identify types: directories are blue, files are white, executable files are bright green, and archived files are red. For now, just note that directories are blue and files are white.

1. **Display Current Directory Path**: Enter `pwd` to display the path of the current directory. This will show your home directory path, such as `/home/stanbaek`.

1. **Change Directory**: To move from your home directory to the Downloads directory, enter `cd Downloads`. The `cd` command stands for "change directory." Enter `ls` again to view the files inside the Downloads directory.

1. **Navigate to Parent Directory**: Use `cd ..` to navigate back to the home directory. In Linux, two dots `..` refer to the directory above (the parent directory). A single dot `.` refers to the current directory. While you don't use `cd .` to switch to the current directory, knowing this can be useful for some commands.

1. **Root Directory**: The root directory is always `/`. Enter `ls /` to list the contents of the root directory, and `cd /` to switch to the root directory. It’s important to understand the difference between `ls Downloads` and `ls /Downloads`. The first command lists the contents of the Downloads directory within the current directory, while the second tries to list a Downloads directory directly under the root, which usually doesn’t exist.

1. **View Root Directory Contents**: Enter `cd /` followed by `ls` to view the files inside the root directory.

1. **Return to Home Directory**: Enter `cd` to move back to your home directory. Note that `cd` is equivalent to `cd ~`.

1. **Complex Paths**: The `ls` and `cd` commands can also be used with more complex paths. For example, enter `ls /opt/ros/humble` to view the contents of the "humble" directory inside "ros," which is inside "opt." Switch to this directory using `cd /opt/ros/humble`. To move back up three parent directories, use `cd ../../..`.

1. **Autocomplete**: Type `cd /o` followed by the tab key. It will autocomplete to `cd /opt/`. Press the tab key again to see options like `google` and `ros`. Type `r` and press the tab key to choose `ros`, then press the tab key again to choose `humble` as it is the only directory inside `ros`.

    ```{image} ./figures/Lab1_Autocomplete.gif
    :width: 520
    :align: center
    ```
    <br>

### Listing Files

Using the desktop GUI to list and move files is easier than using the Terminal and keyboard, but it's an important skill that you’ll appreciate as you advance with ROS and Linux.

1. **Basic Listing**: We've already looked at `ls`, which lists the files in the working directory. You're more likely to use a command like `ls -lah`. The bit after the command (the `-lah`) is known as the argument. This option modifies the behavior of the command. The `-l` argument lists files and directories in "long" format. Each file and directory is now on a single line, and before each file is a lot of text. First, you'll see letters and dashes, like `drwxr-xr-x`. These are `permissions`, and we breifly discussed in class.

    ```{image} ./figures/Lab1_ls-lah.png
    :width: 520
    :align: center
    ```
    <br>

1. **File Count**: After the permission letters, there's a single number. This is the number of files in the item. If it's a file, it will be 1, but if it's a directory, it will be at least 2. Each directory contains two hidden files: one with a single dot (`.`) and one with two dots (`..`). Directories containing files or other directories will have a higher number.

1. **Owner and Group**: Next, you'll see your username twice on each line. The first is the owner of the file, and the second is the group. Typically, these will be the same, and you'll see either `root` or your username. You can enter `ls -l /` to view the files and directories in the root directory that belongs to the root account.

1. **File Size**: The next number relates to the size of the files, in bytes. The `h` argument in `ls -lah` humanizes the number, making it easier to read.

1. **Hidden Files**: Be aware that many hidden files in Linux are listed using the `-a` argument. Hidden files and directories begin with a dot (`.`), so you should never start a file or directory with a dot, unless you want to hide it. Typically, you can combine all three arguments into the command `ls -lah`.

### Creating and Deleting Files

Creating and deleting files is a fundamental computing skill. When using the Linux Terminal, remember that deleted files are not sent to the system recycle bin, so extra care is needed.

1. **Create a File**: Enter `cd` to move to the home directory. Type `touch testfile` and `ls -l` to view the files. You'll see a new file called `testfile` with a size of 0 because it's empty.

1. **Case Sensitivity**: Linux is case sensitive. If you enter `touch Testfile` (with a capital T), it creates a second file called `Testfile`. Enter `ls -l` to see both files. To avoid confusion, most people use lowercase letters consistently.

1. **Update Timestamp**: Enter `ls -l` and note the timestamp of `testfile`. Then enter `touch testfile` followed by `ls -l` and notice the updated time. The touch command updates files or directories with the current system time.

1. **No Spaces in File Names**: Avoid using spaces in file names. Entering `touch test file` creates two files: `test` and `file`. Instead, use an underscore (`_`), like `touch test_file`.

1. **Delete Files**: If you've followed the steps, you should have five files: `testfile`, `Testfile`, `test`, `file`, and `test_file`. To delete files, use the `rm` command. Enter `rm Testfile` to delete the file named `Testfile`. Enter `ls -l` to confirm its deletion.

1. **Use Wildcards**: Enter `ls test*` to view files that match the word `test` followed by any characters. The `*` wildcard means "any characters here." Enter `rm test*` to delete `test`, `testfile`, and `test_file`. Finally, enter `rm file` to delete it.

### Creating and Removing Directories

After learning to create files, you’ll want to know how to make directories and move items around.

1. **Create a Directory**: Enter `ls` to view directories in the home directory. Use `mkdir` to create directories. Enter `mkdir testdir` and `ls` again to see it.

1. **Timestamp Update**: Unlike touch, `mkdir` does not update the timestamp if used on an existing directory.

1. **Multiple Directories**: You can create multiple directories at once with `mkdir`. Enter `mkdir testdir2 testdir3` and `ls` to see several directories.

1. **Nested Directories**: You can create directories within directories using the directory path. Enter `mkdir Documents/photos` to create a `photos` directory inside `Documents`. The parent directory must exist, so `mkdir articles/reports` will fail if `articles` does not exist.

1. **Create Directory Path**: Use the `-p` option with `mkdir` to create a directory path. Enter `mkdir -p articles/reports` and `ls` to view the `articles` directory, and `ls articles` to see the `reports` directory inside.

1. **Remove a Directory**: Ensure you're in the home directory by entering `cd`. Then enter `ls` to view contents. Use `rmdir` to delete directories. Enter `rmdir testdir3` and `ls` to confirm deletion.

1. **Non-Empty Directories**: Try to delete the `articles` directory containing the `reports` directory. Enter `rmdir articles`. You will get an error. The `rmdir` command only removes empty directories. To delete non-empty directories, use `rm` with the `-r` (recursive) option. Enter `rm -r articles` to delete the `articles` directory containing the `reports` directory.

1. **Interactive Deletion**: Use the `-i` (interactive) option with `rm` to prompt before each deletion. Enter `rm -ri test*` and enter `Y` or `y` for each prompt. Using `-i` is a good practice with the `rm` command.


### Copying, Moving, and Renaming Files

In Linux, renaming a file is essentially moving it from one name to another, and copying a file involves moving it without deleting the original.

1. **Create Test File and Directory**: Enter `touch testfile` and `mkdir testdir` to create a test file and directory in your home directory. Enter `ls` to confirm their presence.

1. **Move Files and Directories**: Use the `mv` command with two arguments: source and destination. Enter `mv testfile testdir` to move `testfile` into `testdir`. Enter `ls` to see it’s no longer in the home directory, and `ls testdir` to see it inside `testdir`. Create a new directory with `mkdir newparent`.

1. **Move Directories**: Move directories with files using the same command. Enter `mv testdir newparent` to move `testdir` into `newparent`. Enter `cd newparent/testdir` and `ls` to see `testfile` inside `testdir`.

1. **Move Up Levels**: Use the double dot (`..`) to move up directories. Enter `ls -la` to view files, including single and double dot entries. Move `testfile` up one level with `mv testfile ..` and enter `cd ..` to move to the parent directory.

    ```{hint}
    Use the tab key to autocomplete paths for the following step.
    ```

1. **Longer Paths**: Move files using longer paths. Enter `cd ~` to return to the home directory, then `mv newparent/testfile newparent/testdir/testfile` to move `testfile` back to `testdir`. Enter `ls newparent/testdir` to confirm.

1. **Rename Files**: Rename and move a file at the same time. Enter `mv newparent/testdir/testfile newparent/testfile2`. Enter `ls newparent` to see `testfile2` in the `newparent` directory.

### Useful Commands

Linux is a vast and versatile command line language with hundreds of commands you can learn. Here are a few that can help you get more from your Ubuntu.

1. **View Processor Details**: Enter `cat /proc/cpuinfo` to see details about the processors.

1. **Using `cat`**: The `cat` command lists the contents of a text file, such as `cpuinfo`. You can also open this text file using gedit, a GUI text editor. Enter `gedit /proc/cpuinfo` to view the file in gedit. Note that `cpuinfo` is read-only.

1. **View Memory Information**: Enter `cat /proc/meminfo` to get information about your memory.

1. **Process Status**: Type `ps` to see two items: `bash` and `ps`. To view processes used by other users (including those started by root), enter `ps -a`. This option shows processes for all users but does not include background processes. For that, enter `ps -A` or `ps -e`, which shows every process on the system, including background processes. You may need to pipe it through `less` using `ps -e | less`. The `less` command allows you to view the contents one screen at a time. Press `q` to exit `less`.

1. **Real-Time Process Monitoring**: While `ps` is useful, you may need to view processes in real-time, especially to check CPU and memory usage. Use the `top` command for this.

1. **Shutdown and Restart**: To shut down the computer from the command line, enter `sudo shutdown -h now`. The `-h` option stands for "halt." To restart, enter `sudo shutdown -r`.

### Input, Output, and Pipes

1. **Redirect Output**: Change the standard output using the `>` character after your command. For example, `ls -l /etc` lists all items in the `/etc` directory. Using `ls -l /etc > etc.txt` outputs the list to a new text file called `etc.txt`.

1. **View Output File**: The `etc.txt` file now contains the output from the `ls` command. Check it using `cat etc.txt` or `nano etc.txt`. The output from `ls -l` was sent to this file instead of the screen. Press `Ctrl+X` to quit `nano`. The `>` character allows you to output to files, but you can also get input from a file.

1. **Using Pipes**: As you advance in Linux, you create more powerful commands using the pipe character (`|`). Enter `cat ~/.bashrc` to display the `.bashrc` file's content. Now enter `cat ~/.bashrc | wc`. The output from `cat` is piped into the `wc` (word count) command, showing the number of lines, words, and characters in the document.

1. **Chaining Commands**: You can pipe commands multiple times. Enter `cat ~/.bashrc | sort | grep source*` to list lines starting with "source" in alphabetical order. The output from `cat` is passed to `sort`, and `sort`'s output is passed to `grep`, which filters out content starting with "source."

















































1. Run the commands to move to the home folder and make a new directory:

    ```bash
    cd 
    mkdir master_ws
    ```

1. The `master_ws` is the ROS workspace on the Master computer. 
1. Change directories into this master workspace and create a subdirectory. 

    ```bash
    cd 
    mkdir src
    ```


    ```{tip} In Linux, if you select text using the mouse, you can paste it by clicking the middle mouse button.
    ```







1. Run the commands to move to the home folder and make a new directory:

    ```bash
    cd 
    mkdir my_folder
    ```

    ```{tip} In Linux, if you select text using the mouse, you can paste it by clicking the middle mouse button.
    ```
    
1. Change directories into your new folder and create a new bash script to drive to drive the TurtleBot3:

    ```bash
    cd my_folder
    touch move_turtlebot.sh
    ```

1. A bash script is a regular text file that allows you to run any command you would normally execute within the terminal. We will use it to run a few ROS command-line tools to move our TurtleBot.

1. Use the `Nano` text editor to edit files directly within the terminal. While there are several terminal-based text editors available (and endless debates about which is best, such as Vim or Emacs), Nano is a simple, user-friendly option ideal for quick edits. You are welcome to use whichever editor you prefer, but the instructions in this course will focus mainly on Nano.

    ```{note} On the Master computer, you can use GUI-based editors such as VS Code, Gedit, or Sublime Text. However, we will start with a terminal-based editor to build familiarity with command-line tools. Note that on the Raspberry Pi inside the robot, GUI-based options are not available.    
    ```

1. Open the new bash script for editing:

    ```bash
    nano move_turtlebot.sh
    ```
1. Copy the following code into the script:

    ```bash
    #!/bin/bash

    ARG1=$1

    if [ $ARG1 == 'forward' ]; then
        rostopic pub /cmd_vel geometry_msgs/Twist "linear:
        x: 0.15
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0"
            
    elif [ $ARG1 == 'rotate' ]; then
        rostopic pub /cmd_vel geometry_msgs/Twist "linear:
        x: 0.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.5"
            
    elif [ $ARG1 == 'stop' ]; then
        rostopic pub /cmd_vel geometry_msgs/Twist "linear:
        x: 0.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0"
    else
    echo "Please enter one of the following:
        forward
        rotate
        stop"
    fi
    ```

1. Typing `ctrl+s` saves the file and then typing `ctrl+x` exits Nano.

1. Again, a bash script just runs commands exactly as you would within a terminal. The above code takes an argument and publishes a *Twist* message over the `/cmd_vel` topic to drive the robot accordingly.

1. Before running this script, let's get our ROS environment setup:

1. Open a new terminal and type `roscore`.
1. Open a new terminal tab and run our TurtleBot3 simulation: 


    `ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch`

Now, in a new terminal, run the script you created:

`./move_turtlebot.sh`

Did you get an error? That is because the permissions have not been properly set and you do not have execute permissions. You can observe the permissions of a file by typing `ls -la`.

For the `move_turtlebot.sh` file you should see `-rw-rw-r--`. The first position indicates file type ('d' for directory, '-' for regular file), the next three positions indicate read (r), write (w), and execute (x) permissions for the file owner (u), the next three indicate permisions for the group owner of the file (g), and the last three characters indicate permissions for all other users (o).

You can change the permissions of a file or directory using the command `chmod`:

> ⌨️ **Syntax:**  `chmod <groups to assign the permissions><permissions to assign/remove> <file/folder>`

For example, if we wanted to give the Owner execute permissions, you can enter the command:

`chmod u+x move_turtlebot.sh` 

Typically we will give all users executable permissions (`chmod +x move_turtlebot.sh`). This isn't the most secure thing to do, but in our controlled environment, it isn't an issue. If you type `ls -la` now you should see the 'x' permission added for each permission group (`-rwxrwxr-x`).

Try running your script again:

`./move_turtlebot.sh rotate`

> 📝️ **Note:** You can remove permissions by utilizing the '-' character instead of '+'.

Move to your github repo that you previously created:

`cd ~/master_ws/src/ece387_master_sp23-USERNAME/master/`

Now we are going to create a new ROS package.  You learned this in the previous homework assignment, but we will continue practicing:

`catkin_create_pkg module02 std_msgs rospy roscpp`

Before we go any further, it is always a good idea to compile your workspace with the new package.  To do that, we will use a command `catkin_make` from within the top-level of the workspace:

`cd ~/master_ws`

`catkin_make`

Now go back to the `my_folder` that you created at the beginning of the lesson and create a new bash script, `bash_script.sh`, to accomplish the following:

1. Moves into the package you just created (you will need to figure out the complete path to this folder)
1. Creates a directory called `my_scripts`
1. Moves into that directory
1. Creates a file called **move_turtlebot_square.py**
1. Lists all files showing the permissions
1. Modifies the permissions of the **move_turtlebot_square.py** file so all groups have executable permissions
1. Lists all files again showing the updated permissions

Now run the script. If successful you should see the file listed without and then with execute permissions.

We are done with this script so let's remove it (you will need to either use the complete path to the file you want to remove, or be in the directory of the file you want to remove). The `rm` command can remove folders or files. If you want to learn more about a command there are two helpful tools: **Help**: `rm --help`; **Manual**: `man rm`.

Type the following to remove our bash script: 

`rm bash_script.sh`. 

```{note} 
To delete a whole folder add the `-r` tag to remove directories and thier contents recursively (e.g., `rm -r my_folder`, but don't remove your folder just yet).
```

We can copy (`cp`, just like ctrl+c in a GUI) and move (`mv`, just like ctrl+x in a GUI) files and folders as well. Let's copy the `move_turtlebot.sh` to the `my_scripts` folder you created earlier:

`cp move_turtlebot.sh ~/master_ws/src/ece387_master_sp23-USERNAME/master/module02/my_scripts`

> ⌨️ **Syntax:**  `cp <source> <destination>`

```{note} 
For the above to work, you must already be in the same folder as the `move_turtlebot.sh` file. Otherwise you have to use the absolute file path, such as `~/my_folder/move_turtlebot.sh`.
```

You can now delete your `my_folder` folder.

`cd ..`

`rm -r my_folder`

Change directories to your `my_scripts` folder. We can use a ROS tool, `roscd`, to change directories to ROS packages without using the absolute file path:

`roscd module02/my_scripts`

> ⌨️ **Syntax:**  `roscd <package/folder>`

Edit the **move_turtlebot_square.py** file and paste the following contents:

```python
#!/usr/bin/env python3
import rospy, time, math
from geometry_msgs.msg import Twist

class MoveTurtleBot():
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)    # 10 Hz
        
    def publish_cmd_vel_once(self):
        """
        In case publishing fails on first attempt
        """
        while not self.ctrl_c:
            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
                
    def shutdownhook(self):
        rospy.loginfo("Shutting down. Stopping TurtleBot!")
        self.stop_turtlebot()
        self.ctrl_c = True
        
    def stop_turtlebot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_cmd_vel_once()
        
    def move_time(self, moving_time = 10.0, lin_spd = 0.2, ang_spd = 0.2):
        self.cmd.linear.x = lin_spd
        self.cmd.angular.z = ang_spd
        
        self.publish_cmd_vel_once()
        time.sleep(moving_time)
        
    def move_square(self):
        i = 0
        
        while not self.ctrl_c:
            # Move Forward
            self.move_time(moving_time = 2.0, lin_spd = 0.2, ang_spd = 0)
            # Turn
            ang_spd = 0.5    # rad/sec
            moving_time = math.radians(90)/ang_spd
            self.move_time(moving_time = moving_time, lin_spd = 0.0, ang_spd = ang_spd)
            
        
if __name__ == '__main__':
    rospy.init_node('move_turtlebot')
    move_object = MoveTurtleBot()
    try:
        move_object.move_square()
    except rospy.ROSInterruptException:
        pass
```

There is a lot going on in this script, but hopefully after the previous lesson some of it makes sense. To summarize, the script creates a node, `move_turtlebot`, that publishes **Twist** messages to the **/cmd_vel** topic to drive the robot in a square: drive forward, turn 90 degrees, drive forward, repeat. This script will run until killed using `ctrl+c`.

The script is already executable, so you can run it using ROS!

`rosrun module02 move_turtlebot_square.py`

```{note} Note:** It won't be a perfect square as the robot doesn't turn perfectly, but it will be close!
```

The robot is now driving in a square until the script is killed. If you select the terminal and hit `ctrl+c`, it will kill the script.

Run the script again and this time hit `ctrl+z`. You can see that the robot is still running, but the commands are not updating. This is because `ctrl+z` suspends the current process, but does not kill it. We can observe all running processes on Linux by typing `ps -faux`. As you can see, there are a lot! The `grep` command allows us to filter these processes. Try the following:

`ps -faux | grep move_turtlebot_square.py`

The first entry should be our process and the leftmost number is the process ID (PID). We can kill any process using this number and the `kill` command:

`kill PID` replacing PID with the number listed. 

If you hit enter again, you should see that the process was killed. Unfortunately, the TurtleBot will just continue to execute the last command sent, so you need to kill the simulation as well. Just select that terminal and hit `ctrl+c`.

The `grep` tool is very powerful and can be used with any Linux command. For example, if you wanted to see all turtlebot packages available to us, we could type the following:

`rospack list`

There are a lot, so this isn't very helpful, but we can filter this command!

`rospack list | grep turtlebot`

The vertical line, '|', pipes the results of the first command into the second command, so we can filter all packages looking only for turtlebot packages.

## ROS
Below you will run ROS commands. The "!" character in the front allows us to run bash commands from Jupter and would **NOT** be used in the command line.

Accomplish the following on the master by adding the commands necessary below:

List all running nodes:


```python

```

List the active topics:


```

```

Display running nodes and communication between them:


```python

```

Exit the rqt_graph.

Show information about a the **/cmd_vel** topic such as what type of messages are sent over the topic and publishing and subscribing nodes.


```python

```

Display information about the message that is sent over the **/cmd_vel** topic.


```python

```

Display messages sent over the **/cmd_vel** topic:


```python

```

## Checkpoint
Once complete, get checked off by an instructor showing the output of each of the above.

```{note} You will use all of the above ROS commands for each lab to write your lab reports. You could create a bash script to run these commands automatically ;)
```

Additionally, place screen captures showing each of the commands running or the windows they bring up into a folder within ../module02/Pictures on your master repo.  Then push the repo to get credit for this work.

## Summary
You have seen a lot of different Linux/ROS commands during this lesson but it only scratches the surface. There are tons of online resources available if you want to learn more. I recommend working through a few of these tutorials: http://www.ee.surrey.ac.uk/Teaching/Unix/. They are fairly quick and willl give you more insight into the tools we discussed.

## Cleanup
In each terminal window, close the node by typing `ctrl+c`. Exit any SSH connections. In each of the notebooks reset the Jupter kernel and clear output. Now it is safe to exit out of this window. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.
