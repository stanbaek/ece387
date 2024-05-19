
# Master Setup
This guide will walk through the steps to install Ubuntu Desktop 20.04 LTS, ROS Noetic, and all dependencies on a desktop computer. This computer system is utilized in the United States Air Force Academy's Electrical and Computer Engineering department in an embedded network with the ground robot, a TurtleBot3 Burger. The master system is used to host *roscore*, utilize ROS GUI tools, and create secure connections with the TurtleBot3. This guide is adapted from the [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview).

---


## Hardware
For our application, we are using [Intel NUC Kits](https://www.intel.com/content/www/us/en/products/details/nuc/kits.html) but these instructions will work on any AMD64 architecture. 


## Software
### Download Ubuntu and flash USB
For the desktop machine you will first need to download [Ubuntu Desktop 20.04 LTS](https://releases.ubuntu.com/focal/). 

Once downloaded, follow the instructions to create a [bootable Ubuntu USB stick](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview) within Ubuntu. The guide provides links to create USB sticks from Windows and macOS as well.

Once the bootable USB stick is created, follow the guide to [Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) selecting a useful computer name such as `master0`. The NUC requires you to press and hold F10 on startup to boot from a USB stick.

<!-- #region -->
#### Setup GitHub SSH Keys
The following assumes you already have a GitHub account.

Create SSH keys to use with your GitHub account by typing the following using the same email as you GitHub login:

```bash
cd
ssh-keygen -t ed25519 -C "github@email.com"
```

When prompted to "Enter a file in which to save the key", hit **enter**.

At the prompt, type a secure password.

Start the ssh-agent in the background and add your SSH private key to the ssh-agent:

```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

Open the public key with your favorite command line editor (this is easier to accomplish via an SSH connection from a desktop machine with a GUI so you can copy the public key to your GitHub account).

```bash
nano ~/.ssh/id_ed25519.pub
```

Copy the contents of the file (maximize the window and ensure you copy the entire contents up to the GitHub email).

Open a web browser and sign in to your GitHub account.

In the upper-right corner of any page, click your profile photo, then click **Settings**:

![logo](Figures/ssh1.png)

In the user settings sidebar, click **SSH and GPG keys**:

![logo](Figures/ssh2.png)

Click **New SSH key**:

![logo](Figures/ssh3.png)

In the "Title" field, add a descriptive label for the new key, such as "master0".

Paste your key into the "Key" field (contents of the `.pub` file).

Click **Add SSH key**.
<!-- #endregion -->

<!-- #region -->
#### Update Alternatives
Python3 is installed in Ubuntu20 by default. Some ROS packages utilize the "python" command instead of "python3" so we need to create a new executable, "/usr/bin/python" that will call Python3 (basically use the command "python" to call Python3):

```bash
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10
```

#### Additional Software

```
sudo apt install jupyter-notebook
jupyter contrib nbextension install --user
```
<!-- #endregion -->

<!-- #region -->
### ROS Noetic
At this point, the Ubuntu environment is setup. Now we will setup the ROS requirements for the master. All of these instructions are adapted from the [ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu). ROS Noetic is the latest version of ROS 1 that supports Ubuntu Focal.

#### Installation
Accept software from packages.ros.org:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up keys:

```bash
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Install ROS Noetic:

```bash
sudo apt update
sudo apt -y install ros-noetic-desktop-full
```

The full version provides theminimum packaging, build, communications libraries, GUI tools, and 2D/3D simulation and perception packages.

Install ROS dependencies for building packages:

```bash
sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-pip xterm build-essential
```

Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

Source the ROS setup file:
```bash
source /opt/ros/noetic/setup.bash
```

Create your ROS workspace:

```bash
mkdir -p ~/master_ws/src
cd ~/master_ws/
catkin_make
```

Setup ROS environment variables and setup scripts within the `~/.bashrc` file. Open the `~/.bashrc` file with your favorite command line editor and add the following to the bottom:

```bash
source /opt/ros/noetic/setup.bash
source ~/master_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/master_ws/src:/opt/ros/noetic/share
export ROS_HOSTNAME=`hostname` # note these are backticks, not apostrophes
export ROS_MASTER_URI=http://MASTER_IP:11311 # replace "MASTER_IP" with IP address/hostname of your master
export EDITOR='nano -w' # replace with editor of choice used with rosed command
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01 # replace with LDS-02 if using new LIDAR
```

Any time you make changes to your `~/.bashrc` file you must source it:

```bash
source ~/.bashrc
```

#### Dependencies
There are a number of ROS packages required to operate the TurtleBot3.

##### ROS Dependencies
```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

```

##### TurtleBot3 Dependencies
```bash
sudo apt install ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3
```

##### [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation):
```bash
cd ~/master_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

##### [ECE387 Curriculum](https://github.com/AF-ROBOTICS/ece387_curriculum)
```bash
git clone git@github.com:AF-ROBOTICS/ece387_curriculum.git
```

The **ece387_curriculum** package includes all dependencies needed to run the TurtleBot3 nodes. We can automatically install these dependencies using the ROSDEP tool:

```bash
cd ~/master_ws
rosdep install --from-paths src --ignore-src -r -y
```

This may take a while.

Now we can make and source our workspace:

```bash
catkin_make
source ~/.bashrc
```

The last set of dependencies we need to install are Python dependencies. These are listed within our **ece387_curriculum** package and can be installed using the `pip3` tool:

```bash
roscd ece387_curriculum
pip3 install -r requirements.txt
```

> 📝️ **Note:** the "dlib" package will take quite a while to install.
<!-- #endregion -->
