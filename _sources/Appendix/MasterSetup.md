
# Master Setup
This guide will walk through the steps to install Ubuntu Desktop 22.04 LTS, ROS2 Humble, and all dependencies on a desktop computer. This computer system is utilized in the United States Air Force Academy's Electrical and Computer Engineering department in an embedded network with the ground robot, a TurtleBot3 Burger. The master system is used to utilize ROS GUI tools and create secure connections with the TurtleBot3. 

This guide is adapted from the [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview).

---


## Hardware
For our application, we are using [Intel NUC Kits](https://www.intel.com/content/www/us/en/products/details/nuc/kits.html) but these instructions will work on any AMD64 architecture. 


## Software
### Download Ubuntu and flash USB
For the desktop machine you will first need to download [Ubuntu Desktop 22.04 LTS](https://releases.ubuntu.com/jammy/). 

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

```{image} ./Figures/ssh1.png
:width: 200
:align: center
```


In the user settings sidebar, click **SSH and GPG keys**:

```{image} ./Figures/ssh2.png
:width: 200
:align: center
```

Click **New SSH key**:

```{image} ./Figures/ssh3.png
:width: 600
:align: center
```
<br>

In the ``Title`` field, add a descriptive label for the new key, such as ``master0``.

Paste your key into the ``Key`` field (contents of the `.pub` file).

Click **Add SSH key**.

#### Update Alternatives
Python3 is installed in Ubuntu20 by default. Some ROS packages utilize the ``python`` command instead of ``python3`` so we need to create a new executable, ``/usr/bin/python`` that will call Python3 (basically use the command ``python`` to call ``Python3``):

```bash
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10
```


### ROS2 Humble

At this point, the Ubuntu environment is setup. Now we will setup the ROS 2 requirements for the master. All of these instructions are adapted from the [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/Installation.html). ROS 2 Humble is the latest version of ROS 2 that supports Ubuntu 22.04 LTS (Jammy Jellyfish).

#### Installation

Follow [the official ROS2 documentation](https://docs.ros.org/en/humble/Installation.html) to install the ROS2 Humble.
- For most Linux users, [Debian package installation method](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) is strongly recommended.
- Install ``ros-humble-desktop``.

Install ROS dependencies for building packages:

```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```

Install TurtleBot3 packages

```bash
source ~/.bashrc
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
```

Create your ROS workspace:

```bash
mkdir -p ~/master_ws/src
cd ~/master_ws/
```

Setup ROS environment variables and setup scripts within the `~/.bashrc` file. Open the `~/.bashrc` file with your favorite command line editor and add the following to the bottom:

```bash
source /opt/ros/humble/setup.bash
source ~/master_ws/install/setup.bash
export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> ~/.bashrc
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01 # replace with LDS-02 if using new LIDAR
```

Any time you make changes to your `~/.bashrc` file you must source it:

```bash
source ~/.bashrc
```


##### [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation):
```bash
cd ~/master_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```


> 📝️ **Note:** the "dlib" package will take quite a while to install.
<!-- #endregion -->
