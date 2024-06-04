# 🙋 FAQ

(faq-general)=
## General

### _/usr/bin/env: ‘python3\r’: No such file or directory_

$ rosrun lab1 mouse_client.py
/usr/bin/env: ‘python3\r’: No such file or directory


The problem are your line ending characters. Your file was created or edited on a Windows system and uses Windows/DOS-style line endings (CR+LF), whereas Linux systems like Ubuntu require Unix-style line endings (LF).

- Sublime: Open the desired file with Sublime and from the top menu select View -> Line Endings and then the Windows(CRLF) or Unix(LF). That’s it.
- VS Code: At the bottom right of the screen in VS Code there is a little button that says `LF` or `CRLF`: Click that button and change it to your preference.

<br>


### Gazebo Error

If you have an error when you run the following command
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

The error message looks like:

```bash
[gzclient-2] gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion `px != 0' failed.
[ERROR] [gzclient-2]: process has died [pid 7768, exit code -6, cmd 'gzclient'].
```

Then, the solution is to source Gazebo's setup file, i.e.:

```bash
. /usr/share/gazebo/setup.sh
```

This is needed to set some necessary environment variables in case they're going to be overridden, which is a common use case. 



### _ERROR: cannot launch node of type_

```
[robot98-0]: ERROR: cannot launch node of type [lab4/stop_detector.py]: lab4
ROS path [0]=/opt/ros/noetic/share/ros
ROS path [1]=/opt/ros/noetic/share
```

Ensure you have the `env-loader` field in the `machine` block of the lauch file.
```
    <machine
      name="robot0"
      address="robot0"
      env-loader="/home/pi/robot_ws/devel/remote_env_loader.sh"
      default="true"
      user="pi"
    />
```

