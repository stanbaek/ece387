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

