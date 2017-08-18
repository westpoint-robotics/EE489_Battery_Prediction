# urg_node

This set of instructions will help you install and test the Hokuyo urg_node ROS package.  Details about the package can be found [here](http://wiki.ros.org/urg_node).


### Install the driver (ROS Package) for publishing the laser scan data.
1. Install the package
    + `sudo apt-get install ros-kinetic-urg-node`
2. Change directories to the root of your Catkin Workspace: 
    + `cd ~/catkin_ws`
    + `catkin_make`
3. Configure the Hokuyo Laser; change directories to where usb device (udev) rules are stored.
    + `cd /etc/udev/rules.d/`
4. Switch to super user mode in order to update Udev rules:
    + `sudo su`
5. Create udev rules so that the read-write permissions for the laser are automatically setup every time it is connected.
    + `nano 99-urg.rules`
  - Add the following line in the newly created file and then save upon exit by entering Ctrl-x, then answer 'y', and finally hit 'Enter'.
    + `SUBSYSTEMS=="usb", ATTRS{idVendor}=="15d1", ATTRS{idProduct}=="0000", MODE="0666", SYMLINK+="lidar_hokuyo", GROUP="dialout"`
6. Reload the rules
    + `udevadm control --reload-rules`
7. Exit from super user mode
    + `exit`
8. Check to ensure your user account is part of the 'dialout' group.  As part of the 'dialout' group, your user account will have read and write access to the tty serial ports. 
    + `groups`
    + See if 'dialout' is listed in the terminal output.  If not, then add your user account to the 'dialout' group by typing:
    + `sudo adduser your_username dialout`
    + Note:  You won't see that your user account has been added to the dialout group until you log out and log back in.  

###  Test the Hokuyo Laser

1. Ensure the URG-04LX is connected to two USB ports on the computer. 
2. Start ROS core
    + `roscore`

3. In a seperate terminal, start the publisher node by typing:
    + `rosrun urg_node urg_node`

4. If the node started correctly, then you should be able to see the /scan topic by entering:
    + `rostopic list`
5. You can echo (or display) the information being published to that topic by typing:
    + `rostopic echo /scan`
6. You can gain more information about the /scan topic, as well as more information on the data structure of the LaserScan message type by entering these commands:
    + `rostopic info /scan`
    + `rosmsg show sensor_msg/LaserScan`
7. You can also see more information about a particular message type by using online [ROS documentation](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
8. The LaserScan data structure is part of the [sensor_msgs](http://wiki.ros.org/sensor_msgs) group of data types, but it is also common in ROS to use data types a part of [std_msgs](http://wiki.ros.org/std_msgs).

