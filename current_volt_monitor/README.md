# Waypoint Navigation using a GPS and IMU

This set of instructions supplements your lab handout.


### Create a publisher for the Adafruit 9DOF sensor.
1. Provided in this Lab 7 Github folder you will see a Python script titled, 'imu_9dof_driver.py'.  Your task is to convert this code into a ROS publisher.
    + You will see that the driver reads the chip's accelerometer, magnetometer, and gyroscope.  
    + From the accelerometer and magnetometer data, the code calculates a tilt-compensated heading.  Other information such as roll and pitch angles are also calculated.
    + At a minimum, your code must publish the tilt-compensated heading to a topic, which you will use to navigate your robot toward a waypoint.   
    + You should publish this information at a rate of 10 Hz.  The sensor is capable of providing data at faster rate, but this isn't necessary for your application.  

### Incorporate the example code for calculating bearing and distance in your ROS waypoint navigator node.
1.  Provided in this Lab Github folder you will see a Python script titled, 'calc_bearing_and_distance.py'.  Experiment with this code and validate its operation using Google Maps to obtain sets of Lat/Lon coordinates.  Also, validate the accuracy of the UTM function by obtaining UTM coordinates from the following [website](https://mappingsupport.com/p/coordinates-utm-google-maps.html).  You can use the Python script to compare the output of both methods for obtaining bearning and distance.
2.  Use these functions in your navigation node that controls the Traxxas.  
3.  You may wish to only use the bearing and distance resulting from UTM coordinates, or you may wish to average the result of both methods before providing your robot a direction to follow toward a waypoint.

### Record waypoints to a csv file using the provided ROS package.
1.  There is a ROS package in this Github folder titled 'gps_waypoint_recorder'.  Download this directory and copy onto your Raspberry Pi in the `~\catkin_ws\src` directory.  Afterwards, type `cd ..` to get to the `\catkin_ws` directory, then enter `catkin_make`.  Finally, enter `rospack profile`.  Once finished, the package should be installed.  You can check by entering `rospack find gps_waypoint_recorder`.   
2.  Review the contents of the package and ensure you understand the code.
3.  If you reviewed the code thoroughly, then you should know that a button is required and which GPIO pin it should be connected.
4.  Use the launch file provided to start all the nodes needed to record GPS data (i.e., lat, lon, utm_x, and utm_y).
5.  After launching the waypoint recorder, type `rqt_graph` in a new terminal window.  Do you understand what all the nodes are doing?  For instance, which node is providing the UTM coordinates?
6.  Use this waypoint recorder package to log GPS waypoints to a CSV file.  Once started, take your Traxxas to a desired waypoint, then press the pushbutton.  It will record the latitude/longitude & UTM coordinates to a CVS file.  You can repeat this process of moving the Traxxas to as many waypoints as desired.     
7.  Note:  The program is designed to "append" newly recorded waypoints to the csv file.  This allows a series of different waypoints to be included in the same file without overwriting the previous ones.  However, you will need to manually delete old waypoints from the csv file if they are no longer desired.  Therefore, it is best to inspect the waypoint csv before commanding your robot to proceed to a series of waypoints. 

### Read waypoints from a CSV.
1.  There are two files in this Github directory that demonstrate how to read only one waypoint from a file containing multiple waypoints.  These files are called 'waypoint_reader_example.py' and 'waypoints.csv'.  
2.  Given the scenario where a robot must travel to a series of consecutive waypoints, the Python script shows how to extract only one waypoint (i.e., a row) from a file that contains multiple waypoints (i.e., rows).  
3.  The csv file is provided with a list of several waypoints so that the Python script can be tested.
4.  You should incorporate this code into your own navigation node.  For instance, the node should initially read in the first waypoint from the file and then travel to that point.  Once reached, it should read in a new waypoint and continue the process until there are no more waypoints in the list.

### Use callbacks to update global variables asynchronously, while processing state machine logic using an indefinite while() loop.
1.  Despite multiple nodes running concurrently in ROS, it is possible to implement a state machine using an indefinite while() loop that executes in a sequential fashion.  The use of callbacks in the publisher/subscriber framework of ROS means that multiple processes are running in parallel and that variables are changing asynchronously, but this doesn't mean we can't control logic and decision making to execute in sequential order.  
2.  The trick is to use callbacks only for updating global variables which are referenced whenever needed in a "main" while() loop.  The purpose of the callbacks is to continually update the global variables, so that when they are referenced, the latest information is being used in the while() control loop.  
3.  In this Github folder, there is a Python script titled, "ROS_while_loop_example.py".  It is template which shows how to implement such a model where callbacks update global variables and a while() loop executes instructions sequentially.  Pay particular attention to the comments provided in the script and why the spin() instruction, which is normally used in simple subscribers, is not used in this type of control (i.e., state machine) node.  
