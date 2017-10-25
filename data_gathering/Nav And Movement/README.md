The "imu_data_output.py" file subscribes to the Xsens imu in order to gather information on the GPS latitude and 
longitude, orientation, and velocity. This program publishes to the topic "\heading_info."

In order to run this code, make sure that you run: "roslaunch usma_xsens mti300.launch" and then "python imu_data_output.py"

Working On: putting all manual launch commands into the MTI300 launch file, integrating the latitude and longitude (because 
there is no fix in the basement).
