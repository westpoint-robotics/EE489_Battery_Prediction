#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

lat = 0.0
lon = 0.0
utm_xCoord = 0.0
utm_yCoord = 0.0
prev_button_state = False

def latLon_callback(data):
    global lat
    global lon
    lat = data.latitude
    lon = data.longitude

def utm_callback(data):
    global utm_xCoord
    global utm_yCoord
    utm_xCoord = data.pose.pose.position.x
    utm_yCoord = data.pose.pose.position.y
	
def button_callback(button):
    global prev_button_state
    #Look for transition in button state (i.e., look for toggle in state)
    if button.data != prev_button_state:
        print "Recording GPS & UTM coordinates for this location"
        with open("/home/eecs/catkin_ws/src/gps_waypoint_recorder/src/wayPoints.csv", "a") as wayPointFile:
            wayPointFile.write(str(lat)+','+str(lon)+','+str(utm_xCoord)+','+str(utm_yCoord)+'\n')
        print "Finished recording point"
    prev_button_state = button.data

def gps_subscriber():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('coord_recorder', anonymous=True)

    rospy.Subscriber("fix", NavSatFix, latLon_callback)
    rospy.Subscriber("odom",Odometry, utm_callback)
    rospy.Subscriber("button_state",Bool, button_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    gps_subscriber()
