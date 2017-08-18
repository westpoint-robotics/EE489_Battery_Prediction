#!/usr/bin/env python
import rospy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


####### Global Variables ###############################
utm_xCoord = 0
utm_yCoord = 0
lat = 0
lon = 0


####### Functions (non-callback) #######################

def linearMap(val, in_min, in_max, out_min, out_max):
    return int ((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


####### Callbacks ######################################
def latLon_callback(data):
    global lat
    global lon
    lat = data.latitude
    lon = data.longitude
    #calculate distance & bearing

def utm_callback(data):
    global utm_xCoord
    global utm_yCoord
    utm_xCoord = data.pose.pose.position.x
    utm_yCoord = data.pose.pose.position.y
    #calculate distance & bearing
	
####### Node Definitions ###############################   

def gps_subscriber():

    rospy.init_node('navigator_node', anonymous=True)

    rospy.Subscriber("fix", NavSatFix, latLon_callback)
    rospy.Subscriber("odom",Odometry, utm_callback)
    
    # Don't use rospy.spin() as shown in subscriber tutorials;
    # For a subscriber that just waits for messages to be published, then 
    # spin() keeps Python from exiting until the node is stopped; however, 
    # spin() causes problems if we wish to loop indefinitely using a while() loop;
    # the while() loop below will keep node alive until the node is shutdown;
    

if __name__ == '__main__':
    try:    
        gps_subscriber() # define subscriber node
        
        rate = rospy.Rate(10) # 10Hz; control the rate of your while loop here, instead of using Python's time library
        while not rospy.is_shutdown():    
            # enter state machine here
            
            rate.sleep()
    except KeyboardInterrupt:
        print "keyboard interrupt...exiting"
    finally:
        pwm.setPWM(0, 0, neutral) # initialize steering to neutral
        pwm.setPWM(1, 0, neutral) # initialize speed to neutral
            



