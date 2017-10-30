#!/usr/bin/env python

#import necessary message types
import rospy, numpy, math
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import UInt32
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

#initialize global variables
orient = [0,0,0,0]
angular_vel = [0,0,0]
linear_accel = [0,0,0]
vel = [0,0,0]
velocity = 0
heading = 0
pitchAngle = 0
rollAngle = 0

#calculates the tilt compensated heading
def calculate_heading(orientation,pitch,roll):
     x = (orientation[0] * math.cos(pitch)) + (orientation[2] * math.sin(pitch))
     y = (orientation[0] * math.sin(roll) * math.sin(pitch)) + orientation[1]*math.cos(roll)-(orientation[2]*math.sin(roll)*math.cos(pitch))
     heading = math.atan2(y,x)*(180/math.pi)
     if heading < 0:
          heading = heading +360
     return heading

#calculates the pitch angle
def calculate_pitchAngle(linear_acceleration,roll):
     pitch = math.atan2(-linear_acceleration[0], ((linear_acceleration[1] * math.sin(roll)) + (linear_acceleration[2] * math.cos(roll))))
     return pitch

#calculates the roll angle
def calculate_rollAngle(linear_acceleration):
     roll = math.atan2(linear_acceleration[1],linear_acceleration[2])
     return roll

#calculates velocity vector
def calculate_velocity(vel):
     v = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
     return v

#receives information from /xsens/imu/data
def imu_callback(data):
     global heading
     global pitchAngle
     global rollAngle
     orient = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
     angular_vel = [data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z]
     linear_accel = [data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z]
     rollAngle = calculate_rollAngle(linear_accel)
     pitchAngle = calculate_pitchAngle(linear_accel,rollAngle)
     heading = calculate_heading(orient,pitchAngle,rollAngle)

#receives information from /xsens/velocity
def velocity_callback(data):
     global velocity
     vel = [data.twist.linear.x,data.twist.linear.y,data.twist.linear.z]
     velocity = calculate_velocity(vel)

#subscribes to xsens/imu/data AND xsens/velocity
def listener():
    #rospy.init_node('xsens_monitor',anonymous=True)
    rospy.Subscriber("/imu/raw",Imu,imu_callback)
    rospy.Subscriber("/velocity",TwistStamped,velocity_callback)
    #rospy.spin()

#publishes an array of float values containing the tilt compensated heading, pitch angle, roll angle, and velocity
def talker():
    #rospy.init_node('condensed_imu',anonymous=True)
    pub = rospy.Publisher('heading_info',Float64MultiArray,queue_size=10)
    #rospy.init_node('condensed_imu',anonymous=True)
    rate = rospy.Rate(10)
    dim = [MultiArrayDimension('heading,roll angle, pitch angle',3,3), MultiArrayDimension('velocity',1,1)] #string label, int size of given dimension, int stride of given dimension
    layout = MultiArrayLayout(dim,0) #array of dimension properties, data offset for padding
    while not rospy.is_shutdown():
        heading_info = Float64MultiArray(layout,[heading,rollAngle,pitchAngle,velocity])
        pub.publish(heading_info)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('condensed_imu',anonymous=True)

    try:        
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass
