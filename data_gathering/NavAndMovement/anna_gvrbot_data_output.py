#!/usr/bin/env python

#import necessary message types
import rospy
import csv
import time
from gvrbot.msg import GvrbotMobilityData
count = 0

#lists for holding data
Rcurrent_list = []
Lcurrent_list = []
Rspeed_list = []
Lspeed_list = []
Lencoder_list = []
Rencoder_list = []
Lvelocity_list = []
Rvelocity_list = []
distance_list = []


feetperbit = 0.000176811
Lold = -5
Rold = -5
Lencodercount = 0
Rencodercount = 0

def calculateEncoder(current,old):
    change = abs(current-old)

    if(change > 1000): #the encoder count restarted (2^16)
        if(current - old > 0): #negative slope restart
            change = old + (65536-current)
        else: #positive slope restart
            change = (65536-old)+current

    return change

#receives information from /gvrbot_mobility_data
def mobilityData_callback(data):
    with open('mobilitydata.csv','a') as f:
        writer = csv.writer(f)
        global count
        global Lold
        global Rold
        global Lencodercount
        global Rencodercount 
        if count==0: #enters blank row for new data group
            writer.writerow(["","","","","","","",""])
            writer.writerow(["Time (s)","LEncoder","REncoder","LPosition","RPosition","LVelocity","RVelocity","Distance"])
        count += 1
        RCurrent = data.right_motor_current
        LCurrent = data.left_motor_current
        RSpeed = data.right_drive_motor_speed
        LSpeed = data.left_drive_motor_speed
        REncoder = data.right_drive_motor_raw_encoder_count
        LEncoder = data.left_drive_motor_raw_encoder_count

        #calculates the position (feet) from known encoder data
        if(Lold == -5):
            Lold = LEncoder
            Rold = REncoder
        Lencodercount += calculateEncoder(LEncoder,Lold)
        Lold = LEncoder
        Rencodercount += calculateEncoder(REncoder,Rold)
        Rold = REncoder
        Lposition = Lencodercount * feetperbit
        Rposition = Rencodercount * feetperbit
        #end of position calculation block
        RVelocity = data.right_track_velocity
        LVelocity = data.left_track_velocity
        Distance = data.distance_travelled
        writer.writerow([time.time(),LEncoder, REncoder, Lposition,Rposition, LVelocity, RVelocity, Distance])


#subscribes to /gvrbot_mobility_data
#Message Type: gvrbot/GvrbotMobilityData
def listener():
    rospy.init_node('gvrMobility_data', anonymous=True)
    rospy.Subscriber("/gvrbot_mobility_data",GvrbotMobilityData,mobilityData_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
