#!/usr/bin/env python

#Version 2: Also connects and pushes information into a mysql database


#import necessary message types
import mysql.connector

import rospy
import csv
import time
import numpy
import math
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
from datetime import datetime
from gvrbot.msg import GvrbotMobilityData
from gvrbot.msg import GvrbotOrientation
import smbus2

bus = SMBus(1)

count = 0
orient = [0, 0, 0, 0]
heading = 0
pitchAngle = 0
rollAngle = 0
yawAngle = 0
Lposition = 0
Rposition = 0
RVelocity = 0
LVelocity = 0
Distance = 0
VinV = 0
AdinV = 0
DEBUG = False
voltage = 0
current = 0
temperature = 0
relsoc = 0
abssoc = 0
ttoempty = 0
remcap = 0
fullcap = 0
changingcurr = 0
changingvolt = 0
maxerror = 0

#database initiailization
db = mysql.connector.connect(host="192.168.200.82", user="root", password="toor", database="batterystudy")
curs = db.cursor()

#database table initialization
table = raw_input("Make a new table name: ")

##### need to add  addtable = "CREATE TABLE " + str(table) + "(time DECIMAL(20,2), temperature NUMERIC, voltage NUMERIC, current NUMERIC, relsoc NUMERIC, abssoc NUMERIC, ttoempty NUMERIC,remcap NUMERIC, fullcap NUMERIC, changecurr NUMERIC, changevolt NUMERIC, maxerror NUMERIC);"
##### also need to add a INTEGER for Vin and Adin
addtable = "CREATE TABLE " +str(table) + "(time DECIMAL(20,2), Lposition NUMERIC, Rposition NUMERIC, Lvelocity NUMERIC, Rvelocity NUMERIC, Distance NUMERIC);"

#table management
try:
    curs.execute(addtable)
except:
    newerror = raw_input("Are you sure you want to overwrite table (This will delete table?) y/n:")
    if newerror == ("y" or "yes"):
        deletetable = "DROP TABLE " + str(table)
        print "Deleting " + str(table)
        curs.execute(deletetable)
        curs.execute(addtable)
    else:
        print "Restart the program and use a different table name"
        time.sleep(2)
        exit()

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

#calculates the tilt compensated heading
def calculate_heading(orientation, pitch, roll):
    global heading
    x = (orientation[0] * math.cos(pitch)) + (orientation[2] * math.sin(pitch))
    y = (orientation[0] * math.sin(roll) * math.sin(pitch)) + orientation[1]*math.cos(roll)-(orientation[2]*math.sin(roll)*math.cos(pitch))
    heading = math.atan2(y, x)*(180/math.pi)
    if heading < 0:
        heading = heading +360
    return heading

def calculateEncoder(current, old):
    change = abs(current-old)

    if change > 1000: #the encoder count restarted (2^16)
        if current - old > 0: #negative slope restart
            change = old + (65536-current)
        else: #positive slope restart
            change = (65536-old)+current

    return change

def imu_callback(data):
    global heading
    global orient
    orient = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    heading = calculate_heading(orient, pitchAngle, rollAngle)

def orientation_callback(data):
    global rollAngle
    global pitchAngle
    global yawAngle
    rollAngle = data.roll
    pitchAngle = data.pitch
    yawAngle = data.yaw

#receives information from /gvrbot_mobility_data
def mobilityData_callback(data):
    global count
    global Lold
    global Rold
    global Lencodercount
    global Rencodercount
    global Lposition
    global Rposition
    global RVelocity
    global LVelocity
    global Distance
    RCurrent = data.right_motor_current
    LCurrent = data.left_motor_current
    RSpeed = data.right_drive_motor_speed
    LSpeed = data.left_drive_motor_speed
    REncoder = data.right_drive_motor_raw_encoder_count
    LEncoder = data.left_drive_motor_raw_encoder_count

    #calculates the position (feet) from known encoder data
    if Lold == -5:
        Lold = LEncoder
        Rold = REncoder
    Lencodercount += calculateEncoder(LEncoder, Lold)
    Lold = LEncoder
    Rencodercount += calculateEncoder(REncoder, Rold)
    Rold = REncoder
    Lposition = Lencodercount * feetperbit
    Rposition = Rencodercount * feetperbit
    #end of position calculation block
    RVelocity = data.right_track_velocity
    LVelocity = data.left_track_velocity
    Distance = data.distance_travelled

def pollLTC():
    global VinV
    global AdinV
    global bus
    VinV = bus.read_word_data(0x6c, 0x02)
    AdinV = bus.read_word_data(0x6c, 0x04)

def pollBatt():
    global voltage
    global current
    global temperature
    global relsoc
    global abssoc
    global ttoempty
    global remcap
    global fullcap
    global changingcurr
    global changingvolt
    global maxerror
    voltage = bus.read_word_data(0x0b, 0x09)  # voltage
    current = bus.read_word_data(0x0b, 0x0a)  # current
    temperature = bus.read_word_data(0x0b, 0x08)  # temperature
    relsoc = bus.read_word_data(0x0b, 0x0d)  # relative state of charge
    abssoc = bus.read_word_data(0x0b, 0x0e)  # absolute state of charge
    ttoempty = bus.read_word_data(0x0b, 0x12)  # Average time to empty
    remcap = bus.read_word_data(0x0b, 0x0f)  # Remaining Capacity
    fullcap = bus.read_word_data(0x0b, 0x10)  # Full charge Capacity
    changingcurr = bus.read_word_data(0x0b, 0x14) #Changing current
    changingvolt = bus.read_word_data(0x0b, 0x15) #changing voltage
    maxerror = bus.read_word_data(0x0b, 0x0c) # max error soc

def post():
    pollLTC() # AdinV #VinV
    pollBatt()
    ##### need to add  "INSERT INTO " + str(table) + "(temperature, voltage, current, relsoc, abssoc, ttoempty, remcap, fullcap, changecurr, changevolt, maxerror ) VALUES (" + str(temperature) + ", " + str(voltage) + "," + str(current) + "," + str(relsoc) + "," + str(abssoc) + "," + str(ttoempty) + "," + str(remcap) + "," + str(fullcap) + "," + str(changingcurr) + "," + str(changingvolt) + "," + str(maxerror) +  ")"
    ##### also need to add a INTEGER for Vin (Vinv) and Adin (AdinV)
    myquery = "INSERT INTO " + str(table) + "(time,Lposition, Rposition, LVelocity, RVelocity, Distance, heading, rollAngle, pitchAngle, yawAngle) VALUES (" + str(time.time())[4:]+", "+str(Lposition)+", "+str(Rposition)+", "+str(LVelocity)+", "+str(RVelocity)+", "+str(Distance)+", "+str(heading)+", "+str(rollAngle)+", "+str(pitchAngle)+", "+str(yawAngle)+")"
    curs.execute(myquery)
    db.commit()

#subscribes to /gvrbot_mobility_data
#Message Type: gvrbot/GvrbotMobilityData
def listener():
    rospy.Subscriber("/gvrbot_mobility_data", GvrbotMobilityData, mobilityData_callback)
    rospy.Subscriber("/gvrbot_orientation", GvrbotOrientation, orientation_callback)
    rospy.Subscriber("/gvrbot/imu/data", Imu, imu_callback)
    with open('mobilitydata.csv', 'a') as f:
        writer = csv.writer(f)
        try:
            while 1:
                writer.writerow([time.time(), Lposition, Rposition, LVelocity, RVelocity, Distance, heading, rollAngle, pitchAngle, yawAngle])
                post()
                time.sleep(1)
        except KeyboardInterrupt as e:
            print '[+] Ctrl+C detected. Exiting...'
            import sys
            sys.exit(0)


if __name__ == '__main__':
    inp = 2
    while inp not in ['0', '1', 'y', 'n']:
        inp = raw_input("Should debug mode be turned on? [y/n] [1/0]").lower()
        if inp in ['0','n']:
            DEBUG = False
        if inp in ['1', 'y']:
            DEBUG = True
    rospy.init_node('gvrMobility_data', anonymous=True)

    try:
        if DEBUG: print '[+] Starting'
        listener()
    except rospy.ROSInterruptException:
        pass
