#!/usr/bin/env python
import rospy,time,smbus,numpy,math,subprocess

#Global variables

#Device addresses
ACC_ADDRESS = 0x19      #7 bit address
MAG_ADDRESS = 0x1E 
GYRO_ADDRESS =0x6B 
#accelerometer-related registers
CTRL_REG1_A  = 0x20

CTRL_REG4_A  = 0x23
OUT_X_L_A = 0x28
OUT_X_H_A = 0x29
OUT_Y_L_A = 0x2A
OUT_Y_H_A = 0x2B
OUT_Z_L_A = 0x2C
OUT_Z_H_A = 0x2D
#magnetometer-related
CRA_REG_M = 0x00
MR_REG_M = 0x02
OUT_X_L_M = 0x04
OUT_X_H_M = 0x03
OUT_Z_L_M = 0x06
OUT_Z_H_M = 0x05
OUT_Y_L_M = 0x08
OUT_Y_H_M = 0x07
#gyro-related registers
CTRL_REG1_G = 0x20 #turns on gyro
CTRL_REG2_G = 0x21 #can set a high-pass filter for gyro
OUT_X_L_G = 0x28
OUT_X_H_G = 0x29
OUT_Y_L_G = 0x2A
OUT_Y_H_G = 0x2B
OUT_Z_L_G = 0x2C
OUT_Z_H_G = 0x2D

#define objects
bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

#initialize accelerometer
bus.write_byte_data(ACC_ADDRESS,CTRL_REG1_A,0b01000111) #x,y,z enabled; 50Hz; default +/-2g

#initialize magnetometer
bus.write_byte_data(MAG_ADDRESS,CRA_REG_M,0x14) #change the ODR from 15 Hz to 30 Hz
bus.write_byte_data(MAG_ADDRESS,MR_REG_M,0x00) #put in continuous mode (out of sleep mode)

#initialize gyroscope
bus.write_byte_data(GYRO_ADDRESS,CTRL_REG1_G,0x0F) #turn on gyro and set to normal operation

#define functions

def readSensor(sensorAdd,xH,xL,yH,yL,zH,zL):
    try:
        xMSB = numpy.uint8(bus.read_byte_data(sensorAdd,xH))
        xLSB = numpy.uint8(bus.read_byte_data(sensorAdd,xL)) 

        yMSB = numpy.uint8(bus.read_byte_data(sensorAdd,yH)) 
        yLSB = numpy.uint8(bus.read_byte_data(sensorAdd,yL)) 

        zMSB = bus.read_byte_data(sensorAdd,zH) 
        zLSB = bus.read_byte_data(sensorAdd,zL)
    except IOError:
        print "I2C IO Error"
        subprocess.call(['i2cdetect', '-y', '1']) #try to use i2cdetect command to reinitialize i2c bus

    x = (xMSB << 8) | xLSB
    x_float = numpy.int16(x) #value is expressed as two's complement, so need numpy casing as int16
    z = (zMSB << 8) | zLSB
    z_float = numpy.int16(z) #value is expressed as two's complement, so need numpy casing as int16
    y = (yMSB << 8) | yLSB
    y_float = numpy.int16(y) #value is expressed as two's complement, so need numpy casing as int16
    return x_float,y_float,z_float
        
def calcRollAngle(aY,aZ):
    rollAngle = math.atan2(aY,aZ)
    return rollAngle

def calcPitchAngle(aX,aY,aZ,rollAngle):
    pitchAngle = math.atan2(-aX, ((aY * math.sin(rollAngle)) + (aZ * math.cos(rollAngle))))
    return pitchAngle

def calcBasicHeading(x,y):
    headingDeg = math.atan2(y,x) * (180/math.pi)
    if (headingDeg<0):
            headingDeg = headingDeg+360
    return headingDeg

def tiltCompHeading(mX,mY,mZ,pitchAngle,rollAngle):
    Mx2 = (mX * math.cos(pitchAngle)) + (mZ*math.sin(pitchAngle))
    My2 = (mX * math.sin(rollAngle) * math.sin(pitchAngle)) + mY*math.cos(rollAngle) - (mZ*math.sin(rollAngle)*math.cos(pitchAngle))    
    #Mz2 = (-mX * math.cos(rollAngle)*math.sin(pitchAngle)) + mY*math.sin(rollAngle) + (mZ*math.cos(rollAngle)*math.cos(pitchAngle))    
    compHeadingDeg = math.atan2(My2,Mx2) * (180/math.pi)
    if compHeadingDeg <0:
        compHeadingDeg = compHeadingDeg + 360
    return compHeadingDeg

################### MAIN ###############################

	    
if __name__ == '__main__':
    try:
        while (True):
            #get magnetometer reading
            mX,mY,mZ = readSensor(MAG_ADDRESS, OUT_X_H_M, OUT_X_L_M, OUT_Y_H_M, OUT_Y_L_M, OUT_Z_H_M, OUT_Z_L_M)          
            #find basic (non-tilt compensated heading)
            headingDeg = calcBasicHeading(mX,mY)
            #Read accelerometer
            aX,aY,aZ = readSensor(ACC_ADDRESS, OUT_X_H_A, OUT_X_L_A, OUT_Y_H_A, OUT_Y_L_A, OUT_Z_H_A, OUT_Z_L_A)
            #Calculate roll and pitch angles
            rollAngle = calcRollAngle(aY,aZ)
            rollAngleDeg = rollAngle * (180/math.pi)
            pitchAngle = calcPitchAngle(aX,aY,aZ,rollAngle)
            pitchAngleDeg = pitchAngle * (180/math.pi)
            #find tilt compensated heading
            compHeadingDeg = tiltCompHeading(mX,mY,mZ,pitchAngle,rollAngle)
            print compHeadingDeg         
            #Read gyroscope
            gX,gY,gZ = readSensor(GYRO_ADDRESS, OUT_X_H_G, OUT_X_L_G, OUT_Y_H_G, OUT_Y_L_G, OUT_Z_H_G, OUT_Z_L_G)
            
            #sleep to slow down read attempts; otherwise, it won't work
            #do not include this sleep statement in ROS publisher (the command rate.sleep() does this for you in ROS)
            #set ROS rate in publisher to -> rate = rospy.Rate(10) -> for 10 Hz in your code; should be sufficient based on Traxxas speed
            time.sleep(0.1)
    except KeyboardInterrupt:
        print "exiting...keyboard"
    finally:
        print "exiting...final exception"

