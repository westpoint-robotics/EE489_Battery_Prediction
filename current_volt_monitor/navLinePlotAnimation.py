import numpy as np
import matplotlib.pyplot as plt

'''
Author:  LTC Lowrance

Version:  This version uses a line plot that dynamically adjusts the axes after each point is plotted. 

Purpose:  Plot GPS & Odometry coordinates, which were recorded by a robot and transformed into a local frame perspective.  
          The program plots the coordinates using an animation.
          
Details:  Program opens two separate cvs files (odometry & gps) & stores the data (time & coordinates) in lists.  
          Afterwards, the script plots the odom & gps coordinates sequentially in time as they were recorded.  
          If Odom coordinates were updated more frequently than GPS (1Hz), 
          then plotter skips some odom coordinates and only plots the point nearest in time when the current GPS coordinate was stored.

'''

################## Read CSV Files ############################################
            
#open files and append data to a series of lists (time & x,y coordinates)
#define lists for storing data from cvs files 
odomTime = []
odomX = []
odomY = []
gpsTime = []
gpsX = []
gpsY = []

with open('/home/eecs/workspace/odomCoordinates.csv','r') as odomPoints: #change path to file as necessary
    for line in odomPoints:     #iterate over every line in file
        odomPointList = line.strip().split(',') #convert string into list of strings; slit into list based on commas in string 
        odomPointList = map(float,odomPointList) #convert list of strings into list of floats
        odomTime.append(odomPointList[0])
        odomX.append(odomPointList[1])
        odomY.append(odomPointList[2])
with open('/home/eecs/workspace/gpsTransformedCoords.csv','r') as gpsPoints: #change path to file as necessary
    for line in gpsPoints:     #iterate over every line in file
        gpsPointList = line.strip().split(',') #convert string into list of strings; slit into list based on commas in string 
        gpsPointList = map(float,gpsPointList) #convert list of strings into list of floats
        gpsTime.append(gpsPointList[0])
        gpsX.append(gpsPointList[1])
        gpsY.append(gpsPointList[2])
#################################################################################


################# Animation Code ################################################

#define two lists for appending points progressively for animation
xOdom= []
yOdom = []
xGPS = []
yGPS = []
#plt.gca().set_color_cycle(['blue', 'green'])
plt.gca().set_prop_cycle(None)
#setup x,y plot
plt.plot(xOdom,yOdom, label='Odometry')
plt.plot(xGPS,yGPS, label='GPS')
#add legend
plt.legend(loc='lower right')
#turn on interact plot
plt.ion()
#add labels & title
plt.xlabel("x Coordinate (meters)")
plt.ylabel("y Coordinate (meters)")
plt.title("GPS and Odometry Comparsion from Robot's World Frame View")

#loop through coordinates for animation
#use gps as base time (plot points on a roughly 1Hz timescale)
currentOdomIndex = 0
for i in range(0,len(gpsTime)):    
    #get GPS point
    xGPS.append(gpsX[i])
    yGPS.append(gpsY[i])
    #find Odom point near the time GPS point was recorded    
    for j in range(currentOdomIndex,len(odomTime)):
        if odomTime[j] < gpsTime[i]: #then, too early in time compared to GPS, skip sample & go to next
            pass            
        else:            
            xOdom.append(odomX[j])
            yOdom.append(odomY[j])
            currentOdomIndex = j
            break
    #plot latest points  
    plt.gca().lines[0].set_xdata(xOdom)
    plt.gca().lines[0].set_ydata(yOdom)
    plt.gca().lines[1].set_xdata(xGPS)
    plt.gca().lines[1].set_ydata(yGPS)
    plt.gca().relim()
    plt.gca().autoscale_view()    
    plt.pause(0.5) #pause for half second between points being plotted

#This keeps the plot alive after animation is finished
while True:
   plt.pause(0.5)
   
    
    
    

