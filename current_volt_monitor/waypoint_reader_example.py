#!/usr/bin/env python

# Example to show how to read a .csv file containing waypoints and return a particular row (or waypoint number)


def getWayPoint(wayPointNum):
    #this function iterates over each line (i.e., row) in a file containing waypoints
    #it returns the contents of the desired line (i.e., the current waypoint needed)
    #1 input parameter:  the desired waypoint
    #2 output parameters:  flag on whether waypoint found; and, waypoint coordinates  
    count = 0 				#counter keeps track of line number in file	
    flg_nothingFound = True 		#flag; set to true if waypoint number is NOT found 
    with open('/home/eecs/workspace/waypoints.csv','r') as wayPoints: #change path to file as necessary
        for line in wayPoints: 		#iterate over every line in file
            count = count + 1 		#track line number of file
            if count == wayPointNum:  	#check to see if this is the desired waypoint number (i.e. row)    
                flg_nothingFound = False 	#found a match!  Set flag.
                wayPointList = line.strip().split(',') #convert string into list of strings; slit into list based on commas in string 
                wayPointList = map(float,wayPointList) #convert list of strings into list of floats
    if flg_nothingFound == True:     # didn't find line number (no more wayPoints in file)
        print "stopping robot; there are no more waypoints"
        wayPointList = [] #return empty waypoint list if waypoint number not in file
    
    return flg_nothingFound, wayPointList


############# MAIN #########################################


wayPointNum = 5 #current waypoint number (i.e., line) in file that robot is traveling to
flg_nothingFound, wayPointList = getWayPoint(wayPointNum)
print wayPointList #list = [lat,lon, utm_x, utm_y]
