import math

#variables for current GPS Lat / Lon Readings
currentLat = 41.391240
currentLon = -73.956217
destLat = 41.393035
destLon = -73.953398

#variables for current UTM coordinates
currentX = 587262
currentY = 4582716
destX = 587499
destY = 4582919

#declination angle based on geographic location
#see #https://www.ngdc.noaa.gov/geomag-web/
#needed for "grid-to-magnetic" angle
declinationAngle = 13


########### Functions ############################################################################
def haversine(currentLat,currentLon, destLat, destLon):
    #Calculate the great circle distance between two points 
    #on the earth (specified in decimal degrees - Lat/Lon coords) using Haversine Formula
    
    haversineDistance = math.acos( math.sin(currentLat*math.pi/180)*math.sin(destLat*math.pi/180) + math.cos(currentLat*math.pi/180)*math.cos(destLat*math.pi/180)*math.cos(destLon*math.pi/180-currentLon*math.pi/180) ) * 6371000
    
    haversineAngle = ( math.atan2(math.cos(currentLat)*math.sin(destLat)-math.sin(currentLat)*math.cos(destLat)*math.cos(destLon-currentLon), math.sin(destLon-currentLon)*math.cos(destLat)) ) * (180/math.pi) 
    
    #transform angle perspective - Haversine calculates angle with the perspective that 90 degrees points North
    #for magnetic field reference, we need North to correspond to 0 degrees, so subtract 90
    magBearing = haversineAngle - 90
    #account for declination angle (Westerly declination, so add offset)
    magBearing = magBearing + declinationAngle  
    #account for angle wrap
    if magBearing < 0:
        magBearing = magBearing + 360 
    elif magBearing > 360:
        magBearing = magBearing - 360
    return haversineDistance, magBearing 
    
def distAndBearing_utm(currentX, currentY, destX, destY):
    #calculate distance & bearing using UTM coordinates (x,y)-type coordinates
    dx = destX - currentX
    dy = destY - currentY
    #calculate distance between the two points
    utm_dist = math.sqrt( (dx)**2 + (dy)**2 )
    #calculate the angle between the points
    utm_angle = math.atan(dy/float(dx)) * (180/math.pi)
    
    #If we treat the current (X,Y) point as the origin, then destination (X,Y) lies in a quadrant (either I,II,III, or IV), because ->
    #the dx and dy (above) results in a + or - difference, which indicates the destination quadrant.
    #The quadrant will determine the type of angle adjustment needed magnetically (based on N,S,E, and W heading)
    if dx > 0 and dy > 0: #then destination is in quadrant I (between N and E); atan angle is positive
        utm_angleTF = 90 - utm_angle
    elif dx < 0 and dy > 0: #then destination is in quadrant II (between N and W)
        #atan angle calculation is negative; (get rid of neg. sign, then add to 270 deg-West)
        utm_angleTF = 270 + (-1 * utm_angle)
    elif dx < 0 and dy < 0: #then destination is in quadrant III (between (W and S); atan angle is positive
        utm_angleTF = 270 - utm_angle
    else: # dx > 0 and dy <0, then quad IV (between S and E)
        #angle calculation is negative; (get rid of neg. sign, then add to 90 deg-East)
        utm_angleTF = 90 + (-1 * utm_angle)
    
    #account for declination angle (Westerly declination angle, so add offset)
    magUtmBearing = utm_angleTF + declinationAngle #add offset due to Westerly declination 
    #account for angle wrap
    if magUtmBearing < 0:
        magUtmBearing = magUtmBearing + 360 
    elif magUtmBearing > 360:
        magUtmBearing = magUtmBearing - 360
    
    return utm_dist, magUtmBearing 


####### MAIN ########################################################
dist, bearing = haversine(currentLat,currentLon, destLat, destLon)
print "Distance & Bearning based on Lat/Lon is:  ", dist, bearing
utm_dist, utm_angle = distAndBearing_utm(currentX, currentY, destX, destY)
print "Distance & Bearning based on UTM is:  ", utm_dist, utm_angle
    
