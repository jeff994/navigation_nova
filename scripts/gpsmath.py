#!/usr/bin/env python
import math 
import rospy

from math import radians, cos, sin, asin, sqrt, atan2, degrees

# Calculate distance between two gps coordinates 
def haversine(lon1, lat1, lon2, lat2):
	#convert to radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1,lon2, lat2])
	#haversine
	dlon = lon2 - lon1
	dlat = lat2 - lat1
	a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
	c = 2 * asin(sqrt(a))
	r = 6371 #radius of earth in kilometers
	
	distance =  c * r
	distance = distance * 1000.0 * 1000.0  					# convert distance to mm
	return distance 

# Calculate angle between two diffent gps positions 
def bearing(lon1, lat1, lon2, lat2):  #from position 1 to 2
	#convert to radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
	
	bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
	bearing = degrees(bearing)
	bearing = (bearing + 360) % 360

	return bearing


#calculate the new latitude based on the current gps coordinates and angle and distance 
def get_gps(lon1, lat1, dist, bearing):
	lon1, lat1, bearing	= map(radians, [lon1, lat1,bearing])
	r = 6371 * 1000.0 * 1000.0
	delta = dist/r
	lat2 = asin(sin(lat1) * cos(delta) + cos (lat1) * sin(delta)* cos(bearing))
	lon2 = lon1 + atan2(sin(bearing) * sin (delta) * cos(lat1), cos(delta) - sin (lat1) * sin(lat2))
	#rospy.loginfo("TEST: %f, %f", lon2, lat2)
	lon2 = degrees(lon2)
	lat2 = degrees(lat2)
	return lon2, lat2

def format_bearing(bearing):
	if(bearing < 0):
		while(bearing < 0):
			bearing += 360
		return bearing 
	
	if(bearing >= 360):
		while(bearing >= 360):
			bearing -= 360
	return bearing 
