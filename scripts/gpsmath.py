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

	return c * r

# Calculate angle between two diffent gps positions 
def bearing(lon1, lat1, lon2, lat2):  #from position 1 to 2
	#convert to radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
	
	bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
	bearing = degrees(bearing)
	bearing = (bearing + 360) % 360

	return bearing


#calculate the new latitude based on the current gps coordinates and angle and distance 
def getgps(lon1, lat1, dist, bearing):
	r = 6371 * 1000 * 1000; 
	lat2 = asin(sin(lat1) * cos(dist/r) + cos (lat1) * con(bearing))
	lon2 = lon1 + atan2(sin(bearing) * sin (dist/r) * con(lat1) - sin (lat1) * sin(lat2));
	return [lon2, lat2];
