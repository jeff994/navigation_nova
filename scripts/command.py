import rospy
import serial
import string

from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees

compass_data = 0	#degrees, true north is 0 degrees
dist_travelled = 0	#cm
x_now = 0  	#cm
y_now = 0	#cm
bearing_now = 0 #degrees
r = 35 		#cm, distance between center of robot to wheel
x_target = 0	#cm
y_target = 0 	#cm, should always be 0, because we will be moving in a straight line
bearing_target = 0 	#degrees

#defining or acquiring the GPS coordinates
gps_num = 4
start = [103.962386,1.340549]
gps_lon = [103.962386,103.962389,103.962456,103.962461,103.962381] #S,A,B,C,D
gps_lat = [1.340549,1.3407,1.340696,1.340589,1.340599]

#defining serial port to write to (the commands)
ser = serial.Serial()
ser.port = "/dev/ttyACM2" #depends on the device port name
ser.baudrate = 9600
ser.open()

def haversine(lon1, lat1, lon2, lat2):
	#convert to radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1,lon2, lat2])
	#haversine
	dlon = lon2 - lon1
	dkat = lat2 - lat1
	a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
	c = 2 * asin(sqrt(a))
	r = 6371 #radius of earth in kilometers

	return c * r

def bearing(lon1, lat1, lon2, lat2):  #from position 1 to 2
	#convert to radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
	
	bearing = atan2(sin(lon1-lon2)*cos(lat1), cos(lat2)*sin(lat1)-sin(lat2)*cos(lat1)*cos(lon1-lon2))
	bearing = degrees(bearing)
	bearing = (bearing + 360) % 360

	return bearing

def job_details(angle_now, first_point, second_point)
	#handles from first_point to second_point
	distance = haversine(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #km
	angle_next = bearing(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #deg  
		
	#handles turning angle		
	turn_angle = angle_next - angle_now
	if turn_angle > 180.0 :
		turn_angle = turn_angle - 360.0
		
	#handles forward distance in m
	distance = distance * 1000.0

	return [angle_next, turn_angle, distance]

def job_generator(init_bearing, loops):
	job_des = []
	job_num = []
	angle_now = init_bearing;   #all this angle now might be taken from compass if accurate
	
	#handles from start to first point
	job = job_details(angle_now, 0, 1)
	job_des.extend(['T','F'])
	job_num.extend([job[1],job[2]])
	angle_now = job[0]
		
	#handles how many loops
	for i in range (loops) :
		for k in range (gps_num):
			if k < gps_num - 1 :
				job = job_details(angle_now, k + 1, k + 2)
				job_des.extend(['T','F'])
				job_num.extend([job[1],job[2]])
				angle_now = job[0]
				
			else : 
				job = job_details(angle_now, k + 1, 1)
				job_des.extend(['T','F'])
				job_num.extend([job[1],job[2]])
				angle_now = job[0]
	
	#handles closing loop, going back to start
	job = job_details(angle_now, 1, 0)
	job_des.extend(['T','F'])
	job_num.extend([job[1],job[2]])
	angle_now = job[0]
	#final turn to init_bearing
	job_des.append('T')
	ending_turn_angle = init_angle - angle_now
	if ending_turn_angle > 180 :
		ending_turn_angle = ending_turn_angle - 360.0
	job_num.append(ending_turn_angle)

	return [job_des, job_num]	

def compass_callback(data):
	global compass_data
	#update compass_data
	compass_data = int(data.data)
	#rospy.loginfo("compass : %s", data.data)

def encoder_callback(data):
	#accumulate encoder data
	global x_now
	global y_now
	global r
	global compass_data
	data_string = data.data
	left_encode, right_encode = data_string.split(" ")
	dist = (int(left_encode) + int(right_encode))/2.0
	x_now = x_now + 

	#sending serial command
	send_command(

def send_command(command_string, speed):
	#handle the format of the string
	stringToSend = 'S%s00000%dE' % (command_string, speed) #might need to add \n behind the E
	#sending the string
	if ser.isOpen():
		ser.write(stringToSend)
	else:
		rospy.loginfo("Commanding Serial port not connected")

def main_listener():
	rospy.init_node('commander')
	rospy.Subscriber('compass', String, compass_callback)
	rospy.Subscriber('encoder', String, encoder_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		main_listener()
	except rospy.ROSInterruptException:
		pass
