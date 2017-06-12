#!/usr/bin/env python
import rospy
import string

from std_msgs.msg import String

battery_level = 100

#status callback variables
on_obstacle  				= 0
manual_mode 	 			= 0
obstacle_avoidance_mode 	= 0
has_obstacle 				= 0
interaction_mode 			= 0

motor_1_ok 	 				= 1
motor_2_ok 	 	 			= 1
encoder_ok 					= 1
gyroscope_ok 				= 1
reverse_sensor_ok 		 	= 1
distance_sensor_ok 			= 1


#designed to dynamically update the robot status 
def get_status(data):
	#accumulate encoder data
	#Step 1: Get encoder data and convert them to number for later use 
	#Get left encoder and right encoder 
	data_string = data.data
	device, status = data_string.split(" ")
	rospy.loginfo("Device %s is with status %s", device, status)    
	return

#subscribes to different topic 
def main_listener():
	rospy.init_node('robot_status')
	rospy.Subscriber('status', String, get_status)
	rospy.spin()

if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		#job_generator_move_1m()
		main_listener()
	except rospy.ROSInterruptException:
		pass
