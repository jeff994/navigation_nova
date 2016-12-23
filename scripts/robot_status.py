#!/usr/bin/env python
import rospy
import string


#designed to dynamically update the robot status 
def publish_stats(data):
	#accumulate encoder data
	#Step 1: Get encoder data and convert them to number for later use 
	#Get left encoder and right encoder 
	data_string = data.data
	left_encode, right_encode = data_string.split(" ")

	#convert encoder number to floading point number, make sure all subsquent calculation is on floating point mode 
	if (robot_drive.robot_on_mission ==1 ):
		rospy.loginfo(str(data_string))
	left_encode  = float(left_encode)
    	right_encode = float(right_encode)
    
	return



#subscribes to different topic 
def main_listener():
	rospy.init_node('robot_status')
	rospy.Subscriber('encoder', String, publish_status)

if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		#job_generator_move_1m()
		main_listener()
	except rospy.ROSInterruptException:
		pass
