import rospy
import serial
import string
import robot_drive 
import robot_obstacle
import json
from std_msgs.msg import String

pub_param 		= rospy.Publisher('parameters', String, queue_size = 10)
pub_gps			= rospy.Publisher('gps', 		String, queue_size=10)
pub_command 	= rospy.Publisher('command', 	String, queue_size=10)

# Used to publish parameters 
def publish_parameters():
	#@yuqing_publishparam
	info={}  
	info["ENABLE"]      =    robot_drive.robot_enabled
	info["MOVING"]      =    robot_drive.robot_moving 
	info["MISSION"]     =    robot_drive.robot_on_mission 
	info["OBSTACLE"]    =    robot_obstacle.robot_on_obstacle  
	info["DIRECTION"]   =    robot_drive.move_direction
	info["SPEED"]       =    robot_drive.speed_now  
	info["LONG"]        =    robot_drive.lon_now
	info["LAT"]         =    robot_drive.lat_now
	info["BEARING"]     =    robot_drive.bearing_now
	data={}
	data["parameters"]  =    info
	  
	parameters = json.dumps(data)
	#rospy.loginfo(parameters)
	pub_param.publish(parameters)

def publish_gps():
	stringToSend = '%f %f %f' % (robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now) #might need to add \n behind the E
	pub_gps.publish(stringToSend)


# Helper function which can send commands to robot 
def publish_command(command_string, speed):
	#sending the string
	#handle the format of the string
	stringToSend = 'S%s00000%dE\n' % (command_string, speed) #might need to add \n behind the E
	pub_command.publish(stringToSend)
	rospy.loginfo(str(stringToSend))