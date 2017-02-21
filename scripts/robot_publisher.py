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