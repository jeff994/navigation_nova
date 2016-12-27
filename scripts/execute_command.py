#!/usr/bin/env python
import rospy
import time
import serial
import string
from std_msgs.msg import String

commands_list = [] 
command_buffer = 100
receiving_index = 0
executing_index = 0
ser = serial.Serial()

def callback(data):
	global commands_list
   	global receiving_index
	global command_buffer
    	commands_list[receiving_index] = data.data
    	rospy.loginfo("I heard %s", commands_list[receiving_index])
	receiving_index = (receiving_index + 1) % command_buffer 

def init_command_buffer( size ):
    	global commands_list 
    	if(len(commands_list) == size):
		return 
    	for i in range(size - len(commands_list)):
        	commands_list.append('')

def open_serial():
    	global ser
    	if ser.isOpen():
        	return 1 
    #testing robot port
    #ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
    #real port
    	ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75435363138351A09171-if00"
    	ser.baudrate = 9600
    	ser.open()
    	if ser.isOpen():
        	return 1
    	return 0 

def execute_command():
	global commands_list 
    	global executing_index
    	global receiving_index
	global commad_buffer

	#rospy.logwarn("Serial port is not open, waiting for re-connection")
    	if(executing_index == receiving_index): 
        	#rospy.logwarn("Not receiving any commands, please wait")
       	 	time.sleep(0.1)
        	return

    	if(receiving_index != (executing_index + 1) % command_buffer):
        	rospy.logwarn("Two commans received in the same time %d, %d", receiving_index, executing_index)

    	while True: 
        	stringToSend = commands_list[executing_index]
        	executing_index = (executing_index + 1) % command_buffer
		if open_serial():
			#rospy.loginfo("Testing serial port")
			rospy.loginfo(stringToSend)
			ser.write(stringToSend)	
			time.sleep(0.05)
			#ser.write("/n")
            	else: 
                	rospy.loginfo("Commanding Serial port not connected")
                	break
		if(executing_index == receiving_index):
			break; 
		

def executer():
	rospy.init_node('commad_executer', anonymous=True)
    	rospy.Subscriber("command", String, callback)
    	while not rospy.is_shutdown():
        	execute_command()

    # spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()

if __name__ == '__main__':
	try:
        	init_command_buffer(command_buffer)
        	executer()
    	except rospy.ROSInterruptException:
        #ser.close()  #this doesn't seem to work well
        	pass
