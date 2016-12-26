#!/usr/bin/env python
import rospy
from std_msgs.msg import String

commands_list = [] 
command_buffer = 100
receiving_index = 0
executing_index = 0


def callback(data):
    global commands_list
    global receiving_index
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    commands_list[receiving_index] = str(data.data)
    receiving_index = (receiving_index + 1) % command_buffer 

def open_serial():
    global ser
    if ser.isOpen():
        return 1 
    #testing robot port
    ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
    #real port
    #ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
    ser.baudrate = 9600
    ser.open()
    if ser.isOpen():
        return 1
    return 0 

def execute_command():
    global commands_list 
    global executing_index
    global receiving_index

    if(executing_index == receiving_index): 
        rospy.logwarn("Serial port is not open, waiting for re-connection")
        time.sleep(0.1)
        return

    if(receiving_index != (executing_index + 1) % command_buffer):
        rospy.logwarn("Two commans received in the same time")

    while(receiving_index != (executing_index + 1) % command_buffer):
        stringToSend = commands_list[executing_index]
        if open_serial():
            if ser.isOpen():
		ser.write(stringToSend)
                executing_index = (executing_index + 1) % command_buffer 
            else: 
                rospy.loginfo("Commanding Serial port not connected")
                break
        else: 
             rospy.loginfo("Commanding Serial port not connected")
             break

def executer():
    rospy.init_node('commad_executer', anonymous=True)
    rospy.Subscriber("command", String, callback)
    while rospy.is_shutdown():
        execute_command()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        executer()
    except rospy.ROSInterruptException:
        #ser.close()  #this doesn't seem to work well
        pass
