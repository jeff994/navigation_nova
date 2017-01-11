#!/usr/bin/env python
import rospy
import serial
import string
import time

from datetime import datetime

from std_msgs.msg import String

ser = serial.Serial()


#add a timer to measure the serial port read line waiting time
start = 0.0
end = 0.0

status_pub = rospy.Publisher('status', String, queue_size = 100)


def open_serial():
    global ser
    if ser.isOpen():
        return 1 

    #real robot port
    ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_85438363039351206271-if00"
    ser.baudrate = 9600

    try:
        ser.open()
    except serial.serialutil.SerialException as ex:
        rospy.logerr(ex)
        return 0 
    if ser.isOpen():
        return 1
    return 0 

def encoder():
    global ser
    pub = rospy.Publisher('driver_obstacle', String, queue_size = 10)
    rospy.init_node('driver_obstacle', anonymous=True)
    rate = rospy.Rate(10)
    
    rospy.loginfo("start driver obstacle")
    
    #@yuqing_obstacledriverread
    while not rospy.is_shutdown():
        if open_serial() == 0:
            rospy.loginfo("Not able to open serail port for the reverse sensor")
            status_pub.publish('driver obstacle 0')
            time.sleep(0.1)
            continue

        bytesToRead = ser.readline()
        rospy.loginfo(bytesToRead)
        
        pub.publish(str(bytesToRead))
        status_pub.publish('driver obstacle 1')
        

if __name__ == '__main__':
    try:
        encoder()
    except rospy.ROSInterruptException:
        #ser.close()  #this doesn't seem to work well
        pass
        
        
