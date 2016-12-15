import rospy
import serial
import string
from std_msgs.msg import String

ser = serial.Serial()
ser.port = "/dev/ttyACM1"  #depends on the device port name
ser.baudrate = 9600
ser.open()

def encoder():
	pub = rospy.Publisher('encoder', String, queue_size = 10)
	rospy.init_node('encoder', anonymous=True)
	rate = rospy.Rate(20)
	while ser.isOpen():
		bytesToRead = ser.readline()
		bytesToRead = bytesToRead.strip('\n')
		
		#separates the data into readable things
		r_encoder, r_direction, l_encoder, l_direction = bytesToRead.split(" ")
		if r_direction == "1" :
			r_encoder = -int(r_encoder)
		elif l_direction == "1" :
			l_encoder = -int(l_encoder)
		else :
			r_encoder = int(r_encoder)
			l_encoder = int(l_encoder)

		#turning them into strings
		bytesToPublish = '%d %d' % (l_encoder, r_encoder)
		
		#publishing data in string for standardization
		#rospy.loginfo(str(bytesToPublish))
		pub.publish(str(bytesToPublish))
		rate.sleep()

if __name__ == '__main__':
	try:
		encoder()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
