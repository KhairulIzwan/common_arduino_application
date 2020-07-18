#!/usr/bin/env python3

#Title: The ROS node that displays and calculates the LED's stage
#Author: https://www.intorobotics.com/how-to-use-rosserial-with-two-arduinos-and-raspberry-pi/
#Description: A ROS node will receive a random number from the first Arduino board. The node will run on Raspberry Pi and will command the LED on the second Arduino board.

"""
Re-arrange the code in order to make it works! [Khairul Izwan (24-01-2020)]
"""

import rospy
 
from std_msgs.msg import Int32
from std_msgs.msg import String
 
var=None
 
#define the display text
def callback(msg):
	global var
	var = msg.data
	
	if var <= 2500:
		#send message to turn OFF the LED
		varP=0
		rospy.loginfo("The output is OFF and the var is: %s", var)
		
	else:
		#send message to turn ON the LED
		varP=1
		rospy.loginfo("The output is ON and the var is: %s", var)
		
	pub.publish(varP)

if __name__=='__main__':
	rospy.init_node('random_LED')
	rospy.Subscriber('random_number',Int32, callback)
	pub=rospy.Publisher('LED', Int32, queue_size=1)
#	rate=rospy.Rate(10)
	rospy.spin()

#while not rospy.is_shutdown():
#	if var <= 2500:
#		#send message to turn OFF the LED
#		varP="OFF"
#		rospy.loginfo("The output is OFF and the var is: %s", var)
#	else:
#		#send message to turn ON the LED
#		varP="ON"
#		rospy.loginfo("The output is ON and the var is: %s", var)

#pub.publish(varP)
#rate.sleep()
