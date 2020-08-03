#!/usr/bin/env python

################################################################################
## {Description}: Subcribe the RPY reading from MPU6050 (rosserial)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time

# import the necessary ROS packages
from std_msgs.msg import String
from std_msgs.msg import Float32
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import CameraInfo

#from cv_bridge import CvBridge
#from cv_bridge import CvBridgeError

import rospy

class RPY:
	def __init__(self):

#		self.bridge = CvBridge()
#		self.image_received = False

		rospy.logwarn("RPY Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to Float32 msg
		self.roll_topic = "/roll"
		self.roll_sub = rospy.Subscriber(self.roll_topic, Float32, self.cbRoll)
		
		# Subscribe to Float32 msg
		self.pitch_topic = "/pitch"
		self.pitch_sub = rospy.Subscriber(self.pitch_topic, Float32, self.cbPitch)
		
		# Subscribe to Float32 msg
		self.yaw_topic = "/yaw"
		self.yaw_sub = rospy.Subscriber(self.yaw_topic, Float32, self.cbYaw)

#		# Subscribe to CameraInfo msg
#		self.cameraInfo_topic = "/cv_camera/camera_info"
#		self.cameraInfo_sub = rospy.Subscriber(self.cameraInfo_topic, CameraInfo, 
#			self.cbCameraInfo)

		# Allow up to one second to connection
		rospy.sleep(1)

#	# Convert image to OpenCV format
#	def cbImage(self, msg):

#		try:
#			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#			# comment if the image is mirrored
#			self.cv_image = cv2.flip(self.cv_image, 1)
#		except CvBridgeError as e:
#			print(e)

#		if self.cv_image is not None:
#			self.image_received = True
#		else:
#			self.image_received = False

#	# Get CameraInfo
#	def cbCameraInfo(self, msg):

#		self.imgWidth = msg.width
#		self.imgHeight = msg.height

	# Get Roll
	def cbRoll(self, msg):

		self.roll = msg.data
		
	# Get Pitch
	def cbPitch(self, msg):

		self.pitch = msg.data
		
	# Get Yaw
	def cbYaw(self, msg):

		self.yaw = msg.data

#	# Image information callback
#	def cbInfo(self):

#		fontFace = cv2.FONT_HERSHEY_DUPLEX
#		fontScale = 0.5
#		color = (255, 255, 255)
#		thickness = 1
#		lineType = cv2.LINE_AA
#		bottomLeftOrigin = False # if True (text upside down)

#		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

#		cv2.putText(self.cv_image, "{}".format(self.timestr), (10, 20), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Sample", (10, self.imgHeight-10), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "(%d, %d)" % (self.imgWidth, self.imgHeight), 
#			(self.imgWidth-100, self.imgHeight-10), fontFace, fontScale, 
#			color, thickness, lineType, bottomLeftOrigin)
			
	# Data information callback
	def cbInfo(self):

		rospy.loginfo("Roll: %.2f Pitch: %.2f Yaw: %.2f" % (self.roll, self.pitch, self.yaw))

#	# Show the output frame
#	def cbShowImage(self):

#		cv2.imshow("RPY", self.cv_image)
#		cv2.waitKey(1)

#	# Preview image + info
#	def cbPreview(self):

#		if self.image_received:
#			self.cbInfo()
#			self.cbShowImage()
#		else:
#			rospy.logerr("No images recieved")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("RPY Node [OFFLINE]...")
#		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('rpy', anonymous=False)
	mpu6050 = RPY()

	# Camera preview
	while not rospy.is_shutdown():
		mpu6050.cbInfo()
