#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco
import numpy as np
import rospy
import math

frequency = 10
no_frequency_constraint = False

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)

		# rate = rospy.Rate(10)
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker

		# params required for aruco detection
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
		self.parameters = aruco.DetectorParameters_create()

	# Callback function of camera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			self.detect_aruco()
			self.save_msg()
		except CvBridgeError as e:
			print(e)
	def save_msg(self) : 
		'''Saves the last recorded parameters to self.msg. Call this before publishing'''
		self.marker_msg.id = self.aruco_id
		self.marker_msg.x = self.x
		self.marker_msg.y = self.y
		self.marker_msg.yaw = self.yaw	
	def publish_data(self, event = None) :
		'''Publish self.msg to /marker_detection topic'''
		self.marker_pub.publish(self.marker_msg)
	def detect_aruco(self) : 
		'''Pose estimation of aruco tag on self.img'''
		gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
		corners, ids, _ = aruco.detectMarkers(gray_img, self.aruco_dict, parameters = self.parameters)
		_id = ids[0][0]
		corners = corners[0][0]
		center = 0.5 * (corners[0] + corners[2])
		self.aruco_id = _id
		tl = corners[0]
		tr = corners[1]

		vector = (tr - tl)

		if vector[0] == 0 : 
			angle = [90, 270][vector[1] < 0]
		elif vector[1] == 0 : 
			angle = [0, 180][vector[0] < 0]
		else :
			angle = math.degrees(math.atan(vector[1] / vector[0]))
			angle = [180 + angle, angle][vector[0] > 0]
        
		self.yaw = (-angle + 90) % 360
		self.x = center[0]
		self.y = center[1]

if __name__ == '__main__':
	image_proc_obj = image_proc()
	# Publish every 0.1 seconds : 10 Hz
	rospy.Timer(rospy.Duration(1.0 / 10.0), image_proc_obj.publish_data)
	rospy.spin()
