import math
from math import sqrt, tanh, exp
from time import time

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from rospy.impl.tcpros_service import ServiceProxy
from rospy.topics import Publisher, Subscriber
from gazebo_ros_link_attacher.srv import Gripper
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco

def F(name) : 
    rospy.wait_for_service(name + '/mavros/param/set')  # Waiting untill the service starts 
    try:
        paramService = rospy.ServiceProxy(name + '/mavros/param/set', ParamSet) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
        paramid = 'COM_RCL_EXCEPT'
        paramvalue = ParamValue(integer = (1 << 9) - 1) # values stored bitwise. If you want to activate OFFBOARD, which is 2, you have to put 1 in 2nd index bit, i.e 1 << 2 = bin(1 0 0)
        paramService(paramid, paramvalue)        
    except rospy.ServiceException as e:
        print ("Service arming call failed: %s"%e)

name = '/edrone0'

paramState = rospy.ServiceProxy(name + '/mavros/param/get', ParamGet)

while not rospy.is_shutdown() :
    if not ((paramState('COM_RCL_EXCEPT').value.integer >> 2) & 1) : 
        F(name)
    else :
        break