#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name pick_n_place which controls the drone in offboard mode, picks a package, places it at its destination and returns. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                      /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                        /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
    /activate_gripper                                                                             /gripper_check
         
    
'''

import math
from math import sqrt, tanh, exp
from tarfile import TarFile
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

from strawberry_stacker.task_4.scripts.multidronev2 import stateMoniter

RUNMODE = 1 

precision = 0.1 / (1.5 ** 0.33)
linPrecision = 0.2 

TARGETMODE = 0
DEFAULTTARGETMODE = 1

FALLOFFMODE = 0
DEFAULTFALLOFFMODE = 1

STARTFALLOFFAT = 0.03 
ENDFALLOFFAT = 0.002 

TIMEOUTTHRESH = 1

CAMMATRIX = np.array([238.3515418007097, 0.0, 200.5, 0.0, 238.3515418007097, 200.5, 0.0, 0.0, 1.0], dtype = np.float32).reshape((3, 3))
DISTORTIONCOEFFS = np.array([0, 0, 0, 0, 0], dtype = np.float32)
xPrecision = 0.001

class stateMoniter:
    def __init__(self, name):
        self.state = State()
        # Instantiate a setpoints message
        self.x = 0
        self.y = 0
        self.z = 0
        self.canPick = std_msgs.msg.String()
        self.bridge = CvBridge()
        self.img = np.empty([])

        Subscriber(name + '/mavros/local_position/pose', PoseStamped, self.posiCallBack)
        Subscriber(name + '/gripper_check', std_msgs.msg.String, self.gripperCallBack)
        Subscriber(name + '/camera/image_raw', Image, self.camCallBack)
        Subscriber(name + "/mavros/state",State, self.stateCb)
        
    def stateCb(self, msg : State) -> None:
        # Callback function for topic /mavros/state
        self.state = msg

    def posiCallBack(self, msg : PoseStamped) -> None :
        # Stores current position of the drone to the monitor instance
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
    
    def gripperCallBack(self, msg : std_msgs.msg.String) -> None :
        self.canPick = msg
    
    def camCallBack(self, msg : Image) :
        try :
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err :
            print(err)

class control :

    TASKCODES = [0, 1, 2, 3, 4] # 0 -> Takeoff, 1 -> Move to next setpoint/travelling, 2-> Landing, 3-> Pick, 4-> Drop

    def __init__(self, name = '/edrone0', homeoffset = (-1, 1)) :
        # Initialise rosnode
        rospy.init_node('control', anonymous=True)
        
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        self.parameters = aruco.DetectorParameters_create()

        self.name = name
        self.homeoffset = homeoffset

        self.target = (0, 0, 0)

        self.state = stateMoniter(self.name)

        self.params_service = rospy.ServiceProxy(name + '/mavros/param/get', ParamGet)

        self.tasks = []
    
    def disableFailSafe(self) -> None :
        # Calling to /mavros/param/set to disable failsafe for offboard and print fail message on failure
        rospy.wait_for_service(self.name + '/mavros/param/set')  # Waiting untill the service starts 
        try:
            paramService = rospy.ServiceProxy(self.name + '/mavros/param/set', ParamSet) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            paramid = 'COM_RCL_EXCEPT'
            paramvalue = ParamValue(integer = (1 << 9) - 1) # values stored bitwise. If you want to activate OFFBOARD, which is 2, you have to put 1 in 2nd index bit, i.e 1 << 2 = bin(1 0 0)
            paramService(paramid, paramvalue)        
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
    
    def Arm(self) -> None :
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service(self.name + '/mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy(self.name + '/mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True) # Arm
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
    
    def Disarm(self) -> None :
        # Calling to /mavros/cmd/arming to disarm the drone and print fail message on failure
        rospy.wait_for_service(self.name + '/mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy(self.name + '/mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(False) # Disarm
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
    
    def set_offboard_mode(self) -> None :
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure

        rospy.wait_for_service(self.name + '/mavros/set_mode')  # Waiting untill the service starts 
        msg = SetModeRequest()
        msg.base_mode = 0                           # 0 is custom mode.
        msg.custom_mode = 'OFFBOARD'
        try:
            modeService = rospy.ServiceProxy(self.name + '/mavros/set_mode', SetMode) # Creating a proxy service for the rosservice named /mavros/set_mode to set the mode the drone to OFFBOARD 
            modeService(msg)
        except rospy.ServiceException as e:
            print ("Service set mode call failed: %s"%e)
    
    def set_land_mode(self) -> None:
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure

        rospy.wait_for_service(self.name + '/mavros/set_mode')  # Waiting untill the service starts 
        # msg = SetMode()._request_class
        msg = SetModeRequest()
        msg.base_mode = 0                           # 0 is custom mode.
        msg.custom_mode = 'AUTO.LAND'
        try:
            modeService = rospy.ServiceProxy(self.name + 'mavros/set_mode', SetMode) # Creating a proxy service for the rosservice named /mavros/set_mode to set the mode the drone to OFFBOARD 
            modeService(msg)
        except rospy.ServiceException as e:
            print ("Service set mode call failed: %s"%e)
    
    def gripper(self, arg : bool) -> bool :
        assert type(arg) is bool

        rospy.wait_for_service(self.name + '/activate_gripper')  # Waiting untill the service starts 
        msg = arg
        try:
            gripperService = rospy.ServiceProxy(self.name + '/activate_gripper', Gripper) # Creating a proxy service for the rosservice named /mavros/set_mode to set the mode the drone to OFFBOARD 
            retVal = gripperService(msg)
            return retVal.result
             
        except rospy.ServiceException as e:
            print ("Service activate gripper call failed: %s"%e)

    def inPosition(self, _mode = None) -> bool :
        # Checks if the drone has reached the current target, given the precision values and checking mode
        # More details in line 40 and 66
        global TARGETMODE
        global DEFAULTTARGETMODE
        prevMode = TARGETMODE
        if _mode is not None :
            TARGETMODE = _mode
        if TARGETMODE == 0 : 
            TARGETMODE = prevMode
            rad = sqrt((self.state.x - self.target[0]) ** 2 + (self.state.y - self.target[1]) ** 2 + (self.state.z - self.target[2]) ** 2)
            return rad < precision
        elif TARGETMODE == 1 : 
            TARGETMODE = prevMode
            return abs(self.state.x - self.target[0]) < (precision / 2) and abs(self.state.y - self.target[1]) < (precision / 2) and abs(self.state.z - self.target[2]) < (precision / 2)
        else :
            print('Bad argument for reachedDestination : ', TARGETMODE)
            exit()  

    def update(self) :
        if self.tasks[0] == 0 : # Takeoff
            pass
        elif self.tasks[0] == 1 : # Travelling, move to next setpoint
            pass
        elif self.tasks[0] == 2 : # Landing
            pass
        elif self.tasks[0] == 3 : # Pick up Box
            pass
        elif self.tasks[0] == 4 : #  Drop Box
            pass
        else :
            print('Unknown task with code', self.tasks[0], 'found. Terminating.')
            exit()
            
def reachedDestination(stateMt : stateMoniter, targetCoordinates : tuple, _mode = None) -> bool :
    # Checks if the drone has reached the current target, given the precision values and checking mode
    # More details in line 40 and 66
    global TARGETMODE
    global DEFAULTTARGETMODE
    prevMode = TARGETMODE
    if _mode is not None :
        TARGETMODE = _mode
    if TARGETMODE == 0 : 
        TARGETMODE = prevMode
        rad = sqrt((stateMt.x - targetCoordinates[0]) ** 2 + (stateMt.y - targetCoordinates[1]) ** 2 + (stateMt.z - targetCoordinates[2]) ** 2)
        return rad < precision
    elif TARGETMODE == 1 : 
        TARGETMODE = prevMode
        return abs(stateMt.x - targetCoordinates[0]) < (precision / 2) and abs(stateMt.y - targetCoordinates[1]) < (precision / 2) and abs(stateMt.z - targetCoordinates[2]) < (precision / 2)
    else :
        print('Bad argument for reachedDestination : ', TARGETMODE)
        exit()

def sigmoid(x : float) -> float : 
    # Defining signmoid function as it it not there in math library
    z = exp(-x)
    return 1 / (1 + z)

def fallOff(val : float, _mode : int = None) -> float : 
    # Using a falloff function of  choice.
    # The input value is in the range of 0 - 1 and output is in the range of 0 - 1 where as we approach 0, the output falls off rapidly
    # More details in line 47
    global FALLOFFMODE
    global DEFAULTFALLOFFMODE
    prevMode = FALLOFFMODE # Save the mode
    if _mode is not None : # if some mode is given in arguments, use that mode 
        FALLOFFMODE = _mode
    if FALLOFFMODE == 0 :
        FALLOFFMODE = prevMode
        return tanh(val) / tanh(1)
    elif FALLOFFMODE == 1 : 
        FALLOFFMODE = prevMode
        return (sigmoid(val) - 0.5) / (sigmoid(1) - 0.5)
    elif FALLOFFMODE == 2 : 
        FALLOFFMODE = prevMode
        return val    
    else :
        print('Bad argument for fallOff : ', FALLOFFMODE)
        exit()

