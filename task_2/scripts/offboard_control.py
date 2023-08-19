#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''

from math import sqrt, tanh, exp
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Run mode tells whether the code has been added to launch file or run separately run in terminal
# It is neccesary because when we are running the code in launch file, we need to wait for a few seconds
# before checking for connection, in order for PX4 to settle down
# Usage in line 297
# RUNMODE = 0 : It is run from launch file
# RUNMODE = 1 : It is run from separate terminal
RUNMODE = 0 # WARNING, MUST CHECK : Since I added this to my launch file. Kindly change it to 1 if you have not added it to the launch file

# Precision variable is important in setting the accuracy of path followed, but has an adversarial impact on time of flight
# More on this in line 41, 42 and 61
# Usage in line 213 reachedDestination function
precision = 0.1 / (1.5 ** 0.33) # Idea behind using cube root is because it will be used for volume calculation.

## Target mode defines how it is checked that the drone has reached the setpoint/target
## TARGETMODE = 0 : It lies in a sphere of radius = precision, with center at the setpoint
## TARGETMODE = 1 : It lies in a precision x precision x precision cube, with center at the setpoint
## DEFAULTMODE is used when the mode given is out of the range of defined values
TARGETMODE = 0
DEFAULTTARGETMODE = 1

## Falloff mode defines the function used to lower velocity as the drone closes in on the setpoint
## TARGETMODE = 0 : Hyperbolic Tangent
## TARGETMODE = 1 : Sigmoid
## TARGETMODE = 2 : Linear
## DEFAULTMODE is used when the mode given is out of the range of defined values
FALLOFFMODE = 0
DEFAULTFALLOFFMODE = 1

# Fall off parameters
STARTFALLOFFAT = 0.03 # Start velocity fall off when 3% of the journey is left
ENDFALLOFFAT = 0.002 # continue constant value when 0.2 % of the journey is left

# FALLOFF : 
## Adding fall off to the velocity is helpful in avoiding overshooting and ensuring a fairly accurate path.
## If we keep on lowering precision arbitrarily then overshooting corrections will take ages and we have to compromise between time of flight and accuracy of the path
## That was the basic motivation behind adding a falloff to velocity
## As of now the only good falloff functions I could think of are Identity, Sigmoid and Hyperbolic Tan
## We will explore on other falloff functions in the coming tasks.

# Checking if destination is reached :
## Checking if the drone lies in a bounding cube of edge length l, vs a bounding sphere of radius precision makes a lot of difference
## If we are travelling along X axis, there will be fairly less error along Y and Z axes. and most of the error must be in X axis.
## We could use even custom bounding cuboid or ellipsoids that point in the direction of travel as that will be the direction of maximum error.
## We will explore on other bounding volumes in the coming tasks.

class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    def setFailSafe(self) -> None :
        # Calling to /mavros/param/set to disable failsafe for offboard and print fail message on failure
        rospy.wait_for_service('mavros/param/set')  # Waiting untill the service starts 
        try:
            paramService = rospy.ServiceProxy('mavros/param/set', ParamSet) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            paramid = 'COM_RCL_EXCEPT'
            paramvalue = ParamValue(integer = (1 << 9) - 1) # values stored bitwise. If you want to activate OFFBOARD, which is 2, you have to put 1 in 2nd index bit, i.e 1 << 2 = bin(1 0 0)
            paramService(paramid, paramvalue)        
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
    
    def setArm(self) -> None:
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True) # Arm
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly declare other service proxies 

    def setDisArm(self) -> None:
        # Calling to /mavros/cmd/arming to disarm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(False) # Disarm
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def offboard_set_mode(self) -> None:
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure

        rospy.wait_for_service('mavros/set_mode')  # Waiting untill the service starts 
        # msg = SetMode()._request_class
        msg = SetModeRequest()
        msg.base_mode = 0                           # 0 is custom mode.
        msg.custom_mode = 'OFFBOARD'
        try:
            modeService = rospy.ServiceProxy('mavros/set_mode', SetMode) # Creating a proxy service for the rosservice named /mavros/set_mode to set the mode the drone to OFFBOARD 
            modeService(msg)
        except rospy.ServiceException as e:
            print ("Service set mode call failed: %s"%e)
    
    def land_set_mode(self) -> None:
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure

        rospy.wait_for_service('mavros/set_mode')  # Waiting untill the service starts 
        # msg = SetMode()._request_class
        msg = SetModeRequest()
        msg.base_mode = 0                           # 0 is custom mode.
        msg.custom_mode = 'AUTO.RTL'
        try:
            modeService = rospy.ServiceProxy('mavros/set_mode', SetMode) # Creating a proxy service for the rosservice named /mavros/set_mode to set the mode the drone to OFFBOARD 
            modeService(msg)
        except rospy.ServiceException as e:
            print ("Service set mode call failed: %s"%e)
   
    
   
class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.x = 0
        self.y = 0
        self.z = 0
        
    def stateCb(self, msg : State) -> None:
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    
    def posiCallBack(self, msg : PoseStamped) -> None :
        # Stores current position of the drone to the monitor instance
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

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
        return reachedDestination(val, _mode = DEFAULTFALLOFFMODE)

def velFallOff(stateMt : stateMoniter, setpoints : list, i : int) -> Twist : 
    # Reduces the velocity as we approach the target, using a falloff function
    # More details in line 59
    assert i > 0
    pos = setpoints[i][0]
    _pos = setpoints[i - 1][0]
    total = sqrt((pos.pose.position.x - _pos.pose.position.x) ** 2 + (pos.pose.position.y - stateMt.y) ** 2 + (pos.pose.position.z- _pos.pose.position.z) ** 2)
    left =  sqrt((pos.pose.position.x - stateMt.x) ** 2 + (pos.pose.position.y - stateMt.y) ** 2 + (pos.pose.position.z - stateMt.z) ** 2)

    newVel = setpoints[i][1]

    tup = (newVel.linear.x, newVel.linear.y, newVel.linear.z)
    ratio = left / total
    ratio = max(ratio, ENDFALLOFFAT)

    if ratio < STARTFALLOFFAT :
        multiplier = fallOff(ratio / STARTFALLOFFAT)

        x = tup[0] * multiplier
        y = tup[1] * multiplier
        z = tup[2] * multiplier

        newVel.linear.x = x
        newVel.linear.y = y
        newVel.linear.z = z
    return newVel

def sigmoid(x : float) -> float : 
    # Defining signmoid function as it it not there in math library
    z = exp(-x)
    return 1 / (1 + z)

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
        return reachedDestination(targetCoordinates, _mode = DEFAULTTARGETMODE)

def addSetpoint(coords : tuple, velocity : tuple, setpoints : list) -> None :
    # Neatly adds setpoints, given coordinates and velocity vector
    # coords must be a 3D tuple having x, y, z values
    # velocity must be likewise a 3D tuple having linear velocity componenets along x, y and z axes.
    # setpoints is the global list of setpoints
    
    # Set your position here
    pos = PoseStamped()
    pos.pose.position.x, pos.pose.position.y, pos.pose.position.z = coords

    # Set your velocity here
    vel = Twist()
    vel.linear.x, vel.linear.y, vel.linear.z = velocity

    setpoints.append((pos, vel))

def main() -> None:

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, stateMt.posiCallBack)

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [] #List to setpoints

    '''
    Testing purpose only, maybe deleted
    # Create empty message containers 
    pos = PoseStamped()
    pos.pose.position.x = -5
    pos.pose.position.y = 0
    pos.pose.position.z = 1

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 5

    var = (pos, vel)
    '''

    # Create Setpoints 
    addSetpoint((0, 0, 0), (0, 0, 0), setpoints) # Empty setpoint, to initiate offboard
    addSetpoint((0, 0, 10), (0, 0, 5), setpoints) # Takeoff setpoint
    addSetpoint((10, 0, 10), (5, 0, 0), setpoints) # 1st setpoint
    addSetpoint((10, 10, 10), (0, 5, 0), setpoints) # 2nd setpoint
    addSetpoint((0, 10, 10), (-5, 0, 0), setpoints) # 3rd setpoint
    addSetpoint((0, 0, 10), (0, -5, 0), setpoints) # 4th setpoint
    addSetpoint((0, 0, 0), (0, 0, -5), setpoints) # Landing setpoint

    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Service proxy
    # Calling to /mavros/param/get to get the param value and check if failsafe has been disabled successfully
    paramState = rospy.ServiceProxy('mavros/param/get', ParamGet) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone

    if RUNMODE == 0 : # sleeps for 5 seconds in order to let PX4 settle down, when run from launch file
        rospy.loginfo('Sleeping')
        rospy.sleep(5)

    while not stateMt.state.connected :
        rate.sleep()

    print('Connected now')

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs some setpoints to be already published at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints.  
    '''
    for i in range(100) : # Here 100 has been taken as in the documentation too 100 were taken.
        local_pos_pub.publish(setpoints[0][0])
        local_vel_pub.publish(setpoints[0][1])
        rate.sleep()

    print('Published dummy set points')
    
    # Switching the state to offboard
    while True : 
        if not stateMt.state.armed : 
            ofb_ctl.setArm()
        if not (((paramState('COM_RCL_EXCEPT').value.integer & (1 << 2)) >> 2) & 1) : 
            ofb_ctl.setFailSafe()
        if not stateMt.state.mode=="OFFBOARD":
            ofb_ctl.offboard_set_mode()
        else :
            break
        
        rate.sleep()
    print ("OFFBOARD mode activated")

    '''
    Testing purpose, may be removed
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 10
    while not rospy.is_shutdown() :
        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        rate.sleep()
    '''

    '''
    Step 1: Set the setpoint 
    Step 2: Then wait till the drone reaches the setpoint, 
    Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
    Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  

    Write your algorithm here 
    '''
    for i in range(1, len(setpoints)) : 
        while  not rospy.is_shutdown() :
            # If reached the setpoint, move to next setpoint
            if reachedDestination(stateMt, (setpoints[i][0].pose.position.x, setpoints[i][0].pose.position.y, setpoints[i][0].pose.position.z)) :
                break
            
            # Publish position
            local_pos_pub.publish(setpoints[i][0])
            # Publish velocity
            _vel = velFallOff(stateMt, setpoints, i)
            local_vel_pub.publish(_vel)
            
            rate.sleep()
        
        if i == 0 :
            pass
        elif i == 1 : 
            print('Takeoff Completed')
        elif i == 6 : 
            print('Landing Completed')
        else :
            print('Reached next point')

    print('Disarming now')

    # Disarming the drone
    while stateMt.state.armed:
        ofb_ctl.setDisArm()
        rate.sleep()

    print("Disarmed!!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass