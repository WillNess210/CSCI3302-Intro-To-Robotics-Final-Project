#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

MOCAP_TOPIC_NAME = "/vrpn_client_node/num_7/pose"
publisher_arm = None
subscriber_mocap = None
ARM_SET_X = 0
ARM_SET_Y = 0
ARM_SET_Z = 0
CYCLE_TIME = 2

def main():
    init()
    while not rospy.is_shutdown():
        global publisher_arm
        startTime = time.time()
        # ARM CONTROLLING LOGIC GOES BELOW HERE
        controlArm(publisher_arm)
        # ARM CONTROLLING LOGIC ENDS HERE
        endTime = time.time()
        if endTime - startTime < CYCLE_TIME:
            rospy.sleep(CYCLE_TIME - (endTime - startTime))

def init():
    global publisher_arm
    global subscriber_mocap
    rospy.init_node("catcher_driver")

    # SUBSCRIBER- geometry_msgs/PoseStamped
    subscriber_mocap = rospy.Subscriber(MOCAP_TOPIC_NAME, PoseStamped, callback_update_mocap)

    # PUBLISHER
    #publisher_arm = rospy.Publisher('topic_name', message_type , queue_size=10) TODO fill in parameters




#
# MOCAP
#
#Store ball location history
pos_hist = [[], [], [], [], [], [], []] # time, arm x, arm y, arm z, ball x, ball y, ball z
history_length = 100
LOOKBACK_LENGTH = -2 # -2 means use the previous 2 datapoints, -3 means use the previous 3 datapoints, etc..
TIME = 0
ARM_X = 1
ARM_Y = 2
ARM_Z = 3
BALL_X = 4
BALL_Y = 5
BALL_Z = 6
# ROBOT SETTINGS
CATCH_AT_X = 3
LOWEST_Y = 1
HIGHEST_Y = 2
LOWEST_Z = -1
HIGHEST_Z = 1
# method that adds sensor values to pos_history datastructure
def addSensors(aX, aY, aZ, bX, bY, bZ):
    global pos_hist
    pos_hist[TIME].append(time.time()) # ADD TIME STAMP
    pos_hist[ARM_X].append(aX)
    pos_hist[ARM_Y].append(aY)
    pos_hist[ARM_Z].append(aZ)
    pos_hist[BALL_X].append(bX)
    pos_hist[BALL_Y].append(bY)
    pos_hist[BALL_Z].append(bZ)
    if(len(pos_hist) > history_length):
        pos_hist = pos_hist[int(len(pos_hist)/2):] # cut list in half, but keeping the most recent

#Calculate final arm position function, uses pos_hist
def calcPosition():
    global pos_hist
    # ONLY RUN IF POSSIBLE
    if len(pos_hist[TIME]) < 2:
        print("Not setting setpoints. Not enough data")
        return None, None, None
    # CALCULATE LATEST VEL X & TIME LEFT
    dX = pos_hist[BALL_X][-1] - pos_hist[BALL_X][LOOKBACK_LENGTH]
    dT = (pos_hist[TIME][-1] - pos_hist[TIME][LOOKBACK_LENGTH])
    vX = dX/dT
    distanceLeft = pos_hist[BALL_X][-1] - pos_hist[ARM_X][-1]
    timeLeft = distanceLeft / vX
    # CALCULATE VELOCITY & INTERCEPT
    vY = (pos_hist[BALL_Y][-1] - pos_hist[BALL_Y][LOOKBACK_LENGTH])/dT
    interceptY = pos_hist[BALL_Y][-1] + vY*timeLeft - 4.9*timeLeft**2 #Y_0 + VY*T - 4.9T^2
    # CALCULATE END POSITION FOR Z
    dZ = pos_hist[BALL_Z][-1] - pos_hist[BALL_Z][LOOKBACK_LENGTH]
    vZ = dZ / dT
    interceptZ = pos_hist[BALL_Z][-1] + vZ * timeLeft
    # CHECK IF THIS IS NOT A CATCHABLE BALL
    if interceptY < LOWEST_Y or interceptY > HIGHEST_Y:
        print("Not setting setpoints. interceptY isn't realistic ")
        return None, None, None
    if interceptZ < LOWEST_Z or interceptZ > HIGHEST_Z:
        print("Not setting setpoints. interceptZ isn't realistic ")
        return None, None, None
    # CALCULATE FINAL POSITIONS
    fX = CATCH_AT_X
    fY = interceptY
    fZ = interceptZ
    return fX, fY, fZ

# MOCAP INCOMING DATA LOGIC
def callback_update_mocap(data):
    print("received mocap data")
    global ARM_SET_X, ARM_SET_Y, ARM_SET_Z # SET ARM FINAL POSITION IN THESE VARIABLES
    # TODO add in logic that takes mocap data and sets global X, Y, Z setpints for arm
    bX = data.pose.position.x
    bY = data.pose.position.z
    bZ = data.pose.position.y
    # CURRENTLY USING SETPOINTS AS WHERE ARM IS
    # TODO CHANGE THIS TO ACTUAL ARM POSITION. IS THIS A TOPIC WE CAN LISTEN TO?
    addSensors(ARM_SET_X, ARM_SET_Y, ARM_SET_Z, bX, bY, bZ)
    fX, fY, fZ = calcPosition()
    if(fX == None):
        return
    ARM_SET_X = fX
    ARM_SET_Y = fY
    ARM_SET_Z = fZ
    print("Setting setpoints to: ", ARM_SET_X, ARM_SET_Y, ARM_SET_Z)


#
# ARM
#

# ARM CONTROLLING LOGIC
def controlArm(arm_pub):
    global ARM_SET_X, ARM_SET_Y, ARM_SET_Z # USE THESE VARAIBLES AS SET POINTS
    print("Driving arm to: ", ARM_SET_X, ARM_SET_Y, ARM_SET_Z)
    # arm_pub.publish(arm_msg) # EXAMPLE CALL
    # TODO add in logic to set arm position to set X,Y,Z

if __name__ == "__main__":
    main()