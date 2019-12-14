#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
import numpy as np

MOCAP_TOPIC_NAME = "/vrpn_client_node/num_7/pose"
publisher_arm = None
publisher_intercept = None
publisher_path = None
publisher_ball = None
subscriber_mocap = None
publisher_vx = None
publisher_vy = None
publisher_vz = None
ARM_SET_X = -0.35 # SET THIS TO ARM CATCH VALUE
ARM_SET_Y = 0
ARM_SET_Z = 0
CYCLE_TIME = 0.01
ball_paths = None
latest_seq_update = -1

def main():
    global publisher_arm
    global ball_paths, latest_seq_update
    init()
    bpsize = len(ball_paths)
    while not rospy.is_shutdown():
        startTime = time.time()
        # ARM CONTROLLING LOGIC GOES BELOW HERE
        controlArm(publisher_arm)
        # ARM CONTROLLING LOGIC ENDS HERE
        # INTERCEPT PUBLISHER LOGIC GOES BELOW HERE
        msg = PointStamped()
        msg.point.x = ARM_SET_X
        msg.point.y = ARM_SET_Y
        msg.point.z = ARM_SET_Z
        msg.header.frame_id = "world"
        now = rospy.get_rostime()
        msg.header.stamp.secs = now.secs
        msg.header.stamp.nsecs = now.nsecs
        publisher_intercept.publish(msg)
        # INTERCEPT PUBLISHER LOGIC GOES ABOVE HERE
        # PATH PUBLISHER LOGIC GOES BELOW HERE
        if len(ball_paths) > bpsize:
            msg2 = Path()
            msg2.poses = ball_paths
            msg2.header.frame_id = "world"
            now = rospy.get_rostime()
            msg2.header.stamp.secs = now.secs
            msg2.header.stamp.nsecs = now.nsecs
            publisher_path.publish(msg2)
            bpsize = len(ball_paths)
        # PATH PUBLISER LOGIC GOES ABOVE HERE
        endTime = time.time()
        if endTime - startTime < CYCLE_TIME:
            rospy.sleep(CYCLE_TIME - (endTime - startTime))

def init():
    global publisher_arm, publisher_intercept, publisher_path, publisher_pred_path, publisher_ball, publisher_vx, publisher_vy, publisher_vz
    global subscriber_mocap
    global ball_paths
    rospy.init_node("catcher_driver")

    # path setup
    ball_paths = []

    # SUBSCRIBER- geometry_msgs/PoseStamped
    subscriber_mocap = rospy.Subscriber(MOCAP_TOPIC_NAME, PoseStamped, callback_update_mocap)

    # PUBLISHER
    #publisher_arm = rospy.Publisher('topic_name', message_type , queue_size=10) # TODO fill in parameters
    publisher_intercept = rospy.Publisher('intercept', PointStamped, queue_size=10)
    publisher_path = rospy.Publisher('ball_path', Path, queue_size=10)
    publisher_pred_path = rospy.Publisher('pred_ball_path', Path, queue_size=10)
    publisher_ball = rospy.Publisher('ball', PointStamped, queue_size=10)
    publisher_vx = rospy.Publisher('ball_vx', Path, queue_size=10)
    publisher_vy = rospy.Publisher('ball_vy', Path, queue_size=10)
    publisher_vz = rospy.Publisher('ball_vz', Path, queue_size=10)




#
# MOCAP
#
#Store ball location history
pos_hist = [[], [], [], [], [], [], []] # time, arm x, arm y, arm z, ball x, ball y, ball z
history_length = 100
LOOKBACK_LENGTH = 5 # 2 means use the previous 2 datapoints, 3 means use the previous 3 datapoints, etc..
TIME = 0
ARM_X = 1
ARM_Y = 2
ARM_Z = 3
BALL_X = 4
BALL_Y = 5
BALL_Z = 6
# ROBOT SETTINGS
CATCH_AT_X = ARM_SET_X
LOWEST_Y = 1
HIGHEST_Y = 2
LOWEST_Z = -1
HIGHEST_Z = 1

def getLookbackLength():
    global LOOKBACK_LENGTH, pos_hist
    usePrevious = min(LOOKBACK_LENGTH, max(2, len(pos_hist[TIME])))
    return -1 * usePrevious

# method that adds sensor values to pos_history datastructure
def addSensors(t, aX, aY, aZ, bX, bY, bZ):
    global pos_hist
    pos_hist[TIME].append(t) # ADD TIME STAMP
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
    global pos_hist, publisher_pred_path, publisher_vx, publisher_vy, publisher_vz
    # ONLY RUN IF POSSIBLE
    if len(pos_hist[TIME]) < 2:
        print("Not setting setpoints. Not enough data")
        return None, None, None
    # CALCULATE LATEST VEL X & TIME LEFT
    dX = pos_hist[BALL_X][-1] - pos_hist[BALL_X][getLookbackLength()]
    dT = (pos_hist[TIME][-1] - pos_hist[TIME][getLookbackLength()])
    vX = dX/dT
    distanceLeft = pos_hist[BALL_X][-1] - pos_hist[ARM_X][-1]
    timeLeft = abs(distanceLeft / vX)
    # CALCULATE VELOCITY & INTERCEPT
    vY = (pos_hist[BALL_Y][-1] - pos_hist[BALL_Y][getLookbackLength()])/dT
    interceptY = pos_hist[BALL_Y][-1] + vY*timeLeft - 4.9*timeLeft**2 #Y_0 + VY*T - 4.9T^2
    # CALCULATE END POSITION FOR Z
    dZ = pos_hist[BALL_Z][-1] - pos_hist[BALL_Z][getLookbackLength()]
    vZ = dZ / dT
    interceptZ = pos_hist[BALL_Z][-1] + vZ * timeLeft
    print ("INTERCEPT BALL AT: ", CATCH_AT_X, interceptY, interceptZ)
    # PUBLISH PREDICTED PATH TO THE VISUALIZER
    pred_path = []
    for i in np.arange(0, 1, 0.01):
        nX = pos_hist[BALL_X][-1] + vX * (i * timeLeft)
        nY = pos_hist[BALL_Y][-1] + vY * (i * timeLeft) - 4.9*((i * timeLeft)**2)
        nZ = pos_hist[BALL_Z][-1] + vX * (i * timeLeft)
        newPos = PoseStamped()
        newPos.pose.position.x = nX
        newPos.pose.position.y = nZ
        newPos.pose.position.z = nY
        newPos.header.frame_id = "world"
        now = rospy.get_rostime()
        newPos.header.stamp.secs = now.secs
        newPos.header.stamp.nsecs = now.nsecs
        pred_path.append(newPos)
    pred_path_msg = Path()
    pred_path_msg.poses = pred_path
    pred_path_msg.header.frame_id = "world"
    now = rospy.get_rostime()
    pred_path_msg.header.stamp.secs = now.secs
    pred_path_msg.header.stamp.nsecs = now.nsecs
    publisher_pred_path.publish(pred_path_msg)
    # PUBLISH VX, VY, VZ TO THE VISUALIZER
    vXM = Path()
    vXO = PoseStamped()
    vXO.pose.position.x = pos_hist[BALL_X][-1]
    vXO.pose.position.y = pos_hist[BALL_Z][-1]
    vXO.pose.position.z = pos_hist[BALL_Y][-1]
    vXF = PoseStamped()
    vXF.pose.position.x = pos_hist[BALL_X][-1] + vX
    vXF.pose.position.y = pos_hist[BALL_Z][-1]
    vXF.pose.position.z = pos_hist[BALL_Y][-1]
    vXM.poses = [vXO, vXF]
    vXM.header = pred_path_msg.header
    publisher_vx.publish(vXM)
    #y
    vYM = Path()
    vYO = PoseStamped()
    vYO.pose.position.x = pos_hist[BALL_X][-1]
    vYO.pose.position.y = pos_hist[BALL_Z][-1]
    vYO.pose.position.z = pos_hist[BALL_Y][-1]
    vYF = PoseStamped()
    vYF.pose.position.x = pos_hist[BALL_X][-1]
    vYF.pose.position.y = pos_hist[BALL_Z][-1]
    vYF.pose.position.z = pos_hist[BALL_Y][-1] + vY
    vYM.poses = [vYO, vYF]
    vYM.header = pred_path_msg.header
    publisher_vy.publish(vYM)
    #z
    vZM = Path()
    vZO = PoseStamped()
    vZO.pose.position.x = pos_hist[BALL_X][-1]
    vZO.pose.position.y = pos_hist[BALL_Z][-1]
    vZO.pose.position.z = pos_hist[BALL_Y][-1]
    vZF = PoseStamped()
    vZF.pose.position.x = pos_hist[BALL_X][-1]
    vZF.pose.position.y = pos_hist[BALL_Z][-1] + vZ
    vZF.pose.position.z = pos_hist[BALL_Y][-1]
    vZM.poses = [vZO, vZF]
    vZM.header = pred_path_msg.header
    publisher_vz.publish(vZM)
    print("velocities: " , vX, vY, vZ)
    # CHECK IF THIS IS NOT A CATCHABLE BALL
    if interceptY < LOWEST_Y or interceptY > HIGHEST_Y:
        print("Not setting setpoints. interceptY isn't realistic ")
        return None, None, None
    if interceptZ < LOWEST_Z or interceptZ > HIGHEST_Z:
        print("Not setting setpoints. interceptZ isn't realistic: ")
        return None, None, None
    # CALCULATE FINAL POSITIONS
    fX = CATCH_AT_X
    fY = interceptY
    fZ = interceptZ
    
    return fX, fY, fZ

# MOCAP INCOMING DATA LOGIC
def callback_update_mocap(data):
    global ARM_SET_X, ARM_SET_Y, ARM_SET_Z # SET ARM FINAL POSITION IN THESE VARIABLES
    global ball_paths, latest_seq_update, publisher_ball
    if data.header.seq > latest_seq_update:
        latest_seq_update = data.header.seq
        ball_paths.append(data)
    ball_point = PointStamped()
    ball_point.header = data.header
    ball_point.point.x = data.pose.position.x
    ball_point.point.y = data.pose.position.y
    ball_point.point.z = data.pose.position.z
    publisher_ball.publish(ball_point)
    print("received mocap data")
    
    # TODO add in logic that takes mocap data and sets global X, Y, Z setpints for arm
    bX = data.pose.position.x
    bY = data.pose.position.z
    bZ = data.pose.position.y
    data_time = rospy.Time(data.header.stamp.secs, data.header.stamp.nsecs)
    print("Received: ", bX, bY, bZ, " at time ", data_time.to_sec())
    # CURRENTLY USING SETPOINTS AS WHERE ARM IS
    # TODO CHANGE THIS TO ACTUAL ARM POSITION. IS THIS A TOPIC WE CAN LISTEN TO?
    addSensors(data_time.to_sec(), ARM_SET_X, ARM_SET_Y, ARM_SET_Z, bX, bY, bZ)
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
    #print("Driving arm to: ", ARM_SET_X, ARM_SET_Y, ARM_SET_Z)
    # arm_pub.publish(arm_msg) # EXAMPLE CALL
    # TODO add in logic to set arm position to set X,Y,Z

if __name__ == "__main__":
    main()