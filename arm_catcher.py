#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
import numpy as np
import statistics as stats

MOCAP_TOPIC_NAME = "/vrpn_client_node/num_7/pose"
publisher_arm = None
publisher_intercept = None
publisher_path = None
publisher_ball = None
subscriber_mocap = None
publisher_vx = None
publisher_vy = None
publisher_vz = None
publisher_catchzone = None
publisher_catchcube = None
publisher_setarm_vis = None
publisher_setarm_path = None
UNIQUE_ARM_SET_HISTORY = [[], [], [], []]
ARM_SET_X = 0.25 # SET THIS TO ARM CATCH VALUE
ARM_SET_Y = 0 # STARTING POINT
ARM_SET_Z = 0 # STARTING POINT
# ROBOT SETTINGS
CATCH_AT_X = ARM_SET_X
LOWEST_Y = -1
HIGHEST_Y = 1
LOWEST_Z = 1
HIGHEST_Z = 2
hasCaught = False
visualize = True


CYCLE_TIME = 0.01
ball_paths = None
latest_seq_update = -1
X_MODIFIER = 1.0 # increase to overestimate x velocity, decrease to underestimate. 1 means normal
Y_MODIFIER = 1.0 # increase to overestimate x velocity, decrease to underestimate. 1 means normal
Z_MODIFIER = 1.0 # increase to overestimate x velocity, decrease to underestimate. 1 means normal

def main():
    global publisher_arm, publisher_catchzone, publisher_catchcube
    global ball_paths, latest_seq_update, visualize
    global CATCH_AT_X, LOWEST_Y, LOWEST_Z, HIGHEST_Y, HIGHEST_Z, ARM_SET_Y, ARM_SET_Z, hasCaught
    init()       
    # start loop
    bpsize = len(ball_paths)
    while not rospy.is_shutdown():
        startTime = time.time()
        # init catchzone
        if visualize:
            catchzone = PolygonStamped()
            catchzone.header.frame_id = "world"
            now = rospy.get_rostime()
            catchzone.header.stamp.secs = now.secs
            catchzone.header.stamp.nsecs = now.nsecs
            BL = Point32()
            BL.x = CATCH_AT_X
            BL.y = LOWEST_Y
            BL.z = LOWEST_Z
            BR = Point32()
            BR.x = CATCH_AT_X
            BR.y = LOWEST_Y
            BR.z = HIGHEST_Z
            TL = Point32()
            TL.x = CATCH_AT_X
            TL.y = HIGHEST_Y
            TL.z = HIGHEST_Z
            TR = Point32()
            TR.x = CATCH_AT_X
            TR.y = HIGHEST_Y
            TR.z = LOWEST_Z
            poly_pts = [BL, BR, TL, TR]
            catchzone.polygon.points = poly_pts
            publisher_catchzone.publish(catchzone)
            catchcube = Marker()
            catchcube.type = Marker.CUBE
            catchcube.action = Marker.ADD
            catchcube.header = catchzone.header
            catchcube.pose.position.x = CATCH_AT_X
            catchcube.pose.position.y = LOWEST_Y + (HIGHEST_Y - LOWEST_Y) / 2.0
            catchcube.pose.position.z = LOWEST_Z + (HIGHEST_Z - LOWEST_Z) / 2.0
            catchcube.scale.x = 0.05
            catchcube.scale.y = abs(HIGHEST_Y - LOWEST_Y)
            catchcube.scale.z = abs(HIGHEST_Z - LOWEST_Z)
            if hasCaught:
                catchcube.color.r = 0.0
                catchcube.color.g = 0.3
                catchcube.color.b = 0.0
            else:
                catchcube.color.r = 0.0
                catchcube.color.g = 0.3
                catchcube.color.b = 0.7
            catchcube.color.a = 0.35
            publisher_catchcube.publish(catchcube)
        # ARM CONTROLLING LOGIC GOES BELOW HERE
        if hasCaught == False:
            controlArm(publisher_arm)
        # ARM CONTROLLING LOGIC ENDS HERE
        # PATH PUBLISHER LOGIC GOES BELOW HERE
        if visualize:
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
    global publisher_arm, publisher_intercept, publisher_path, publisher_pred_path, publisher_ball, publisher_vx, publisher_vy, publisher_vz, publisher_catchzone, publisher_catchcube, publisher_setarm_vis
    global subscriber_mocap, publisher_setarm_path
    global ball_paths
    global hasCaught
    hasCaught = False
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
    publisher_catchzone = rospy.Publisher('catch_zone', PolygonStamped, queue_size=10)
    publisher_catchcube = rospy.Publisher('catch_cube', Marker, queue_size=10)
    publisher_setarm_vis = rospy.Publisher('arm_set_Vis', PointStamped, queue_size=10)
    publisher_setarm_path = rospy.Publisher('arm_set_path', Path, queue_size=10)




#
# MOCAP
#
#Store ball location history
pos_hist = [[], [], [], [], [], [], []] # time, arm x, arm y, arm z, ball x, ball y, ball z
history_length = 100
LOOKBACK_LENGTH = 10 # 2 means use the previous 2 datapoints, 3 means use the previous 3 datapoints, etc..
TIME = 0
ARM_X = 1
ARM_Y = 2
ARM_Z = 3
BALL_X = 4
BALL_Y = 5
BALL_Z = 6


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
    global pos_hist, publisher_pred_path, publisher_vx, publisher_vy, publisher_vz, CATCH_AT_X, X_MODIFIER, Y_MODIFIER, Z_MODIFIER, UNIQUE_ARM_SET_HISTORY, publisher_intercept, publisher_setarm_vis
    global hasCaught, visualize
    # check to see if loop repeated
    if len(pos_hist[TIME]) >= 2:
        if pos_hist[TIME][-1] < pos_hist[TIME][-2]:
            pos_hist[TIME] = []
            pos_hist[ARM_X] = []
            pos_hist[ARM_Y] = []
            pos_hist[ARM_Z] = []
            pos_hist[BALL_X] = []
            pos_hist[BALL_Y] = []
            pos_hist[BALL_Z] = []
            UNIQUE_ARM_SET_HISTORY[TIME] = []
            UNIQUE_ARM_SET_HISTORY[ARM_X] = []
            UNIQUE_ARM_SET_HISTORY[ARM_Y] = []
            UNIQUE_ARM_SET_HISTORY[ARM_Z] = []
            hasCaught = False

    # Only run if possible
    if len(pos_hist[TIME]) < 2:
        print("Not setting setpoints. Not enough data")
        return None, None, None
    # Calculating differences 
    lookback = getLookbackLength()
    dX = pos_hist[BALL_X][-1] - pos_hist[BALL_X][lookback]
    dY = pos_hist[BALL_Y][-1] - pos_hist[BALL_Y][lookback]
    dZ = pos_hist[BALL_Z][-1] - pos_hist[BALL_Z][lookback]
    dT = pos_hist[TIME][-1] - pos_hist[TIME][lookback]
    # Calculating velocities
    vX = dX / dT * X_MODIFIER
    vY = dY / dT * Y_MODIFIER
    vZ = dZ / dT * Z_MODIFIER
    # Checking if ball is headed towards arm
    if (vX > 0 and pos_hist[BALL_X][-1]  >= pos_hist[ARM_X][-1]) or (vX < 0 and pos_hist[BALL_X][-1] <= pos_hist[ARM_X][-1]):
        print("Not setting setpoints. Ball not headed towards arm")
        return None, None, None
    # Calculating time left & intercepts
    interceptX = CATCH_AT_X
    distanceLeft = abs(pos_hist[BALL_X][-1] - interceptX)
    timeLeft = abs(distanceLeft / vX)
    interceptY = pos_hist[BALL_Y][-1] + vY * timeLeft
    interceptZ = pos_hist[BALL_Z][-1] + vZ * timeLeft - 4.9 * (timeLeft**2)
    if visualize:
        # PUBLISH PREDICTED PATH TO THE VISUALIZER
        pred_path = []
        for i in np.arange(0, 1, 0.01):
            nX = pos_hist[BALL_X][-1] + vX * (i * timeLeft)
            nY = pos_hist[BALL_Y][-1] + vY * (i * timeLeft)
            nZ = pos_hist[BALL_Z][-1] + vZ * (i * timeLeft) - 4.9*((i * timeLeft)**2)
            newPos = PoseStamped()
            newPos.pose.position.x = nX
            newPos.pose.position.y = nY
            newPos.pose.position.z = nZ
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
        vXO.pose.position.y = pos_hist[BALL_Y][-1]
        vXO.pose.position.z = pos_hist[BALL_Z][-1]
        vXF = PoseStamped()
        vXF.pose.position.x = pos_hist[BALL_X][-1] + vX
        vXF.pose.position.y = pos_hist[BALL_Y][-1]
        vXF.pose.position.z = pos_hist[BALL_Z][-1]
        vXM.poses = [vXO, vXF]
        vXM.header = pred_path_msg.header
        publisher_vx.publish(vXM)
        #y
        vYM = Path()
        vYO = PoseStamped()
        vYO.pose.position.x = pos_hist[BALL_X][-1]
        vYO.pose.position.y = pos_hist[BALL_Y][-1]
        vYO.pose.position.z = pos_hist[BALL_Z][-1]
        vYF = PoseStamped()
        vYF.pose.position.x = pos_hist[BALL_X][-1]
        vYF.pose.position.y = pos_hist[BALL_Y][-1] + vY
        vYF.pose.position.z = pos_hist[BALL_Z][-1]
        vYM.poses = [vYO, vYF]
        vYM.header = pred_path_msg.header
        publisher_vy.publish(vYM)
        #z
        vZM = Path()
        vZO = PoseStamped()
        vZO.pose.position.x = pos_hist[BALL_X][-1]
        vZO.pose.position.y = pos_hist[BALL_Y][-1]
        vZO.pose.position.z = pos_hist[BALL_Z][-1]
        vZF = PoseStamped()
        vZF.pose.position.x = pos_hist[BALL_X][-1]
        vZF.pose.position.y = pos_hist[BALL_Y][-1]
        vZF.pose.position.z = pos_hist[BALL_Z][-1] + vZ
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
    # PUBLISH INTERCEPT
    if visualize:
        int_msg = PointStamped()
        int_msg.header.frame_id = "world"
        int_msg.header.stamp = rospy.Time.now()
        int_msg.point.x = interceptX
        int_msg.point.y = interceptY
        int_msg.point.z = interceptZ
        publisher_intercept.publish(int_msg)
    # CALCULATE FINAL POSITIONS
    fX = CATCH_AT_X
    fY = interceptY
    fZ = interceptZ

    return fX, fY, fZ

lX = -1.0
# MOCAP INCOMING DATA LOGIC
def callback_update_mocap(data):
    global ARM_SET_X, ARM_SET_Y, ARM_SET_Z # SET ARM FINAL POSITION IN THESE VARIABLES
    global ball_paths, latest_seq_update, publisher_ball, UNIQUE_ARM_SET_HISTORY, publisher_setarm_vis, publisher_setarm_path, pos_hist, X_MODIFIER
    global lX, hasCaught, visualize
    if visualize:
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

    bX = data.pose.position.x
    bY = data.pose.position.y
    bZ = data.pose.position.z
    data_time = rospy.Time(data.header.stamp.secs, data.header.stamp.nsecs)
    print("Received: ", bX, bY, bZ, " at time ", data_time.to_sec())
    # CHECK IF BALL HAS PASSED PLANE
    if not (lX == -1.0):
        if(np.sign(lX - ARM_SET_X) != np.sign(bX - ARM_SET_X)):
            hasCaught = True
    lX = bX
    # CURRENTLY USING SETPOINTS AS WHERE ARM IS
    # TODO CHANGE THIS TO ACTUAL ARM POSITION. IS THIS A TOPIC WE CAN LISTEN TO?
    addSensors(data_time.to_sec(), ARM_SET_X, ARM_SET_Y, ARM_SET_Z, bX, bY, bZ)
    fX, fY, fZ = calcPosition()
    if(fX == None):
        return
    ARM_SET_X = fX
    ARM_SET_Y = fY
    ARM_SET_Z = fZ
    UNIQUE_ARM_SET_HISTORY[TIME].append(data_time.to_sec())
    UNIQUE_ARM_SET_HISTORY[ARM_X].append(ARM_SET_X)
    UNIQUE_ARM_SET_HISTORY[ARM_Y].append(ARM_SET_Y)
    UNIQUE_ARM_SET_HISTORY[ARM_Z].append(ARM_SET_Z)
    # SMOOTHED WEIGHTED ENDPOINT METHOD
    '''
    if(len(UNIQUE_ARM_SET_HISTORY) > 4):
        UNIQUE_ARM_SET_HISTORY = UNIQUE_ARM_SET_HISTORY[:4]
    if(len(UNIQUE_ARM_SET_HISTORY) == 4):
        ARM_SET_Y = UNIQUE_ARM_SET_HISTORY[ARM_Y][-1] * 0.5 + UNIQUE_ARM_SET_HISTORY[ARM_Y][-2] * 0.25 + UNIQUE_ARM_SET_HISTORY[ARM_Y][-3] * 0.125 + UNIQUE_ARM_SET_HISTORY[ARM_Y][-1] * 0.125
        ARM_SET_X = UNIQUE_ARM_SET_HISTORY[ARM_X][-1] * 0.5 + UNIQUE_ARM_SET_HISTORY[ARM_X][-2] * 0.25 + UNIQUE_ARM_SET_HISTORY[ARM_X][-3] * 0.125 + UNIQUE_ARM_SET_HISTORY[ARM_X][-1] * 0.125
    elif(len(UNIQUE_ARM_SET_HISTORY) == 3):
        ARM_SET_Y = UNIQUE_ARM_SET_HISTORY[ARM_Y][-1] * 0.6 + UNIQUE_ARM_SET_HISTORY[ARM_Y][-2] * 0.25 + UNIQUE_ARM_SET_HISTORY[ARM_Y][-3] * 0.15
        ARM_SET_X = UNIQUE_ARM_SET_HISTORY[ARM_X][-1] * 0.6 + UNIQUE_ARM_SET_HISTORY[ARM_X][-2] * 0.25 + UNIQUE_ARM_SET_HISTORY[ARM_X][-3] * 0.15
    elif(len(UNIQUE_ARM_SET_HISTORY) == 2):
        ARM_SET_Y = UNIQUE_ARM_SET_HISTORY[ARM_Y][-1] * 0.67 + UNIQUE_ARM_SET_HISTORY[ARM_Y][-2] * 0.33
        ARM_SET_X = UNIQUE_ARM_SET_HISTORY[ARM_X][-1] * 0.67 + UNIQUE_ARM_SET_HISTORY[ARM_X][-2] * 0.33 '''
    # TIME BASED WEIGHTING METHOD
    TIME_RANGE = 0.5
    recentPts = [[], [], [], []]
    i = -1
    while(abs(i) <= len(UNIQUE_ARM_SET_HISTORY[TIME]) and UNIQUE_ARM_SET_HISTORY[TIME][i] >= UNIQUE_ARM_SET_HISTORY[TIME][-1] - TIME_RANGE):
        recentPts[TIME].append(UNIQUE_ARM_SET_HISTORY[TIME][i])
        recentPts[ARM_X].append(UNIQUE_ARM_SET_HISTORY[ARM_X][i])
        recentPts[ARM_Y].append(UNIQUE_ARM_SET_HISTORY[ARM_Y][i])
        recentPts[ARM_Z].append(UNIQUE_ARM_SET_HISTORY[ARM_Z][i])
        i -= 1
    if(len(recentPts) > 1):
        ARM_SET_Y = stats.mean(recentPts[ARM_Y])
        ARM_SET_Z = stats.mean(recentPts[ARM_Z])
        print("TIME: ", len(recentPts[TIME]))
    print("Setting setpoints to: ", ARM_SET_X, ARM_SET_Y, ARM_SET_Z)
    if visualize:
        setarm_vis_msg = PointStamped()
        setarm_vis_msg.header.frame_id = "world"
        setarm_vis_msg.header.stamp = rospy.Time.now()
        setarm_vis_msg.point.x = ARM_SET_X
        setarm_vis_msg.point.y = ARM_SET_Y
        setarm_vis_msg.point.z = ARM_SET_Z
        publisher_setarm_vis.publish(setarm_vis_msg)
        # visualizing setarm path
        set_path = Path()
        set_path.header.frame_id = "world"
        set_path.header.stamp = rospy.Time.now()
        set_path.poses = []
        lookback = getLookbackLength()
        vX = (pos_hist[BALL_X][-1] - pos_hist[BALL_X][lookback]) / (pos_hist[TIME][-1] - pos_hist[TIME][lookback]) * X_MODIFIER
        timeLeft = abs((pos_hist[BALL_X][-1] - ARM_SET_X)/vX)
        pvY = (ARM_SET_Y - pos_hist[BALL_Y][-1])/timeLeft
        pvZ = (ARM_SET_Z - pos_hist[BALL_Z][-1] + 4.9 * (timeLeft ** 2))/timeLeft
        for i in np.arange(0, 1, 0.01):
            nX = pos_hist[BALL_X][-1] + vX * (i * timeLeft)
            nY = pos_hist[BALL_Y][-1] + pvY * (i * timeLeft)
            nZ = pos_hist[BALL_Z][-1] + pvZ * (i * timeLeft) - 4.9*((i * timeLeft)**2)
            newPos = PoseStamped()
            newPos.pose.position.x = nX
            newPos.pose.position.y = nY
            newPos.pose.position.z = nZ
            newPos.header.frame_id = "world"
            newPos.header.stamp = rospy.Time.now()
            set_path.poses.append(newPos)
        publisher_setarm_path.publish(set_path)
    


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


















    ## OLD CALCPOS, BEFORE REWRITE
'''
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

    return fX, fY, fZ'''