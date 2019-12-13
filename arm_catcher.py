#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String

publisher_arm = None
subscriber_mocap = None
CYCLE_TIME = 0.1 # In seconds
ARM_SET_X = 0
ARM_SET_Y = 0
ARM_SET_Z = 0

def main():
    init()
    while not rospy.is_shutdown():
        global publisher_arm
        startTime = time.time()
        print("TURN")
        # ARM CONTROLLING LOGIC GOES BELOW HERE
        
        # ARM CONTROLLING LOGIC ENDS HERE
        endTime = time.time()
        if endTime - startTime < CYCLE_TIME:
            rospy.sleep(CYCLE_TIME - (endTime - startTime))

def init():
    global publisher_arm
    global subscriber_mocap
    rospy.init_node("catcher_driver")

    # SUBSCRIBER
    #subscriber_mocap = rospy.Subscriber('topic_name', message_type, callback_update_mocap) TODO fill in parameters

    # PUBLISHER
    #publisher_arm = rospy.Publisher('topic_name', message_type , queue_size=10) TODO fill in parameters

# MOCAP INCOMING DATA LOGIC
def callback_update_mocap(data):
    print("received mocap data")
    # TODO add in logic that takes mocap data and sets global X, Y, Z setpints for arm

# ARM CONTROLLING LOGIC
def controlArm(arm_pub):
    global ARM_SET_X, ARM_SET_Y, ARM_SET_Z
    print("controlling arm")
    # arm_pub.publish(arm_msg) # EXAMPLE CALL
    # TODO add in logic to set arm position to set X,Y,Z

if __name__ == "__main__":
    main()