import intera_interface
import rospy
import copy 
import time
import random

from geometry_msgs.msg import Pose, Point, Quaternion

g_limb = None
g_orientation_hand_down = None
g_position_neutral = None

def init():
    global g_limb, g_orientation_hand_down, g_position_neutral
    rospy.init_node('cairo_sawyer_ik_example')
    g_limb = intera_interface.Limb('right')

    # This quaternion will have the hand face straight down (ideal for picking tasks)
    g_orientation_hand_down = Quaternion()
    g_orientation_hand_down.x = 0.704238785359
    g_orientation_hand_down.y =0.709956638597
    g_orientation_hand_down.z = -0.00229009932359
    g_orientation_hand_down.w = 0.00201493272073

    # This is the default neutral position for the robot's hand (no guarantee this will move the joints to neutral though)
    g_position_neutral = Point()
    OX = 0.5
    OY = 0.0
    OZ = 0.432 # 
    g_position_neutral.x = OX
    g_position_neutral.y = OY
    g_position_neutral.z = OZ
    g_limb.move_to_neutral()

def main():
    init()

    OX = 0.5
    OY = 0.0
    OZ = 0.432
    helper(OX, OY, OZ)
    '''print("SENDING TO", OX, OY, OZ)
    waitToStart = time.time()
    while time.time() - waitToStart < 2:
        h = 3
    while(True):
        startTime = time.time()
        NX = OX
        NY = OY + (random.random() - 0.5)
        NZ = (random.random()) * 0.864
        helper(NX, NY, NZ)
        print("SENDING TO", NX, NY, NZ)
        while time.time() - startTime < 4:
            h=3'''
            

def helper(x,y,z):
 # Create a new pose (Position and Orientation) to solve for
    target_pose = Pose()
    target_pose.orientation = copy.deepcopy(g_orientation_hand_down)
    target_pose.position.x = x # Add 20cm to the x axis position of the hand
    target_pose.position.y = y
    target_pose.position.z = z



    # Call the IK service to solve for joint angles for the desired pose
    target_joint_angles = g_limb.ik_request(target_pose, "right_hand")

    # The IK Service returns false if it can't find a joint configuration
    if target_joint_angles is False:
        rospy.logerr("Couldn't solve for position %s" % str(target_pose))
        return

    # Set the robot speed (takes a value between 0 and 1)
    g_limb.set_joint_position_speed(.2)

    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish
    g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
    print("Submitted request to move arm to", x, y, z)

if __name__ == "__main__":
    print("Starting program...")
    main()
