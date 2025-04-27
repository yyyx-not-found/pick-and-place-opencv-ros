#!/usr/bin/env python

import sys
import copy
import time
import rospy

import numpy as np
from lab6_header import *
from lab6_func import *
from blob_search import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of green, yellow, and red blocks
xw_yw_G = []
xw_yw_Y = []
xw_yw_R = []

# Any other global variable you want to define
# Hints: where to put the blocks?
block_edge = 19
target_xw_yw = [100, 0]
n_blocks = 0

# ========================= Student's code ends here ===========================

################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, angle, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    # global variable1
    # global variable2

    error = 0

    # Pick 
    print(f"Start: {start_xw_yw_zw} (Orientation: {angle})")
    start_xw_yw_zw_above = copy.deepcopy(start_xw_yw_zw)
    start_xw_yw_zw_above[-1] = 100
    if angle < 0:
        start_theta = lab_invk(*start_xw_yw_zw, 0)
        start_above_theta = lab_invk(*start_xw_yw_zw_above, 0)
        theta_rot = lab_invk(*start_xw_yw_zw, -angle)
        above_theta_rot = lab_invk(*start_xw_yw_zw_above, -angle)
    else:
        start_theta = lab_invk(*start_xw_yw_zw, angle)
        start_above_theta = lab_invk(*start_xw_yw_zw_above, angle)
        theta_rot = lab_invk(*start_xw_yw_zw, 0)
        above_theta_rot = lab_invk(*start_xw_yw_zw_above, 0)

    move_arm(pub_cmd, loop_rate, start_above_theta, vel, accel)
    move_arm(pub_cmd, loop_rate, start_theta, vel/2, accel/2)
    gripper(pub_cmd, loop_rate, suction_on)
    rospy.sleep(1)
    if current_io_0 and not digital_in_0:
        gripper(pub_cmd, loop_rate, suction_off)
        rospy.logerr("No block found.")
        error = 1
        return error
    
    move_arm(pub_cmd, loop_rate, start_above_theta, vel/2, accel/2)
    rospy.sleep(0.5)

    # Rotate
    move_arm(pub_cmd, loop_rate, above_theta_rot, vel/2, accel/2)
    rospy.sleep(0.5)
    
    # Place
    print(f"Target: {target_xw_yw_zw}")
    target_xw_yw_zw_above = copy.deepcopy(target_xw_yw_zw)
    target_xw_yw_zw_above[-1] = 100
    if angle < 0:
        target_theta = lab_invk(*target_xw_yw_zw, -angle)
        target_above_theta = lab_invk(*target_xw_yw_zw_above, -angle)
    else:
        target_theta = lab_invk(*target_xw_yw_zw, 0)
        target_above_theta = lab_invk(*target_xw_yw_zw_above, 0)
    
    move_arm(pub_cmd, loop_rate, target_above_theta, vel/2, accel/2)
    rospy.sleep(0.5)
    move_arm(pub_cmd, loop_rate, target_theta, vel/2, accel/2)
    gripper(pub_cmd, loop_rate, suction_off)
    rospy.sleep(1)
    move_arm(pub_cmd, loop_rate, target_above_theta, vel/2, accel/2)
    
    # ========================= Student's code ends here ===========================

    return error

class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_G # store found green blocks in this list
        global xw_yw_Y # store found yellow blocks in this list
        global xw_yw_R # store found red blocks in this list

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_Y.

        # Remember, xw_yw_G & xw_yw_Y are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        xw_yw_G = blob_search(cv_image, "green")
        xw_yw_Y = blob_search(cv_image, "yellow")
        xw_yw_R = blob_search(cv_image, "red")


"""
Program run from here
"""
def main():

    global go_away
    global xw_yw_R
    global xw_yw_G
    global xw_yw_Y
    global n_blocks
    global target_xw_yw

    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab6node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)

    # ========================= Student's code starts here =========================

    """
    Hints: use the found xw_yw_G, xw_yw_Y to move the blocks correspondingly. You will
    need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
    """
    # For camera calibration
    # print(IMG2W(334, 67))

    xw_yw_G_copy = np.array(xw_yw_G)
    xw_yw_Y_copy = np.array(xw_yw_Y)
    xw_yw_R_copy = np.array(xw_yw_R)

    print("========= Green ==========")
    print(xw_yw_G_copy)
    print("========= Yellow ==========")
    print(xw_yw_Y_copy)
    print("========= Red ==========")
    print(xw_yw_R_copy)
    print("===========================")

    # Group blocks (We assume green is always the center, and yellow is at the front)
    blocks = []
    used_indices_Y = set()
    used_indices_G = set()
    used_indices_R = set()
    for i_Y, pos_Y in enumerate(xw_yw_Y_copy):
        if i_Y in used_indices_Y:
            continue
        for i_R, pos_R in enumerate(xw_yw_R_copy):
            if i_R in used_indices_R:
                continue
            for i_G, pos_G in enumerate(xw_yw_G_copy):
                if i_G in used_indices_G:
                    continue

                # rospy.loginfo("==============================")
                # rospy.loginfo("Check the following pairs:")
                # rospy.loginfo(f"Yellow: {pos_Y}")
                # rospy.loginfo(f"Green: {pos_G}")
                # rospy.loginfo(f"Red: {pos_R}")
                # rospy.loginfo(f"d(T, G) = {np.linalg.norm(pos_Y - pos_G, ord=2)}")
                # rospy.loginfo(f"d(R, G) = {np.linalg.norm(pos_R - pos_G, ord=2)}")
                # rospy.loginfo(f"mid_pt = {(pos_Y + pos_G) / 2}")
                # rospy.loginfo("==============================")

                # Check (pos_Y, pos_G, pos_B) satisfies block structure
                Y_is_connect_to_G = np.linalg.norm(pos_Y - pos_G, ord=2) <= block_edge + 10
                R_is_connect_to_G = np.linalg.norm(pos_R - pos_G, ord=2) <= block_edge + 10
                mid_pt = (pos_Y + pos_G) / 2
                G_is_between_Y_and_R = np.linalg.norm(pos_G - mid_pt, ord=2) <= 20
                if Y_is_connect_to_G and R_is_connect_to_G and G_is_between_Y_and_R:
                    # Calculate orientation and angle need to rotate
                    direction = pos_Y - pos_G
                    angle = np.degrees(np.arctan2(direction[1], direction[0]))

                    # Add block
                    blocks.append((pos_Y, pos_G, pos_R, angle))
                    used_indices_Y.add(i_Y)
                    used_indices_G.add(i_G)
                    used_indices_R.add(i_R)

    for i, block in enumerate(blocks):
        rospy.loginfo(f"Block {i}: {block}")
        _, pos_G, _, angle = block
        xw_start, yw_start = pos_G
        xw_target, yw_target = target_xw_yw
        if move_block(pub_command, loop_rate, [xw_start, yw_start, block_edge], [xw_target, yw_target, block_edge * (n_blocks + 1)], angle, vel, accel) == 0:
            n_blocks += 1

    # ========================= Student's code ends here ===========================

    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
