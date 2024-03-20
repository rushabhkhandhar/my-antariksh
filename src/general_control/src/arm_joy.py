#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

# Initialize ROS node
rospy.init_node('arm_control')

# Define global variables for arm control
global base, rev_1, rev_2, gripper_x, gripper_y, bevel1, bevel2, bevel3, bevel4, rev_4_1, rev_4_2, button_11
base = 0
rev_1 = 0
rev_2 = 0
gripper_x = 0
gripper_y = 0
bevel1 = 0
bevel2 = 0
bevel3 = 0
bevel4 = 0
rev_4_1 = 0
rev_4_2 = 0
button_11 = 0

# Publisher for arm control commands
arm_cmd_pub = rospy.Publisher('command_for_arm', Joy, queue_size=10)

# Callback function to handle received Joy messages
def arm_control_callback(data):
    global base, rev_1, rev_2, gripper_x, gripper_y, bevel1, bevel2, bevel3, bevel4, rev_4_1, rev_4_2, button_11
    base = data.axes[2]
    rev_1 = data.axes[1]
    rev_2 = data.axes[0]
    gripper_x = data.axes[4]
    gripper_y = data.axes[5]

    bevel1 = data.buttons[2]
    bevel2 = data.buttons[3]
    bevel3 = data.buttons[4]
    bevel4 = data.buttons[5]

    rev_4_1 = data.buttons[0]
    rev_4_2 = data.buttons[1]

    button_11 = data.buttons[11]

# Subscriber for arm control commands
rospy.Subscriber("command_for_arm", Joy, arm_control_callback)

# Main loop
def main_loop():
    global base, rev_1, rev_2, gripper_x, gripper_y, bevel1, bevel2, bevel3, bevel4, rev_4_1, rev_4_2, button_11
    rospy.loginfo("Arm control node started")
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Create Joy message to control the arm
        joy_msg = Joy()
        joy_msg.axes = [0, 0, 0, 0, 0, 0]  # Initialize all axes to zero
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]  # Initialize all buttons to zero

        # Map joystick axes and buttons to control the arm according to global variables
        joy_msg.axes[2] = base
        joy_msg.axes[1] = rev_1
        joy_msg.axes[0] = rev_2
        joy_msg.axes[4] = gripper_x
        joy_msg.axes[5] = gripper_y

        joy_msg.buttons[2] = bevel1
        joy_msg.buttons[3] = bevel2
        joy_msg.buttons[4] = bevel3
        joy_msg.buttons[5] = bevel4

        joy_msg.buttons[0] = rev_4_1
        joy_msg.buttons[1] = rev_4_2

        joy_msg.buttons[11] = button_11

        # Publish arm control command
        arm_cmd_pub.publish(joy_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
