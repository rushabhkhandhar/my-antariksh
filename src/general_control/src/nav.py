#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import sys, select, termios, tty

# Define the mapping of keyboard keys to rover commands
key_mapping = {
    'w': 1,  # Move forward
    's': 2,  # Move backward
    ' ': 3,  # Stop
    'a': 4,  # Rotate clockwise
    'd': 5,  # Rotate anticlockwise
    'x': 6   # Stop motor
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    rospy.init_node("rover_keyboard_control") # Initialize ROS node

    # Define the publisher for sending commands to the rover
    command_pub = rospy.Publisher("command_for_ros", Int32, queue_size=10)

    settings = termios.tcgetattr(sys.stdin)

    rospy.loginfo("Use 'w' to move forward, 's' to move backward, 'space' to stop, 'a' to rotate clockwise, 'd' to rotate anticlockwise, and 'x' to stop motor.")

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key in key_mapping:
                command = key_mapping[key]
                rospy.loginfo("Sending command: {}".format(command))
                command_pub.publish(command)
            elif key == '\x03':
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
