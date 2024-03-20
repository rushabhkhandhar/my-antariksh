#!/usr/bin/env python3

import rospy
from std_msgs.msg import Char
import sys, select, termios, tty


# Define the mapping of keyboard keys to Arduino commands
key_mapping = {
    'w': ord('a'),  # Move base forward
    's': ord('b'),  # Move base backward
    'a': ord('c'),  #  Move rev1 clockwise
    'd': ord('d'),  # Move rev1 anti-clockwise
    'j': ord('e'),  # Move rev2 clockwise
    'l': ord('f'),  # Move rev2 anti-clockwise
    'i': ord('g'),  # Move gripper close 
    'k': ord('h'),  # Move gripper open
    'u': ord('i'),  # Move bevel1 and bevel2 forward
    'o': ord('j'),  # Move bevel1 and bevel2 backward
    'p': ord('k'),  # Move bevel1 forward and bevel2 backward
    'n': ord('l'),  # Move bevel1 backward and bevel2 forward
    'q': ord('m'),  # Move fourth_joint clockwise
    'e': ord('n'),  # Move fourth_joint anti-clockwise
    ' ': ord('x')   # Stop all movements
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def send_command_to_arduino(command):
    pub.publish(command)  # Publish the command
    print("Sent command:", chr(command))  # Print the command being sent

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('command_sender', anonymous=True)  # Initialize the ROS node
    pub = rospy.Publisher('command_for_arm', Char, queue_size=10)  # Create a publisher

    current_command = 'x'  # Initialize with stop command

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key in key_mapping:
                command = key_mapping[key]
                if command != current_command:  # Check if the command has changed
                    send_command_to_arduino(command)
                    current_command = command
            elif key == '\x03':  # Ctrl+C
                break
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
