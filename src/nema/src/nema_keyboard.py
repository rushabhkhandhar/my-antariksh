#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

# Initialize ROS node
rospy.init_node('stepper_control')

# ROS publisher for sending movement commands
pub = rospy.Publisher('stepper_command', Float32, queue_size=10)

def main():
    while not rospy.is_shutdown():
        try:
            angle = float(input("Enter angle (-36 to 36): "))
            if angle >= -36 and angle <= 36:
                pub.publish(angle)
            elif angle == 100:
                # jash na rakha hai to bring it to origin
                pub.publish(angle)
            else:
                print("Angle must be between -36 and 36")
        except ValueError:
            print("Invalid input. Please enter a number.")

if __name__ == '__main__':
    main()
