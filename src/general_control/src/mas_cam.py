#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

# Initialize ROS node
rospy.init_node('joystick_pan_tilt_control')

# Initialize pan and tilt angles
pan_angle = 0
tilt_angle = 90

# Publishers for pan and tilt angles
pan_pub = rospy.Publisher('pan_angle', Int32, queue_size=10)
tilt_pub = rospy.Publisher('tilt_angle', Int32, queue_size=10)

# Callback function for joystick input
def joystick_callback(data):
    global pan_angle, tilt_angle
    pan_angle += int(data.axes[0] * 6)  
    tilt_angle += int(data.axes[1] * 6) 
    # Publish pan and tilt angles
    pan_pub.publish(pan_angle)
    tilt_pub.publish(tilt_angle)

# Subscribe to joystick input
rospy.Subscriber('joy', Joy, joystick_callback)

# Main loop
def main_loop():
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
