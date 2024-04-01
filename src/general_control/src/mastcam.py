#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32


# Initialize ROS node
rospy.init_node('joystick_pan_tilt_control')

pan_angle = 0
tilt_angle = 0

pan_pub = rospy.Publisher('pan_angle', Int32, queue_size=10)
tilt_pub = rospy.Publisher('tilt_angle', Int32, queue_size=10)

axes_movement_allowed = False

def joystick_callback(data):
    global pan_angle, tilt_angle, axes_movement_allowed
    if data.buttons[4] == 1: 
        axes_movement_allowed = True
    else:
        axes_movement_allowed = False

    if axes_movement_allowed:
        pan_angle += int(data.axes[0] * 6)  
        tilt_angle += int(data.axes[1] * 6) 
      
        pan_pub.publish(pan_angle)
        tilt_pub.publish(tilt_angle)

rospy.Subscriber('joy', Joy, joystick_callback)

def main_loop():
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass

