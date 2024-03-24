#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from math import tan


wheelbase = 1.0  

def ackermann_steering(steering_angle, wheelbase):
    if steering_angle == 0:
        return 0, 0

    inner_radius = wheelbase / abs(tan(steering_angle))
    outer_radius = inner_radius + wheelbase

    inner_angle = tan(wheelbase / inner_radius)
    outer_angle = tan(wheelbase / outer_radius)

    if steering_angle > 0:
        return inner_angle, outer_angle
    else:
        return outer_angle, inner_angle

def joystick_callback(data):
    # Map joystick axes to linear and angular velocities
    speed = data.axes[1] * 1.0  # Forward/backward movement

    # Initialize velocities
    left_velocity = 0
    right_velocity = 0

    # Check the state of the buttons for each type of motion
    spot_turn_button = data.buttons[3]  
    ackermann_button = data.buttons[0]  
    crabbing_button = data.buttons[2] 

    if spot_turn_button == 1:
        # If the spot turn button is pressed, perform a spot turn
        rotation = data.axes[0] * 1.0  # Left/right rotation
        left_velocity = rotation
        right_velocity = -rotation
    elif ackermann_button == 1:
        # If the Ackermann button is pressed, perform Ackermann steering
        steering_angle = data.axes[1] * 1.0  # Steering angle

        front_left_angle, front_right_angle = ackermann_steering(steering_angle, wheelbase)
        rear_left_velocity, rear_right_velocity = speed, speed
    elif crabbing_button == 1:
       
        # Assuming that for crabbing motion, all wheels move at the same speed in the same direction
        left_velocity = speed
        right_velocity = speed

    # Create a Twist message
    twist = Twist()
    twist.linear.x = left_velocity
    twist.angular.z = right_velocity

    # Publish the velocities and angles
    pub.publish(twist)

def holonomic_control():
    # Initialize ROS node
    rospy.init_node('holonomic_control')

    # Create a publisher to publish movement commands
    global pub
    pub = rospy.Publisher('holonomic_command', Twist, queue_size=10)

    # Subscribe to joystick messages
    rospy.Subscriber("joy", Joy, joystick_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    holonomic_control()
    