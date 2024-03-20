#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

# Initialize ROS node
rospy.init_node('joy_to_motor_control')

# Initialize motor speeds
speed1 = 0
speed2 = 0
button=0

# Define callback function for joystick input
def joy_callback(data):
    global speed1, speed2,button
    # Assuming axes 0 and 1 correspond to left-right and up-down movement of joystick
    button=data.axes[5]  

    speed1 = int(data.axes[0] )  # Map joystick Y-axis to motor speed
    speed2 = int(data.axes[1] )   # Map joystick X-axis to motor speed
    
  
    
# Subscribe to joystick topic
rospy.Subscriber("joy", Joy, joy_callback)

# Publisher for motor control commands
motor_cmd_pub = rospy.Publisher('motor_commands', Int32, queue_size=10)

# Main loop
def main_loop():
    global speed1, speed2,button
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Publish motor speeds
        if button:
            motor_cmd_pub.publish(Int32(speed1))
            motor_cmd_pub.publish(Int32(speed2))
        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
