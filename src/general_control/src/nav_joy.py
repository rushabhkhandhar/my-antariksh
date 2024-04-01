#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

# Initialize ROS node
rospy.init_node('joy_to_motor_control')

# Initialize motor speeds and button state
speed1 = 0
speed2 = 0
button_pressed = False

# Define callback function for joystick input
def joy_callback(data):
    global speed1, speed2, button_pressed
    button_pressed = data.buttons[5]
    
    # Map joystick axes to motor speeds
    x_axis = data.axes[0]  
    y_axis = data.axes[1] 
    
    # Calculate motor speeds based on joystick axes
    speed1 = int(150 * (y_axis + x_axis))  
    speed2 = int(150 * (y_axis - x_axis))  

# Subscribe to joystick topic
rospy.Subscriber("joy", Joy, joy_callback)

# Publisher for motor control commands
motor_cmd_pub = rospy.Publisher('motor_commands', Int32, queue_size=10)

# Main loop
def main_loop():
    global speed1, speed2, button_pressed
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        
        if button_pressed:
            motor_cmd_pub.publish(Int32(speed1))
            motor_cmd_pub.publish(Int32(speed2))
        else:
            motor_cmd_pub.publish(Int32(0)) 
        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass

