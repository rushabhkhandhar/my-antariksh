import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

# Initialize ROS node
rospy.init_node('stepper_control')

# ROS publisher for sending movement commands
pub = rospy.Publisher('stepper_command', Float32, queue_size=10)

def joystick_callback(data):
    # Extract the joystick axes values
    x_axis = data.axes[0]
    y_axis = data.axes[1]

    # Convert the joystick axes values to angle
    angle = y_axis * 36

    # Publish the angle
    pub.publish(angle)

    # Console the angle
    print("Angle:", angle)

def main():
    # Subscribe to the joystick topic
    rospy.Subscriber('joy', Joy, joystick_callback)

    # Spin the ROS node
    rospy.spin()

if __name__ == '__main__':
    main()
 