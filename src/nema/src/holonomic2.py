import rospy 
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from math import tan

# Define the wheelbase of the robot
wheelbase = 1.0  # Replace with your actual wheelbase

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

    # Check the state of the buttons for each type of motion
    spot_turn_button = data.buttons[0]  # Replace with your actual button index
    ackermann_button = data.buttons[1]  # Replace with your actual button index
    crabbing_button = data.buttons[2]  # Replace with your actual button index

    if spot_turn_button == 1:
        # If the spot turn button is pressed, perform a spot turn
        rotation = data.axes[0] * 1.0  # Left/right rotation

        # Adjust the rotation velocity based on the throttle input
        rotation_velocity = rotation * speed  # Adjust this scaling factor as needed

        # Apply the rotation velocity to both left and right wheels
        left_velocity = rotation_velocity
        right_velocity = -rotation_velocity

        # Publish the velocities and angles
        publish_velocity_angles(left_velocity, right_velocity, 0, 0)
    elif ackermann_button == 1:
        # If the Ackermann button is pressed, perform Ackermann steering
        steering_angle = data.axes[2] * 1.0  # Steering angle

        # Adjust the steering angle based on the throttle input
        steering_angle *= speed  # Adjust this scaling factor as needed

        # Calculate the front left and right angles for Ackermann steering
        front_left_angle, front_right_angle = ackermann_steering(steering_angle, wheelbase)

        # For a car-like rover, the steering angle is applied to the front wheels
        # and the speed is applied to the rear wheels
        rear_left_velocity, rear_right_velocity = speed, speed

        # Publish the velocities and angles
        publish_velocity_angles(front_left_angle, front_right_angle, rear_left_velocity, rear_right_velocity)
    elif crabbing_button == 1:
        # If the crabbing button is pressed, perform crabbing motion
        # Assuming that for crabbing motion, all wheels move at the same speed in the same direction
        velocity = speed  # Adjust this scaling factor as needed

        # Publish the velocities and angles for crabbing motion
        publish_velocity_angles(velocity, velocity, velocity, velocity)
    else:
        # If neither button is pressed, stop the robot
        # Publish zero velocities
        publish_velocity_angles(0, 0, 0, 0)

def publish_velocity_angles(front_left_angle, front_right_angle, rear_left_velocity, rear_right_velocity):
    # Publish the velocities and angles
    cmd_vel = Twist()
    cmd_vel.linear.x = rear_left_velocity  # Adjust according to your robot's motion model
    cmd_vel.linear.y = rear_right_velocity  # Adjust according to your robot's motion model
    cmd_vel.angular.x = front_left_angle  # Adjust according to your robot's motion model
    cmd_vel.angular.y = front_right_angle  # Adjust according to your robot's motion model
    pub.publish(cmd_vel)

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
