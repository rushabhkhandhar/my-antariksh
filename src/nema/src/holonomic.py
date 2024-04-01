#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
import math

m1_pub = rospy.Publisher('/m1_speed', Float32, queue_size=10)
m2_pub = rospy.Publisher('/m2_speed', Float32, queue_size=10)
fl_pub = rospy.Publisher('/fl', Float32, queue_size=10)
fr_pub = rospy.Publisher('/fr', Float32, queue_size=10)
rl_pub = rospy.Publisher('/rl', Float32, queue_size=10)
rr_pub = rospy.Publisher('/rr', Float32, queue_size=10)
linear_vel = 0
angular_vel = 0

def main():
    rospy.init_node('rover_control', anonymous=True)
    rospy.Subscriber('/joy', Joy, joystick_callback)
    rospy.spin()

def joystick_callback(data):
    global linear_vel

    x = data.axes[0]
    y = data.axes[1]

    if x == 0 and y == 0:
        angle = 0
    else:
        angle = math.degrees(math.atan2(x, y))

    if data.buttons[5]:
        linear_vel = data.axes[1] * 150
        control_linear_only(linear_vel)
    else:
        linear_vel = 0
        control_all_axes(angle, linear_vel, data)

def control_linear_only(vel):
    m1_pub.publish(vel)
    m2_pub.publish(vel)
    fl_pub.publish(0)
    fr_pub.publish(0)
    rl_pub.publish(0)
    rr_pub.publish(0)

def control_all_axes(angle, vel, data):
    track_width = 1.0  # distance between the front wheels
    ackermann = data.buttons[0]  # will have to check the index number of mapping
    spot_turn = data.buttons[1]  # will have to check the index number of mapping
    crabbing = data.buttons[2]  # will have to check the index number of mapping

    if spot_turn:
        fl_pub.publish(angle)
        fr_pub.publish(-angle)
        rl_pub.publish(-angle)
        rr_pub.publish(angle)

        left_m = vel
        right_m = vel

        m1_pub.publish(left_m)
        m2_pub.publish(right_m)

    elif crabbing:
        fl_pub.publish(angle)
        fr_pub.publish(angle)
        rl_pub.publish(angle)
        rr_pub.publish(angle)

        left_m = -vel
        right_m = vel

        m1_pub.publish(left_m)
        m2_pub.publish(right_m)

    elif ackermann:
        inner_angle = angle - (track_width / 2)
        outer_angle = angle + (track_width / 2)

        if angle > 0:
            fl_pub.publish(outer_angle)
            fr_pub.publish(inner_angle)
            rl_pub.publish(0)
            rr_pub.publish(0)
        else:
            fl_pub.publish(inner_angle)
            fr_pub.publish(outer_angle)
            rl_pub.publish(0)
            rr_pub.publish(0)

        left_m = -vel
        right_m = vel

        m1_pub.publish(left_m)
        m2_pub.publish(right_m)

    else:
        fl_pub.publish(angle)
        fr_pub.publish(angle)
        rl_pub.publish(angle)
        rr_pub.publish(angle)

        if not (spot_turn or crabbing or ackermann):
            left_m = -vel
            right_m = vel

            m1_pub.publish(left_m)
            m2_pub.publish(right_m)

if __name__ == '__main__':
    main()
