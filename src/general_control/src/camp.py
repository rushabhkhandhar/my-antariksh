#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import curses

# Initialize ROS node
rospy.init_node('keyboard_pan_tilt_control')

# Initialize curses for keyboard input
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(True)

# Initialize pan and tilt angles
pan_angle = 0
tilt_angle = 90

# Publisher for pan-tilt angles
pan_tilt_pub = rospy.Publisher('pan_tilt_angles', Int32MultiArray, queue_size=10)

# Main loop
def main_loop():
    global pan_angle, tilt_angle
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Publish pan-tilt angles
        angles_msg = Int32MultiArray(data=[pan_angle, tilt_angle])
        pan_tilt_pub.publish(angles_msg)
        rate.sleep()

def main(stdscr):
    global pan_angle, tilt_angle
    stdscr.clear()
    stdscr.addstr("Use arrow keys to control pan and tilt. Press 'q' to quit.\n")
    stdscr.refresh()
    while True:
        key = stdscr.getch()
        if key == curses.KEY_UP:
            tilt_angle += 1
        elif key == curses.KEY_DOWN:
            tilt_angle -= 1
        elif key == curses.KEY_RIGHT:
            pan_angle += 1
        elif key == curses.KEY_LEFT:
            pan_angle -= 1
        elif key == ord('q'):
            break
        stdscr.clear()
        stdscr.addstr("Pan angle: {}\n".format(pan_angle))
        stdscr.addstr("Tilt angle: {}\n".format(tilt_angle))
        stdscr.refresh()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
        main_loop()
    except rospy.ROSInterruptException:
        pass