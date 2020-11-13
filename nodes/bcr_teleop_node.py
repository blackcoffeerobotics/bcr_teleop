#! /usr/bin/env python

"""
    Author: Gaurav Gupta
    Email: gaurav@blackcoffeerobotics.com
    Ref: https://stackoverflow.com/questions/510357/how-to-read-a-single-character-from-the-user
"""


import rospy
from geometry_msgs.msg import Twist
import time


def publish_event(event):
    vel_pub.publish(cmd_vel_msg)


class _Getch:
    """Gets a single character from standard input. Does not echo to the
    screen."""

    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty
        import sys

    def __call__(self):
        import sys
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


if __name__ == '__main__':

    rospy.init_node('key_teleop_node')
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Timer(rospy.Duration(0.05), publish_event)
    cmd_vel_msg = Twist()
    getch = _Getch()
    rate = rospy.Rate(5)
    rospy.loginfo("w/a/s/d keyboard. Press q to terminate")
    try:
        while (not rospy.is_shutdown()):
            key_in = getch()
            if key_in == "w":
                cmd_vel_msg.linear.x += 0.1
            elif key_in == "s":
                cmd_vel_msg.linear.x -= 0.1
            elif key_in == "d":
                cmd_vel_msg.angular.z -= 0.1
            elif key_in == "a":
                cmd_vel_msg.angular.z += 0.1
            else:
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = 0
                if (key_in == "q"):
                    rate.sleep()
                    break
            print("linear: " + str(cmd_vel_msg.linear.x),
                  "\tangular" + str(cmd_vel_msg.angular.z))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
