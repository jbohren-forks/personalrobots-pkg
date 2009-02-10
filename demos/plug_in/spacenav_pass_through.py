#! /usr/bin/env python

import roslib
roslib.load_manifest('plug_in')

import rospy
from std_msgs.msg import *
from robot_msgs.msg import *
from joy.msg import Joy

rospy.init_node('wrencher', anonymous=True)
pub = rospy.Publisher('/arm_constraint/command', Wrench)
gripper = rospy.Publisher('/gripper_effort/set_command', Float64)

closing = False
def callback(joy):
    global closing
    def f(x):
        return 70 * x**2
    w = Wrench()
    w.force.x = f(joy.axes[0])
    w.force.y = f(joy.axes[1])
    w.force.z = f(joy.axes[2])
    w.torque.x = f(joy.axes[3])
    w.torque.y = f(joy.axes[4])
    w.torque.z = f(joy.axes[5])
    pub.publish(w)

    if joy.buttons[1] > 0:
        closing = False
        gripper.publish(Float64(5))
    elif joy.buttons[0] > 0:
        closing = True
        gripper.publish(Float64(-5))
    elif closing:
        gripper.publish(Float64(-5))
    else:
        gripper.publish(Float64(0))

rospy.Subscriber('/spacenav/joy', Joy, callback)
rospy.spin()
