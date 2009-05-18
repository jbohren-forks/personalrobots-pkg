#!/usr/bin/env python
PKG = 'audio'
import roslib; roslib.load_manifest(PKG)

import rospy
from robot_msgs.msg import PoseDot

import os

# if robot base receives a command to move backwards in the x-axis,
# then play sound file (TruckBackUp.wav) from Logitech USB speaker (hw:1,0).
# data is an instance of a PoseDot extracted from the cmd_vel topic.
def callback(data):
    if data.vel.vx < 0:  
        os.system("aplay -D hw:1,0 TruckBackUp.wav")

# subscribe to velocity command (cmd_vel)
# to see if the forward velocity (vel.vw) is negative
def backingup():
    rospy.init_node('backingup', anonymous=True)
    rospy.Subscriber("cmd_vel", PoseDot, callback)

    rospy.spin()

if __name__ == '__main__':
    backingup()

