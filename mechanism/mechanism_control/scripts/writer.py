#!/usr/bin/env python
PKG = 'mechanism_control'

import rostools
rostools.update_path(PKG)

import sys, traceback, logging, rospy
from robot_msgs.msg import MechanismState

print "File name is: %s" %sys.argv[1]
filehandle = open(sys.argv[1],'w')

NAME = 'joint_writer'

def callback(data):
    for a in data.actuator_states:
      filehandle.write("%.4f " %a.timestamp)
    for j in data.joint_states:
      filehandle.write("%.4f " %j.position)
      filehandle.write("%.4f " %j.velocity)
      filehandle.write("%.4f " %j.commanded_effort)
      filehandle.write("\n")
   

def listener_with_user_data():
    rospy.TopicSub("/mechanism_state", MechanismState, callback)
    rospy.ready(NAME, anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_with_user_data()
    except KeyboardInterrupt, e:
        pass
    print "exiting"

