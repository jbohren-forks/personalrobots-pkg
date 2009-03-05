#!/usr/bin/env python
import roslib
roslib.load_manifest('kinematic_calibration')

import time
import rospy
import sys
from std_msgs.msg import Empty
from pr2_mechanism_controllers.srv import *
from pr2_mechanism_controllers.msg import *
from robot_msgs.msg import *

cmd_count = 0

if __name__ == '__main__':
  print "Running python code"
  rospy.init_node('flag_tester', sys.argv, anonymous=False)

  capture_publisher = rospy.Publisher("grabber/capture", Empty)

  headers_string = rospy.get_param('~joint_headers')
  print "Got headers from param:\n%s" % headers_string
  commands_string = rospy.get_param('~joint_commands')
  print "Got commands from param:\n%s" % commands_string

  actuator_name = rospy.get_param('~actuator')
  print "Got actuator name from param:\n%s" % actuator_name

  headers = headers_string.split()

  cmd_all = [[float(x) for x in cur_line.split()] for cur_line in commands_string.split("\n")]
  cmd = [x for x in cmd_all if len(x)==len(headers)]
  print "Commands"
  print cmd

  arm_controller = 'right_arm_trajectory_controller'
  arm_query_topic = arm_controller + '/TrajectoryQuery'
  arm_start_topic = arm_controller + '/TrajectoryStart'

  print "Waiting Query Service: " + arm_query_topic
  rospy.wait_for_service(arm_query_topic)
  print "Found Query Service!"

  print "Waiting Start Service: " + arm_start_topic
  rospy.wait_for_service(arm_start_topic)
  print "Found Start Service!"

  query_srv = rospy.ServiceProxy(arm_query_topic, TrajectoryQuery)
  query_resp = query_srv.call(TrajectoryQueryRequest(0))

  start_srv = rospy.ServiceProxy(arm_start_topic, TrajectoryStart)

  print "Headers"
  print headers

  print "Queried Jointnames:"
  print query_resp.jointnames

  print "***** Remapping Arm Commands *****"
  arm_mapping = [headers.index(x) for x in query_resp.jointnames]
  print "Mapping:"
  print arm_mapping
  arm_cmd = [[y[x] for x in arm_mapping] for y in cmd]
  print "First remapped arm cmd:"
  print arm_cmd[0]
  print "Successfully Remapped Arm Commands"

  print "*** Arm Command 0 ***"
  joint_traj = JointTraj([JointTrajPoint(arm_cmd[0],0)])
  start_resp = start_srv.call(TrajectoryStartRequest(joint_traj,0,1))

  print "  Response:"
  print "    Traj ID: %u" % start_resp.trajectoryid
  
  prev_id = start_resp.trajectoryid

  while (not rospy.is_shutdown()) :
    print "*** Arm Command 1 ***"
    joint_traj = JointTraj([JointTrajPoint(arm_cmd[1],0)])
    start_resp = start_srv.call(TrajectoryStartRequest(joint_traj,0,1))

    print "  Response:"
    print "    Traj ID: %u" % start_resp.trajectoryid

    done = False
    while not done :
      query_resp = query_srv.call(TrajectoryQueryRequest(prev_id))
      done = (query_resp.done == 1)
      time.sleep(.1)
      print "Waiting... resp=%u"%query_resp.done
    capture_publisher.publish(Empty())

    print "*** Arm Command 0 ***"
    joint_traj = JointTraj([JointTrajPoint(arm_cmd[0],0)])
    start_resp = start_srv.call(TrajectoryStartRequest(joint_traj,0,1))
    
    print "  Response:"
    print "    Traj ID: %u" % start_resp.trajectoryid
  
    prev_id = start_resp.trajectoryid

  #rospy.spin()

