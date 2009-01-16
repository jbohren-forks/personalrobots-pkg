#!/usr/bin/env python
import rostools
rostools.update_path('kinematic_calibration')

import rospy
import sys
from roscpp.msg import Empty
from pr2_mechanism_controllers.srv import *
from pr2_mechanism_controllers.msg import *
from robot_msgs.msg import JointCmd

cmd_count = 0

def goto_next_callback(data, callback_args):
	arm_cmd = callback_args[0]
	start_topic = callback_args[1]
	head_cmd = callback_args[2]
	head_pub = callback_args[3]
	
	global cmd_count
	print "Callback Called"
	print "  Waiting for Service:\n  (%s)..." % start_topic
	rospy.wait_for_service(start_topic)
	print "  Service Found!"
	print "  Sending Cmd #%u" % cmd_count
	if cmd_count < len(arm_cmd) :
		print "  *** Arm Command ***"
		print arm_cmd[cmd_count]
		start_srv = rospy.ServiceProxy(start_topic, TrajectoryStart)
		joint_traj = JointTraj([JointTrajPoint(arm_cmd[cmd_count],0)])
		start_resp = start_srv.call(TrajectoryStartRequest(joint_traj,0,1))
		print "Response:"
		print "  Traj ID: %u" % start_resp.trajectoryid
		print "  Timestamps:"
		print start_resp.timestamps

		print "  ***  Head Command ***"		
		print head_cmd[cmd_count]
		head_pub.publish(JointCmd(['head_pan_joint', 'head_tilt_joint'],[0.0,0.0],head_cmd[cmd_count],[0.0, 0.0],[0.0, 0.0]))

	else :
		print "No More Commands left in queue!"


	print "************"
	cmd_count = cmd_count + 1
        

if __name__ == '__main__':
	print "Running python code"
	rospy.init_node('arm_commander', sys.argv, anonymous=False)

	headers_string = rospy.get_param('~joint_headers')
        print "Got headers from param:\n%s" % headers_string
	commands_string = rospy.get_param('~joint_commands')
        print "Got commands from param:\n%s" % commands_string

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

        query_srv = rospy.ServiceProxy(arm_query_topic, TrajectoryQuery)
        query_resp = query_srv.call(TrajectoryQueryRequest(0))
	head_publisher = rospy.Publisher('head_controller/set_command_array', JointCmd)

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

	print "***** Remapping Head Commands *****"
	head_mapping = [headers.index(x) for x in ['HeadPan', 'HeadTilt']]
	head_cmd = [[y[x] for x in head_mapping] for y in cmd]
	print "First remapped head cmd:"
	print head_cmd[0]
	print "Successfully Remapped Head Commands"

	sub = rospy.Subscriber("~goto_next", Empty, goto_next_callback,
			       [arm_cmd, arm_start_topic, head_cmd, head_publisher])

	print "***** Ready to Send Trajectories *****"

	rospy.spin()

