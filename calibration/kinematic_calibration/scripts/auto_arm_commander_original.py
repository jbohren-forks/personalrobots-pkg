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
from kinematic_calibration.msg import *

traj_actuator_names = ['r_shoulder_pan_motor',
		       'r_shoulder_lift_motor',
		       'r_upper_arm_roll_motor',
		       'r_elbow_flex_motor',
		       'r_forearm_roll_motor',
		       'r_wrist_l_motor',
		       'r_wrist_r_motor']


arm_mapping = [ ]
headers = [ ]
joint_state_hist = [ ]

ready_to_capture = False
done_capturing = False

N = 100

print_count = 0

def mech_state_callback(data, interval_publisher):
	
	#print "Callback Called"
	global arm_mapping
	global headers
	global print_count
	global joint_states_hist
	global ready_to_capture
	global done_capturing

	traj_joint_names = [headers[x] for x in arm_mapping]


	ms_actuator_names = [x.name for x in data.actuator_states]
	ms_joint_names = [x.name for x in data.joint_states]


	ms_actuator_mapping = [ms_actuator_names.index(x) for x in traj_actuator_names]
	ms_joint_mapping = [ms_joint_names.index(x) for x in traj_joint_names]

	joint_state_hist.append([data.header.stamp, [data.joint_states[x].position for x in ms_joint_mapping ]] ) 
	while (len(joint_state_hist) > N) :
		joint_state_hist.pop(0)

	print_count = print_count+1 ;
	if (ready_to_capture) :
		if (len(joint_state_hist) == N) :
			state_maxes = [ max( [x[1][k] for x in joint_state_hist] ) for k in range(0,len(traj_actuator_names)) ]
			state_mins  = [ min( [x[1][k] for x in joint_state_hist] ) for k in range(0,len(traj_actuator_names)) ]
			state_range = [state_maxes[k]-state_mins[k]             for k in range(0,len(traj_actuator_names)) ]

			#print state_maxes
			#print state_mins

			if ( max(state_range) > .00000001 ) :
				if (print_count % 50 == 0) :
					print "Moving"
					print state_range
			else :
				if (print_count % 50 == 0) :
					print "Stationary"
				interval_publisher.publish(Interval(roslib.msg.Header(None, None, None),
							            joint_state_hist[0][0], joint_state_hist[-1][0]))
				ready_to_capture = False
				done_capturing = True
		

if __name__ == '__main__':
	print "Running python code"
	rospy.init_node('auto_arm_commander', sys.argv, anonymous=False)

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

	interval_publisher = rospy.Publisher('stationary_interval', Interval)
#	interval_publisher = [ ]
	sub = rospy.Subscriber("mechanism_state", MechanismState, mech_state_callback, interval_publisher)

	#rospy.spin()

	start_srv = rospy.ServiceProxy(arm_start_topic, TrajectoryStart)
	for k in range(1, len(arm_cmd)) :
		print "Command Traj #%u"%k
		joint_traj = JointTraj([JointTrajPoint(arm_cmd[k],0)])
		start_resp = start_srv.call(TrajectoryStartRequest(joint_traj,0,1))
		print "  Response:"
		print "    Traj ID: %u" % start_resp.trajectoryid

		done = False
		while not done :
			query_resp = query_srv.call(TrajectoryQueryRequest(start_resp.trajectoryid))
			done = (query_resp.done == 1 or query_resp.done == 4)
			time.sleep(.1)
			print "Waiting... resp=%u"%query_resp.done

		done_capturing = False
		ready_to_capture = True
		print "Waiting to capture"
		while not done_capturing and not rospy.is_shutdown() :
			time.sleep(.1)



	#print "***** Ready to Send Trajectories *****"

	rospy.spin()

