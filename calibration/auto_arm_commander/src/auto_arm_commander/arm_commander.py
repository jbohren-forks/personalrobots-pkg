# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib
roslib.load_manifest('auto_arm_commander')

import time
import rospy
import sys
#from std_msgs.msg import Empty
from pr2_mechanism_controllers.srv import *
#from pr2_mechanism_controllers.msg import 
from robot_msgs.msg import *


class ArmCommander() :
    def __init__(self, arm_controller_name):
        print "Initializing Object"

        # Grab and initialize command set from parameters
        joint_headers_string = rospy.get_param('~joint_headers')
        joint_data_string = rospy.get_param('~joint_commands')

        self.init_joint_headers(joint_headers_string)
        self.init_joint_data(joint_data_string)

        
        rospy.wait_for_service(arm_controller_name + '/TrajectoryQuery')

        # Build service proxies for arm controller
        self.query_srv = rospy.ServiceProxy(arm_controller_name + '/TrajectoryQuery', TrajectoryQuery)
        self.start_srv = rospy.ServiceProxy(arm_controller_name + '/TrajectoryStart', TrajectoryStart)

        # Get list of controlled joints
        query_resp = self.query_srv.call(TrajectoryQueryRequest(0))
        self._traj_joint_names = query_resp.jointnames
        self.build_cmd_list(query_resp.jointnames)

    def init_joint_headers(self, joint_headers) :
        self.joint_headers = joint_headers.split()
    
    def init_joint_data(self, joint_data) :
	joint_data_all = [[float(x) for x in cur_line.split()] for cur_line in joint_data.split("\n")]
        self.joint_data = [x for x in joint_data_all if len(x)==len(self.joint_headers)]

    def build_cmd_list(self, controlled_joints) :
        arm_mapping = [self.joint_headers.index(x) for x in controlled_joints]
        self.cmds   = [[y[x] for x in arm_mapping] for y in self.joint_data]

    def num_cmds(self) :
        return len(self.cmds)

    def cmd_arm(self, cmd_index) :
        joint_traj = JointTraj([JointTrajPoint(self.cmds[cmd_index],0)])
        start_resp = self.start_srv.call(TrajectoryStartRequest(joint_traj,0,1))
        #print "StartingNewTraj: [CmdID=%u]  [TrajID=%u] " % (cmd_index, start_resp.trajectoryid)

        waiting = True
        while waiting :
            query_resp = self.query_srv.call(TrajectoryQueryRequest(start_resp.trajectoryid))
            # Keep waiting if the traj is queued up or active
            waiting = (query_resp.done == 0 or query_resp.done == 2 )
            time.sleep(.1)
            #print query_resp.done

        return query_resp.done

    def get_traj_joint_names(self) :
        return self._traj_joint_names
