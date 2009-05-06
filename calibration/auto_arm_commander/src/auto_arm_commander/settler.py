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
import threading

from pr2_mechanism_controllers.srv import *
from robot_msgs.msg import *
from roslib import rostime
from auto_arm_commander.msg_cache import MsgCache


class MechStateStats() :
    def __init__(self) :
        pass

class Settler() :
    def __init__(self, max_len) :
        self._cache = MsgCache(max_len)

    def start(self, topic) :
        self._cache.start("mechanism_state", MechanismState)

    # Extract a vector of joint positions from a MechanismState message
    def _grab_joint_pos(self, msg, req_joint_names) :
	msg_joint_names   = [x.name for x in msg.joint_states]
	mapping           = [msg_joint_names.index(x) for x in req_joint_names]
        return [msg.joint_states[x].position for x in mapping]

    def get_stats_interval(self, joints, start, end) :
        seg = self._cache.get_segment_interval(start, end)
        return self._process_stats(joints, seg)
        
    def get_stats_latest(self, joints, N) :
        seg = self._cache.get_segment_latest(N)
        return self._process_stats(joints, seg)

    def get_timed_elem(self, elem_time) :
        return cache_.get_segment_interval(elem_time, elem_time)

    def _process_stats(self, joints, seg) :
        stats = MechStateStats()
        if len(seg) > 0 :
            pos_raw = [self._grab_joint_pos(x, joints) for x in seg]
            pos = [  [x[k] for x in pos_raw] for k in range(0, len(joints)) ]
            ranges = [ max(x) - min(x) for x in pos ]
        else :
            ranges = [ [ ] for x in joints ]
        stamps = [x.header.stamp for x in seg]
        stats.N = len(seg)
        stats.ranges = ranges
        stats.start = min(stamps)
        stats.end = max(stamps)
        stats.seg = seg
        return stats
