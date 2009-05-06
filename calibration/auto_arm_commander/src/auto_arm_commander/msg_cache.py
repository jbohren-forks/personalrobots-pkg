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

class MsgCache() :
    def __init__(self, max_len) :
        self._max_len = max_len
        self._q = [ ]
        self._lock = threading.Lock()

    def start(self, topic, type) :
        self._sub = rospy.Subscriber(topic, type, self.callback)

    def callback(self, msg) :
        self._lock.acquire()
        self._q.append(msg)
        while len(self._q) > self._max_len :
            self._q.pop(0)
        self._lock.release()

    # Grabs a section of the cache that falls within the specified time bounds
    def get_segment_interval(self, start, end) :
        self._lock.acquire()
        mech_seg = [x for x in self._q if (x.header.stamp >= start and x.header.stamp <= end)]
        self._lock.release()
        return mech_seg

    def get_segment_after(self, start) :
        self._lock.acquire()
        mech_seg = [x for x in self._q if (x.header.stamp >= start)]
        self._lock.release()
        return mech_seg

    def get_segment_latest(self, N) :
        self._lock.acquire()
        mech_seg = self._q[-N:]
        self._lock.release()
        return mech_seg

