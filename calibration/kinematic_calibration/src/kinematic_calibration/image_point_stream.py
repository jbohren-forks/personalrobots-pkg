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
roslib.load_manifest('kinematic_calibration')

import time
import rospy
import sys

from kinematic_calibration.msg_cache import MsgCache
from kinematic_calibration.msg import *
from image_msgs.msg import *
from roslib import rostime

class ImagePointStats() :
    def __init__(self) :
        pass

class ImagePointStream() :
    def __init__(self, config) :
        self._cache = MsgCache( config['cache_size'] )
        self._topic = config['topic']
        self._skew  = config['skew']
        self._tol   = config['tol']
        self._sensor= config['sensor']
        self._target= config['target']
        self._min_samples = config['min_samples'] ;


    def start(self) :
        self._cache.start(self._topic, ImagePointStamped)

    # Compute N, avg, ranges
    def _process_stats(self, segment) :
        stats = ImagePointStats()
        stats.N = len(segment)
        if (stats.N > 0) :
            seg_x = [elem.image_point.x for elem in segment]
            seg_y = [elem.image_point.y for elem in segment]
            stats.ranges = [max(data) - min(data) for data in [seg_x, seg_y] ]
            stats.avg    = [sum(data)/float(stats.N)    for data in [seg_x, seg_y] ]
        else :
            stats.ranges = [ ]
            stats.avg = [ ]
        return stats

    # Compute for interval, including skew
    def get_stats_interval(self, start, end) :
        skew_start = rostime.Time.from_seconds(start.to_seconds() + self._skew)
        skew_end   = rostime.Time.from_seconds(end.to_seconds()   - self._skew)
        seg = self._cache.get_segment_interval(skew_start, skew_end)
        return self._process_stats(seg)

    def is_stable(self, stats) :
        if (stats.N < self._min_samples) :
            return False
        if (stats.ranges[0] > self._tol or stats.ranges[1] > self._tol) :
            return False
        return True

    def build_sample(self, stats) :
        msg = SensorSample()
        msg.sensor  = self._sensor
        msg.target  = self._target
        msg.m = stats.avg
        return msg
        
