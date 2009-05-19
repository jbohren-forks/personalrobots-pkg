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
roslib.load_manifest('dense_laser_assembler')

import time
import rospy
import sys
import threading
from laser_scan.msg import *
from robot_msgs.msg import *
from roslib import rostime

# Stores a single laser scan plus some meta information
class LaserCacheElem() :
    pass


class DenseLaserCache() :
    def __init__(self, req_callback, max_mech_len, max_laser_done_len, max_laser_wait_len) :

        # Define callback for interval cloud req
        self._req_callback = req_callback

        # Specify all queue lengths
        self._max_mech_len = max_mech_len
        self._max_laser_done_len = max_laser_done_len
        self._max_laser_wait_len = max_laser_wait_len

        # Init mutexes
        self._mech_lock = threading.Lock()
        self._laser_done_lock = threading.Lock()
        self._laser_wait_lock = threading.Lock()
        self._interval_req_lock = threading.Lock()

        # Define queues
        self._mech_q = [ ]        # Mechanism State Data
        self._laser_done_q = []   # Laser scans that have already been processed
        self._laser_wait_q = []   # Laser scans that we've received but haven't/can't processed yet
        self._interval_req_q = [] # Stores list of intervals that we need to process

    def add_mech_state(self, msg) :
        self._mech_lock.acquire()
        self._mech_q.append(msg)
        while len(self._mech_q) > self._max_mech_len :
            self._mech_q.pop(0)
        self._mech_lock.release()

    def add_scan(self, msg) :
        self._laser_wait_lock.acquire()
        self._laser_wait_q.append(msg)
        while len(self._laser_wait_q) > self._max_laser_wait_len :
            self._laser_wait_q.pop(0)
        self._laser_wait_lock.release()

    # Iterate through the list of scans we're waiting to process, and process each one
    # if we're ready. A scan is ready to be processed if there is at least one
    # MechanismState message available before and after the scan. The processed scan is
    # then added to the laser_done queue
    def process_scans(self) :
        self._mech_lock.acquire()
        local_mech_q = self._mech_q
        self._mech_lock.release()

        if len(local_mech_q) > 0 :
            oldest_mech = local_mech_q[0]
            newest_mech = local_mech_q[-1]
        else :
            return
        
        self._laser_wait_lock.acquire()
        laser_pending_q = [x for x in self._laser_wait_q
                             if x.header.stamp < newest_mech.header.stamp and
                                x.header.stamp > oldest_mech.header.stamp]
        # Purge entire queue, except scans in the future
        self._laser_wait_q = [x for x in self._laser_wait_q
                                if x.header.stamp >= newest_mech.header.stamp]
        self._laser_wait_lock.release()

        # Now process the pending queue
        processed = [self._process_pending(x,local_mech_q) for x in laser_pending_q]

        #if len(processed) > 0 :
        #    print 'processed %i scans' % len(processed)

        self._laser_done_lock.acquire()
        self._laser_done_q = self._laser_done_q + processed
        while len(self._laser_done_q) > self._max_laser_done_len :
            self._laser_done_q.pop(0)
        self._laser_done_lock.release()

    # Process a scan, assuming that we have MechansimState data available before and after the scan
    # \param scan_msg The message that we want to process
    # \param the time-order queue of mechanism_state data
    # \param The processed scan element
    def _process_pending(self, scan_msg, mech_q):
        mech_before = [x for x in mech_q if x.header.stamp < scan_msg.header.stamp][-1]
        mech_after  = [x for x in mech_q if x.header.stamp > scan_msg.header.stamp][0]

        ind = [x.name for x in mech_before.joint_states].index('laser_tilt_mount_joint')
        pos_before = mech_before.joint_states[ind].position
        elapsed_before = (scan_msg.header.stamp - mech_before.header.stamp).to_seconds()

        ind = [x.name for x in mech_after.joint_states].index('laser_tilt_mount_joint')
        pos_after = mech_after.joint_states[ind].position
        elapsed_after = (mech_after.header.stamp - scan_msg.header.stamp).to_seconds()

        elapsed_total = elapsed_before + elapsed_after
        # Linear interp
        pos_during = elapsed_after/elapsed_total*pos_before + elapsed_before/elapsed_total*pos_after
        elem = LaserCacheElem()
        elem.pos  = pos_during
        elem.scan = scan_msg
        return elem

    # Returns a list of processed scan elems between the start and end time
    def get_dense_cloud(self, start, end) :
        self._laser_done_lock.acquire()
        trimmed_done = [x for x in self._laser_done_q
                          if x.scan.header.stamp <= end and
                             x.scan.header.stamp >= start]
        self._laser_done_lock.release()
        return trimmed_done

    # Returns the time associated with the latest scan
    def get_latest_done_scan_time(self) :
        self._laser_done_lock.acquire()
        if len(self._laser_done_q) > 0 :
            latest_time = self._laser_done_q[-1].scan.header.stamp
        else :
            latest_time = rostime.Time().from_seconds(0.0)
        self._laser_done_lock.release()
        return latest_time 

    # Adds a request for extracting data during an interval. This request will be attempted to
    # processed when process_interval_reqs is called.
    def add_interval_req(self, start, end) :
        self._interval_req_lock.acquire()
        self._interval_req_q.append([start, end])
        self._interval_req_lock.release()
        self.process_interval_reqs()

    # Attempts to service all the interval requests added by calling add_interval_req. A req can
    # be serviced if there exists a processed scan elem with a timestamp greater than the end time
    # of the interval request
    def process_interval_reqs(self) :

        latest_time = self.get_latest_done_scan_time()

        # Split list into request we're ready to process, on one's we're not ready for
        self._interval_req_lock.acquire()
        pending_reqs = [x for x in self._interval_req_q if x[1] <  latest_time]
        idle_reqs =    [x for x in self._interval_req_q if x[1] >= latest_time]
        self._interval_req_q = idle_reqs
        self._interval_req_lock.release()

        # Compute clouds for the requests we're ready to process
        clouds = [self.get_dense_cloud(x[0], x[1]) for x in pending_reqs]

        for x in clouds :
            self._req_callback(x)
