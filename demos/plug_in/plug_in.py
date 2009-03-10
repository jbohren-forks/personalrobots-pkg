#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Stuart Glaser

import sys, time

import roslib
roslib.load_manifest('plug_in')
import rospy
from robot_msgs.msg import *
from robot_srvs.srv import *

CONTROLLER = 'arm_constraint'

class Tracker:
    def __init__(self, topic, Msg):
        self.sub = rospy.Subscriber(topic, Msg, self.callback)
        self.msg = None

    def callback(self, msg):
        self.msg = msg

mechanism_state = Tracker('/mechanism_state', MechanismState)
def last_time():
    global mechanism_state
    if mechanism_state.msg:
        return mechanism_state.msg.header.stamp
    return 0



def main():
    rospy.init_node('plug_in')

    track_outlet_pose = Tracker('/outlet_detector/pose', PoseStamped)
    track_plug_pose = Tracker('/plug_detector/pose', PoseStamped)

    # Waits for an estimate of the outlet pose
    print "Waiting for outlet pose..."
    outlet_pose = None
    msg = None
    while not msg:
        msg = track_outlet_pose.msg
        if rospy.is_shutdown(): return
        time.sleep(0.1)
    outlet_pose = msg
    print "Found outlet"

    # Waits for an estimate of the plug pose
    print "Waiting for plug pose..."
    plug_pose = None
    msg = None
    while not msg:
        msg = track_plug_pose.msg
        if rospy.is_shutdown(): return
        time.sleep(0.1)
    plug_pose = msg
    print "Found plug"

    print "Please start the constraint controller..."

    # Sets the tool frame
    rospy.wait_for_service("/%s/set_tool_frame" % CONTROLLER)
    if rospy.is_shutdown(): return
    set_tool_frame = rospy.ServiceProxy("/%s/set_tool_frame" % CONTROLLER, SetPoseStamped)
    set_tool_frame(plug_pose)
    print "Tool frame set"

    # Publishes commands
    time.sleep(1.0)
    print "Starting to publish"
    pub_command = rospy.Publisher("/%s/outlet_pose" % CONTROLLER, PoseStamped)
    msg = PoseStamped()
    cnt = 0
    while not rospy.is_shutdown():
        cnt += 1
        outlet_pose.header.stamp = last_time()
        pub_command.publish(outlet_pose)
        if cnt % 3 == 0:
            try:
                set_tool_frame(track_plug_pose.msg)
            except rospy.service.ServiceException, ex:
                print "set_tool frame service went down.  Reconnecting..."
                rospy.wait_for_service("/%s/set_tool_frame" % CONTROLLER)
                if rospy.is_shutdown(): return
                set_tool_frame = rospy.ServiceProxy("/%s/set_tool_frame" % CONTROLLER, SetPoseStamped)
                print "got it.  Continuing"
        time.sleep(1.0)

if __name__ == '__main__': main()
