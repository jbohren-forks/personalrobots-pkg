#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
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

# Action watcher: Who watches the actions?
#
# Author: Stuart Glaser

import time, sys

import roslib; roslib.load_manifest('robot_actions')
import rospy
import rostopic
from robot_actions.msg import *
from std_msgs.msg import *

STATUS = ['Undefined', 'Success', 'Aborted', 'Preempted', 'Active']

class ActionWatcher:
    def __init__(self, name):
        self.name = name
        self.last_status = 0
        self.goal_received = rospy.get_rostime()
        self.last_preemption = roslib.rostime.Time(0.0)

        try:
            self.activate_class, _, __ = rostopic.get_topic_class(self.name + '/activate')
            self.feedback_class, _, __ = rostopic.get_topic_class(self.name + '/feedback')

            rospy.Subscriber(self.name + '/activate', self.activate_class, self.activate_cb)
            rospy.Subscriber(self.name + '/feedback', self.feedback_class, self.feedback_cb)
            rospy.Subscriber(self.name + '/preempt', Empty, self.preempt_cb)
        except roslib.packages.InvalidROSPkgException, ex:
            print "Cannot watch %s because you do not have the messages built" % name

    def activate_cb(self, msg):
        print "- %s: new goal" % self.name
        self.goal_received = rospy.get_rostime()

    def preempt_cb(self, msg):
        now = rospy.get_rostime()
        if (now - self.last_preemption).to_seconds() > 0.5:
            print "- %s preempted" % self.name
        self.last_preemption = now

    def feedback_cb(self, msg):
        if msg.status.value != self.last_status:
            duration_str = ''
            if msg.status.value == 1 and self.last_status != 0:
                duration = rospy.get_rostime() - self.goal_received
                duration_str = "(%.1f seconds)" % duration.to_seconds()
            print "- %s: status changed %s -> %s  %s" % (self.name,
                                                         STATUS[self.last_status],
                                                         STATUS[msg.status.value],
                                                         duration_str)
            self.last_status = msg.status.value

def get_actions(topics):
    ACTIVATE = 1
    PREEMPT = 2
    FEEDBACK = 4
    are_actions = {}
    for t in topics:
        name = t[0]
        action_name = name.rsplit('/', 1)[0]
        if name.endswith('activate'):
            are_actions[action_name] = are_actions.get(action_name, 0) | ACTIVATE
        elif name.endswith('preempt'):
            are_actions[action_name] = are_actions.get(action_name, 0) | PREEMPT
        elif name.endswith('feedback'):
            are_actions[action_name] = are_actions.get(action_name, 0) | FEEDBACK
    actions = []
    for k,v in are_actions.items():
        if v == (ACTIVATE | PREEMPT | FEEDBACK):
            actions.append(k)
    return actions

def main():
    rospy.init_node('action_watcher', anonymous=True)
    actions = []
    watchers = []

    while not rospy.is_shutdown():
        latest_actions = get_actions(rospy.get_published_topics())
        new_actions = []

        for a in latest_actions:
            if a not in actions:
                actions.append(a)
                new_actions.append(a)
                print "Found action: %s" % a
        for a in new_actions:
            watchers.append(ActionWatcher(a))
        
        time.sleep(0.5)
    rospy.spin()

if __name__ == '__main__': main()
