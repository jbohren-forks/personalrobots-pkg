#!/usr/bin/env python
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

import roslib; roslib.load_manifest('robot_actions_tools')
import roslib.names
import roslib.scriptutil
import rospy

NAME='robot_actions_tools'

import sys

import robot_actions



#####################################################################################
# All this stuff is strait from rostopic. Maybe it should be in a library.
#####################################################################################
def msgevalgen(pattern):
    if not pattern or pattern == '/':
        return None
    def msgeval(msg):
        # I will probably replace this with some less beautiful but more efficient
        return eval('msg'+'.'.join(pattern.split('/')))
    return msgeval
def _get_topic_type(topic):
    val = succeed(roslib.scriptutil.get_master().getPublishedTopics('/', '/'))
    matches = [(t, t_type) for t, t_type in val if t == topic or topic.startswith(t+'/')]
    if matches:
        #TODO logic for multiple matches if we are prefix matching
        t, t_type = matches[0]
        if t_type == roslib.names.ANYTYPE:
            return None, None, None
        if t_type == topic:
            return t_type, None
        return t_type, t, msgevalgen(topic[len(t):])
    else:
        return None, None, None
    
def get_topic_type(topic):
    topic_type, real_topic, msg_eval = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, msg_eval
    else:
        print >> sys.stderr, "WARNING: topic [%s] does not appear to be published yet"%topic
        while not rospy.is_shutdown():
            topic_type, real_topic, msg_eval = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, msg_eval
            else:
                time.sleep(0.1)

def get_topic_class(topic):
    topic_type, real_topic, msg_eval = get_topic_type(topic)
    return roslib.scriptutil.get_message_class(topic_type), real_topic, msg_eval

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise RosTopicException("remote call failed: %s"%msg)
    return val
#####################################################################################







controllers = {}

def set_topic_key(topic, msg):
    global controllers
    if (msg.status.value == msg.status.UNDEFINED):
        controllers[topic] = "Undefined"
    elif (msg.status.value == msg.status.SUCCESS):
        controllers[topic] = "Success"
    elif (msg.status.value == msg.status.ABORTED):
        controllers[topic] = "Aborted"
    elif (msg.status.value == msg.status.PREEMPTED):
        controllers[topic] = "Preempted"
    elif (msg.status.value == msg.status.ACTIVE):
        controllers[topic] = "Active"
    else:
        controllers[topic] = "Bad status (" + str(msg.status.value) + ")"
    

def create_callback(topic):
    return lambda msg: set_topic_key(topic, msg)

def display_action_states(seconds):
    global controllers

    master = roslib.scriptutil.get_master()
    state = succeed(master.getSystemState('/rostopic'))

    pub_topics = succeed(roslib.scriptutil.get_master().getPublishedTopics('/rostopic', '/'))    
    controllers = {}
    topic_types = {}
    for topic in pub_topics:
        msg_class, real_topic, msg_eval = get_topic_class(topic[0])
        test_message = msg_class()
        is_controller = False
        try:
            is_controller = isinstance(test_message.status, robot_actions.msg.ActionStatus)
        except:
            is_controller = False
        if (is_controller):
            controllers[real_topic] = "Unknown (controller did not publish)"
            topic_types[real_topic] = msg_class
    
    print "list_action_state.py: robot_actions debug tool starting. Run with --help for info."

    rospy.init_node(NAME, anonymous=True)
    subs = []
    for topic in controllers:
        subs.append(rospy.Subscriber(topic, topic_types[topic], create_callback(topic)))


    print "Waiting", seconds, "seconds for messages."

    rospy.sleep(seconds)

    print "Done. The following actions are running:"

    for topic in controllers:
        print "\t-"+ topic.replace("/feedback", "/").strip("/") + ": " + str(controllers[topic])




if __name__ == '__main__':
    seconds = 5.0
    run = True
    if (len(sys.argv) > 1):
        if (sys.argv[1] == "--help"):
            name = sys.argv[0].strip("./")
            print "Usage of", name, ":"
            print "   1.", name, "--help"
            print "   2.", name, ""
            print "   3.", name, "--seconds <n>"
            print 
            print name, "is a tool used to debug robot_actions. Robot_actions are used in ROS as a general interface"
            print "between various parts of the system and the executive. They have five states: active, inactive, undefined,"
            print "aborted, and preempted.", name, "is used to tell what state the actions are in at the current time."
            print
            print "Case 1: print help"
            print "Case 2: collect data for five seconds and display the result."
            print "Case 3: specifiy the number of seconds <n> to collect data for and display the result."
            run = False
        if (sys.argv[1] == "--seconds" and len(sys.argv) > 2):
            try:
                seconds = float(sys.argv[2])
                if (seconds <= 0.0):
                    raise 1
            except:
                print "You must enter a postive real number for seconds. You entered:", sys.argv[2]
                run = False
    if (run):
        display_action_states(seconds)
