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

import sys, time
import roslib; roslib.load_manifest('plug_in')
import rospy
from robot_msgs.msg import PoseStamped, TransformStamped
import tf.msg
from std_srvs.srv import Empty

class Tracker:
    def __init__(self, topic, Msg):
        self.sub = rospy.Subscriber(topic, Msg, self.callback)
        self.msg = None

    def callback(self, msg):
        self.msg = msg

def pose_to_transform_stamped(p):
    t = TransformStamped()
    t.header.stamp = p.header.stamp
    #t.header.frame_id = p.header.frame_id
    t.parent_id = p.header.frame_id
    t.transform.translation.x = p.pose.position.x
    t.transform.translation.y = p.pose.position.y
    t.transform.translation.z = p.pose.position.z
    t.transform.rotation = p.pose.orientation
    return t

def main():

    # TODO: outlet pose remains stable in the base frame.  Transform
    # to there, and then continuously publish

    track_outlet_pose = Tracker('/plugs_core_actions/outlet_pose', PoseStamped)

    def reset():
        track_outlet_pose.msg = None
    rospy.Service('/outlet_pose_filter/reset', Empty, reset)

    tf_pub = rospy.Publisher('/tf_message', tf.msg.tfMessage)
    tf_msg = tf.msg.tfMessage()
    tf_msg.transforms = [TransformStamped()]

    rospy.init_node('outlet_filter')

    seq = 0
    outlet_msg = None
    while not rospy.is_shutdown():
        if outlet_msg: # and outlet_msg.pose.orientation.z > 0: # HACK
            tf_msg.transforms = [pose_to_transform_stamped(outlet_msg)]
            tf_msg.transforms[0].header.seq = seq
            tf_msg.transforms[0].header.frame_id = 'outlet_pose'
            tf_msg.transforms[0].header.stamp = rospy.get_rostime()
            tf_pub.publish(tf_msg)
            seq += 1
            outlet_msg = track_outlet_pose.msg
        else:
            outlet_msg = track_outlet_pose.msg

        time.sleep(0.05)

if __name__ == '__main__': main()
