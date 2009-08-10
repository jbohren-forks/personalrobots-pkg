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

# Author: Dallas Goecker
import time
import random
import roslib
roslib.load_manifest('texas')
import rospy
from std_msgs.msg import Float64
from mechanism_msgs.msg import JointStates
from texas.msg import TexasCmd

STRAIGHT = 0.82
ROTATION_JOINT = 'bl_caster_rotation_joint'
SPEED = 6.0
PI = 3.14159

class LastMessage():
    def __init__(self, topic, msg_type):
        self.msg = None
        rospy.Subscriber(topic, msg_type, self.callback)

    def last(self):
        return self.msg

    def callback(self, msg):
        self._last_time = rospy.get_time()
        self.msg = msg

    def get_timeout(self):
        return rospy.get_time() - self._last_time

def main():
    angle = STRAIGHT
    speed = -SPEED
    last_time = 0
    rospy.init_node('tdrive', anonymous=True)
    last_state = LastMessage('/joint_states', JointStates)
    tx_cmd = LastMessage('texas_cmd', TexasCmd)

    pub_steer = rospy.Publisher("/caster/steer_velocity", Float64)
    pub_drive = rospy.Publisher("/caster/drive_velocity", Float64)
    pub_steer.publish(Float64(0.0))
    pub_drive.publish(Float64(0.0))
    print "Waiting for a mechanism_state message..."
    while not last_state.msg and not rospy.is_shutdown(): pass
    while not tx_cmd.msg and not rospy.is_shutdown(): pass
    while not rospy.is_shutdown():
        time.sleep(0.01)
        
        # Get angle, velocity from command
        drive_cmd = tx_cmd.last()
        drive_vel = drive_cmd.velocity
        drive_angle = drive_cmd.angle
        # Safety timeout
        if tx_cmd.get_timeout() > 3:
            drive_vel = 0.0

        joint_state = last_state.last()
        rotation_state = None
        for state in joint_state.joints:
            if state.name == ROTATION_JOINT:
                rotation_state = state
                break
        if not rotation_state:
            print "The %s joint was not found in the mechanism state" % ROTATION_JOINT

        # Steers the caster to be straight
        pub_steer.publish(Float64(6.0 * (drive_angle - rotation_state.position)))
        pub_drive.publish(Float64(drive_vel))

if __name__ == '__main__':
    main()
