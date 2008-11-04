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

import rostools; rostools.update_path('teleop_spacenav')
import rospy, sys, math
from robot_srvs.srv import GetVector
from std_msgs.msg import Vector3

def print_usage(code = 0):
    print sys.argv[0], '<cartesian velocity controller topic>'
    sys.exit(code)


def sign(x):
    if x < 0:
        return -1
    return 1


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage(1)

    topic = sys.argv[1]
    rospy.wait_for_service(topic + '/get_actual')
    get_position = rospy.ServiceProxy(topic + '/get_actual', GetVector)
    publisher = rospy.Publisher(topic + '/command', Vector3)

    pos = get_position().v

    i = 0
    def spacenav_updated(msg):
        global i
        def t(x):
            #return x * 0.005
            return sign(x) * abs(x * 0.005) ** 1.5
        msg.x = t(msg.x)
        msg.y = t(msg.y)
        msg.z = t(msg.z)
        publisher.publish(msg)

    rospy.Subscriber("/spacenav/offset", Vector3, spacenav_updated)
    rospy.init_node('spacenav_teleop')
    rospy.spin()

'''
TODO:
reset to current position occasionally
'''
