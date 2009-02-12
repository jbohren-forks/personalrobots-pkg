#!/usr/bin/python

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
#  * Neither the name of the Willow Garage nor the names of its
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

import roslib
roslib.update_path('kinematic_calibration')
import rospy
import sys
import rosrecord

filename = sys.argv[1]
framecounter = 1

print "Starting Playback"
f = filename
keys = set()



headers = [ ]

headers_file = open("joint_names.txt","w")
pos_file = open("joint_pos.txt","w")
for topic, msg, t in rosrecord.logplayer(f):
  print topic

  if rospy.is_shutdown():
    break

  if topic == "/grabber/calibration_data" :
    print framecounter

    joint_names = [x.name for x in msg.mechanism_state.joint_states]
    joint_pos   = [x.position for x in msg.mechanism_state.joint_states]

    if (len(headers) == 0) :
      headers = joint_names
      headers_file.writelines([x + "\n" for x in headers])
      headers_file.writelines(['\n'])

    # Reshuffle the joint_pos to match the headers
    pos_remapped = [joint_pos[joint_names.index(x)] for x in headers]

    pos_file.writelines(["% 15.10f "%x for x in pos_remapped])
    pos_file.writelines(["\n"])
    framecounter+=1

pos_file.close()
headers_file.close()


