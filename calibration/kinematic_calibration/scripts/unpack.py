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

import rostools
rostools.update_path('kinematic_calibration')
import rospy
import sys
import Image as Image
import rosrecord

class dcamImage:
  def __init__(self, m):
    if hasattr(m, "byte_data"):
      ma = m.byte_data
      self.data = ma.data
    else:
      ma = m.uint8_data # MultiArray
      self.data = "".join([chr(x) for x in ma.data])
    d = ma.layout.dim
    assert d[0].label == "height"
    assert d[1].label == "width"
    self.size = (d[1].size, d[0].size)
    self.mode = "L"

  def tostring(self):
    return self.data

filename = sys.argv[1]
framecounter = 1

if 0:
  start,end = 20000,21300
  skipto = 12634364243
  start,end = 0,3300
else:
  skipto = None
  start,end = 0,30000

print "Starting Playback"
f = filename
keys = set()
for topic, msg, t in rosrecord.logplayer(f):
  print topic
  if skipto and (f.tell() < skipto):
    f.seek(skipto)

  if rospy.is_shutdown():
    break

  if topic == "/grabber/calibration_data" :
    if framecounter == end:
      break
    if start <= framecounter and (framecounter % 1) == 0:
      print framecounter

      left = dcamImage(msg.raw_stereo.left_image) ;
      right = dcamImage(msg.raw_stereo.right_image) ;

      Image.fromstring("L", (640,480), left.tostring()).save("unpacked/%06d_L.png" % framecounter)
      Image.fromstring("L", (640,480), right.tostring()).save("unpacked/%06d_R.png" % framecounter)

      file = open("unpacked/%06d_JointNames.txt" % framecounter,"w")
      file.writelines([x.name + '\n' for x in msg.mechanism_state.joint_states])
      file.close()

      file = open("unpacked/%06d_JointPos.txt" % framecounter, "w")
      file.writelines(['% 15.10f\n' % x.position for x in msg.mechanism_state.joint_states])
      file.close()

      file = open("unpacked/%06d_JointVel.txt" % framecounter, "w")
      file.writelines(['% 15.10f\n' % x.position for x in msg.mechanism_state.joint_states])
      file.close()

      file = open("unpacked/%06d_L_proj.txt" % framecounter, "w")
      file.writelines(['% 15.10f\n' % x for x in msg.raw_stereo.left_info.P])
      file.close()

      file = open("unpacked/%06d_R_proj.txt" % framecounter, "w")
      file.writelines(['% 15.10f\n' % x for x in msg.raw_stereo.right_info.P])
      file.close()

      framecounter += 1
