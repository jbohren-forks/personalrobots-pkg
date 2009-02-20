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
roslib.load_manifest('vslam')

import sys
import time
import getopt

from math import *

from image_msgs.msg import RawStereo
import vslam.msg
import rospy

import Image
import time
import math

import pylab
# interactive mode on
pylab.ion()
timefig = pylab.figure(1)
timesub = pylab.subplot(111)

def handle_roadmap(msg):
  print "recv message"
  print msg.nodes
  print msg.edges
  print
  pylab.scatter([n.x for n in msg.nodes], [n.y for n in msg.nodes])
  pylab.quiver([ n.x for n in msg.nodes ], [n.y for n in msg.nodes], [ math.cos(n.theta) for n in msg.nodes ], [math.sin(n.theta) for n in msg.nodes])
  for i,n in enumerate(msg.nodes):
    pylab.annotate('%d' % i, (n.x, n.y))
  for e in msg.edges:
    i0,i1 = e.node0, e.node1
    pylab.plot([ msg.nodes[i0].x, msg.nodes[i1].x ], [ msg.nodes[i0].y, msg.nodes[i1].y ])
  pylab.draw()

def main(args):
  rospy.init_node('snapper')
  rospy.TopicSub('/roadmap', vslam.msg.Roadmap, handle_roadmap)
  rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
