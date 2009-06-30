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
roslib.load_manifest('vision_tests')

import sys
import time
import getopt
import math

import rospy

import sensor_msgs.msg

class source:

  def __init__(self):
    self.pub = rospy.Publisher("/stereo/raw_stereo", sensor_msgs.msg.RawStereo)
    self.rate = 30.0
    self.duration = 10.0

  def spin(self):
    time.sleep(1.0)
    started = time.time()
    counter = 0
    while not rospy.core.is_shutdown():
      now = time.time()
      expected = int((now - started) * self.rate)
      #print "counter", counter, "expected", expected
      if counter < expected:
        img_msg = sensor_msgs.msg.RawStereo()
        img_msg.left_image.uint8_data.data = "L" * (640 * 480)
        img_msg.right_image.uint8_data.data = "R" * (640 * 480)
        self.pub.publish(img_msg)
        counter = counter + 1
      else:
        next_frame_due = started + ((counter + 1) / self.rate)
        duration = next_frame_due - now
        time.sleep(duration)

      took = now - started
      if took > self.duration:
        print "source: sent %d frames in %f, so %f fps" % (counter, took, counter / took)
        assert abs(((counter / took) - self.rate) != 0.1)
        return

def main(args):
  s = source()
  rospy.init_node('source')
  try:
    s.spin()
    rospy.spin()
    outcome = 'test completed'
  except KeyboardInterrupt:
    print "shutting down"
    outcome = 'keyboard interrupt'
  rospy.core.signal_shutdown(outcome)

if __name__ == '__main__':
  main(sys.argv)
