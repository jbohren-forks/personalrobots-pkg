#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
import Queue
import threading
import random

import rospy

import sensor_msgs.msg

class sink:

  def __init__(self):
    self.queue = Queue.Queue(1)
    self.processing_delay = 0.33333
    self.expecting = 27
    self.messages = []
    t0 = threading.Thread(target=self.sink_raw_stereo, args=())
    t0.start()
    self.massive = " " * 25000000
    rospy.Subscriber('/stereo/raw_stereo', sensor_msgs.msg.RawStereo, self.handle_raw_stereo, queue_size=2, buff_size=8000000)

  def handle_raw_stereo(self, msg):

    print "rcv msg", msg.header.seq
    assert len(msg.left_image.uint8_data.data) == (640 * 480)
    assert len(msg.right_image.uint8_data.data) == (640 * 480)

    # If something older was in the queue, remove it
    try:
      _ = self.queue.get(False)
    except Queue.Empty:
      pass
    self.queue.put(msg)

  def sink_raw_stereo(self):
    while True:
      msg = self.queue.get(self.queue)

      if len(self.messages) == 0:
        # first message
        self.started = time.time()
      self.messages.append(msg.header.seq)
      if len(self.messages) == self.expecting:
        took = time.time() - self.started
        print "%d messages received in %f: %s" % (len(self.messages), took, str(self.messages))
        delays = [ b-a for a,b in zip(self.messages, self.messages[1:])]
        print "delays %d-%d" % (min(delays), max(delays))
        print "%3d: %4d" % (0, self.messages[0])
        for i in range(1, len(self.messages)):
          print "%3d: %4d %d" % (i, self.messages[i], self.messages[i] - self.messages[i - 1])
        break

      mode = 'massive'

      if mode == 'sleep':
        time.sleep(self.processing_delay)
      elif mode == 'cpu':
        started = time.time()
        while (time.time() < (started + self.processing_delay)):
          x = math.sqrt(random.random())
      elif mode == 'massive':
        started = time.time()
        while (time.time() < (started + self.processing_delay)):
          x = self.massive[random.randrange(len(self.massive))]

    rospy.core.signal_shutdown('completed')

def main(args):
  s = sink()
  rospy.init_node('sink')
  rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
