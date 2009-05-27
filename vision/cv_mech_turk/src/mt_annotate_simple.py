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

"""                                                                             
usage: %(progname)s --session=SESSION --server=SERVER [--imgdir=images/]
        
"""
import roslib; roslib.load_manifest('cv_mech_turk')


import os
import sys
import time
import getopt

from math import *

from image_msgs.msg import RawStereo
import image_msgs.msg
import rospy
import Image

import time

import submit_img

class MtAnnotateSimple:


  def __init__(self):

    srv_name="vm6.willowgarage.com:8080"
    target_session="willow-env3-test"
    images_dir="images/";
    img_mode="gray";

    self.init_internal(srv_name, target_session,images_dir,img_mode)

  def init_internal(self,srv_name, target_session,images_dir,img_mode):
    self.mech = submit_img.MechSubmiter(srv_name, target_session)
    self.img_dir=images_dir;
    if not os.path.exists(images_dir):
      os.makedirs(images_dir);
    self.img_mode=img_mode;
  
  def handle_raw_stereo(self, msg):
    print "Snapper image"
    image = msg.uint8_data.data
    if self.img_mode=="gray":
      i = Image.fromstring("L", (640,480), image)
    else:
      i = Image.fromstring("RGB", (640,480), image)
    fn = "foo-%d.%09d.jpg" % (msg.header.stamp.secs, msg.header.stamp.nsecs)
    full_fn=os.path.join(self.img_dir,fn);
    i.save(full_fn)
    #ext_id = self.mech.submit(full_fn)
    #print full_fn, ext_id
    print full_fn
    time.sleep(1.0)




def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]

  rospy.init_node('mt_annotate_simple')

  s = Snapper()
  rospy.Subscriber('image_to_annotate', image_msgs.msg.Image, s.handle_raw_stereo, queue_size=2, buff_size=7000000)
  rospy.spin()
  
               

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
