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

from sensor_msgs.msg import RawStereo
import sensor_msgs.msg
import rospy
import Image

import time

import submit_img

class Snapper:


  def __init__(self):

    try:
      srv_name=rospy.get_param("~server")
    except:
      srv_name="vm6.willowgarage.com:8080"

    #target_session="prf-2009-05-21-22-29-00"

    try:
      self.target_session=rospy.get_param("~session")
    except:
      self.target_session="demo-session-L1s"

    try:
      self.block_submit=rospy.get_param("~block_submit")
    except:
      self.block_submit="0"

    try:
      self.save_imgs_dir=rospy.get_param("~img_dir")
    except:
      self.save_imgs_dir="."

    print self.save_imgs_dir

    images_dir=self.save_imgs_dir+"images/"+self.target_session+"/";
    img_mode="gray";
 
    try:
      self.frame_id=rospy.get_param("~frame");
    except:
      self.frame_id="stereo_l_stereo_camera_frame"

    rospy.logwarn("INIT DONE")
    print "INIT DONE"
    self.init_internal(srv_name, self.target_session,images_dir,img_mode)

  def init_internal(self,srv_name, target_session,images_dir,img_mode):
    self.mech = submit_img.MechSubmiter(srv_name, target_session)
    self.img_dir=images_dir;
    if not os.path.exists(images_dir):
      os.makedirs(images_dir);
    self.img_mode=img_mode;


    self.sub=rospy.Subscriber('image', sensor_msgs.msg.Image, self.handle_image)
  
  def handle_image(self, msg):
    rospy.loginfo("Snapper image")
    print "Snapper image"


    image = msg.uint8_data.data
    image_sz = (640,480);
    if self.img_mode=="gray":
      i = Image.fromstring("L", image_sz, image)
    else:
      i = Image.fromstring("RGB", image_sz, image)

    ref_time = "%d.%09d" % (msg.header.stamp.secs, msg.header.stamp.nsecs)
    fn = "foo-%d.%09d.jpg" % (msg.header.stamp.secs, msg.header.stamp.nsecs)
    full_fn=os.path.join(self.img_dir,fn);
    i.save(full_fn)

    if not self.block_submit==1:
      ext_id = self.mech.submit(full_fn,{'image_size':"640,480",
                                         'topic_in':rospy.resolve_name('image'),
                                         'topic_out':rospy.resolve_name('annotation'),
                                         'ref_time':ref_time,
                                         'frame_id':self.frame_id,
                                         })
      rospy.loginfo("Submitted %s %s"% ( full_fn, ext_id))
      print "Submitted %s %s"% ( full_fn, ext_id)
    else:
      rospy.loginfo("Submission to session %s blocked by parameter ~block_submit=1", self.target_session)
      print "Submission blocked to session %s by parameter ~block_submit=1" % self.target_session
      ext_id="n/a"



def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  print "Starting a snapper"
  rospy.init_node('snapper')

  s = Snapper()
  rospy.spin()
      
               

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
