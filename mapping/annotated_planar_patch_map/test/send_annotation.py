#! /usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

import roslib
roslib.load_manifest('annotated_planar_patch_map')
import rospy
import random
from image_msgs.msg import RawStereo
from std_msgs.msg import Empty

from image_msgs.msg import *;

from cv_mech_turk.msg import *

class SendAnnotation:
  def __init__(self):

    self.sub_ = rospy.Subscriber("/stereo/stereo_info",StereoInfo, self.onStereo)
    self.message_topic_ = "/annotations_2d"

    self.pub_ = rospy.Publisher(self.message_topic_, ExternalAnnotation)

  
  def onStereo(self,msg_in):
      msg=ExternalAnnotation();
      msg.reference_time=msg_in.header.stamp;
      msg.reference_frame="stereo_l_stereo_camera_frame";
      poly=AnnotationPolygon();
      poly.object_name="wall";
      poly.control_points.append(AnnotationPt2D(0,0))
      poly.control_points.append(AnnotationPt2D(640,0))
      poly.control_points.append(AnnotationPt2D(640,480))
      poly.control_points.append(AnnotationPt2D(0,480))
      msg.polygons.append(poly)
      print msg
      self.pub_.publish(msg);

if __name__ == '__main__':
  try:

    rospy.init_node("annotation_sender")
    s=SendAnnotation();
    rospy.spin();

  except KeyboardInterrupt, e:
    pass
  print "exiting"
