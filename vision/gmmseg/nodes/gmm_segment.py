#!/usr/bin/python
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
#
## @author Hai Nguyen/hai@gatech.edu
## @package gmm_segment
#Provides Gaussian mixture segmentation service.
#Subscribes to topic 'image' of type std_msgs.msg.Image.

import rostools
rostools.updatePath('gmmseg')
import rospy
import std_msgs.msg.Image as RImage 
import gmmseg.texseg as tg
import gmmseg.srv as srv
from threading import RLock
import pyrob.util as ut
import sys, time, math
import opencv.highgui as hg
import time

CAMERA_FOV    = 60.0 
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480

class SegmentNode:
    def __init__(self):
        self.image = None
        self.lock  = RLock()
        t30     = math.tan(math.radians(CAMERA_FOV/2.0))
        self.fx = CAMERA_WIDTH  / t30
        self.fy = CAMERA_HEIGHT / t30

    def segment(self, request):
        print 'Hey Advait, I want to segment the image'
        while self.image == None:
            time.sleep(0.1)

        self.lock.acquire()
        sego = tg.segment_center_object(self.image, display_on=False)
        self.lock.release()

        z     = request.height
        u     = sego.fg_object_ellipse.center[0] - (CAMERA_WIDTH / 2.0)
        v     = sego.fg_object_ellipse.center[1] - (CAMERA_HEIGHT / 2.0)
        x     = u * z / self.fx
        y     = v * z / self.fy
        theta = sego.fg_object_ellipse.angle

        if theta > 90:
          theta = theta - 180
        if theta < -90:
          theta = theta + 180

        theta = math.radians(theta)

#        if True:
#          image_list = sego.get_images_for_display()
#          image_list.append( cvimg )
#          
#          curtime = time.localtime()
#          date_name = time.strftime('%Y%m%d%I%M', curtime)
#        
#          for i,img in enumerate(image_list):
#            fname = date_name+'_image%d.png'%(i)
#            hg.cvSaveImage( fname, img )


        return srv.hrl_graspResponse(x, y, request.height, theta)

    def set_image(self, image):
        if self.lock.acquire(blocking=False):
            self.image = ut.ros2cv(image)
            self.lock.release()
        hg.cvShowImage('image', self.image)
        hg.cvWaitKey(5)

def run_me_server():
    sn       = SegmentNode()
    hg.cvNamedWindow('image', 1)

    rospy.TopicSub('image', RImage, sn.set_image)
    rospy.ready(sys.argv[0])
    segment_service = rospy.Service('hrl_grasp', srv.hrl_grasp, sn.segment)
    segment_service.register()
    rospy.spin()

if __name__ == '__main__':
    run_me_server()
