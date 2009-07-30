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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

PKG = 'bagserver'
import roslib; roslib.load_manifest(PKG)

import rospy

import sys
from threading import Thread
import threading

import unittest

from sensor_msgs.msg import *
from roslib.msg import *
import tf
from bagserver.srv import *

class TestBagserver(unittest.TestCase):

    def __init__(self,test_name):
        unittest.TestCase.__init__(self,test_name);


    def onImage(self,msg):
        
        print msg.header.stamp, "image"
        self.num_img += 1;


    def testTimeline(self):

        rospy.wait_for_service('timeline_info')
        tl_info = rospy.ServiceProxy('timeline_info', TimelineInfo)
        self.info=tl_info();

        self.assertEqual(len(self.info.topics),2)
        self.assertEqual(self.info.topics[0],"/stereo/left/image")
        self.assertEqual(self.info.topics[1],"/stereo/right/image_rect")



    def testFetchImage(self):
        self.img_topic="/stereo/left/image";
        rospy.wait_for_service('fetch_image')
        self.img_service = rospy.ServiceProxy('fetch_image', FetchImage)


        req=FetchImageRequest();
        req.topic=self.img_topic
        req.begin.secs=1244500682; 
        req.begin.nsecs=760380000;
        resp=self.img_service(req);

        self.assertEqual(resp.result.header.stamp.nsecs,req.begin.nsecs);
        self.assertEqual(resp.result.header.stamp.secs,req.begin.secs);

        req.begin.secs=1244500682; 
        req.begin.nsecs=0;
        resp2=self.img_service(req);




        self.assertEqual(resp.result.header.stamp,resp2.result.header.stamp);


        req.begin.secs=1244500683; 
        req.begin.nsecs=0;
        resp3=self.img_service(req);

        self.assertNotEqual(resp.result.header.stamp,resp3.result.header.stamp);


    def testHist(self):

        self.num_img=0;
        self.img_topic="/stereo/left/image";
        rospy.wait_for_service('hist')

        self.img_sub_topic="/hist/stereo/left/image";

        self.img_sub = rospy.Subscriber(self.img_sub_topic, sensor_msgs.msg.Image,self.onImage,None,100,30000000)

        self.hist_service = rospy.ServiceProxy('hist', History)
        rospy.sleep(1);


        self.timed_test_ok=False
        req=HistoryRequest();
        req.begin.secs=1244500678;
        req.begin.nsecs=566643000;
        req.end.secs=1244500690;
        req.end.nsecs=0;
        req.topic=self.img_topic;

        def call_svc():
            resp=self.hist_service(req)

        svc_thread=threading.Thread(None,call_svc)
        svc_thread.start()


        rospy.sleep(5);

            
        self.assertEqual(self.num_img,12)
        return


if __name__ == "__main__":
    import rostest
    rospy.init_node("test_bagserver");


    rostest.rosrun('bagserver','testContent',TestBagserver)

