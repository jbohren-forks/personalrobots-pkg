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

PKG='bag_image_view'
import roslib;roslib.load_manifest(PKG);

import Image

import rospy

import wx
import wx.aui
from wx import xrc

from bagserver.srv import *

import sensor_msgs.msg



class ImageViewPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent,-1,wx.DefaultPosition,wx.Size(700,550))

 # Associate some events with methods of this class
        rospy.wait_for_service('timeline_info')
        tl_info = rospy.ServiceProxy('timeline_info', TimelineInfo)
        self.info=tl_info();

        self.img_topic="/stereo/left/image";

        rospy.wait_for_service('fetch_image')
        self.img_service = rospy.ServiceProxy('fetch_image', FetchImage)

        self.img_pub = rospy.Publisher('image', sensor_msgs.msg.Image)


        # Add a panel and some controls to display the size and position
        panel = wx.Panel(self, -1)
        label1 = wx.StaticText(panel, -1, "Image:")
        label2 = wx.StaticText(panel, -1, "Image time:")
        label3 = wx.StaticText(panel, -1, "Time control:")
        label4 = wx.StaticText(panel, -1, "Image topic:")
        self.timeTextCtrl = wx.TextCtrl(panel, -1, "",wx.DefaultPosition,wx.Size(200,30))
        self.imageCtrl = wx.StaticBitmap(panel, -1,wx.NullBitmap, wx.DefaultPosition,wx.Size(640,480));
        self.posCtrl = wx.Slider(panel, -1, self.info.begin.secs, self.info.begin.secs, self.info.end.secs, wx.DefaultPosition, wx.Size(400,40),wx.SL_LABELS +  wx.SL_HORIZONTAL)
        self.topicCtrl = wx.Choice(panel, -1, wx.DefaultPosition, wx.Size(300,40), self.info.topics)
        self.img_topic=self.info.topics[0];

        self.panel = panel

        # Use some sizers for layout of the widgets
        sizer = wx.FlexGridSizer(3, 2, 5, 5)
        sizer.Add(label1)
        sizer.Add(self.imageCtrl)
        sizer.Add(label2)
        sizer.Add(self.timeTextCtrl)
        sizer.Add(label3)
        sizer.Add(self.posCtrl)
        sizer.Add(label4)
        sizer.Add(self.topicCtrl)

        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 15)
        panel.SetSizerAndFit(border)
        self.Fit()

        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)
        self.Bind(wx.EVT_CHOICE, self.topicSelection)

    def topicSelection(self, event):
        self.img_topic=self.info.topics[self.topicCtrl.GetSelection()];
        print self.img_topic
 
    def sliderUpdate(self, event):

        req=FetchImageRequest();
        req.locator="";
        req.begin.secs=self.posCtrl.GetValue();
        req.begin.nsecs=0;
        req.topic=self.img_topic;

        try:
            resp = self.img_service(req);
        except:
            print "reconnecting to service";
            self.img_service = rospy.ServiceProxy('fetch_image', FetchImage)            
            return
        

        self.img_pub.publish(resp.result);

        msg=resp.result;

        if msg.encoding=="mono":
            image_mode="gray"
        elif  msg.encoding=="rgb":
            image_mode="RGB";
        else:
            rospy.logerror("Image encoding is not supported %s",msg.encoding);
            return
        
        
        ma = msg.uint8_data # MultiArray                                                                                                                                                 
        dim = dict([ (d.label,d.size) for d in ma.layout.dim ])
        (w,h) = (dim['width'], dim['height'])

        image = msg.uint8_data.data
        image_sz = (w,h);
        
        
        if image_mode=="gray":
            i = Image.fromstring("L", image_sz, image)
        elif image_mode=="RGB":
            i = Image.fromstring("RGB", image_sz, image)
        else:
            rospy.logerror("Unknown image mode");
            return
        


        wxImg = wx.EmptyImage(i.size[0],i.size[1])
        wxImg.SetData(i.convert("RGB").tostring())
        wxImg.SetAlphaData(i.convert("RGBA").tostring()[3::4])
        
        self.imageCtrl.SetBitmap(wx.BitmapFromImage(wxImg))

        self.timeTextCtrl.Clear()
        self.timeTextCtrl.AppendText("%d.%d"%(msg.header.stamp.secs,msg.header.stamp.nsecs));
