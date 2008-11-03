#!/usr/bin/env python
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

# Author: Melonee Wise

PKG = "joint_qualification_controllers"

import rostools; rostools.update_path(PKG)

import wx

import numpy
import wxmpl
import matplotlib

from robot_msgs.msg import *
from std_msgs.msg import *
from robot_srvs.srv import *

import sys
import os
import string
from time import sleep

import rospy


class App(wx.App):
  def OnInit(self):
    rospy.ready("Hysteresis", anonymous=True)
    self.data_topic = rospy.TopicSub("/hysteresis_data", ChannelFloat32, self.OnData)
    # Configure the plot panel
    self.plot = wxmpl.PlotApp('Hysteresis Plot')

    self.plot.set_crosshairs(False)
    self.plot.set_selection(False)
    self.plot.set_zoom(False)
    return True
    
  def OnData(self,msg):
    print 'Got data named %s' % (msg.name)
    self.data_dict[msg.name] = msg.vals
    self.data_topic.unregister()
    self.Plot
    
  def Plot(self):
  
    # Plot the values and line of best fit
    fig=self.plot.get_figure()
    axes1 = fig.add_subplot(211)
    axes1.clear()
    axes2 = fig.add_subplot(212)
    axes2.clear()
    axes1.plot(numpy.array(self.data_dict['position']), numpy.array(self.data_dict['effort']), 'r--')
    axes1.set_xlabel('Position')
    axes1.set_ylabel('Effort')

    axes2.plot(numpy.array(self.data_dict['position']), numpy.array(self.data_dict['velocity']), 'b--')
    axes2.set_xlabel('Position')
    axes2.set_ylabel('Velocity')
    
    self.plot.draw()
    self.plot.Show()
    self.data_topic.unregister()
    
if __name__ == "__main__":
  try:
    app = App(0)
    app.MainLoop()
  except Exception, e:
    print e
    
  print 'quit'
