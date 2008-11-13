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
    rospy.ready("TestPlotter", anonymous=True)
    self.data_topic = rospy.TopicSub("/test_data", TestData, self.OnData)
    # Configure the plot panel
    self.plot = wxmpl.PlotApp('Test Plot')

    self.plot.set_crosshairs(True)
    self.plot.set_selection(True)
    self.plot.set_zoom(True)
    return True
    
  def OnData(self,msg):
    print 'Got data named %s' % (msg.test_name)
    self.data = msg
    if self.data.test_name=="hysteresis":
      self.HysteresisPlot()
    elif self.data.test_name=="sinesweep":
      self.SineSweepPlot()
    else:
      print 'this test message cannot be analyzed'
      
  def HysteresisPlot(self):
    self.HysteresisAnalysis()
    print "plotting hysteresis"
    #create the figure
    fig=self.plot.get_figure()
    axes1 = fig.add_subplot(211)
    axes1.clear()
    axes2 = fig.add_subplot(212)
    axes2.clear()
    axes1.plot(numpy.array(self.data.position), numpy.array(self.data.effort), 'r--')
    #show the average effort lines 
    axes1.axhline(y=self.avg1,color='b')
    axes1.axhline(y=0,color='k')
    axes1.axhline(y=self.avg2,color='g')
    axes1.set_xlabel('Position')
    axes1.set_ylabel('Effort')
    #show that a constant velocity was achieved
    axes2.plot(numpy.array(self.data.position), numpy.array(self.data.velocity), 'b--')
    axes2.set_xlabel('Position')
    axes2.set_ylabel('Velocity')
    self.plot.draw()
    
  def SineSweepPlot(self):
    print "plotting sinesweep"
    # Plot the values and line of best fit
    fig=self.plot.get_figure()
    axes1 = fig.add_subplot(211)
    axes1.clear()
    axes2 = fig.add_subplot(212)
    axes2.clear()
    #axes1.plot(numpy.array(self.data_dict['position']), numpy.array(self.data_dict['effort']), 'r--')
    axes1.psd(numpy.array(self.data.effort), NFFT=16384, Fs=1000, Fc=0, color='r')
    axes1.psd(numpy.array(self.data.position), NFFT=16384, Fs=1000, Fc=0)
    axes1.axis([0, 50, 0, -100])

    axes1.set_xlabel('Position PSD')
    #axes2.plot(numpy.array(self.data_dict['position']), numpy.array(self.data_dict['velocity']), 'b--')
    pxx, f = axes2.psd(numpy.array(self.data.velocity), NFFT=16384, Fs=1000, Fc=0)
    axes2.clear()
    axes2.plot(f,pxx)
    index = numpy.argmax(pxx)
    max_value=max(pxx)

    axes2.plot([f[index]],[pxx[index]],'r.', markersize=10);

    axes2.axis([0, 50, 0, 1])
    axes2.set_xlabel('Velocity PSD')
    #axes2.set_ylabel('Velocity')
    self.count=0
    self.plot.draw()

  def HysteresisAnalysis(self):
    #compute the encoder travel
    encodermin=min(numpy.array(self.data.position))
    encodermax=max(numpy.array(self.data.position))
    #find the index to do the average over
    indexmax = numpy.argmax(numpy.array(self.data.position))
    indexmin = numpy.argmin(numpy.array(self.data.position))
    index=min(indexmin,indexmax)
    end =numpy.array(self.data.position).size
    #compute the averages to display
    self.avg1 = numpy.average(numpy.array(self.data.effort)[0:index])
    self.avg2 = numpy.average(numpy.array(self.data.effort)[index:end])
    if encodermin > self.data.arg_value[1] or encodermax < self.data.arg_value[2]:
      print "mechanism is binding and not traveling the complete distace"
      print "min expected: %f  measured: %f" % (self.data.arg_value[1],encodermin)
      print "max expected: %f  measured: %f" % (self.data.arg_value[2],encodermax)
    else:
      print "passed test"
    
    
if __name__ == "__main__":
  try:
    app = App(0)
    app.MainLoop()
  except Exception, e:
    print e
    
  self.data_topic.unregister()
  print 'quit'
