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

# Author: Marius Muja

import wx
import wx.lib.intctrl
import sys
import os

PKG = 'stereodcam_params' # this package name
import roslib; roslib.load_manifest(PKG)
from roslib.scriptutil import get_param_server, script_resolve_name
from roslib.names import ns_join, get_ros_namespace, make_caller_id, make_global_ns, GLOBALNS

import rospy
from std_msgs.msg import Empty

def _get_caller_id():
    return make_caller_id('%s-%s'%(PKG,os.getpid()))


def succeed(args):
    code, msg, val = args
    if code != 1:
        raise RosParamException(msg)
    return val

def _get_param(param):
    return succeed(get_param_server().getParam(_get_caller_id(), param))

def _set_param(param, value):
    succeed(get_param_server().setParam(_get_caller_id(), param, value))



class IntParameterWidget(wx.Panel):
    
    def __init__(self, parent, name, param, update_callback = None):
        
        self.param = param
        self.update_callback = update_callback
        
        wx.Panel.__init__(self, parent ,-1)
        self.label = wx.StaticText(self, -1, name)
        self.edit = wx.lib.intctrl.IntCtrl(self, -1)
        
        # properties
        self.label.SetMinSize((150,-1))
        
        # layout
        sizer =  wx.BoxSizer(wx.HORIZONTAL)
        sizer.Add(self.label,0,0,0)
        sizer.Add(self.edit,0,0,0)
        self.SetSizer(sizer)

        # bindings
        self.Bind(wx.EVT_TEXT, self.onEdit, self.edit)

        self._initValue()


    def _initValue(self):
        try:
            value = _get_param(self.param)
            self.edit.SetValue(value)
        except:
            print "Cannot read value for parameter: "+self.param
        
        
    def _setParam(self, param, value):
        try:
            _set_param(param, value)
            if self.update_callback != None:
                self.update_callback()
        except:
            print "Cannot set parameter: "+self.param

    def onEdit(self, event): 
        str_value = event.GetString()
        #print str_value
        value = int(str_value)
        #print value
        self._setParam(self.param, value)


class BoolParameterWidget(wx.Panel):
    
    def __init__(self, parent, name, param, update_callback = None):
        
        self.param = param
        self.update_callback = update_callback
        
        wx.Panel.__init__(self, parent ,-1)
        self.label = wx.StaticText(self, -1, name)
        self.checkbox = wx.CheckBox(self, -1, "active")
        
        # properties
        self.label.SetMinSize((150,-1))
        
        # layout
        sizer =  wx.BoxSizer(wx.HORIZONTAL)
        sizer.Add(self.label,0,0,0)
        sizer.Add(self.checkbox,0,0,0)
        self.SetSizer(sizer)

        # bindings
        self.Bind(wx.EVT_CHECKBOX, self.onCheckbox, self.checkbox)

        self._initValue()


    def _initValue(self):
        try:
            value = _get_param(self.param)
            self.checkbox.SetValue(value)
        except:
            print "Cannot read value for parameter: "+self.param
        
        
    def _setParam(self, param, value):
        try:
            _set_param(param, value)
            if self.update_callback != None:
                self.update_callback()
        except:
            print "Cannot set parameter: "+self.param

    def onCheckbox(self, event):
        value = event.IsChecked();
        self._setParam(self.param, value)


class RangeParameterWidget(wx.Panel):
    
    def __init__(self, parent, name, param, update_callback = None, has_auto = False):
        
        self.param = param
        self.update_callback = update_callback
        self.has_auto = has_auto
        
        wx.Panel.__init__(self, parent ,-1)
        self.label = wx.StaticText(self, -1, name)
        self.slider = wx.Slider(self, -1, 0, 0, 100)
        self.spin = wx.SpinCtrl(self, -1, "", min=0, max=100)
        if self.has_auto:
            self.checkbox = wx.CheckBox(self, -1, "auto")
        
        # properties
        self.label.SetMinSize((150,-1))
        
        # layout
        sizer =  wx.BoxSizer(wx.HORIZONTAL)
        sizer.Add(self.label,0,0,0)
        sizer.Add(self.slider,1,0,0)
        sizer.Add(self.spin,0,0,0)
        if self.has_auto:
            sizer.Add(self.checkbox,0,0,0)
        self.SetSizer(sizer)

        # bindings
        self.Bind(wx.EVT_COMMAND_SCROLL, self.onSlider, self.slider)
        self.Bind(wx.EVT_SPINCTRL, self.onSpin, self.spin)
        if self.has_auto:
            self.Bind(wx.EVT_CHECKBOX, self.onCheckbox, self.checkbox)

        self._initRange()            
        self._initValue()

    def _initRange(self):
        try:
            min_value = _get_param(self.param+"_min")
            max_value = _get_param(self.param+"_max")
            self.slider.SetRange(min_value, max_value)
            self.spin.SetRange(min_value, max_value)
        except:
            print "Cannot read range for parameter: "+self.param

    def _initValue(self):
        try:
            value = _get_param(self.param)
            self.slider.SetValue(value)
            self.spin.SetValue(value)
        except:
            print "Cannot read value for parameter: "+self.param
        if self.has_auto:
            try:
                value = _get_param(self.param+"_auto")
                self.checkbox.SetValue(value)
                if value:
                    self.slider.Disable()
                    self.spin.Disable()
                else:
                    self.slider.Enable()
                    self.spin.Enable()
            except:
                print "Cannot tell if auto is set for parameter: "+self.param
                
    def _setParam(self, param, value):
        try:
            _set_param(param, value)
            if self.update_callback != None:
                self.update_callback()
        except:
            print "Cannot set parameter: "+param

    def onSlider(self, event): 
        value = event.GetPosition()
        self.spin.SetValue(value)
        self._setParam(self.param, value)

    def onSpin(self, event): 
        value = event.GetInt()
        self.slider.SetValue(value)
        self._setParam(self.param, value)
        
    def onCheckbox(self, event):
        value = event.IsChecked();
        self._setParam(self.param+"_auto", value)
        if value:
            self.slider.Disable()
            self.spin.Disable()
        else:
            self.slider.Enable()
            self.spin.Enable()


class StereoParameters(wx.Frame):
    def __init__(self, *args, **kwds):

        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)

        self.widgets = []
        self.widgets.append(RangeParameterWidget(self, "Exposure", "stereo/exposure", has_auto = True, update_callback = self.update_stereo_params))
        self.widgets.append(RangeParameterWidget(self, "Gain", "stereo/gain", has_auto = True, update_callback = self.update_stereo_params))
        self.widgets.append(RangeParameterWidget(self, "Brightness", "stereo/brightness", has_auto = True, update_callback = self.update_stereo_params))
        self.widgets.append(BoolParameterWidget(self, "Companding", "stereo/companding", update_callback = self.update_stereo_params))
        self.widgets.append(BoolParameterWidget(self, "HDR", "stereo/hdr", update_callback = self.update_stereo_params))
        self.widgets.append(BoolParameterWidget(self, "Unique Check", "stereo/unique_check", update_callback = self.update_stereo_params))
        self.widgets.append(IntParameterWidget(self, "Texture Threshold", "stereo/texture_thresh", update_callback = self.update_stereo_params))
        self.widgets.append(IntParameterWidget(self, "Unique Threshold", "stereo/unique_thresh", update_callback = self.update_stereo_params))
        self.widgets.append(IntParameterWidget(self, "Smoothness Threshold", "stereo/smoothness_thresh", update_callback = self.update_stereo_params))
        self.widgets.append(IntParameterWidget(self, "Horopter", "stereo/horopter", update_callback = self.update_stereo_params))
        self.widgets.append(IntParameterWidget(self, "Speckle Size", "stereo/speckle_size", update_callback = self.update_stereo_params))
        self.widgets.append(IntParameterWidget(self, "Speckle Diff", "stereo/speckle_diff", update_callback = self.update_stereo_params))
        self.widgets.append(IntParameterWidget(self, "Corr Size", "stereo/corr_size", update_callback = self.update_stereo_params))
        self.widgets.append(IntParameterWidget(self, "Num Disp", "stereo/num_disp", update_callback = self.update_stereo_params))
                                                        
        self.close_button = wx.Button(self, -1, "Close")

        self.__set_properties()
        self.__do_layout()
    
        self.Bind(wx.EVT_BUTTON, self.onClose, self.close_button)
    
        self.check_params = rospy.Publisher('stereo/check_params', Empty)
        rospy.init_node('stereodcam_params', anonymous=True)

        
    def update_stereo_params(self):
        self.check_params.publish(Empty())
        

    def __set_properties(self):
        self.SetTitle("Stereo Parameters")
        self.SetSize((498, 500))

    def __do_layout(self):
        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add((10, 10), 0, 0, 0)
        
        for widget in self.widgets:
            vsizer.Add(widget, 1, wx.EXPAND, 0)

        vsizer.Add(self.close_button, 0, wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL, 0)
        vsizer.Add((10, 10), 0, 0, 0)
        hsizer = wx.BoxSizer(wx.HORIZONTAL)
        hsizer.Add((10, 10), 0, 0, 0)
        hsizer.Add(vsizer, 1, wx.EXPAND, 0)
        hsizer.Add((10, 10), 0, 0, 0)
        self.SetSizer(hsizer)

        self.Layout()


    def onClose(self, event):
        sys.exit(0)



class MyApp(wx.App):
    def OnInit(self):
        wx.InitAllImageHandlers()
        frame_2 = StereoParameters(None, -1, "")
        self.SetTopWindow(frame_2)
        frame_2.Show()
        return 1

# end of class MyApp

if __name__ == "__main__":
    app = MyApp(0)
    app.MainLoop()
