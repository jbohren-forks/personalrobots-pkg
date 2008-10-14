#!/usr/bin/python -i 
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
#

# Written by Timothy Hunter <tjhunter@willowgarage.com> 2008

from xml.dom.minidom import parseString

import rostools
import sys
import rospy
rostools.update_path('robot_srvs')
from robot_srvs.srv import *

#TODO: add a controller factory mechanism

# Loads specific controller interfaces.
rostools.update_path('pr2_controllers')
from pr2_controllers import *

# Loads basic controller interfaces.
rostools.update_path('generic_controllers')
from generic_controllers import *

# Loads logging utilities (used for record/replay)
rostools.update_path('wxpy_ros')
import wxpy_ros

import copy

__all__=['RobotInterface']

class StateLogger(wxpy_ros.wxplot.Channel):
  def __init__(self, slotName, style):
    self._records = None
    self._previous_time=None
    print self
    
  def callback(self, m_state, time):
    if self._records is not None:
      print 'adding data'
      if not self._previous_time:
        self._previous_time = time
      self._records.append((time-self._previous_time,[(j_state.name,j_state.position) for j_state in m_state.joint_states]))
      self._previous_time = time

class RobotInterface:
  def __init__(self):
    self._records = []
    self._channel = None
    m_name = '/mechanism_state/'
    m_handler = wxpy_ros.getMessageHandler()
    self._channel = m_handler.subscribe(m_name, '',StateLogger)
  
  def spawn(self, typename, name, param):
    s = rospy.ServiceProxy('spawn_controller', SpawnController)
    resp = s.call(SpawnControllerRequest(param))
    if resp.ok == 1:
        print "Spawned successfully"
        return typename(name)
    else:
        print "Error when spawning", resp.ok
        return None

  def spawnXml(self, param):
    dom = parseString(param)
    node = dom.childNodes[0]
    assert node and node.localName == 'controller'
    typename = eval(node.getAttribute('type').replace('Node','').replace('Controller','Proxy'))
    name = node.getAttribute('topic')
    print typename, name
    return self.spawn(typename, name, param)
  
  def spawnFile(self, fname):
    c1=open(fname)
    return self.spawnXml(c1.read())
  
  def createRecord(self):
    self.stop_record()
    self._records=[]
    print 'Press enter when ready'
    raw_input()
    print 'press enter when finished'
    self.record()
    raw_input()
    self.stop_record()
    return copy.deepcopy(self._records)

  
  def record(self):
    self._channel._records = self._records
    print self._channel._records,self._records
    
  def stop_record(self):
    self._channel._records = None
    self._channel._previous_time = None

  def calibrateArm(self):
    pass
