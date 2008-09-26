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
# Revision $Id: gossipbot.py 1013 2008-05-21 01:08:56Z sfkwc $

## Simple draft of teleop arm in python - not ready yet

import rostools
rostools.update_path('pr2_mechansim_controllers')

import sys, os, string
import rospy
from pr2_mechansim_controllers.srv import *

class ArmInterface:
  def __init__(self, arm_name):
    self.arm_name = arm_name
    self.set_pos_sname = '%s/set_command' % arm_name
    self.get_pos_sname = '%s/get_command' % arm_name
    self.set_joint_gains_sname = '%s/set_joint_gains' % arm_name
    self.set_cp_sname = '%s/set_cartesian_pos' % arm_name
    self.get_cp_sname = '%s/get_cartesian_pos' % arm_name
    rospy.wait_for_service(self.set_pos_sname)
    rospy.wait_for_service(self.get_pos_sname)
    rospy.wait_for_service(self.set_joint_gains_sname)
    rospy.wait_for_service(self.set_cp_sname)
    rospy.wait_for_service(self.get_cp_sname)
    self.set_com = rospy.ServiceProxy(self.set_pos_sname, SetJointPosCmd)
    self.get_com = rospy.ServiceProxy(self.get_pos_sname, GetJointPosCmd)
    self.set_joint_gains = rospy.ServiceProxy(self.set_joint_gains_sname, SetJointGains)
    self.set_cp = rospy.ServiceProxy(self.set_cp_sname, SetCartesianPosCmd)
    self.get_cp = rospy.ServiceProxy(self.get_cp_sname, GetCartesianPosCmd)
    self.cur_com = self.get()
    self.__doc__ = " Crash course:\n \
    your_arm_name.set([list of positions]) \n \
    your_arm_name.get() => get the current position commands \n \
    your_arm_name.setCP() => set the current cartesian position command \n \
    your_arm_name.getCP() => get the current position in the cartesian frame\n \
    your_arm_name.set() => get the current position commands\n \
    ++ some examples ++\n \
    la.set([0,0,0,1,0,0,0]) => send commands for all joints of the left arm to be 0 expect the 4th one\n \
    la.set(j1=1, j7=-1) => changes the previous command to the first joint at 1, the last at -1\n \
    that's it folks!\n \
    "
    
  # Set position commands
  def set(self, list_pos=None,**kwargs):
    if list_pos:
      self.cur_com = list_pos
    for i in kwargs:
      foo=str(i)[1:]
      self.cur_com[int(foo)-1] = kwargs[i]
    self.set_com.call(SetJointPosCmdRequest(self.cur_com))
    return self.get()
  
  # Get position commands
  def get(self):
    self.cur_com = []
    self.cur_com += (self.get_com.call(GetJointPosCmdRequest([0,0,0,0,-0.5,0,0])).positions)
    return self.cur_com

  # Set gains
  def setJointGains(self,name,p,i,d,i_min,i_max):
    self.set_joint_gains.call(SetJointGainsRequest(name,p,i,d,i_min,i_max))
  
  # Set cartesian position commands
  def setCP(self, x, y, z, roll, pitch, yaw):
    self.set_cp.call(SetCartesianPosCmdRequest(x, y, z, roll, pitch ,yaw))
    
  # Get the current cartesian position of the arm
  def getCP(self):
    v = self.get_cp.call(GetCartesianPosCmdRequest())
    return [v.x, v.y, v.z, v.roll, v.pitch, v.yaw]
    
  # Prints some help
  def help(self):
    print self.__doc__
   
if __name__ == "__main__":
    la = ArmInterface('/left_arm_controller')
    la.help()
    la.get()
