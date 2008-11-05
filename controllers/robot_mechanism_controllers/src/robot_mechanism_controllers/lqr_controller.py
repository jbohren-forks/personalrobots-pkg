#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rostools
rostools.update_path('rospy')
rostools.update_path('robot_srvs')
rostools.update_path('robot_mechanism_controllers')

import rospy, sys
from robot_srvs.srv import *
from robot_mechanism_controllers.srv import *
from robot_mechanism_controllers.controller import *

class LQRProxy(Controller):
  def __init__(self, topic_name):
    Controller.__init__(self,topic_name)
    self.cmd_ = rospy.ServiceProxy('/%s/set_command' % self.topic_name, SetLQRCommand).call
    self.target_ = rospy.ServiceProxy('/%s/set_target' % self.topic_name, SetJointCmd).call

  def cmd(self, names, pos, vel, Q, R):
    acc=[]
    effort=[]
    for i in pos:
      acc.append(0)
      effort.append(0)
    print(acc)
    zoo=SetLQRCommandRequest()
    zoo.target=SetJointCmdRequest(effort,pos,vel,acc,names)
    zoo.state_cost=Q
    zoo.input_cost=R
    return self.cmd_(zoo)

  def target(self, name, pos, vel):
    return self.target_(SetJointCmd(name,pos,vel,acc))