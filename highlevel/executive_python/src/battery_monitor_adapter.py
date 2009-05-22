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
roslib.load_manifest('executive_python')
import rospy
from robot_msgs.msg import BatteryState
import os

class BatteryMonitorAdapter:
  def __init__(self, discharge_limit, charge_limit, state_topic, email_addresses, robot_name, mail_program):
    self.discharge_limit = discharge_limit
    self.charge_limit = charge_limit
    rospy.Subscriber(state_topic, BatteryState, self.update)
    self.state = None
    self.state_topic = state_topic
    self.email_addresses = email_addresses
    self.mail_program = mail_program
    self.robot_name = robot_name
    self.mail_sent = False

  def legalState(self):
    return self.state != None

  def update(self, state):
    self.state = state

  def chargeNeeded(self):
    if self.state.energy_capacity == 0.0 or self.state.energy_remaining == 0.0:
      return False
    return (self.state.energy_remaining / self.state.energy_capacity) < self.discharge_limit

  def chargeDone(self):
    if self.state.energy_capacity == 0.0 or self.state.energy_remaining == 0.0:
      return False
    return (self.state.energy_remaining / self.state.energy_capacity) > self.charge_limit

  def sendPluggedEmail(self):
    mail_string = "To: "
    for address in self.email_addresses: mail_string += address + ", "
    mail_string += "\nFrom: %s@willowgarage.com" % self.robot_name
    mail_string += "\nSubject: Thanks! I've been plugged in."
    mail_string += "\n\nIf you're thinking about plugging me in after my last e-mail, don't bother. Someone else beat you to it. Thanks."

    #since the robot's have mail servers installed on them... we'll just pipe out our e-mail
    pipe = os.popen("%s -t" % self.mail_program, 'w')
    pipe.write(mail_string)
    pipe.close()

  def sendUnpluggedEmail(self):
    mail_string = "To: "
    for address in self.email_addresses: mail_string += address + ", "
    mail_string += "\nFrom: %s@willowgarage.com" % self.robot_name
    mail_string += "\nSubject: Thanks! I've been unplugged."
    mail_string += "\n\nIf you're thinking about unplugging me after my last e-mail, don't bother. Someone else beat you to it. Thanks."

    #since the robot's have mail servers installed on them... we'll just pipe out our e-mail
    pipe = os.popen("%s -t" % self.mail_program, 'w')
    pipe.write(mail_string)
    pipe.close()

  def sendPlugEmail(self):
    mail_string = "To: "
    for address in self.email_addresses: mail_string += address + ", "
    mail_string += "\nFrom: %s@willowgarage.com" % self.robot_name
    mail_string += "\nSubject: Battery level on robot below %.2f" % self.discharge_limit
    mail_string += "\n\nMy battery level has fallen below the notification level. You should probably think about plugging me in. \n\nThanks much,\n%s" % self.robot_name

    #since the robot's have mail servers installed on them... we'll just pipe out our e-mail
    pipe = os.popen("%s -t" % self.mail_program, 'w')
    pipe.write(mail_string)
    pipe.close()

  def sendUnPlugEmail(self):
    mail_string = "To: "
    for address in self.email_addresses: mail_string += address + ", "
    mail_string += "\nFrom: %s@willowgarage.com" % self.robot_name
    mail_string += "\nSubject: Battery level on robot above %.2f" % self.charge_limit
    mail_string += "\n\nMy battery level is above the desired charge level. You should probably think about unplugging me. \n\nThanks much,\n%s" % self.robot_name

    #since the robot's have mail servers installed on them... we'll just pipe out our e-mail
    pipe = os.popen("%s -t" % self.mail_program, 'w')
    pipe.write(mail_string)
    pipe.close()

  def sendStuckEmail(self):
    mail_string = "To: "
    for address in self.email_addresses: mail_string += address + ", "
    mail_string += "\nFrom: %s@willowgarage.com" % self.robot_name
    mail_string += "\nSubject: My caster is stuck! Help!"
    mail_string += "\n\nMy caster seems to be stuck! I have a solution to get myself unstuck, but it is dangerous and needs to be tested tomorrow. For now, could you please come wiggle me a bit? Its probably my back right caster. \n\nThanks much,\n%s" % self.robot_name

    #since the robot's have mail servers installed on them... we'll just pipe out our e-mail
    pipe = os.popen("%s -t" % self.mail_program, 'w')
    pipe.write(mail_string)
    pipe.close()

  def sendUnstuckEmail(self):
    mail_string = "To: "
    for address in self.email_addresses: mail_string += address + ", "
    mail_string += "\nFrom: %s@willowgarage.com" % self.robot_name
    mail_string += "\nSubject: Whew, false alarm. I'm unstuck."
    mail_string += "\n\nSorry to cause trouble. It turns out I got myself unstuck. I'll just keep driving... sorry for the trouble and any concern I've caused. \n\nThanks much,\n%s" % self.robot_name

    #since the robot's have mail servers installed on them... we'll just pipe out our e-mail
    pipe = os.popen("%s -t" % self.mail_program, 'w')
    pipe.write(mail_string)
    pipe.close()

  def pluggedIn(self):
    return self.state.power_consumption >= 0.0
