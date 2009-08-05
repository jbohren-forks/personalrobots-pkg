#! /usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
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
roslib.load_manifest('trex_pr2_writing_test')
import rospy

import Queue
import time
import sys
import threading

from robot_actions.msg import ActionStatus
import geometry_msgs.msg
from std_msgs.msg import Empty
import python_actions

import pr2_robot_actions.msg
import robot_actions.msg
import nav_robot_actions.msg

class Switcher:

  def __init__(self, all):
    self.all = set(all)
    self.on = set()
    self.switch_controllers = python_actions.ActionClient("switch_controllers",
                                                     pr2_robot_actions.msg.SwitchControllers,
                                                     pr2_robot_actions.msg.SwitchControllersState, 
                                                     Empty)
    time.sleep(2.0)
    self.update(set())

  def turn_on(self, controllers):
    #assert set(controllers) < self.all, "Unknown controller"
    self.update(self.on | set(controllers))

  def turn_off(self, controllers):
    #assert set(controllers) < self.all, "Unknown controller"
    self.update(self.on - set(controllers))

  # Sugar

  def __iadd__(self, other):
    self.turn_on(set([other]))
    return self

  def __isub__(self, other):
    self.turn_off(set([other]))
    return self

  # List has changed, so send an action to "switch_controllers"
  def update(self, newon):
    self.on = newon
    switchlist = pr2_robot_actions.msg.SwitchControllers()
    switchlist.stop_controllers = sorted(self.all - self.on)
    switchlist.start_controllers = sorted(self.on)
    print switchlist.stop_controllers, switchlist.start_controllers
    as,_ = self.switch_controllers.execute(switchlist, 4.0)
    #if as != python_actions.SUCCESS:
    #  sys.exit(-1);

class Abort(Exception):
  pass
class Preempt(Exception):
  pass

def act(action, goal, timeout):
  as,fb = action.execute(goal, timeout)
  if as == python_actions.SUCCESS:
    return fb
  elif as == python_actions.ABORTED:
    raise Abort
  elif as == python_actions.PREEMPTED:
    raise Preempt
  else:
    assert 0, ("Unexpected action status %d" % as)

if __name__ == '__main__':

  try:

    rospy.init_node("test_move_to_helper")

    find_helper = python_actions.ActionClient("find_helper",
                                              Empty,
                                              pr2_robot_actions.msg.FindHelperState,
                                              pr2_robot_actions.msg.FindHelperResult)

    track_helper = python_actions.ActionClient("track_helper",
                         geometry_msgs.msg.PoseStamped,
                         pr2_robot_actions.msg.TrackHelperState,
                         Empty)


    time.sleep(2)

    all_controllers = set([
       "head_controller" ])
    sw = Switcher(all_controllers)

    sw += "head_controller"
    helper = act(find_helper, Empty(), 200.0)

    print "===> Found person:", helper

    time.sleep(1)

    track_helper_thread = threading.Thread(target = lambda: track_helper.execute(helper.helper_head, 500.0))
    track_helper_thread.start()

    print "===> track_helper started"
    time.sleep(1)

  except KeyboardInterrupt, e:

    pass

  print "exiting"
