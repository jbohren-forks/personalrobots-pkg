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
roslib.load_manifest('writing_test')
import rospy

import Queue
import time
import sys
import threading

from robot_actions.msg import ActionStatus
import robot_msgs.msg
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
    assert set(controllers) < self.all, "Unknown controller"
    self.update(self.on | set(controllers))

  def turn_off(self, controllers):
    assert set(controllers) < self.all, "Unknown controller"
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
    tuck_arm = python_actions.ActionClient("safety_tuck_arms",
                                           Empty,
                                           robot_actions.msg.NoArgumentsActionState,
                                           Empty)

    find_helper = python_actions.ActionClient("find_helper",
                                              Empty,
                                              pr2_robot_actions.msg.FindHelperState,
                                              pr2_robot_actions.msg.FindHelperResult)

    move_base_local = python_actions.ActionClient("move_base_local",
                                                  robot_msgs.msg.PoseStamped,
                                                  nav_robot_actions.msg.MoveBaseState,
                                                  robot_msgs.msg.PoseStamped)

    start_tilt_laser = python_actions.ActionClient("set_laser_tilt",
                                        Empty,
                                        robot_actions.msg.NoArgumentsActionState,
                                        Empty)

    track_helper = python_actions.ActionClient("track_helper",
                         robot_msgs.msg.PoseStamped,
                         pr2_robot_actions.msg.TrackHelperState,
                         Empty)

    ask_for_pen = python_actions.ActionClient("ask_for_pen", Empty, robot_actions.msg.NoArgumentsActionState, Empty)
    ask_for_pen.preempt()

    generate_text_trajectory = python_actions.ActionClient("generate_text_trajectory",
                                    pr2_robot_actions.msg.TextGoal,
                                    pr2_robot_actions.msg.GenerateTextTrajectoryState,
                                    robot_msgs.msg.Path)
    generate_text_trajectory.preempt()

    time.sleep(2)

    r_arm_cartesian_controllers = set([
       "r_arm_cartesian_trajectory_controller",
       "r_arm_cartesian_pose_controller",
       "r_arm_cartesian_twist_controller",
       "r_arm_cartesian_wrench_controller",
       ])
    all_controllers = set([
       "r_gripper_position_controller",
       "head_controller",
       "laser_tilt_controller",
       "r_arm_joint_trajectory_controller"]) | r_arm_cartesian_controllers
    sw = Switcher(all_controllers)

    sw += "r_arm_joint_trajectory_controller"
    act(tuck_arm, Empty(), 20.0)

    sw += "head_controller"
    helper = act(find_helper, Empty(), 200.0)

    print "===> Found person:", helper

    time.sleep(1)

    sw += "laser_tilt_controller"
    act(start_tilt_laser, Empty(), 20.0)

    print "===> Started laser"
    time.sleep(1)

    track_helper_thread = threading.Thread(target = lambda: track_helper.execute(helper.helper_head, 500.0))
    track_helper_thread.start()

    print "===> track_helper started"
    time.sleep(1)

    act(move_base_local, helper.helper_zone, 500.0)

    print "===> reached target point"
    track_helper.preempt()
#    track_helper_thread.join()

    sw -= "r_arm_joint_trajectory_controller"
    sw.turn_on(r_arm_cartesian_controllers)
    sw += "r_gripper_position_controller"
    act(ask_for_pen, Empty(), 20.0)

    if 0:
      state, path = w.execute(pr2_robot_actions.msg.TextGoal("klak", robot_msgs.msg.Point(10,20,0), 30))
      print "EXECUTE returned", state #, path
      #for p in path.poses:
      #  print p.pose.position

  except KeyboardInterrupt, e:

    pass

  print "exiting"
