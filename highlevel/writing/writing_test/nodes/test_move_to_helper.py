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

from robot_actions.msg import ActionStatus
import robot_msgs.msg
from std_msgs.msg import Empty

import python_actions
import threading

if __name__ == '__main__':

  try:

    import pr2_robot_actions.msg
    import robot_actions.msg
    import nav_robot_actions.msg

    rospy.init_node("test_move_to_helper")
    switch_controllers = python_actions.ActionClient("switch_controllers",
                                                     pr2_robot_actions.msg.SwitchControllers,
                                                     pr2_robot_actions.msg.SwitchControllersState, 
                                                     Empty)
    tuck_arm = python_actions.ActionClient("safety_tuck_arms",
                                           Empty,
                                           robot_actions.msg.NoArgumentsActionState,
                                           Empty)
    find_helper = python_actions.ActionClient("find_helper",
                                              Empty,
                                              pr2_robot_actions.msg.FindHelperState,
                                              robot_msgs.msg.PoseStamped)

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

    generate_text_trajectory = python_actions.ActionClient("generate_text_trajectory",
                                    pr2_robot_actions.msg.TextGoal,
                                    pr2_robot_actions.msg.GenerateTextTrajectoryState,
                                    robot_msgs.msg.Path)
    time.sleep(2)

    # Preempt all of the actions
    switch_controllers.preempt()
    tuck_arm.preempt()
    find_helper.preempt()
    move_base_local.preempt()
    track_helper.preempt()

    time.sleep(2)

    class Abort(Exception):
      pass
    class Preempt(Exception):
      pass

    controllers = [
       "r_arm_cartesian_trajectory_controller",
       "r_arm_joint_trajectory_controller",
       "r_gripper_position_controller",
       "head_controller",
       "laser_tilt_controller"]


    def stop_start(stp, strt):
      switchlist = pr2_robot_actions.msg.SwitchControllers()
      switchlist.stop_controllers = stp
      switchlist.start_controllers = strt
      as,_ = switch_controllers.execute(switchlist, 4.0)
      if as != python_actions.SUCCESS:
      sys.exit(-1);

    stop_start(controllers, [])

    stop_start([], ["r_arm_joint_trajectory_controller"])
    as,_ = tuck_arm.execute(Empty(), 20.0)
    if as != python_actions.SUCCESS:
      sys.exit(-1);

    stop_start(["r_arm_joint_trajectory_controller"], ["r_gripper_position_controller", "r_arm_cartesian_trajectory_controller"])
    as,_ = ask_for_pen.execute(Empty(), 20.0)
    if as != python_actions.SUCCESS:
      sys.exit(-1);

    sys.exit(0)

    as,helper_p = find_helper.execute(Empty(), 200.0)
    if as != python_actions.SUCCESS:
      sys.exit(-1);

    print "===> Found person:", helper_p
    time.sleep(1)

    as,_ = start_tilt_laser.execute(Empty(), 20.0)
    if as != python_actions.SUCCESS:
      sys.exit(-1);

    print "===> Started laser"
    time.sleep(1)

    th = threading.Thread(target = lambda: track_helper.execute(helper_p, 500.0))
    th.start()

    print "===> track_helper started"
    time.sleep(1)

    as,_ = move_base_local(helper_p, 500.0)
    if as != python_actions.SUCCESS:
      sys.exit(-1);

    print "===> reached target point"
    track_helper.preempt()

    if 0:
      state, path = w.execute(pr2_robot_actions.msg.TextGoal("klak", robot_msgs.msg.Point(10,20,0), 30))
      print "EXECUTE returned", state #, path
      #for p in path.poses:
      #  print p.pose.position

  except KeyboardInterrupt, e:

    pass

  print "exiting"
