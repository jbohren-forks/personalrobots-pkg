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
roslib.load_manifest('writing_core')
import rospy

import Queue
import time

from robot_actions.msg import ActionStatus
import robot_msgs.msg
from std_msgs.msg import Empty

RESET = ActionStatus.RESET
ACTIVE = ActionStatus.ACTIVE
PREEMPTED = ActionStatus.PREEMPTED
SUCCESS = ActionStatus.SUCCESS
ABORTED = ActionStatus.ABORTED

class ActionClient:

  def __init__(self, node_name, goalmsg, statemsg, feedbackmsg, rate = 10.0):

    self.nn = node_name

    self.goalmsg = goalmsg
    self.statemsg = statemsg
    self.feedbackmsg = feedbackmsg

    self.goals_pub_ = rospy.Publisher(self.nn + "/activate", self.goalmsg)
    self.pre_pub_   = rospy.Publisher(self.nn + "/preempt", Empty)
    self.state_pub_ = rospy.Subscriber(self.nn + "/feedback", self.statemsg, self.onFeedback)

    self.feedback_queue = Queue.Queue()

  def onFeedback(self, msg):
    self.feedback_queue.put(msg)

  def preempt(self):
    self.pre_pub_.publish(Empty())

  def execute(self, goal, timeout = 1.0):
    fbmsg = self.feedback_queue.get()
    while not self.feedback_queue.empty():
      fbmsg = self.feedback_queue.get()

    if not (fbmsg.status.value in [RESET, PREEMPTED, SUCCESS, ABORTED]):
     print 'unexpected status'
     assert 0

    self.goals_pub_.publish(goal)
    state_machine = 'g'
    sent_preempt = False

    t_preempt = time.time() + timeout
    t = time.time()
    while True:
      assert state_machine in ['g', 'a']

      # If this client has not yet sent preempt, wait with a timeout.
      # But if this client has already sent preempt, wait indefinitely.
      if t < t_preempt and not sent_preempt:
        try:
          fbmsg = self.feedback_queue.get(True, t_preempt - t)
        except Queue.Empty:
          pass
      else:
        if not sent_preempt:
          sent_preempt = True
          self.pre_pub_.publish()
        fbmsg = self.feedback_queue.get()

      # 'g'  Goal issued, waiting for action to go active
      # 'a'  Seen active state, waiting for SUCCESS/ABORTED/PREEMPTED
      if state_machine == 'g':
        if fbmsg.status.value == ACTIVE:
          state_machine = 'a'
      elif state_machine == 'a':
        if fbmsg.status.value == SUCCESS:
          return SUCCESS, fbmsg.feedback
        if fbmsg.status.value == ABORTED:
          return ABORTED, fbmsg.feedback
        if fbmsg.status.value == PREEMPTED:
          return PREEMPTED, None
        if fbmsg.status.value != ACTIVE:
          print 'unexpected status after ACTIVE'
          assert 0
      t = time.time()
