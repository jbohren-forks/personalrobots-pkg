#! /usr/bin/python

import rostools
rostools.update_path('trajectory_rollout')

import sys, time, traceback, logging, rospy, random
from std_msgs.msg import Planner2DGoal
from std_msgs.msg import Planner2DState

NAME = 'indefinite_nav'

goals = [[15.585, 14.186, .418], [14.136, 13.455, -2.070], [14.752, 14.573, 1.893]]
first = True

def indefinite_nav():
  def callback(state):
    send_goal(state.done)

  def send_goal(done):
    global first
    if first or done == 1:
      goal_pts = goals[random.randint(0, len(goals) - 1)]
      goal = Planner2DGoal()
      goal.goal.x = goal_pts[0]
      goal.goal.y = goal_pts[1]
      goal.goal.th = goal_pts[2]
      goal.enable = 1
      first = False
      print "New Goal: x: %.2f, y: %.2f, th: %.2f" % (goal.goal.x, goal.goal.y, goal.goal.th)
      pub.publish(goal)

  rospy.Subscriber("state", Planner2DState, callback)
  pub = rospy.Publisher("goal", Planner2DGoal)
  rospy.init_node(NAME, anonymous=True)
  rospy.spin()

if __name__ == '__main__':
  try:
    indefinite_nav()
  except KeyboardInterrupt, e:
    pass
  print "exiting"

