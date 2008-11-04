#! /usr/bin/python
import rostools
rostools.update_path("2dnav_pr2")
import rospy
import time
import math
from std_msgs.msg import *

class GoalDistanceTracker:

  def __init__(self, goal_topic, base_topic):
    rospy.Subscriber(goal_topic, PointStamped, self.new_goal_pos)
    rospy.Subscriber(base_topic, RobotBase2DOdom, self.new_base_pos)
    self.goal = None
    self.base = None

  def new_goal_pos(self, msg):
    self.goal = msg
  def new_base_pos(self, msg):
    self.base = msg

  def dist(self):  # Caution: not particularly thread safe
    if self.goal == None or self.base == None:
      return 0
    return math.sqrt( (self.goal.point.x - self.base.pos.x)**2 + (self.goal.point.y - self.base.pos.y)**2)


dist = GoalDistanceTracker('/head_controller/track_point', '/localizedpose')
rospy.init_node('torso_levitator', anonymous=True)
torso_pub = rospy.Publisher('/torso_controller/set_command', Float64)
while not rospy.is_shutdown():
  height = dist.dist() * 0.3
  torso_pub.publish(Float64(height))
  time.sleep(0.05)
