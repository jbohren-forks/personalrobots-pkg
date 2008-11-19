#! /usr/bin/python

import math 
import rostools
rostools.update_path('2dnav_pr2')

import sys, traceback, logging, rospy
from rosrecord import *
from std_msgs.msg import RobotBase2DOdom

NAME = 'log_parser'

class LogParser:
  def __init__(self):
    self.start_time = None
    self.end_time = None
    self.distance = 0.0
    self.radians = 0.0
    self.last_data = None
    self.msgs = []

  def new_odom(self, msg):
    #if this is the first message, log the time
    if self.last_data == None:
      self.start_time = msg.header.stamp
      self.last_data = msg
      return

    meters, rads = self.dist(msg, self.last_data);
    self.distance += meters
    self.radians += rads
    self.end_time = msg.header.stamp
    self.last_data = msg

  def dist(self, odom1, odom2):
    dist_meters = math.sqrt((odom1.pos.x - odom2.pos.x)**2 + (odom1.pos.y - odom2.pos.y)**2)
    dist_rads = abs((odom1.pos.th % (math.pi)) - (odom2.pos.th % (math.pi)))
    return [dist_meters, dist_rads]

  def time_diff(self):
    start = self.start_time.secs + self.start_time.nsecs / 1e9
    end = self.end_time.secs + self.end_time.nsecs / 1e9
    return end - start
  
  def print_info(self):
    print "Run Time: %.2f (seconds), Distance Traveled: %.2f (meters), Radians Traveled: %.2f (radians)" % (self.time_diff(), self.distance, self.radians)

def parse(filename):
  parser = LogParser()
  for topic, msg in logplayer(filename):
    if topic == "/odom":
      parser.new_odom(msg)
    # unfortunately have to include this as messages spin up the rospy infrastructure
    if rospy.is_shutdown():
      break

  parser.print_info()

if __name__ == '__main__':
  parse(sys.argv[1])


