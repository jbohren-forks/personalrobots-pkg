#! /usr/bin/env python

import rospy
import rosservice
import cStringIO

import rosweb
from std_msgs.msg import String

def config_plugin(context):
  context.register_subtopic("/chatter:more", MoreChatterSubtopic)

class MoreChatterSubtopic(rosweb.ROSWebSubTopic):
  def __init__(self, topic, factory, main_rwt):
    rosweb.ROSWebSubTopic.__init__(self, topic, factory, main_rwt)

  def transform(self, msg):
    newmsg = String(msg.data.upper())
    return newmsg
