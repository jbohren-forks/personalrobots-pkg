#! /usr/bin/env python

import rosweb
from std_msgs.msg import String

def config_plugin(context):
  context.register_subtopic("/chatter:more", UpperChatterSubtopic)

class UpperChatterSubtopic(rosweb.ROSWebSubTopic):
  def __init__(self, topic, factory, main_rwt):
    rosweb.ROSWebSubTopic.__init__(self, topic, factory, main_rwt)

  def transform(self, msg):
    newmsg = String(msg.data.upper())
    return newmsg
