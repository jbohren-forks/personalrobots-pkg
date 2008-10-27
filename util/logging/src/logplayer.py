import rostools
rostools.update_path('logging')

import std_msgs.msg
import struct

class LogPlayer:
  def __init__(self):
    self.handlers = {}

  def open(self, filename):
    self.f = open(filename, "r")
    l = self.f.readline().rstrip() #ROSRECORD V1.1
    assert l == "#ROSRECORD V1.1"

  def addHandler(self, topic, msgtype, cb):
    self.handlers[(topic, msgtype)] = cb
    
  def nextMsg(self):

    topic = self.f.readline().rstrip()
    if topic == "":
      return False
    md5sum = self.f.readline().rstrip()
    datatype = self.f.readline().rstrip()
    ( time_sec, time_nsec, length ) = struct.unpack("LLL", self.f.read(12))
    data = self.f.read(length)

    message_name = datatype.split('/')[-1]
    pytype = eval("std_msgs.msg." + message_name)
    if (topic, pytype) in self.handlers:
      msg = pytype()
      msg.deserialize(data)
      self.handlers[(topic, pytype)](msg)
    return True

filename = "/u/jamesb/2008-10-27-10-02-13-topic.bag"
