import rospy
import struct
import itertools
class pyImageString(object):
  __slots__ = ['publish_count','stamp_secs','stamp_nsecs','imageString','imageFormat','height','width']

  def __init__(self, *args):
    """
    @param args: fields to set, in .flow order. Optionally, stamp_secs and stamp_nsecs may be included as the last two arguments
    """
    self.publish_count = 0
    if args:
       timestampSet = len(args) == 6
       assert len(args) == 4 or timestampSet
       if timestampSet:
           self.stamp_secs = args[-2]
           self.stamp_nsecs = args[-1]
       else:
           self.stamp_secs = self.stamp_nsecs = None
       self.imageString = args[0]
       self.imageFormat = args[1]
       self.height = args[2]
       self.width = args[3]
    else:
       self.stamp_secs = None
       self.stamp_nsecs = None
       self.imageString = None
       self.imageFormat = None
       self.height = None
       self.width = None

  def serialize(self, buff):
    buff.write(struct.pack('<iii', self.publish_count, self.stamp_secs, self.stamp_nsecs))
    length = len(self.imageString)
    buff.write(struct.pack('<I%ss'%length, length, self.imageString))
    length = len(self.imageFormat)
    buff.write(struct.pack('<I%ss'%length, length, self.imageFormat))
    buff.write(struct.pack('<II', self.height, self.width))

  def deserialize(self, str):
    try:
      if str is None or len(str) == 0:
        raise rospy.DeserializationError, 'buffer underfill'
      end = 0
      start = end
      end += struct.calcsize('<iii')
      (self.publish_count, self.stamp_secs, self.stamp_nsecs,) = struct.unpack('<iii',str[start:end])
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%ss'%length
      start = end
      end += struct.calcsize(pattern)
      (self.imageString,) = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%ss'%length
      start = end
      end += struct.calcsize(pattern)
      (self.imageFormat,) = struct.unpack(pattern, str[start:end])
      start = end
      end += struct.calcsize('<II')
      (self.height, self.width,) = struct.unpack('<II',str[start:end])
      return self
    except struct.error, e:
      raise rospy.DeserializationError, e #most likely buffer underfill

class FlowpyImageString(rospy.FlowBase):

  def __init__(self, name, direction):
     super(FlowpyImageString, self).__init__(name, direction, 'pyImageViewer/pyImageString')

  def _receive(self, str, data=None):
    if not data:
      data = pyImageString()
    return data.deserialize(str)
class InflowpyImageString(FlowpyImageString):
  def __init__(self, name):
     super(InflowpyImageString, self).__init__(name, 'inflow')
class OutflowpyImageString(FlowpyImageString):
  def __init__(self, name):
     super(OutflowpyImageString, self).__init__(name, 'outflow')
