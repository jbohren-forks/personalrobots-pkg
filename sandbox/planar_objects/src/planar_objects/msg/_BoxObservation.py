# autogenerated by genmsg_py from BoxObservation.msg. Do not edit.
import roslib.message
import struct

import robot_msgs.msg
## \htmlinclude BoxObservation.msg.html

class BoxObservation(roslib.message.Message):
  _md5sum = "265645feb0c635335862a46ef9b80248"
  _type = "planar_objects/BoxObservation"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """robot_msgs/Transform transform
int32 plane_id

float64 w # width
float64 h # height 

float64 precision # fill ratio of rectangle
float64 recall # coverage ratio of rectangle in plane



================================================================================
MSG: robot_msgs/Transform
Vector3 translation
Quaternion rotation

================================================================================
MSG: robot_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: robot_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w

"""
  __slots__ = ['transform','plane_id','w','h','precision','recall']
  _slot_types = ['robot_msgs/Transform','int32','float64','float64','float64','float64']

  ## Constructor. Any message fields that are implicitly/explicitly
  ## set to None will be assigned a default value. The recommend
  ## use is keyword arguments as this is more robust to future message
  ## changes.  You cannot mix in-order arguments and keyword arguments.
  ##
  ## The available fields are:
  ##   transform,plane_id,w,h,precision,recall
  ##
  ## @param args: complete set of field values, in .msg order
  ## @param kwds: use keyword arguments corresponding to message field names
  ## to set specific fields. 
  def __init__(self, *args, **kwds):
    super(BoxObservation, self).__init__(*args, **kwds)
    #message fields cannot be None, assign default values for those that are
    if self.transform is None:
      self.transform = robot_msgs.msg.Transform()
    if self.plane_id is None:
      self.plane_id = 0
    if self.w is None:
      self.w = 0.
    if self.h is None:
      self.h = 0.
    if self.precision is None:
      self.precision = 0.
    if self.recall is None:
      self.recall = 0.

  ## internal API method
  def _get_types(self): return BoxObservation._slot_types

  ## serialize message into buffer
  ## @param buff StringIO: buffer
  def serialize(self, buff):
    try:
      buff.write(struct.pack('<7di4d', self.transform.translation.x, self.transform.translation.y, self.transform.translation.z, self.transform.rotation.x, self.transform.rotation.y, self.transform.rotation.z, self.transform.rotation.w, self.plane_id, self.w, self.h, self.precision, self.recall))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  ## unpack serialized message in str into this message instance
  ## @param str str: byte array of serialized message
  def deserialize(self, str):
    try:
      if self.transform is None:
        self.transform = robot_msgs.msg.Transform()
      end = 0
      start = end
      end += 92
      (self.transform.translation.x, self.transform.translation.y, self.transform.translation.z, self.transform.rotation.x, self.transform.rotation.y, self.transform.rotation.z, self.transform.rotation.w, self.plane_id, self.w, self.h, self.precision, self.recall,) = struct.unpack('<7di4d',str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  ## serialize message with numpy array types into buffer
  ## @param buff StringIO: buffer
  ## @param numpy module: numpy python module
  def serialize_numpy(self, buff, numpy):
    try:
      buff.write(struct.pack('<7di4d', self.transform.translation.x, self.transform.translation.y, self.transform.translation.z, self.transform.rotation.x, self.transform.rotation.y, self.transform.rotation.z, self.transform.rotation.w, self.plane_id, self.w, self.h, self.precision, self.recall))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  ## unpack serialized message in str into this message instance using numpy for array types
  ## @param str str: byte array of serialized message
  ## @param numpy module: numpy python module
  def deserialize_numpy(self, str, numpy):
    try:
      if self.transform is None:
        self.transform = robot_msgs.msg.Transform()
      end = 0
      start = end
      end += 92
      (self.transform.translation.x, self.transform.translation.y, self.transform.translation.z, self.transform.rotation.x, self.transform.rotation.y, self.transform.rotation.z, self.transform.rotation.w, self.plane_id, self.w, self.h, self.precision, self.recall,) = struct.unpack('<7di4d',str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill
