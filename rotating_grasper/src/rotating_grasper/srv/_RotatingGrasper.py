"""autogenerated by genmsg_py from RotatingGrasperRequest.msg. Do not edit."""
import roslib.message
import struct

import rotating_grasper.msg
import geometry_msgs.msg
import std_msgs.msg

class RotatingGrasperRequest(roslib.message.Message):
  _md5sum = "8d26d701a178b5799d5810420ce1250b"
  _type = "rotating_grasper/RotatingGrasperRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """RotatingGrasp command

================================================================================
MSG: rotating_grasper/RotatingGrasp
Header header
geometry_msgs/Point center
geometry_msgs/Point initial
float64 rotation_rate
float64 outward_angle
float64 object_radius
float64 object_height
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

"""
  __slots__ = ['command']
  _slot_types = ['rotating_grasper/RotatingGrasp']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       command
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(RotatingGrasperRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.command is None:
        self.command = rotating_grasper.msg.RotatingGrasp()
    else:
      self.command = rotating_grasper.msg.RotatingGrasp()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.command.header.seq, _x.command.header.stamp.secs, _x.command.header.stamp.nsecs))
      _x = self.command.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_10d.pack(_x.command.center.x, _x.command.center.y, _x.command.center.z, _x.command.initial.x, _x.command.initial.y, _x.command.initial.z, _x.command.rotation_rate, _x.command.outward_angle, _x.command.object_radius, _x.command.object_height))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.command is None:
        self.command = rotating_grasper.msg.RotatingGrasp()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.command.header.seq, _x.command.header.stamp.secs, _x.command.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.command.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.command.center.x, _x.command.center.y, _x.command.center.z, _x.command.initial.x, _x.command.initial.y, _x.command.initial.z, _x.command.rotation_rate, _x.command.outward_angle, _x.command.object_radius, _x.command.object_height,) = _struct_10d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.command.header.seq, _x.command.header.stamp.secs, _x.command.header.stamp.nsecs))
      _x = self.command.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_10d.pack(_x.command.center.x, _x.command.center.y, _x.command.center.z, _x.command.initial.x, _x.command.initial.y, _x.command.initial.z, _x.command.rotation_rate, _x.command.outward_angle, _x.command.object_radius, _x.command.object_height))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.command is None:
        self.command = rotating_grasper.msg.RotatingGrasp()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.command.header.seq, _x.command.header.stamp.secs, _x.command.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.command.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.command.center.x, _x.command.center.y, _x.command.center.z, _x.command.initial.x, _x.command.initial.y, _x.command.initial.z, _x.command.rotation_rate, _x.command.outward_angle, _x.command.object_radius, _x.command.object_height,) = _struct_10d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_10d = struct.Struct("<10d")
"""autogenerated by genmsg_py from RotatingGrasperResponse.msg. Do not edit."""
import roslib.message
import struct


class RotatingGrasperResponse(roslib.message.Message):
  _md5sum = "358e233cde0c8a8bcfea4ce193f8fc15"
  _type = "rotating_grasper/RotatingGrasperResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool success

"""
  __slots__ = ['success']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       success
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(RotatingGrasperResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.success is None:
        self.success = False
    else:
      self.success = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      buff.write(_struct_B.pack(self.success))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 1
      (self.success,) = _struct_B.unpack(str[start:end])
      self.success = bool(self.success)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      buff.write(_struct_B.pack(self.success))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.success,) = _struct_B.unpack(str[start:end])
      self.success = bool(self.success)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
class RotatingGrasper(roslib.message.ServiceDefinition):
  _type          = 'rotating_grasper/RotatingGrasper'
  _md5sum = 'cfe40b1cda41de206c8de5cb3e88c5ec'
  _request_class  = RotatingGrasperRequest
  _response_class = RotatingGrasperResponse