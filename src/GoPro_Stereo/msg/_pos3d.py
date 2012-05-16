"""autogenerated by genmsg_py from pos3d.msg. Do not edit."""
import roslib.message
import struct


class pos3d(roslib.message.Message):
  _md5sum = "d4b1ecf7db42ce21fc31494bb091e422"
  _type = "GoPro_Stereo/pos3d"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 X
float32 Y
float32 Z
float32 distance
float32 bearing
bool flag

"""
  __slots__ = ['X','Y','Z','distance','bearing','flag']
  _slot_types = ['float32','float32','float32','float32','float32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       X,Y,Z,distance,bearing,flag
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(pos3d, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.X is None:
        self.X = 0.
      if self.Y is None:
        self.Y = 0.
      if self.Z is None:
        self.Z = 0.
      if self.distance is None:
        self.distance = 0.
      if self.bearing is None:
        self.bearing = 0.
      if self.flag is None:
        self.flag = False
    else:
      self.X = 0.
      self.Y = 0.
      self.Z = 0.
      self.distance = 0.
      self.bearing = 0.
      self.flag = False

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
      buff.write(_struct_5fB.pack(_x.X, _x.Y, _x.Z, _x.distance, _x.bearing, _x.flag))
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
      _x = self
      start = end
      end += 21
      (_x.X, _x.Y, _x.Z, _x.distance, _x.bearing, _x.flag,) = _struct_5fB.unpack(str[start:end])
      self.flag = bool(self.flag)
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
      buff.write(_struct_5fB.pack(_x.X, _x.Y, _x.Z, _x.distance, _x.bearing, _x.flag))
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
      _x = self
      start = end
      end += 21
      (_x.X, _x.Y, _x.Z, _x.distance, _x.bearing, _x.flag,) = _struct_5fB.unpack(str[start:end])
      self.flag = bool(self.flag)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_5fB = struct.Struct("<5fB")
