# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from intera_core_msgs/IONodeStatus.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import intera_core_msgs.msg

class IONodeStatus(genpy.Message):
  _md5sum = "260fce3c02f43bd977c92642b3c09c1d"
  _type = "intera_core_msgs/IONodeStatus"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# IO Node Status
time time                    # time the message was created
IOComponentStatus node       # IO Node status
IOComponentStatus[] devices  # status of IO Devices in this node
time[] commands              # recent command timestamps, for syncing
================================================================================
MSG: intera_core_msgs/IOComponentStatus
## IO Component status data
string name            # component name
IOStatus status        # component status
#



================================================================================
MSG: intera_core_msgs/IOStatus
## IO status data
#
string tag             # one of the values listed below
#   down     Inoperative, not fully instantiated
#   ready    OK, fully operational
#   busy     OK, not ready to output data; input data value may be stale
#   unready  OK, not operational; data is invalid
#   error    Error, not operational
string DOWN      = down
string READY     = ready
string BUSY      = busy
string UNREADY   = unready
string ERROR     = error
#
string id             # message id, for internationalization
#
string detail         # optional additional status detail
#
"""
  __slots__ = ['time','node','devices','commands']
  _slot_types = ['time','intera_core_msgs/IOComponentStatus','intera_core_msgs/IOComponentStatus[]','time[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       time,node,devices,commands

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(IONodeStatus, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.time is None:
        self.time = genpy.Time()
      if self.node is None:
        self.node = intera_core_msgs.msg.IOComponentStatus()
      if self.devices is None:
        self.devices = []
      if self.commands is None:
        self.commands = []
    else:
      self.time = genpy.Time()
      self.node = intera_core_msgs.msg.IOComponentStatus()
      self.devices = []
      self.commands = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2I().pack(_x.time.secs, _x.time.nsecs))
      _x = self.node.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.node.status.tag
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.node.status.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.node.status.detail
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.devices)
      buff.write(_struct_I.pack(length))
      for val1 in self.devices:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v1 = val1.status
        _x = _v1.tag
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = _v1.id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = _v1.detail
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.commands)
      buff.write(_struct_I.pack(length))
      for val1 in self.commands:
        _x = val1
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.time is None:
        self.time = genpy.Time()
      if self.node is None:
        self.node = intera_core_msgs.msg.IOComponentStatus()
      if self.devices is None:
        self.devices = None
      if self.commands is None:
        self.commands = None
      end = 0
      _x = self
      start = end
      end += 8
      (_x.time.secs, _x.time.nsecs,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.node.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node.status.tag = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.node.status.tag = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node.status.id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.node.status.id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node.status.detail = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.node.status.detail = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.devices = []
      for i in range(0, length):
        val1 = intera_core_msgs.msg.IOComponentStatus()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        _v2 = val1.status
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v2.tag = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v2.tag = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v2.id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v2.id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v2.detail = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v2.detail = str[start:end]
        self.devices.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.commands = []
      for i in range(0, length):
        val1 = genpy.Time()
        _x = val1
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        self.commands.append(val1)
      self.time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2I().pack(_x.time.secs, _x.time.nsecs))
      _x = self.node.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.node.status.tag
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.node.status.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.node.status.detail
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.devices)
      buff.write(_struct_I.pack(length))
      for val1 in self.devices:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v3 = val1.status
        _x = _v3.tag
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = _v3.id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = _v3.detail
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.commands)
      buff.write(_struct_I.pack(length))
      for val1 in self.commands:
        _x = val1
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.time is None:
        self.time = genpy.Time()
      if self.node is None:
        self.node = intera_core_msgs.msg.IOComponentStatus()
      if self.devices is None:
        self.devices = None
      if self.commands is None:
        self.commands = None
      end = 0
      _x = self
      start = end
      end += 8
      (_x.time.secs, _x.time.nsecs,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.node.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node.status.tag = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.node.status.tag = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node.status.id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.node.status.id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.node.status.detail = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.node.status.detail = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.devices = []
      for i in range(0, length):
        val1 = intera_core_msgs.msg.IOComponentStatus()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        _v4 = val1.status
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v4.tag = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v4.tag = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v4.id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v4.id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v4.detail = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v4.detail = str[start:end]
        self.devices.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.commands = []
      for i in range(0, length):
        val1 = genpy.Time()
        _x = val1
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        self.commands.append(val1)
      self.time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
