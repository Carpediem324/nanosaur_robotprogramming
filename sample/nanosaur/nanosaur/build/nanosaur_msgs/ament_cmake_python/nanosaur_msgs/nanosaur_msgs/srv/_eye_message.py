# generated from rosidl_generator_py/resource/_idl.py.em
# with input from nanosaur_msgs:srv/EyeMessage.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EyeMessage_Request(type):
    """Metaclass of message 'EyeMessage_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'BOTH': 0,
        'LEFT': 1,
        'RIGHT': 2,
        'FULL': 0,
        'WIDE': 1,
        'TWO_LINES': 2,
        'INF': -1,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('nanosaur_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'nanosaur_msgs.srv.EyeMessage_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__eye_message__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__eye_message__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__eye_message__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__eye_message__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__eye_message__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'BOTH': cls.__constants['BOTH'],
            'LEFT': cls.__constants['LEFT'],
            'RIGHT': cls.__constants['RIGHT'],
            'FULL': cls.__constants['FULL'],
            'WIDE': cls.__constants['WIDE'],
            'TWO_LINES': cls.__constants['TWO_LINES'],
            'INF': cls.__constants['INF'],
        }

    @property
    def BOTH(self):
        """Message constant 'BOTH'."""
        return Metaclass_EyeMessage_Request.__constants['BOTH']

    @property
    def LEFT(self):
        """Message constant 'LEFT'."""
        return Metaclass_EyeMessage_Request.__constants['LEFT']

    @property
    def RIGHT(self):
        """Message constant 'RIGHT'."""
        return Metaclass_EyeMessage_Request.__constants['RIGHT']

    @property
    def FULL(self):
        """Message constant 'FULL'."""
        return Metaclass_EyeMessage_Request.__constants['FULL']

    @property
    def WIDE(self):
        """Message constant 'WIDE'."""
        return Metaclass_EyeMessage_Request.__constants['WIDE']

    @property
    def TWO_LINES(self):
        """Message constant 'TWO_LINES'."""
        return Metaclass_EyeMessage_Request.__constants['TWO_LINES']

    @property
    def INF(self):
        """Message constant 'INF'."""
        return Metaclass_EyeMessage_Request.__constants['INF']


class EyeMessage_Request(metaclass=Metaclass_EyeMessage_Request):
    """
    Message class 'EyeMessage_Request'.

    Constants:
      BOTH
      LEFT
      RIGHT
      FULL
      WIDE
      TWO_LINES
      INF
    """

    __slots__ = [
        '_display',
        '_type',
        '_message',
        '_timeout',
    ]

    _fields_and_field_types = {
        'display': 'int8',
        'type': 'int8',
        'message': 'sequence<string>',
        'timeout': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.display = kwargs.get('display', int())
        self.type = kwargs.get('type', int())
        self.message = kwargs.get('message', [])
        self.timeout = kwargs.get('timeout', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.display != other.display:
            return False
        if self.type != other.type:
            return False
        if self.message != other.message:
            return False
        if self.timeout != other.timeout:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def display(self):
        """Message field 'display'."""
        return self._display

    @display.setter
    def display(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'display' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'display' field must be an integer in [-128, 127]"
        self._display = value

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'type' field must be an integer in [-128, 127]"
        self._type = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'message' field must be a set or sequence and each value of type 'str'"
        self._message = value

    @builtins.property
    def timeout(self):
        """Message field 'timeout'."""
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timeout' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'timeout' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._timeout = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_EyeMessage_Response(type):
    """Metaclass of message 'EyeMessage_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('nanosaur_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'nanosaur_msgs.srv.EyeMessage_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__eye_message__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__eye_message__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__eye_message__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__eye_message__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__eye_message__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EyeMessage_Response(metaclass=Metaclass_EyeMessage_Response):
    """Message class 'EyeMessage_Response'."""

    __slots__ = [
        '_done',
    ]

    _fields_and_field_types = {
        'done': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.done = kwargs.get('done', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.done != other.done:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def done(self):
        """Message field 'done'."""
        return self._done

    @done.setter
    def done(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'done' field must be of type 'bool'"
        self._done = value


class Metaclass_EyeMessage(type):
    """Metaclass of service 'EyeMessage'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('nanosaur_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'nanosaur_msgs.srv.EyeMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__eye_message

            from nanosaur_msgs.srv import _eye_message
            if _eye_message.Metaclass_EyeMessage_Request._TYPE_SUPPORT is None:
                _eye_message.Metaclass_EyeMessage_Request.__import_type_support__()
            if _eye_message.Metaclass_EyeMessage_Response._TYPE_SUPPORT is None:
                _eye_message.Metaclass_EyeMessage_Response.__import_type_support__()


class EyeMessage(metaclass=Metaclass_EyeMessage):
    from nanosaur_msgs.srv._eye_message import EyeMessage_Request as Request
    from nanosaur_msgs.srv._eye_message import EyeMessage_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
