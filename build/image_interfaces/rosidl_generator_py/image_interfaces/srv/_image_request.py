# generated from rosidl_generator_py/resource/_idl.py.em
# with input from image_interfaces:srv/ImageRequest.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ImageRequest_Request(type):
    """Metaclass of message 'ImageRequest_Request'."""

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
            module = import_type_support('image_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'image_interfaces.srv.ImageRequest_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__image_request__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__image_request__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__image_request__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__image_request__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__image_request__request

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ImageRequest_Request(metaclass=Metaclass_ImageRequest_Request):
    """Message class 'ImageRequest_Request'."""

    __slots__ = [
        '_image',
    ]

    _fields_and_field_types = {
        'image': 'sensor_msgs/Image',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import Image
        self.image = kwargs.get('image', Image())

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
        if self.image != other.image:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def image(self):
        """Message field 'image'."""
        return self._image

    @image.setter
    def image(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'image' field must be a sub message of type 'Image'"
        self._image = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ImageRequest_Response(type):
    """Metaclass of message 'ImageRequest_Response'."""

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
            module = import_type_support('image_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'image_interfaces.srv.ImageRequest_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__image_request__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__image_request__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__image_request__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__image_request__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__image_request__response

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ImageRequest_Response(metaclass=Metaclass_ImageRequest_Response):
    """Message class 'ImageRequest_Response'."""

    __slots__ = [
        '_num_apples',
        '_apple_coordinates',
    ]

    _fields_and_field_types = {
        'num_apples': 'int16',
        'apple_coordinates': 'sequence<geometry_msgs/Point>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.num_apples = kwargs.get('num_apples', int())
        self.apple_coordinates = kwargs.get('apple_coordinates', [])

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
        if self.num_apples != other.num_apples:
            return False
        if self.apple_coordinates != other.apple_coordinates:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def num_apples(self):
        """Message field 'num_apples'."""
        return self._num_apples

    @num_apples.setter
    def num_apples(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_apples' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'num_apples' field must be an integer in [-32768, 32767]"
        self._num_apples = value

    @builtins.property
    def apple_coordinates(self):
        """Message field 'apple_coordinates'."""
        return self._apple_coordinates

    @apple_coordinates.setter
    def apple_coordinates(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
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
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'apple_coordinates' field must be a set or sequence and each value of type 'Point'"
        self._apple_coordinates = value


class Metaclass_ImageRequest(type):
    """Metaclass of service 'ImageRequest'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('image_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'image_interfaces.srv.ImageRequest')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__image_request

            from image_interfaces.srv import _image_request
            if _image_request.Metaclass_ImageRequest_Request._TYPE_SUPPORT is None:
                _image_request.Metaclass_ImageRequest_Request.__import_type_support__()
            if _image_request.Metaclass_ImageRequest_Response._TYPE_SUPPORT is None:
                _image_request.Metaclass_ImageRequest_Response.__import_type_support__()


class ImageRequest(metaclass=Metaclass_ImageRequest):
    from image_interfaces.srv._image_request import ImageRequest_Request as Request
    from image_interfaces.srv._image_request import ImageRequest_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
