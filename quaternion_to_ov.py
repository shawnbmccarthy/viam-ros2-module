import sys
import ctypes
import logging
import rclpy
import viam
from ctypes import Structure, POINTER, c_double
from threading import Lock, Thread
from utils import RclpyNodeManager
from viam.logging import getLogger
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple
from typing_extensions import Self
from viam.components.movement_sensor import MovementSensor, Orientation, Vector3
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

lib = ctypes.cdll.LoadLibrary("libviam_rust_utils.so")

class OrientationVectorS(Structure):
    pass

class QuaternionS(Structure):
    pass


quaternion_array = c_double * 4
orientation_vector_array = c_double * 4

lib.new_orientation_vector.restype = POINTER(OrientationVectorS)
lib.new_orientation_vector.argtypes = (c_double, c_double, c_double, c_double)
lib.free_orientation_vector_memory.argtypes = (POINTER(OrientationVectorS),)
lib.orientation_vector_from_quaternion.argtypes = (POINTER(QuaternionS),)
lib.orientation_vector_from_quaternion.restype = POINTER(OrientationVectorS)
lib.orientation_vector_get_components.argtypes = (POINTER(OrientationVectorS),)
lib.orientation_vector_get_components.restype = POINTER(orientation_vector_array)

lib.new_quaternion.restype = POINTER(QuaternionS)
lib.new_quaternion.argtypes = (c_double, c_double, c_double, c_double)
lib.free_quaternion_memory.argtypes = (POINTER(QuaternionS),)
lib.quaternion_from_orientation_vector.argtypes = (POINTER(OrientationVectorS),)
lib.quaternion_from_orientation_vector.restype = POINTER(QuaternionS)
lib.quaternion_get_components.argtypes = (POINTER(QuaternionS),)
lib.quaternion_get_components.restype = POINTER(quaternion_array)


def quaternion_to_orientation(real, i, j, k):
    quat_obj = lib.new_quaternion(real, i, j, k)
    orientation_vec = lib.orientation_vector_from_quaternion(quat_obj)
    lib.free_quaternion_memory(quat_obj)
    o_x, o_y, o_z, theta = lib.orientation_vector_get_components(orientation_vec).contents
    lib.free_orientation_vector_memory(orientation_vec)
    return Orientation(o_x=o_x, o_y=o_y, o_z=o_z, theta=theta)


if __name__ == '__main__':
    x = -0.0015423791483044624
    y = -0.015031004324555397
    z = -0.08984685689210892
    w = 0.9958409667015076

    print(quaternion_to_orientation(w, x, y, z))

