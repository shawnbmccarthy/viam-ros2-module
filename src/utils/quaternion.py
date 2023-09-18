"""
This is a utility library to convert quaternions to viams
orientation vector
"""
import ctypes
from ctypes import Structure, POINTER, c_double
from viam.components.movement_sensor import Orientation

# this will be loaded using the LD_LIBRARY PATH
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


def quaternion_to_orientation(real: float, i: float, j: float, k: float) -> Orientation:
    quat_obj = lib.new_quaternion(real, i, j, k)
    orientation_vec = lib.orientation_vector_from_quaternion(quat_obj)
    lib.free_quaternion_memory(quat_obj)
    o_x, o_y, o_z, theta = lib.orientation_vector_get_components(orientation_vec).contents
    lib.free_orientation_vector_memory(orientation_vec)
    return Orientation(o_x=o_x, o_y=o_y, o_z=o_z, theta=theta)
