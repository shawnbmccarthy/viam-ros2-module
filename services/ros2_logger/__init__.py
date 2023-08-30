"""
This file registers the ROS2LoggerService subtype with the Viam Registry, as well as the specific MyROS2LoggerService model.
"""

from viam.resource.registry import (
    Registry,
    ResourceCreatorRegistration,
    ResourceRegistration,
)

from .api import ROS2LoggerClient, ROS2LoggerRPCService, ROS2LoggerService
from .ros2_logger import MyROS2LoggerService

Registry.register_subtype(
    ResourceRegistration(
        ROS2LoggerService,
        ROS2LoggerRPCService,
        lambda name, channel: ROS2LoggerClient(name, channel),
    )
)

Registry.register_resource_creator(
    ROS2LoggerService.SUBTYPE,
    MyROS2LoggerService.MODEL,
    ResourceCreatorRegistration(MyROS2LoggerService.new),
)
