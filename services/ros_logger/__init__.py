"""
This file registers the speech subtype with the Viam Registry, as well as the specific SpeechIOService model.
"""

from viam.resource.registry import Registry, ResourceCreatorRegistration, ResourceRegistration
from services.ros_logger.my_ros_logger import RosLogger



Registry.register_subtype(ResourceRegistration(SpeechService, SpeechRPCService, lambda name, channel: SpeechClient(name, channel)))
#Registry.register_subtype(ResourceRegistration(SpeechService, SpeechRPCService, lambda name, channel: SpeechClient(name, channel)))

Registry.register_subtype(ResourceRegistration())

Registry.register_resource_creator(
    RosLogger.SUBTYPE,
    RosLogger.MODEL,
    ResourceCreatorRegistration(RosLogger.new, RosLogger.validate_config),
)