import logging
from threading import Lock
from typing import ClassVar, Mapping, Sequence

import rclpy
from rcl_interfaces.msg import Log
from typing_extensions import Self
from viam.logging import getLogger
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ServiceConfig
from viam.resource.base import ResourceBase, ResourceName
from viam.resource.types import Model, ModelFamily

from components.viam_ros_node import ViamRosNode

from .api import ROS2LoggerService


class MyROS2LoggerService(ROS2LoggerService, Reconfigurable):
    """
    A service which takes http://docs.ros.org/en/api/rosgraph_msgs/html/msg/Log.html messages from the configured ROS topic e.g. /rosout or /rosout_agg and logs them to Viam
    """

    MODEL: ClassVar[Model] = Model(ModelFamily("viamlabs", "ros2"), "ros2_logger")

    # Instance variables
    ros_topic: str
    log_level: str
    ros_node: ViamRosNode
    logger: logging.Logger

    def __init__(self, name: str):
        super().__init__(name)

    # Constructor
    @classmethod
    def new(
        cls, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        service = cls(config.name)
        service.ros_node = None
        service.logger = getLogger(f"{__name__}.{service.__class__.__name__}")
        service.reconfigure(config, dependencies)
        return service

    # Validates JSON Configuration
    @classmethod
    def validate_config(cls, config: ServiceConfig) -> Sequence[str]:
        ros_topic = config.attributes.fields["ros_topic"].string_value
        if ros_topic == "":
            raise Exception("A ros_topic attribute is required for this service.")
        return []

    # Handles attribute reconfiguration
    def reconfigure(
        self, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        self.ros_topic = config.attributes.fields["ros_topic"].string_value
        if self.ros_node is not None:
            if self.subscription is not None:
                self.ros_node.destroy_subscription(self.subscription)
        else:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        self.log_level = config.attributes.fields["log_level"].string_value

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.ros_node.create_subscription(
            Log, self.ros_topic, self.subscriber_callback, qos_profile=qos_policy
        )
        self.lock = Lock()

    def subscriber_callback(self, ros_log: Log) -> None:
        # ROS Log Messages: http://docs.ros.org/en/api/rosgraph_msgs/html/msg/Log.html

        levels = {"debug": 1, "info": 2, "warn": 4, "error": 8, "critical": 16}
        message = f"node name: {ros_log.name}, message: {ros_log.msg}, severity level: {str(ros_log.level)}"

        if ros_log.level <= 1 and ros_log.level >= levels[self.log_level]:
            self.logger.debug(message)
        elif ros_log.level <= 2 and ros_log.level >= levels[self.log_level]:
            self.logger.info(message)
        elif ros_log.level <= 4 and ros_log.level >= levels[self.log_level]:
            self.logger.warn(message)
        elif ros_log.level <= 8 and ros_log.level >= levels[self.log_level]:
            self.logger.error(message)
        elif ros_log.level <= 16 and ros_log.level >= levels[self.log_level]:
            self.logger.critical(message)
        elif ros_log.level > 16:
            self.logger.info(f"{message}")
        else:
            pass