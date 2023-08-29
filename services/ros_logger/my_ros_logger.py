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

from .api import SummationService


class MyRosLoggerService(SummationService, Reconfigurable):
    """
    A service which takes messages from the ROS topic configured under the service attributes as "ros_topic": with rosout or rosout_agg and logs them to Viam
    """

    MODEL: ClassVar[Model] = Model(ModelFamily("viamlabs", "ros2"), "ros_logger")

    # Instance variables
    ros_topic: str
    ros_node: ViamRosNode
    logger: logging.Logger

    def __init__(self, name: str):
        super().__init__(name)

    # Constructor
    @classmethod
    def new(cls, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
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

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.ros_node.create_subscription(
            Log, self.ros_topic, self.subscriber_callback, qos_profile=qos_policy
        )
        self.lock = Lock()

    def subscriber_callback(self, log:Log) -> None:
        # ROS Log Messages: http://docs.ros.org/en/api/rosgraph_msgs/html/msg/Log.html
        if log.level == 1:
            self.logger.debug(f'Topic: {self.ros_topic}, Node name: {log.name}, Message: {log.msg}, Level: {log.level}')
        elif log.level == 2:
            self.logger.info(f'Topic: {self.ros_topic}, Node name: {log.name}, Message: {log.msg}, Level: {log.level}')
        elif log.level == 4:
            self.logger.warn(f'Topic: {self.ros_topic}, Node name: {log.name}, Message: {log.msg}, Level: {log.level}')
        elif log.level == 8:
            self.logger.error(f'Topic: {self.ros_topic}, Node name: {log.name}, Message: {log.msg}, Level: {log.level}')
        elif log.level == 16:
            self.logger.critical(f'Topic: {self.ros_topic}, Node name: {log.name}, Message: {log.msg}, Level: {log.level}')
        else:
            self.logger.debug(f'UNDEFINED LOG LEVEL for entry: Topic: {self.ros_topic}, Node name: {log.name}, Message: {log.msg}, Level: {log.level}')


    async def sum(self, nums: Sequence[float]) -> float:
        if len(nums) <= 0:
            raise ValueError("Must provided at least one number to sum")

        result = 0
        for num in nums:
            if self.subtract:
                result -= num
            else:
                result += num
        return result
    