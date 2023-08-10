import logging
from threading import Lock
import viam
from utils import RclpyNodeManager
from viam.logging import getLogger
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple
from typing_extensions import Self
from viam.components.base import Base
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Rate
from geometry_msgs.msg import Twist
from .viam_ros_node import ViamRosNode

logger = getLogger(__name__)


class RosBase(Base, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'base')
    is_base_moving: bool
    lock: Lock
    logger: logging.Logger
    publisher: Publisher
    publish_rate: float
    rate: Rate
    ros_node: ViamRosNode
    ros_topic: str
    twist_msg: Twist

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        base = cls(config.name)
        base.ros_node = None
        base.reconfigure(config, dependencies)
        return base

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['ros_topic'].string_value
        publish_rate = float(config.attributes.fields['publish_rate'].string_value)

        if topic == '':
            raise Exception('ros_topic required')

        if publish_rate == 0.0:
            raise Exception('rate required')

        return []

    def ros_publisher_cb(self):
        self.publisher.publish(self.twist_msg)


    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        self.publish_rate = float(config.attributes.fields['publish_rate'].string_value)
        self.twist_msg = Twist()

        if self.ros_node is not None:
            if self.publisher is not None:
                self.ros_node.destroy_publisher(self.publisher)
            if self.rate is not None:
                self.ros_node.destroy_rate(self.rate)
        else:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        self.publisher = self.ros_node.create_publisher(Twist, self.ros_topic, 10)
        self.rate = self.ros_node.create_timer(self.publish_rate, self.ros_publisher_cb)
        self.is_base_moving = False
        self.lock = Lock()

    async def move_straight(self, distance: int, velocity: float, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        raise NotImplementedError()

    async def spin(self, angle: float, velocity: float, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        raise NotImplementedError()

    async def set_power(
            self,
            linear: viam.components.base.Vector3,
            angular: viam.components.base.Vector3,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None, **kwargs
    ) -> None:
        with self.lock:
            self.twist_msg.linear.x = linear.y
            self.twist_msg.angular.z = angular.z
            self.is_base_moving = True
        logger.debug(f'set_power: {self.twist_msg}')

    async def set_velocity(
            self,
            linear: viam.components.base.Vector3,
            angular: viam.components.base.Vector3,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs
    ) -> None:
        with self.lock:
            self.twist_msg.linear.x = linear.y
            self.twist_msg.angular.z = angular.z
            self.is_base_moving = True
        logger.debug(f'set_velocity: {self.twist_msg}')

    async def stop(
            self,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs
    ) -> None:
        with self.lock:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.is_base_moving = False
        logger.debug(f'stop: done')

    async def is_moving(self):
        return self.is_base_moving

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs):
        raise NotImplementedError()

    async def do_command(
            self,
            command: Mapping[str, ValueTypes],
            *,
            timeout: Optional[float] = None,
            **kwargs
    ):
        raise NotImplementedError()


Registry.register_resource_creator(
    Base.SUBTYPE,
    RosBase.MODEL,
    ResourceCreatorRegistration(RosBase.new, RosBase.validate_config)
)
