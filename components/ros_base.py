from threading import Lock, Thread
from ros_utils import multiThreadedExecutor
import viam
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
from geometry_msgs.msg import Twist

logger = getLogger(__name__)


class RosBaseNode(Node):
    def __init__(self, base_topic):
        super().__init__('ros_base_node', enable_rosout=True)
        self.base_pub = self.create_publisher(
            Twist,
            base_topic,
            10
        )
        self.lock = Lock()
        self.twist_msg = Twist()
        self.publisher = self.create_publisher(Twist, base_topic, 10)
        self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f'{self.get_name()} -> {self.twist_msg}')
        self.publisher.publish(self.twist_msg)

    def destroy(self):
        self.destroy_publisher(self.base_pub)
        self.destroy_node()


class RosBase(Base, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'base')
    ros_topic: str
    is_base_moving: bool
    ros_node: Node
    base_thread: Thread

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        base = cls(config.name)
        base.ros_node = None
        base.base_thread = None
        base.reconfigure(config, dependencies)
        return base

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['ros_topic'].string_value
        if topic == '':
            raise Exception('ros_topic required')
        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        self.is_base_moving = False
        #if self.ros_node != None:
        #    self.ros_node.destroy()
        #if self.base_thread:
        #    self.base_thread.join()
        self.ros_node = RosBaseNode(self.ros_topic)

        # TODO: dumb bug need to create one instance of this
        multiThreadedExecutor.add_node(self.ros_node)
        self.is_base_moving = False

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
        with self.ros_node.lock:
            self.ros_node.twist_msg.linear.x = linear.y
            self.ros_node.twist_msg.angular.z = angular.z
            self.is_base_moving = True
        self.ros_node.get_logger().info(f'set_power: done')

    async def set_velocity(
            self,
            linear: viam.components.base.Vector3,
            angular: viam.components.base.Vector3,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs
    ) -> None:
        with self.ros_node.lock:
            self.ros_node.twist_msg.linear.x = linear.y
            self.ros_node.twist_msg.angular.z = angular.z
            self.is_base_moving = True
        self.ros_node.get_logger().info(f'set_velocity: done')

    async def stop(
            self,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs
    ) -> None:
        with self.ros_node.lock:
            self.ros_node.twist_msg.linear.x = 0.0
            self.ros_node.twist_msg.angular.z = 0.0
            self.is_base_moving = False
        self.ros_node.get_logger().info(f'stop: done')

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
