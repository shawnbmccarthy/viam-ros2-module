from threading import Lock, Thread
import rclpy
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
from geometry_msgs.msg import Twist

logger = getLogger(__name__)


class RosBaseNode(Node):
    def __init__(self, base_topic, node_name):
        super().__init__(node_name, enable_rosout=True)
        self.lock = Lock()
        self.twist_msg = Twist()
        self.publisher = self.create_publisher(Twist, base_topic, 10)
        self.create_timer(0.2, self.timer_callback)
        self.get_logger().debug('RosBaseNode(): created node')

    def timer_callback(self):
        self.get_logger().debug(f'timer_callback(): {self.get_name()} -> {self.twist_msg}')
        self.publisher.publish(self.twist_msg)


class RosBase(Base, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'base')
    ros_topic: str
    is_base_moving: bool
    ros_node: Node
    node_name: str
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
        self.node_name = config.attributes.fields['ros_node_name'].string_value

        if self.node_name == '' or self.node_name == None:
            self.node_name = 'VIAM_ROS_BASE_NODE'

        self.is_base_moving = False
        self.ros_node = RosBaseNode(self.ros_topic, self.node_name)
        # TODO: validate reconfigure works 
        #       issue around node name (what happens when we change the node name?)
        rcl_mgr = RclpyNodeManager.get_instance()
        rcl_mgr.remove_node(self.ros_node)
        rcl_mgr.spin_and_add_node(self.ros_node)
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
        self.ros_node.get_logger().debug(f'set_power: {self.ros_node.twist_msg}')

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
        self.ros_node.get_logger().debug(f'set_velocity: {self.ros_node.twist_msg}')

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
        self.ros_node.get_logger().debug(f'stop: done')

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
