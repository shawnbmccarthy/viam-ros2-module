from threading import Lock, Thread
import rclpy
import viam
from utils import RclpyNodeManager
from viam.logging import getLogger
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple
from typing_extensions import Self
from viam.components.movement_sensor import MovementSensor
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from rclpy.node import Node
from geometry_msgs.msg import Imu

logger = getLogger(__name__)


class RosImuNode(Node):
    def __init__(self, imu_topic, node_name):
        super().__init__(node_name, enable_rosout=True)
        self.lock = Lock()
        self.subscriber = self.create_subscription(Imu, imu_topic, self.listener_callback, 10)
        self.get_logger().info('RosImuNode(): created node')

    def listener_callback(self, msg):
        self.get_logger().debug(f'listener_callback(): {self.get_name()} -> {msg.data}')


class RosImu(MovementSensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'imu')
    ros_topic: str
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
            self.node_name = 'VIAM_ROS_IMU_NODE'

        self.ros_node = RosBaseNode(self.ros_topic, self.node_name)
        # TODO: validate reconfigure works 
        #       issue around node name (what happens when we change the node name?)
        rcl_mgr = RclpyNodeManager.get_instance()
        rcl_mgr.remove_node(self.ros_node)
        rcl_mgr.spin_and_add_node(self.ros_node)
        self.is_base_moving = False

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[viam.components.movement_sensor.GeoPoint, float]:
        raise NotImplementedError()

    async def get_linear_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        raise NotImplementedError()

    async def get_angular_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        raise NotImplementedError()


    async def get_linear_acceleration(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        raise NotImplementedError()

    async def get_compass_heading(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> float:
        raise NotImplementedError()

    async def get_orientation(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Orientation:
        raise NotImplementedError()

    async def get_accuracy(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, float]:
        raise NotImplementedError()

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
    ResourceCreatorRegistration(RosImu.new, RosImu.validate_config)
)
