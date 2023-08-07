import sys
import logging
import rclpy
import viam
from threading import Lock, Thread
from utils import quaternion_to_orientation, RclpyNodeManager
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


class RosImuNode(Node):
    def __init__(self, imu_topic, node_name):
        super().__init__(node_name, enable_rosout=True)
        self.lock = Lock()
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscriber = self.create_subscription(Imu, imu_topic, self.subscriber_callback, qos_profile=qos_policy)
        self.msg = None

    def subscriber_callback(self, msg):
        self.get_logger().debug(f'listener_callback(): {self.get_name()} -> {msg}')
        with self.lock:
            self.msg = msg


class RosImuProperties(MovementSensor.Properties):
    def __init__(self):
        super().__init__(
            linear_acceleration_supported=True,
            angular_velocity_supported=True,
            orientation_supported=True,
            position_supported=False,
            compass_heading_supported=False,
            linear_velocity_supported=False
        )

class RosImu(MovementSensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'imu')
    ros_topic: str
    ros_node: Node
    node_name: str
    base_thread: Thread
    logger: logging.Logger
    props: RosImuProperties

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        base = cls(config.name)
        base.ros_node = None
        base.base_thread = None
        base.logger = getLogger(f'{__name__}.{base.__class__.__name__}')
        base.props = RosImuProperties()
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

        self.ros_node = RosImuNode(self.ros_topic, self.node_name)
        rcl_mgr = RclpyNodeManager.get_instance()
        rcl_mgr.remove_node(self.ros_node)
        rcl_mgr.spin_and_add_node(self.ros_node)

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[viam.components.movement_sensor.GeoPoint, float]:
        self.logger.warn('get_position: not implemented')
        raise NotImplementedError()

    async def get_linear_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        self.logger.warn('get_linear_velocity: not implemented')
        raise NotImplementedError()

    async def get_angular_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        if self.ros_node.msg == None:
            raise Exception("ros imu message not ready")
        av = self.ros_node.msg.angular_velocity
        return Vector3(x=av.x, y=av.y, z=av.z)


    async def get_linear_acceleration(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        if self.ros_node.msg == None:
            raise Exception("ros imu message not ready")
        la = self.ros_node.msg.linear_acceleration
        return Vector3(x=la.x, y=la.y, z=la.z)

    async def get_compass_heading(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> float:
        self.logger.warn(f'get_compass_heading: not implemented')
        raise NotImplementedError()

    async def get_orientation(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Orientation:
        if self.ros_node.msg == None:
            raise Exception("ros imu message not ready")
        o = self.ros_node.msg.orientation
        return quaternion_to_orientation(o.w, o.x, o.y, o.z)

    async def get_accuracy(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, float]:
        raise NotImplementedError()

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs):
        return self.props

    async def do_command(
            self,
            command: Mapping[str, ValueTypes],
            *,
            timeout: Optional[float] = None,
            **kwargs
    ):
        raise NotImplementedError()
        


Registry.register_resource_creator(
    MovementSensor.SUBTYPE,
    RosImu.MODEL,
    ResourceCreatorRegistration(RosImu.new, RosImu.validate_config)
)
