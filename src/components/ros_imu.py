import logging
import rclpy
import viam
from threading import Lock
from utils import quaternion_to_orientation
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
from rclpy.subscription import Subscription
from sensor_msgs.msg import Imu
from .viam_ros_node import ViamRosNode


class Unimplemented(Exception):
    def __init__(self):
        super().__init__()


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
    subscription: Subscription
    logger: logging.Logger
    msg: Imu
    lock: Lock
    props: RosImuProperties

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        imu = cls(config.name)
        imu.ros_node = None
        imu.logger = getLogger(f'{__name__}.{imu.__class__.__name__}')
        imu.props = RosImuProperties()
        imu.reconfigure(config, dependencies)
        return imu

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['ros_topic'].string_value
        if topic == '':
            raise Exception('ros_topic required')
        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.ros_topic = config.attributes.fields['ros_topic'].string_value

        if self.ros_node is not None:
            if self.subscription is not None:
                self.ros_node.destroy_subscription(self.subscription)
        else:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.ros_node.create_subscription(
            Imu,
            self.ros_topic,
            self.subscriber_callback,
            qos_profile=qos_policy
        )
        self.lock = Lock()
        self.msg = None

    def subscriber_callback(self, msg):
        with self.lock:
            self.msg = msg

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[viam.components.movement_sensor.GeoPoint, float]:
        self.logger.warning('get_position: not implemented')
        raise Unimplemented()

    async def get_linear_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        self.logger.warning('get_linear_velocity: not implemented')
        raise Unimplemented()

    async def get_angular_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        if self.msg is None:
            raise Exception("ros imu message not ready")
        av = self.msg.angular_velocity
        return Vector3(x=av.x, y=av.y, z=av.z)

    async def get_linear_acceleration(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        if self.msg is None:
            raise Exception("ros imu message not ready")
        la = self.msg.linear_acceleration
        return Vector3(x=la.x, y=la.y, z=la.z)

    async def get_compass_heading(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> float:
        self.logger.warning(f'get_compass_heading: not implemented')
        raise Unimplemented()

    async def get_orientation(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Orientation:
        if self.msg is None:
            raise Exception("ros imu message not ready")
        o = self.msg.orientation
        return quaternion_to_orientation(o.w, o.x, o.y, o.z)

    async def get_accuracy(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, float]:
        raise Unimplemented()

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> MovementSensor.Properties:
        return self.props

    async def do_command(
            self,
            command: Mapping[str, ValueTypes],
            *,
            timeout: Optional[float] = None,
            **kwargs
    ):
        raise Unimplemented()
        

Registry.register_resource_creator(
    MovementSensor.SUBTYPE,
    RosImu.MODEL,
    ResourceCreatorRegistration(RosImu.new, RosImu.validate_config)
)
