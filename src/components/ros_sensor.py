import logging
import rclpy
from threading import Lock
from utils import RclpyNodeManager
from viam.logging import getLogger
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple
from typing_extensions import Self
from viam.components.sensor import Sensor
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from rclpy.node import Node
from irobot_create_msgs.msg import HazardDetectionVector
from .viam_ros_node import ViamRosNode


class RosIrobotHazardNode(Node):
    def __init__(self, hazard_topic, node_name):
        super().__init__(node_name, enable_rosout=True)
        self.lock = Lock()
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscriber = self.create_subscription(
            HazardDetectionVector,
            hazard_topic,
            self.subscriber_callback,
            qos_profile=qos_policy
        )
        self.msg = None

    def subscriber_callback(self, msg):
        self.get_logger().debug(f'subscriber_callback(): {self.get_name()} -> {msg}')
        with self.lock:
            self.msg = msg


class RosSensor(Sensor, Reconfigurable):
    # TODO: Make sensor generic with the ability to customize nodes and return messages
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'sensor')
    ros_topic: str
    ros_node: ViamRosNode
    logger: logging.Logger

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        sensor = cls(config.name)
        sensor.ros_node = None
        sensor.logger = getLogger(f'{__name__}.{sensor.__class__.__name__}')
        sensor.reconfigure(config, dependencies)
        return sensor

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['ros_topic'].string_value
        msg_type = config.attributes.fields['ros_msg_type'].string_value

        if topic == '':
            raise Exception('ros_topic required')

        if msg_type == '':
            raise Exception('ros_msg_type required')
        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.ros_topic = config.attributes.fields['ros_topic'].string_value

        if self.ros_node is not None:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        rcl_mgr = RclpyNodeManager.get_instance()
        rcl_mgr.remove_node(self.ros_node)
        rcl_mgr.spin_and_add_node(self.ros_node)

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]]=None,
        timeout: Optional[float]=None,
        **kwargs
    ) -> Mapping[str, Any]:
        self.logger.info(f'get_readings: {self.ros_node.msg}')
        return {'value': 0}



Registry.register_resource_creator(
    Sensor.SUBTYPE,
    RosSensor.MODEL,
    ResourceCreatorRegistration(RosSensor.new, RosSensor.validate_config)
)
