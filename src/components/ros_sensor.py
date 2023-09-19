"""
sensor.py


"""
import importlib
import logging
import rclpy
from array import array
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
from rclpy.node import Node
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
        with self.lock:
            self.msg = msg


class RosSensor(Sensor, Reconfigurable):
    # TODO: Make sensor generic with the ability to customize nodes and return messages
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'sensor')
    ros_topic: str
    ros_node: ViamRosNode
    ros_msg_type: str
    ros_sensor_cls: ClassVar
    ros_msg_package: str
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
        msg_package = config.attributes.fields['ros_msg_package'].string_value
        msg_type = config.attributes.fields['ros_msg_type'].string_value

        # TODO: Document this
        if topic == '':
            raise Exception('ros_topic required')

        if msg_package == '':
            raise Exception('ros_msg_package required')
        if msg_type == '':
            raise Exception('ros_msg_type required')

        try:
            tmp = importlib.import_module(msg_package)
            if not hasattr(tmp, msg_type):
                raise Exception(f'invalid ros_msg_type, {msg_package} does not have {ros_msg_type}')
        except ModuleNotFoundError as mnfe:
            raise Exception(f'invalid ros_msg_type: {mnfe}')

        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        self.ros_msg_package = config.attributes.fields['ros_msg_package'].string_value
        self.ros_msg_type = config.attributes.fields['ros_msg_type'].string_value

        lib = importlib.import_module(self.ros_msg_package)
        self.ros_sensor_cls = getattr(lib, self.ros_msg_type)

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
            self.ros_sensor_cls,
            self.ros_topic,
            self.subscriber_callback,
            qos_profile=qos_policy
        )
        self.lock = Lock()
        self.msg = None

        self.ros_node = ViamRosNode.get_viam_ros_node()

        rcl_mgr = RclpyNodeManager.get_instance()
        rcl_mgr.remove_node(self.ros_node)
        rcl_mgr.spin_and_add_node(self.ros_node)

    def subscriber_callback(self, msg):
        with self.lock:
            self.msg = msg

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]]=None,
        timeout: Optional[float]=None,
        **kwargs
    ) -> Mapping[str, Any]:
        """
        recursive here

        :param extra:
        :param timeout:
        :param kwargs:
        :return:
        """
        if self.msg is not None:
            t_msg = self.msg
            return build_msg(t_msg)
        return {'value': 'NOT_READY'}


def build_msg(msg):
    """

    :param msg:
    :return:
    """
    r_data = {}
    if hasattr(msg, 'get_fields_and_field_types'):
        fields_and_types = msg.get_fields_and_field_types()
        for key in fields_and_types.keys():
            r_data[key] = build_msg(getattr(msg, key))
    else:
        # builtin type
        msg_type = type(msg)
        if msg_type is list or msg_type is tuple or msg_type is set or msg_type is array:
            l_data = []
            for value in msg:
                l_data.append(build_msg(value))
            return l_data
        elif msg_type is dict:
                d_data = {}
                for key in msg.keys():
                    d_data[key] = build_msg(msg[key])
                return d_data
        else:
            return msg
    return r_data


Registry.register_resource_creator(
    Sensor.SUBTYPE,
    RosSensor.MODEL,
    ResourceCreatorRegistration(RosSensor.new, RosSensor.validate_config)
)
