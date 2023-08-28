from typing import ClassVar, Final, Mapping, Sequence, Optional
from typing_extensions import Self
import logging
import rclpy
from threading import Lock
from viam.services.service_base import ServiceBase
from viam.module.types import Reconfigurable
from viam.resource.types import Model, ModelFamily
from viam.proto.app.robot import ServiceConfig
from viam.resource.base import ResourceBase, ResourceName
from viam.utils import ValueTypes
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import RESOURCE_TYPE_SERVICE, Subtype
from rcl_interfaces.msg import Log

from components.viam_ros_node import ViamRosNode

class RosLogger(ServiceBase, Reconfigurable):
    """
    A service which takes messages from the ros_out_agg topic and loggs them to Viam
    """
    SUBTYPE: Final = Subtype("viamlabs", RESOURCE_TYPE_SERVICE, "ros_logger")
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
        service.reconfigure(config, dependencies)
        return service

    # Validates JSON Configuration
    @classmethod
    def validate_config(cls, config: ServiceConfig) -> Sequence[str]:
        ros_topic = config.attributes.fields["ros_topic"].string_value
        if ros_topic == "":
            raise Exception("A ros_topic attribute is required for this service.")
        ros_topic= [config.attributes.fields["ros_topic"].string_value]
        return [ros_topic]

    # Handles attribute reconfiguration
    def reconfigure(self, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.ros_topic = config.attributes.fields["ros_topic"].string_value

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

    def subscriber_callback(self, log) -> None:
        self.logger.warning("LoggerService")

    # Handles arbitrary commands to the Resource
    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        raise NotImplementedError()
    