import logging

import PIL
import rclpy
import viam
from threading import Lock
from viam.logging import getLogger
from typing import ClassVar, Mapping, Optional, Sequence, Tuple, Union, List
from typing_extensions import Self
from viam.components.camera import Camera, IntrinsicParameters, DistortionParameters
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from rclpy.node import Node
from rclpy.subscription import Subscription
from sensor_msgs.msg import (
    Image,
)  # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
from .viam_ros_node import ViamRosNode
from cv_bridge import CvBridge


class RosCameraProperties(Camera.Properties):
    def __init__(self):
        super().__init__()


class RosCamera(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily("viamlabs", "ros2"), "camera")
    ros_topic: str
    ros_node: Node
    subscription: Subscription
    logger: logging.Logger
    image: PIL.Image
    lock: Lock
    props: RosCameraProperties
    bridge = CvBridge()

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        camera = cls(config.name)
        camera.ros_node = None
        camera.logger = getLogger(f"{__name__}.{camera.__class__.__name__}")
        camera.props = Camera.Properties(
            supports_pcd=False,
            distortion_parameters=DistortionParameters(),
            intrinsic_parameters=IntrinsicParameters(),
        )
        camera.reconfigure(config, dependencies)
        return camera

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields["ros_topic"].string_value
        if topic == "":
            raise Exception("ros_topic required")
        return []


    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
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
            Image, self.ros_topic, self.subscriber_callback, qos_profile=qos_policy
        )
        self.lock = Lock()
        self.image = None


    def subscriber_callback(self, msg):
        with self.lock:
            self.image = PIL.Image.fromArray(self.bridge.imgmsg_to_cv2(msg))
            self.image.close()


    async def get_image(
        self, mime_type="", timeout: Optional[float] = None, **kwargs
    ) -> Union[PIL.Image.Image, viam.components.camera.RawImage]:
        if self.image is None:
            raise Exception("ros image message not ready")
        return self.image


    async def get_images(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Tuple[List[viam.media.video.NamedImage], viam.proto.common.ResponseMetadata]:
        self.logger.warning(f"get_images: not implemented")
        raise NotImplementedError()


    async def get_point_cloud(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Tuple[bytes, str]:
        self.logger.warning(f"get_point_cloud: not implemented")
        raise NotImplementedError()


    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs):
        return self.props


    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        raise NotImplementedError()


Registry.register_resource_creator(
    Camera.SUBTYPE,
    RosCamera.MODEL,
    ResourceCreatorRegistration(RosCamera.new, RosCamera.validate_config),
)
