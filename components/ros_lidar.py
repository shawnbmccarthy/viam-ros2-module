import logging
import numpy as np
import rclpy
import viam

from rclpy.node import Node
from PIL.Image import Image
from sensor_msgs.msg import LaserScan
from threading import Lock
from typing import ClassVar, List, Mapping, Optional, Sequence, Tuple, Union
from typing_extensions import Self
from utils import RclpyNodeManager
from viam.components.camera import Camera, DistortionParameters, IntrinsicParameters, RawImage
from viam.logging import getLogger
from viam.media.video import NamedImage 
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, ResponseMetadata
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily


class RosLidarNode(Node):
    # TODO: enable_rosout should be arg
    def __init__(self, lidar_topic, node_name):
        super().__init__(node_name, enable_rosout=True)
        self.lock = Lock()
        self.msg = None
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscriber = self.create_subscription(LaserScan, lidar_topic, self.subscriber_callback, qos_profile=qos_policy)

    def subscriber_callback(self, msg):
        with self.lock:
            self.get_logger().info(f'msg -> {msg}')
            self.msg = msg


class RosLidarProperties(Camera.Properties):
    def __init__(self):
        super().__init__(
            supports_pcd=True,
            intrinsic_parameters=IntrinsicParameters(width_px=0, height_px=0, focal_x_px=0.0, focal_y_px=0.0, center_x_px=0.0),
            distortion_parameters=DistortionParameters(model='')
        )


class RosLidar(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'lidar')
    logger: logging.Logger
    node_name: str
    ros_topic: str
    ros_node: Node
    ros_lidar_properties: Camera.Properties

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        lidar = cls(config.name)
        lidar.ros_node = None
        lidar.logger = getLogger(f'{__name__}.{lidar.__class__.__name__}')
        lidar.reconfigure(config, dependencies)
        return lidar

    @classmethod
    def  validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['ros_topic'].string_value
        if topic == '':
            raise Exception('ros_topic required')
        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        self.node_name = config.attributes.fields['ros_node_name'].string_value
        self.ros_lidar_properties = Camera.Properties(
            supports_pcd=True,
            intrinsic_parameters=IntrinsicParameters(width_px=0, height_px=0, focal_x_px=0.0, focal_y_px=0.0, center_x_px=0.0),
            distortion_parameters=DistortionParameters(model='')
        )
        if self.node_name == '' or self.node_name == None:
            self.node_name = 'VIAM_ROS_LIDAR_NODE'
        
        rcl_mgr = RclpyNodeManager.get_instance()
        if self.ros_node != None:
            rcl_mgr.remove_node(self.ros_node)

        self.ros_node = RosLidarNode(self.ros_topic, self.node_name)
        rcl_mgr.spin_and_add_node(self.ros_node)

    async def get_image(self, mime_type: str='', *, timeout: Optional[float]=None, **kwargs) -> Union[Image, RawImage]:
        raise NotImplementedError()

    async def get_images(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[List[NamedImage], ResponseMetadata]:
        raise NotImplementedError()

    async def get_point_cloud(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[bytes, str]:
        if self.ros_node.msg == None:
            raise Exception('laserscan msg not ready')
        self.logger.info(f'lidar msg: {self.ros_node.msg}')
        raise NotImplementedError()

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        return self.ros_lidar_properties



Registry.register_resource_creator(
    Camera.SUBTYPE,
    RosLidar.MODEL,
    ResourceCreatorRegistration(RosLidar.new, RosLidar.validate_config)
)

