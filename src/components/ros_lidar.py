import logging
import math
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.subscription import Subscription
from PIL.Image import Image
from sensor_msgs.msg import LaserScan
from threading import Lock
from typing import ClassVar, List, Mapping, Optional, Sequence, Tuple, Union
from typing_extensions import Self
from viam.components.camera import Camera, DistortionParameters, IntrinsicParameters, RawImage
from viam.logging import getLogger
from viam.media.video import NamedImage 
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, ResponseMetadata
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from .viam_ros_node import ViamRosNode
from viam.media.video import CameraMimeType


class RosLidar(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'lidar')
    logger: logging.Logger
    ros_topic: str
    ros_node: Node
    subscription: Subscription
    ros_lidar_properties: Camera.Properties
    lock: Lock
    msg: LaserScan

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
        self.ros_lidar_properties = Camera.Properties(
            supports_pcd=True,
            intrinsic_parameters=IntrinsicParameters(width_px=0, height_px=0, focal_x_px=0.0, focal_y_px=0.0, center_x_px=0.0),
            distortion_parameters=DistortionParameters(model='')
        )
        
        if self.ros_node is not None:
            if self.subscription is not None:
                self.ros_node.destroy_subscription(self.subscription)
        else:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.SYSTEM_DEFAULT,
            depth=1
        )
        self.subscription = self.ros_node.create_subscription(
            LaserScan,
            self.ros_topic,
            self.subscriber_callback,
            qos_profile=qos_policy
        )
        self.lock = Lock()
        self.msg = None

    def subscriber_callback(self, msg):
        with self.lock:
            self.msg = msg

    async def get_image(self, mime_type: str='', *, timeout: Optional[float]=None, **kwargs) -> Union[Image, RawImage]:
        raise NotImplementedError()

    async def get_images(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[List[NamedImage], ResponseMetadata]:
        raise NotImplementedError()

    async def get_point_cloud(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[bytes, str]:
        if self.msg is None:
            raise Exception('laserscan msg not ready')

        version = 'VERSION .7\n'
        fields = 'FIELDS x y z\n'
        size = 'SIZE 4 4 4\n'
        type_of = 'TYPE F F F\n'
        count = 'COUNT 1 1 1\n'
        height = 'HEIGHT 1\n'
        viewpoint = 'VIEWPOINT 0 0 0 1 0 0 0\n'
        data = 'DATA binary\n'
        pdata = []
        for i, r in enumerate(self.msg.ranges):
            if r < self.msg.range_min or r > self.msg.range_max:
                continue

            ang = self.msg.angle_min + (float(i) * self.msg.angle_increment)
            y = math.sin(ang) * r
            x = math.cos(ang) * r
            pdata.append(x)
            pdata.append(y)
            pdata.append(float(0))

        width = f'WIDTH {len(pdata)}\n'
        points = f'POINTS {len(pdata)}\n'
        header = f'{version}{fields}{size}{type_of}{count}{width}{height}{viewpoint}{points}{data}'
        a = np.array(pdata, dtype='f')
        h = bytes(header, 'UTF-8')

        return h + a.tobytes(), CameraMimeType.PCD

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        return self.ros_lidar_properties

Registry.register_resource_creator(
    Camera.SUBTYPE,
    RosLidar.MODEL,
    ResourceCreatorRegistration(RosLidar.new, RosLidar.validate_config)
)

