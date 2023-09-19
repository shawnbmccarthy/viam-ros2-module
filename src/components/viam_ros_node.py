from rclpy.node import Node
from utils import RclpyNodeManager


class ViamRosNode(Node):
    """
    ViamRosNode
    A single node to integrate with the ROS robot

    TODO: Test namespace changes
    """
    node = None

    @classmethod
    def get_viam_ros_node(cls, namespace: str = '', node_name: str = 'viam_ros_node', enable_rosout: bool = True):
        if cls.node is None:
            cls.node = ViamRosNode(namespace, node_name, enable_rosout)
        return cls.node

    def __init__(self, namespace: str = '', node_name: str = 'viam_ros_node', enable_rosout: bool = True):
        super().__init__(node_name, namespace=namespace, enable_rosout=enable_rosout)
        rclpy_mgr = RclpyNodeManager.get_instance()
        rclpy_mgr.spin_and_add_node(self)
        self.get_logger().debug('created base node')
