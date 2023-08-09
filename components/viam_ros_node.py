from rclpy.node import Node
from threading import Lock

class ViamRosNode(Node):
    """
    ViamRosNode
    A single node to integrate with the ROS robot
    """
    def __init__(self, node_name:str='VIAM_ROS_NODE', enable_rosout:bool=True) -> None:
        super().__init__(node_name, enable_rosout)
        self.lock = Lock()
        self.logger = self.get_logger()

    def add_publisher(self, msg_type, callback, rate):
        pass

    def add_subscriber(self, msg_type, callback):
        pass