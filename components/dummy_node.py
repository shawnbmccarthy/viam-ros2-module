from rclpy.node import Node


class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')