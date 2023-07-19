import rclpy
from threading import Thread
from viam.logging import getLogger

class RclpyNodeManager:
    mgr = None

    @classmethod
    def get_instance(cls):
        if cls.mgr is None:
            rclpy.init(args=None)
            cls.mgr = RclpyNodeManager()
        return cls.mgr

    def __init__(self):
        self.spinning_nodes = []
        self.logger = getLogger(__name__)
        self.logger.info('initialized rclpy')
        # NOT USED YET
        # self.executor = rclpy.get_global_executor()

    def spin_and_add_node(self, node):
        self.logger.info(f'attempting to add: {node.get_name()}')
        t = Thread(target=rclpy.spin, args=(node, ), daemon=True)
        t.start()
        self.spinning_nodes.append({'node_name': node.get_name(), 'node': node, 'thread': t})

    def remove_node(self, node):
        ex = rclpy.get_global_executor()
        for i in self.spinning_nodes:
            if i['node_name'] == node.get_name():
                ex.remove_node(node)
                i['thread'].join()

    def shutdown(self):
        self.logger(f'attempting to shutdown')
        for i in self.spinning_nodes:
            n = i['node']
            n.destroy_node()
            i['thread'].join()
        rclpy.shutdown()
