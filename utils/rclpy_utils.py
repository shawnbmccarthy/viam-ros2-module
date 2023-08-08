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
        self.logger = getLogger(__name__)
        self.logger.debug('RclpyNodeManager: initialized rclpy')
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor_thread = None
        self.logger.debug(f'Created RclpyNodeManager {self.executor}')

    def spin_and_add_node(self, node):
        self.logger.debug(f'RclpyNodeManager: attempting to add: {node.get_name()}')
        self.executor.add_node(node)

        if self.executor_thread == None:
            self.logger.debug('creating executor thread for multithreaded executor')
            self.executor_thread = Thread(target=self.executor.spin, daemon=True)
            self.executor_thread.start()
        self.logger.debug(f'RclpyNodeManager: node({node.get_name()}) to executor')

    def remove_node(self, node):
        self.logger.debug(f'RclpyNodeManager: attempting to remove node: {node.get_name()}')
        for n in self.executor.get_nodes():
            if n.get_name() == node.get_name():
                self.logger.info(f'RclpyNodeManager: found {node.get_name()}, removing node')
                self.executor.remove_node(node)
        self.logger.debug(f'RclpyNodeManager: successfully remove node: {node.get_name()}')

    def shutdown(self):
        self.logger.info('shutting down rclpy executor & joining threads')
        rclpy.shutdown()
        self.executor_thread.join()
