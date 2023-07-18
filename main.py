import asyncio
import rclpy
import sys
import threading

from viam.components.base import Base
from viam.module.module import Module
from components import DummyNode, RosBase

rclpy_thread: threading.Thread = None


def rclpy_init():
    global rclpy_thread
    rclpy.init(args=None)
    dn = DummyNode()
    rclpy_thread = threading.Thread(target=rclpy.spin, args=(dn, ), daemon=True)
    rclpy_thread.start()


async def main(addr: str) -> None:
    m = Module(addr)
    m.add_model_from_registry(Base.SUBTYPE, RosBase.MODEL)
    await m.start()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise Exception('need socket path as cmd line arg')
    rclpy.init(args=None)
    asyncio.run(main(sys.argv[1]))
    if rclpy_thread:
        rclpy_thread.join()
