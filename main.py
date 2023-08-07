#!/usr/bin/python3

# TODO: handler does not seem to execute properly
import asyncio
import signal
import sys

from viam.components.base import Base
from viam.components.movement_sensor import MovementSensor
from viam.logging import getLogger
from viam.module.module import Module
from components import RosBase, RosImu
from utils import RclpyNodeManager

logger = getLogger(__name__)

rclpy_mgr = None


def sigterm_handler(_signo, _stack_frame):
    logger.info('attempting rclpy shutdown')
    sys.exit(0)

    
async def main(addr: str) -> None:
    try:
        global rclpy_mgr
        logger.info('starting ros2 module server')
        rclpy_mgr = RclpyNodeManager.get_instance()
        m = Module(addr)
        m.add_model_from_registry(Base.SUBTYPE, RosBase.MODEL)
        m.add_model_from_registry(MovementSensor.SUBTYPE, RosImu.MODEL)
        await m.start()
    finally:
        rclpy_mgr.shutdown()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise Exception('need socket path as cmd line arg')
    signal.signal(signal.SIGTERM, sigterm_handler)
    asyncio.run(main(sys.argv[1]))
